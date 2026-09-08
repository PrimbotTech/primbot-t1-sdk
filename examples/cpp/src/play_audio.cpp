/*
 @file play_audio.cpp
 @brief 音频播放示例

 @description
   通过 PulseAudio TCP 协议（simple-protocol-tcp）将音频文件发送到机器人扬声器播放。

   支持格式：
     - WAV 文件：自动检测 RIFF 头，转换为 48kHz mono S16LE
     - PCM 文件：48kHz mono S16LE，直接传输
     - 多声道 WAV 自动智能混音：识别活跃通道，避免静音通道稀释信号
     - 非 48kHz 采样率自动重采样

 @prerequisites
   示例脚本会通过 SSH 自动在机器人端加载 TCP 播放模块（端口 6001）。
   先检查模块是否已加载，已加载则跳过。

 @usage
   ./play_audio <机器人IP> <音频文件>

 @example
   ./play_audio <机器人IP> mic_mono.wav
 */

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fcntl.h>
#include <poll.h>
#include <string>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <thread>
#include <vector>

namespace {

constexpr int kRate = 48000;
constexpr int kChannels = 1;       // Mono output
constexpr int kSampleWidth = 2;    // 16-bit = 2 bytes
constexpr int kPort = 6001;
constexpr double kChunkDuration = 0.1;  // 100ms per send chunk

std::atomic<bool> g_stop{false};

}  // namespace

static void signal_handler(int) { g_stop.store(true); }

// ---------------------------------------------------------------------------
// Auto-setup: load PulseAudio TCP modules on robot via SSH
// ---------------------------------------------------------------------------

static void setup_robot_audio_tcp(const char *robot_ip) {
  // 先检查模块是否已加载，已加载则跳过，避免重复输入密码
  char cmd[1024];
  snprintf(cmd, sizeof(cmd),
           "ssh -o StrictHostKeyChecking=no -o ConnectTimeout=5 run@%s '"
           "if pactl list modules short 2>/dev/null | grep -q module-simple-protocol-tcp; then "
           "echo MODULE_ALREADY_LOADED; "
           "else "
           "pactl load-module module-simple-protocol-tcp "
           "sink=@DEFAULT_SINK@ playback=true port=%d "
           "format=s16le rate=%d channels=%d listen=0.0.0.0 && "
           "echo MODULE_LOADED; "
           "fi'",
           robot_ip, kPort, kRate, kChannels);

  FILE *fp = popen(cmd, "r");
  if (!fp) {
    printf("SSH 连接失败\n");
    return;
  }
  std::string output;
  char buf[256];
  while (fgets(buf, sizeof(buf), fp)) {
    output += buf;
  }
  int ret = pclose(fp);
  (void)ret;

  if (output.find("MODULE_ALREADY_LOADED") != std::string::npos) {
    printf("播放模块已在机器人端加载（跳过加载）\n");
  } else if (output.find("MODULE_LOADED") != std::string::npos) {
    printf("已在机器人端加载播放模块 (端口 %d)\n", kPort);
  } else {
    printf("警告: 播放模块加载可能未成功（stderr 已输出到终端）\n");
  }
}

// ---------------------------------------------------------------------------
// Binary reading helpers
// ---------------------------------------------------------------------------

static uint16_t read_u16_le(const uint8_t *p) {
  return static_cast<uint16_t>(p[0]) | (static_cast<uint16_t>(p[1]) << 8);
}

static uint32_t read_u32_le(const uint8_t *p) {
  return static_cast<uint32_t>(p[0]) |
         (static_cast<uint32_t>(p[1]) << 8) |
         (static_cast<uint32_t>(p[2]) << 16) |
         (static_cast<uint32_t>(p[3]) << 24);
}

static int16_t read_s16_le(const uint8_t *p) {
  return static_cast<int16_t>(read_u16_le(p));
}

// ---------------------------------------------------------------------------
// File loading
// ---------------------------------------------------------------------------

static bool load_file(const char *path, std::vector<uint8_t> &out) {
  FILE *f = fopen(path, "rb");
  if (!f) {
    perror(path);
    return false;
  }
  fseek(f, 0, SEEK_END);
  long size = ftell(f);
  fseek(f, 0, SEEK_SET);
  if (size <= 0) { fclose(f); return false; }
  out.resize(static_cast<size_t>(size));
  size_t n = fread(out.data(), 1, out.size(), f);
  fclose(f);
  return n == out.size();
}

// ---------------------------------------------------------------------------
// WAV loader with format conversion (operates on in-memory data)
// ---------------------------------------------------------------------------

static bool is_wav_data(const std::vector<uint8_t> &data) {
  return data.size() >= 4 && memcmp(data.data(), "RIFF", 4) == 0;
}

static bool convert_wav_to_pcm(const std::vector<uint8_t> &file,
                               std::vector<uint8_t> &pcm_out) {
  if (file.size() < 44 || memcmp(file.data(), "RIFF", 4) != 0 ||
      memcmp(file.data() + 8, "WAVE", 4) != 0) {
    printf("错误: 无效的 WAV 文件\n");
    return false;
  }

  // Walk chunks to find "fmt " and "data"
  size_t pos = 12;
  int src_rate = 0, src_channels = 0, src_sampwidth = 0;
  const uint8_t *audio_data = nullptr;
  size_t audio_size = 0;

  while (pos + 8 <= file.size()) {
    uint32_t chunk_size = read_u32_le(file.data() + pos + 4);

    if (memcmp(file.data() + pos, "fmt ", 4) == 0 && chunk_size >= 16) {
      uint16_t format = read_u16_le(file.data() + pos + 8);
      if (format != 1) {
        printf("错误: 仅支持 PCM 格式 WAV (format=%d)\n", format);
        return false;
      }
      src_channels = read_u16_le(file.data() + pos + 10);
      src_rate = static_cast<int>(read_u32_le(file.data() + pos + 12));
      uint16_t bps = read_u16_le(file.data() + pos + 22);
      src_sampwidth = bps / 8;
      if (src_sampwidth != 2) {
        printf("错误: 仅支持 16-bit PCM WAV，当前 %d-bit\n", bps);
        return false;
      }
    } else if (memcmp(file.data() + pos, "data", 4) == 0) {
      audio_data = file.data() + pos + 8;
      audio_size = chunk_size;
      if (pos + 8 + audio_size > file.size()) {
        audio_size = file.size() - pos - 8;
      }
    }

    pos += 8 + chunk_size;
    if (chunk_size % 2 != 0) pos++;
  }

  if (!audio_data || audio_size == 0) {
    printf("错误: WAV 文件中未找到 data chunk\n");
    return false;
  }

  size_t n_samples = audio_size / src_sampwidth;
  double src_duration = static_cast<double>(n_samples) / src_channels / src_rate;
  printf("WAV 格式: %dHz, %dch, 16bit, %zu帧 (%.1f秒)\n",
         src_rate, src_channels, n_samples / src_channels, src_duration);

  // Parse source samples
  std::vector<int16_t> samples(n_samples);
  for (size_t i = 0; i < n_samples; ++i) {
    samples[i] = read_s16_le(audio_data + i * src_sampwidth);
  }

  // Mono mixdown if multi-channel
  std::vector<int16_t> mono;
  if (src_channels > 1) {
    // Find active channels: per-channel peak, only mix channels with signal
    std::vector<int32_t> ch_peaks(src_channels, 0);
    for (size_t i = 0; i < n_samples; i += src_channels) {
      for (int c = 0; c < src_channels && i + c < n_samples; ++c) {
        int32_t a = std::abs(static_cast<int32_t>(samples[i + c]));
        if (a > ch_peaks[c]) ch_peaks[c] = a;
      }
    }

    // A channel is "active" if its peak exceeds a noise floor
    std::vector<int> active;
    for (int c = 0; c < src_channels; ++c) {
      if (ch_peaks[c] > 200) active.push_back(c);
    }
    if (active.empty()) {
      for (int c = 0; c < src_channels; ++c) active.push_back(c);
    }

    printf("混音: %d 声道 -> 单声道 (活跃通道:", src_channels);
    for (int c : active) printf(" ch%d", c + 1);
    printf(")\n");
    printf("各声道峰值:");
    for (int c = 0; c < src_channels; ++c) printf(" ch%d=%d", c + 1, ch_peaks[c]);
    printf("\n");

    size_t n_frames = n_samples / src_channels;
    mono.resize(n_frames);
    int n_active = static_cast<int>(active.size());
    for (size_t i = 0; i < n_frames; ++i) {
      int32_t sum = 0;
      for (int c : active) {
        sum += samples[i * src_channels + c];
      }
      int32_t avg = sum / n_active;
      if (avg > 32767) avg = 32767;
      if (avg < -32768) avg = -32768;
      mono[i] = static_cast<int16_t>(avg);
    }
  } else {
    mono = std::move(samples);
  }

  // Resample to 48kHz if needed
  std::vector<int16_t> output;
  if (src_rate != kRate) {
    printf("重采样: %dHz -> %dHz\n", src_rate, kRate);
    double ratio = static_cast<double>(kRate) / src_rate;
    size_t new_len = static_cast<size_t>(mono.size() * ratio);
    output.resize(new_len);
    for (size_t i = 0; i < new_len; ++i) {
      double src_idx = static_cast<double>(i) / ratio;
      size_t idx_low = static_cast<size_t>(src_idx);
      size_t idx_high = std::min(idx_low + 1, mono.size() - 1);
      double frac = src_idx - static_cast<double>(idx_low);
      double val = mono[idx_low] * (1.0 - frac) + mono[idx_high] * frac;
      int32_t clamped = static_cast<int32_t>(val);
      if (clamped > 32767) clamped = 32767;
      if (clamped < -32768) clamped = -32768;
      output[i] = static_cast<int16_t>(clamped);
    }
  } else {
    output = std::move(mono);
  }

  // Convert to PCM bytes
  pcm_out.resize(output.size() * kSampleWidth);
  for (size_t i = 0; i < output.size(); ++i) {
    auto v = static_cast<uint16_t>(output[i]);
    pcm_out[i * 2] = static_cast<uint8_t>(v & 0xFF);
    pcm_out[i * 2 + 1] = static_cast<uint8_t>((v >> 8) & 0xFF);
  }

  return true;
}

// ---------------------------------------------------------------------------
// TCP streaming playback
// ---------------------------------------------------------------------------

static bool connect_with_timeout(int sock, const struct sockaddr *addr,
                                 socklen_t addrlen, int timeout_sec) {
  // Set non-blocking
  int flags = fcntl(sock, F_GETFL, 0);
  if (flags < 0) return false;
  fcntl(sock, F_SETFL, flags | O_NONBLOCK);

  int ret = connect(sock, addr, addrlen);
  if (ret == 0) {
    // Connected immediately
    fcntl(sock, F_SETFL, flags);
    return true;
  }
  if (errno != EINPROGRESS) {
    fcntl(sock, F_SETFL, flags);
    return false;
  }

  // Wait for connection with timeout, checking g_stop periodically
  auto deadline = std::chrono::steady_clock::now() +
                  std::chrono::seconds(timeout_sec);
  while (std::chrono::steady_clock::now() < deadline) {
    if (g_stop.load()) {
      fcntl(sock, F_SETFL, flags);
      errno = EINTR;
      return false;
    }
    auto remaining = std::chrono::duration_cast<std::chrono::milliseconds>(
        deadline - std::chrono::steady_clock::now()).count();
    if (remaining <= 0) break;
    int wait_ms = static_cast<int>(std::min(remaining, 200L));

    struct pollfd pfd{};
    pfd.fd = sock;
    pfd.events = POLLOUT;
    int n = poll(&pfd, 1, wait_ms);
    if (n > 0 && (pfd.revents & POLLOUT)) {
      int err = 0;
      socklen_t len = sizeof(err);
      getsockopt(sock, SOL_SOCKET, SO_ERROR, &err, &len);
      fcntl(sock, F_SETFL, flags);
      if (err == 0) return true;
      errno = err;
      return false;
    }
    if (n < 0 && errno != EINTR) {
      fcntl(sock, F_SETFL, flags);
      return false;
    }
  }

  // Timed out or stopped
  fcntl(sock, F_SETFL, flags);
  errno = ETIMEDOUT;
  return false;
}

static bool play_pcm(const char *robot_ip, const std::vector<uint8_t> &pcm) {
  int sock = socket(AF_INET, SOCK_STREAM, 0);
  if (sock < 0) {
    perror("socket");
    return false;
  }

  struct sockaddr_in addr{};
  addr.sin_family = AF_INET;
  addr.sin_port = htons(kPort);
  if (inet_pton(AF_INET, robot_ip, &addr.sin_addr) <= 0) {
    printf("错误: 无效的 IP 地址: %s\n", robot_ip);
    close(sock);
    return false;
  }

  printf("正在连接 %s:%d ...\n", robot_ip, kPort);
  if (!connect_with_timeout(sock, reinterpret_cast<struct sockaddr *>(&addr),
                            sizeof(addr), 5)) {
    if (g_stop.load()) {
      printf("\n已取消\n");
    } else {
      perror("connect");
      printf("连接失败，请检查机器人 IP 是否正确以及 TCP 播放模块是否已加载。\n");
    }
    close(sock);
    return false;
  }

  int bytes_per_sec = kRate * kChannels * kSampleWidth;
  size_t chunk_bytes = static_cast<size_t>(bytes_per_sec * kChunkDuration);
  double total_secs = static_cast<double>(pcm.size()) / bytes_per_sec;

  printf("已连接 %s:%d，播放中...\n", robot_ip, kPort);

  auto next_send = std::chrono::steady_clock::now();
  size_t offset = 0;

  while (!g_stop.load() && offset < pcm.size()) {
    size_t remaining = pcm.size() - offset;
    size_t send_size = std::min(chunk_bytes, remaining);

    ssize_t sent = send(sock, pcm.data() + offset, send_size, MSG_NOSIGNAL);
    if (sent <= 0) {
      if (sent < 0) perror("send");
      printf("\n连接断开\n");
      break;
    }
    offset += static_cast<size_t>(sent);

    double played = static_cast<double>(offset) / bytes_per_sec;
    printf("\r已播放 %.1f / %.1f 秒", played, total_secs);
    fflush(stdout);

    // Precise timing: compensate for network/processing delay
    next_send += std::chrono::microseconds(
        static_cast<int64_t>(kChunkDuration * 1e6));
    auto now = std::chrono::steady_clock::now();
    if (next_send > now) {
      std::this_thread::sleep_for(next_send - now);
    }
  }

  printf("\n播放结束\n");
  close(sock);
  return true;
}

// ---------------------------------------------------------------------------
// Main
// ---------------------------------------------------------------------------

int main(int argc, char *argv[]) {
  if (argc < 3) {
    printf("用法: %s <机器人IP> <音频文件>\n", argv[0]);
    printf("支持格式: .pcm (48kHz mono S16LE), .wav (自动转换)\n");
    return 1;
  }

  const char *robot_ip = argv[1];
  const char *audio_file = argv[2];

  signal(SIGINT, signal_handler);
  signal(SIGTERM, signal_handler);

  // Auto-load TCP playback module on robot via SSH
  setup_robot_audio_tcp(robot_ip);
  if (g_stop.load()) {
    printf("\n已取消\n");
    return 0;
  }

  // Load raw file data (local)
  std::vector<uint8_t> raw;
  if (!load_file(audio_file, raw)) return 1;

  // Convert to PCM (auto-detect WAV or treat as raw PCM)
  std::vector<uint8_t> pcm;
  if (is_wav_data(raw)) {
    printf("检测到 WAV 文件，正在转换...\n");
    if (!convert_wav_to_pcm(raw, pcm)) return 1;
    int bytes_per_sec = kRate * kChannels * kSampleWidth;
    printf("转换完成: %zu 字节 (%.1f 秒)\n",
           pcm.size(), static_cast<double>(pcm.size()) / bytes_per_sec);
  } else {
    printf("按 PCM 格式使用 (假设 48kHz mono S16LE)\n");
    pcm = std::move(raw);
  }

  if (pcm.empty()) {
    printf("错误: 音频数据为空\n");
    return 1;
  }

  return play_pcm(robot_ip, pcm) ? 0 : 1;
}
