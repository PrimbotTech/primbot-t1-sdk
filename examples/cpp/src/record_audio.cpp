/*
 @file record_audio.cpp
 @brief 录音示例

 @description
   通过 PulseAudio TCP 协议从机器人麦克风阵列录音。

   录音格式：8声道 48kHz S16LE（机器人麦克风原始格式）
   声道说明：
     - ch1-2：扬声器回放
     - ch3-6：麦克风信号（ch5-6 为主要拾音通道）
     - ch7-8：全零

   输出文件：
     - raw_8ch.wav：完整 8 声道原始录音
     - mic_mono.wav：ch5-6 混合的单声道录音（10 倍放大）
   同时输出各通道音量分析，帮助识别活跃麦克风。

 @prerequisites
   - 示例脚本会通过 SSH 自动在机器人端加载 TCP 录音模块（端口 4713）
   - 本机已安装 parec：sudo apt install pulseaudio-utils

 @usage
   ./record_audio <机器人IP> [录音秒数]

 @example
   ./record_audio <机器人IP> 5
 */

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <csignal>
#include <cerrno>
#include <fcntl.h>
#include <string>
#include <sys/wait.h>
#include <thread>
#include <unistd.h>
#include <vector>

namespace {

constexpr int kRate = 48000;
constexpr int kChannels = 8;
constexpr int kSampleWidth = 2;  // 16-bit = 2 bytes
constexpr int kBytesPerFrame = kChannels * kSampleWidth;
constexpr const char *kDevice = "alsa_input.platform-aw89403_sound.pro-input-0";

std::atomic<bool> g_stop{false};

}  // namespace

static void signal_handler(int) { g_stop.store(true); }

// ---------------------------------------------------------------------------
// Auto-setup: load PulseAudio TCP modules on robot via SSH
// ---------------------------------------------------------------------------

static void setup_robot_audio_tcp(const char *robot_ip) {
  char cmd[512];
  snprintf(cmd, sizeof(cmd),
           "ssh -o StrictHostKeyChecking=no -o ConnectTimeout=5 run@%s "
           "\"pactl load-module module-native-protocol-tcp "
           "auth-anonymous=1 listen=0.0.0.0\" "
           "2>/dev/null",
           robot_ip);
  int ret = system(cmd);
  if (ret == 0) {
    printf("已在机器人端加载录音模块 (端口 4713)\n");
  }
  // Non-zero is fine — module may already be loaded
}

// ---------------------------------------------------------------------------
// WAV file writing helpers
// ---------------------------------------------------------------------------

static void write_u16_le(FILE *f, uint16_t v) {
  uint8_t buf[2] = {
      static_cast<uint8_t>(v & 0xFF),
      static_cast<uint8_t>((v >> 8) & 0xFF),
  };
  fwrite(buf, 1, 2, f);
}

static void write_u32_le(FILE *f, uint32_t v) {
  uint8_t buf[4] = {
      static_cast<uint8_t>(v & 0xFF),
      static_cast<uint8_t>((v >> 8) & 0xFF),
      static_cast<uint8_t>((v >> 16) & 0xFF),
      static_cast<uint8_t>((v >> 24) & 0xFF),
  };
  fwrite(buf, 1, 4, f);
}

static bool write_wav(const char *path, const std::vector<uint8_t> &data,
                      int channels, int rate) {
  FILE *f = fopen(path, "wb");
  if (!f) {
    perror(path);
    return false;
  }

  uint32_t data_size = static_cast<uint32_t>(data.size());
  uint32_t riff_size = 36 + data_size;

  fwrite("RIFF", 1, 4, f);
  write_u32_le(f, riff_size);
  fwrite("WAVE", 1, 4, f);

  // fmt chunk
  fwrite("fmt ", 1, 4, f);
  write_u32_le(f, 16);                              // chunk size
  write_u16_le(f, 1);                               // PCM format
  write_u16_le(f, static_cast<uint16_t>(channels));  // channels
  write_u32_le(f, static_cast<uint32_t>(rate));      // sample rate
  write_u32_le(f, static_cast<uint32_t>(rate * channels * kSampleWidth));  // byte rate
  write_u16_le(f, static_cast<uint16_t>(channels * kSampleWidth));         // block align
  write_u16_le(f, 16);                              // bits per sample

  // data chunk
  fwrite("data", 1, 4, f);
  write_u32_le(f, data_size);
  fwrite(data.data(), 1, data.size(), f);

  fclose(f);
  printf("保存: %s\n", path);
  return true;
}

// ---------------------------------------------------------------------------
// PCM sample helpers
// ---------------------------------------------------------------------------

static inline int16_t read_s16_le(const uint8_t *p) {
  return static_cast<int16_t>(
      static_cast<uint16_t>(p[0]) | (static_cast<uint16_t>(p[1]) << 8));
}

static inline void write_s16_le(uint8_t *p, int16_t v) {
  auto uv = static_cast<uint16_t>(v);
  p[0] = static_cast<uint8_t>(uv & 0xFF);
  p[1] = static_cast<uint8_t>((uv >> 8) & 0xFF);
}

static inline int16_t clamp_s16(int32_t v) {
  if (v > 32767) return 32767;
  if (v < -32768) return -32768;
  return static_cast<int16_t>(v);
}

// ---------------------------------------------------------------------------
// Main
// ---------------------------------------------------------------------------

int main(int argc, char *argv[]) {
  if (argc < 2) {
    printf("用法: %s <机器人IP> [录音秒数]\n", argv[0]);
    return 1;
  }

  signal(SIGINT, signal_handler);
  signal(SIGTERM, signal_handler);

  const char *robot_ip = argv[1];
  int duration = (argc > 2) ? atoi(argv[2]) : 5;
  if (duration <= 0) duration = 5;

  // Auto-load TCP recording module on robot via SSH
  setup_robot_audio_tcp(robot_ip);

  // ---- Build parec argv ----
  std::string pulse_server_value = "tcp:" + std::string(robot_ip) + ":4713";
  std::string s_channels = std::to_string(kChannels);
  std::string s_rate = std::to_string(kRate);
  std::string s_device = std::string(kDevice);

  printf("连接 %s，录音 %d 秒...\n", robot_ip, duration);

  // ---- Fork/exec parec so we can kill it cleanly ----
  int pipefd[2];
  if (::pipe(pipefd) < 0) {
    perror("pipe");
    return 1;
  }

  pid_t child = fork();
  if (child < 0) {
    perror("fork");
    return 1;
  }

  if (child == 0) {
    // Child: redirect stdout to pipe, exec parec
    ::close(pipefd[0]);
    ::dup2(pipefd[1], STDOUT_FILENO);
    ::close(pipefd[1]);

    // Redirect stderr to /dev/null
    int devnull = ::open("/dev/null", O_WRONLY);
    if (devnull >= 0) {
      ::dup2(devnull, STDERR_FILENO);
      ::close(devnull);
    }

    ::setenv("PULSE_SERVER", pulse_server_value.c_str(), 1);
    ::execlp("parec", "parec",
             "--channels", s_channels.c_str(),
             "--rate", s_rate.c_str(),
             "--format", "s16le",
             "--device", s_device.c_str(),
             nullptr);
    perror("execlp parec");
    _exit(1);
  }

  // Parent: close write end
  ::close(pipefd[1]);

  // ---- Read PCM data with threaded progress display ----
  int bytes_per_sec = kRate * kBytesPerFrame;
  size_t total_bytes = static_cast<size_t>(bytes_per_sec) * duration;
  std::vector<uint8_t> audio;
  audio.reserve(total_bytes);

  auto start_time = std::chrono::steady_clock::now();
  std::atomic<bool> stop_display{false};

  // Progress display thread: updates every 100ms with smooth timing
  std::thread progress_thread([&]() {
    while (!stop_display.load()) {
      auto now = std::chrono::steady_clock::now();
      double elapsed = std::chrono::duration<double>(now - start_time).count();
      double display_sec = std::min(elapsed, static_cast<double>(duration));
      double received_sec = static_cast<double>(audio.size()) / bytes_per_sec;
      printf("\r录音中 %.1f / %d 秒  (已接收 %.1f 秒)", display_sec, duration, received_sec);
      fflush(stdout);
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
  });

  // Main thread: read data until target bytes received or signal
  uint8_t buf[65536];
  while (audio.size() < total_bytes && !g_stop.load()) {
    ssize_t n = ::read(pipefd[0], buf, sizeof(buf));
    if (n < 0) {
      if (errno == EINTR) continue;
      break;
    }
    if (n == 0) break;  // EOF
    audio.insert(audio.end(), buf, buf + n);
  }

  // Kill parec child and wait for exit
  ::kill(child, SIGTERM);
  int status;
  ::waitpid(child, &status, 0);

  // Stop progress thread and wait for it to finish
  stop_display.store(true);
  progress_thread.join();
  printf("\n");

  ::close(pipefd[0]);

  size_t n_frames = audio.size() / kBytesPerFrame;
  double actual_secs = static_cast<double>(n_frames) / kRate;

  printf("录音 %zu 字节 (%.1f 秒)\n", audio.size(), actual_secs);

  if (n_frames == 0) {
    printf("未录制到数据，请检查：\n");
    printf("  1. 机器人 IP 是否正确\n");
    printf("  2. TCP 录音模块是否已加载\n");
    printf("  3. 是否已安装 pulseaudio-utils: sudo apt install pulseaudio-utils\n");
    return 1;
  }

  // ---- Save 8-channel WAV ----
  write_wav("raw_8ch.wav", audio, kChannels, kRate);

  // ---- Per-channel analysis + mono extraction ----
  printf("分析中...\n");

  std::vector<int64_t> ch_sum(kChannels, 0);
  std::vector<int16_t> ch_peak(kChannels, 0);
  std::vector<uint8_t> mono;
  mono.reserve(n_frames * kSampleWidth);

  for (size_t i = 0; i < n_frames; ++i) {
    const uint8_t *frame = audio.data() + i * kBytesPerFrame;
    int16_t samples[kChannels];
    for (int c = 0; c < kChannels; ++c) {
      samples[c] = read_s16_le(frame + c * kSampleWidth);
      int16_t abs_val = static_cast<int16_t>(std::abs(samples[c]));
      ch_sum[c] += abs_val;
      if (abs_val > ch_peak[c]) ch_peak[c] = abs_val;
    }
    // Mix ch5-6 (index 4,5), amplify 10x
    int32_t mixed = (static_cast<int32_t>(samples[4]) + samples[5]) / 2 * 10;
    int16_t mono_sample = clamp_s16(mixed);
    uint8_t sample_buf[2];
    write_s16_le(sample_buf, mono_sample);
    mono.insert(mono.end(), sample_buf, sample_buf + 2);
  }

  // ---- Save mono WAV ----
  write_wav("mic_mono.wav", mono, 1, kRate);

  // ---- Print per-channel stats ----
  printf("\n各通道音量:\n");
  for (int c = 0; c < kChannels; ++c) {
    double avg = static_cast<double>(ch_sum[c]) / n_frames;
    const char *tag = (avg > 50) ? "有声" : "静音";
    printf("  通道 %d: 平均 %6.1f  峰值 %5d  %s\n", c + 1, avg, ch_peak[c], tag);
  }

  printf("\n保存: raw_8ch.wav, mic_mono.wav\n");
  printf("播放: paplay mic_mono.wav\n");
  return 0;
}
