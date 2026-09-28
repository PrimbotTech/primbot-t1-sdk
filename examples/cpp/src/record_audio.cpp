/*
 @file record_audio.cpp
 @brief 录音示例（自动检测内置麦/外置麦）

 @description
   支持本机（共享内存）和远程（TCP）两种录音模式。
   自动检测当前 PulseAudio 默认录音源，适配不同麦克风：

   内置麦（麦克风阵列 aw89403）：
     - 8声道 48kHz S16LE
     - ch1-2：回采, ch3-8：麦克
     - 输出：raw_8ch.wav + mic_mono.wav（ch5-6 混合, 10倍放大）

   外置麦（USB 麦克风）：
     - 2声道 48kHz S24LE（24-bit）
     - ch1-2：左右声道
     - 输出：raw_2ch.wav + mic_mono.wav（ch1-2 混合, 24→16bit）

 @prerequisites
   - 远程模式：示例脚本会通过 SSH 自动在机器人端加载 TCP 录音模块（端口 4713）
   - 本机模式：无需额外配置，直接使用 PulseAudio 本地 socket
   - 本机已安装 parec：sudo apt install pulseaudio-utils
   - 使用 SSH 密钥时：将售后提供的密钥目录（<机器人SN>_soc0/）放到 SDK 根目录下
     （与 examples/ 同级），并修改权限：
     sudo chown -R $USER:$USER ./<机器人SN>_soc0/
     chmod 600 ./<机器人SN>_soc0/id_ed25519
     然后在 SDK 根目录下执行示例命令，使用 -i ./<机器人SN>_soc0/id_ed25519 指定密钥。

 @usage
   ./build/aimdk_examples_cpp/record_audio [-i SSH_KEY] <机器人IP> [录音秒数]

   机器人IP 为 127.0.0.1 或 localhost 时自动使用本机模式（共享内存），
   否则使用远程模式（TCP）。

   可选参数：
     -i SSH_KEY    SSH 私钥文件路径（机器人需要密钥登录）

 @example
   # 本机录音（共享内存）
   ./build/aimdk_examples_cpp/record_audio 127.0.0.1 5

   # 远程录音（TCP）
   ./build/aimdk_examples_cpp/record_audio <机器人IP> 5

   # 远程录音（使用 SSH 密钥，网线连接时 IP 为 10.1.1.100）
   ./build/aimdk_examples_cpp/record_audio -i ./<机器人SN>_soc0/id_ed25519 10.1.1.100 5
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
#include <cstring>
#include <fcntl.h>
#include <string>
#include <sys/stat.h>
#include <sys/wait.h>
#include <thread>
#include <unistd.h>
#include <vector>

namespace {

constexpr int kRate = 48000;
constexpr const char *kBuiltinDevice = "alsa_input.platform-aw89403_sound.pro-input-0";

std::atomic<bool> g_stop{false};

// ---- Microphone profile (built-in array or USB mic) ----
struct MicProfile {
  std::string device_name;
  int channels         = 8;
  int sample_width     = 2;     // bytes per sample (2 = 16-bit, 3 = 24-bit)
  int bits_per_sample  = 16;
  std::string format   = "s16le";
  bool is_builtin      = true;
};

}  // namespace

static void signal_handler(int) { g_stop.store(true); }

// ---------------------------------------------------------------------------
// Local / remote detection
// ---------------------------------------------------------------------------

static bool is_local_robot(const char *robot_ip) {
  if (strcmp(robot_ip, "127.0.0.1") == 0 ||
      strcmp(robot_ip, "localhost") == 0 ||
      strcmp(robot_ip, "::1") == 0) {
    return true;
  }
  struct stat st;
  if (stat("/robot/software", &st) == 0 && S_ISDIR(st.st_mode)) {
    return true;
  }
  return false;
}

static std::string get_local_pulse_server() {
  const char *sockets[] = { "/run/user/1000/pulse/native" };
  for (const char *s : sockets) {
    struct stat st;
    if (stat(s, &st) == 0) {
      return std::string("unix:") + s;
    }
  }
  return "";
}

// ---------------------------------------------------------------------------
// Microphone profile detection
// ---------------------------------------------------------------------------

static std::string get_default_source(bool local_mode, const char *robot_ip, const char *ssh_key = nullptr) {
  std::string cmd;
  if (local_mode) {
    cmd = "pactl get-default-source 2>/dev/null";
  } else {
    char buf[512];
    if (ssh_key) {
      snprintf(buf, sizeof(buf),
               "ssh -o StrictHostKeyChecking=no -o ConnectTimeout=5 -i %s run@%s '"
               "PULSE_SERVER=unix:/run/user/1000/pulse/native "
               "pactl get-default-source 2>/dev/null'",
               ssh_key, robot_ip);
    } else {
      snprintf(buf, sizeof(buf),
               "ssh -o StrictHostKeyChecking=no -o ConnectTimeout=5 run@%s '"
               "PULSE_SERVER=unix:/run/user/1000/pulse/native "
               "pactl get-default-source 2>/dev/null'",
               robot_ip);
    }
    cmd = buf;
  }

  FILE *fp = popen(cmd.c_str(), "r");
  if (!fp) return "";

  std::string result;
  char buf[512];
  while (fgets(buf, sizeof(buf), fp)) result += buf;
  pclose(fp);

  // Trim trailing whitespace
  while (!result.empty() &&
         (result.back() == '\n' || result.back() == '\r' || result.back() == ' '))
    result.pop_back();
  return result;
}

// ---------------------------------------------------------------------------
// Auto-setup: load PulseAudio TCP modules on robot via SSH
//             同时检测默认录音源，合并为一次 SSH 调用避免重复输入密码
// ---------------------------------------------------------------------------

// 返回值: 默认录音源名称（空字符串表示检测失败）
// 副作用: 通过 SSH 检查/加载 TCP 录音模块，打印状态信息
static std::string setup_and_detect_source(const char *robot_ip, const char *ssh_key = nullptr) {
  // 将 TCP 模块加载检查和默认录音源查询合并到一次 SSH 会话中
  char cmd[2048];
  if (ssh_key) {
    snprintf(cmd, sizeof(cmd),
             "ssh -o StrictHostKeyChecking=no -o ConnectTimeout=5 -i %s run@%s '"
             "export PULSE_SERVER=unix:/run/user/1000/pulse/native; "
             "if pactl list modules short 2>/dev/null | grep -q module-native-protocol-tcp; then "
             "echo __TCP_MODULE__:ALREADY_LOADED; "
             "else "
             "pactl load-module module-native-protocol-tcp "
             "auth-anonymous=1 listen=0.0.0.0 >/dev/null 2>&1 && "
             "echo __TCP_MODULE__:LOADED; "
             "fi; "
             "echo __DEFAULT_SOURCE__; "
             "pactl get-default-source 2>/dev/null"
             "'",
             ssh_key, robot_ip);
  } else {
    snprintf(cmd, sizeof(cmd),
             "ssh -o StrictHostKeyChecking=no -o ConnectTimeout=5 run@%s '"
             "export PULSE_SERVER=unix:/run/user/1000/pulse/native; "
             "if pactl list modules short 2>/dev/null | grep -q module-native-protocol-tcp; then "
             "echo __TCP_MODULE__:ALREADY_LOADED; "
             "else "
             "pactl load-module module-native-protocol-tcp "
             "auth-anonymous=1 listen=0.0.0.0 >/dev/null 2>&1 && "
             "echo __TCP_MODULE__:LOADED; "
             "fi; "
             "echo __DEFAULT_SOURCE__; "
             "pactl get-default-source 2>/dev/null"
             "'",
             robot_ip);
  }

  FILE *fp = popen(cmd, "r");
  if (!fp) {
    printf("SSH 连接失败\n");
    return "";
  }
  std::string output;
  char buf[512];
  while (fgets(buf, sizeof(buf), fp)) {
    output += buf;
  }
  int ret = pclose(fp);
  (void)ret;

  // 解析 TCP 模块加载状态
  if (output.find("__TCP_MODULE__:ALREADY_LOADED") != std::string::npos) {
    printf("录音模块已在机器人端加载（跳过加载）\n");
  } else if (output.find("__TCP_MODULE__:LOADED") != std::string::npos) {
    printf("已在机器人端加载录音模块 (端口 4713)\n");
  } else {
    printf("警告: 录音模块加载可能未成功（stderr 已输出到终端）\n");
  }

  // 解析默认录音源名称（__DEFAULT_SOURCE__ 标记之后的第一行非空内容）
  std::string source;
  auto pos = output.find("__DEFAULT_SOURCE__");
  if (pos != std::string::npos) {
    pos = output.find('\n', pos);
    if (pos != std::string::npos) {
      pos++;  // skip '\n'
      auto end = output.find('\n', pos);
      if (end == std::string::npos) end = output.size();
      source = output.substr(pos, end - pos);
      // Trim trailing whitespace
      while (!source.empty() &&
             (source.back() == '\n' || source.back() == '\r' || source.back() == ' '))
        source.pop_back();
    }
  }
  return source;
}

// ---------------------------------------------------------------------------
// Parse source name into MicProfile (shared by local and remote paths)
// ---------------------------------------------------------------------------

static MicProfile parse_mic_profile(const std::string &source) {
  MicProfile profile;
  if (source.empty()) {
    printf("无法检测默认录音源，使用内置麦默认配置\n");
    profile.device_name = kBuiltinDevice;
    return profile;
  }

  profile.device_name = source;

  if (source.find("aw89403") != std::string::npos) {
    profile = {source, 8, 2, 16, "s16le", true};
    printf("检测到内置麦（aw89403）: %s\n", source.c_str());
  } else {
    profile = {source, 2, 3, 24, "s24le", false};
    printf("检测到外置麦（USB）: %s\n", source.c_str());
  }

  printf("  格式: %s, %dch, %dHz, %d-bit\n",
         profile.format.c_str(), profile.channels, kRate, profile.bits_per_sample);
  return profile;
}

// ---------------------------------------------------------------------------
// Microphone profile detection (local-only path)
// ---------------------------------------------------------------------------

static MicProfile detect_mic_profile(bool local_mode, const char *robot_ip, const char *ssh_key = nullptr) {
  std::string source = get_default_source(local_mode, robot_ip, ssh_key);
  return parse_mic_profile(source);
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
                      int channels, int rate, int bits_per_sample) {
  FILE *f = fopen(path, "wb");
  if (!f) {
    perror(path);
    return false;
  }

  int bytes_per_sample = bits_per_sample / 8;
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
  write_u32_le(f, static_cast<uint32_t>(rate * channels * bytes_per_sample));  // byte rate
  write_u16_le(f, static_cast<uint16_t>(channels * bytes_per_sample));         // block align
  write_u16_le(f, static_cast<uint16_t>(bits_per_sample));                     // bits per sample

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
// S24LE (24-bit signed little-endian) sample helpers
// ---------------------------------------------------------------------------

static inline int32_t read_s24_le(const uint8_t *p) {
  uint32_t val = static_cast<uint32_t>(p[0]) |
                 (static_cast<uint32_t>(p[1]) << 8) |
                 (static_cast<uint32_t>(p[2]) << 16);
  // Sign-extend from 24 bits to 32 bits
  if (val & 0x800000) {
    val |= 0xFF000000;
  }
  return static_cast<int32_t>(val);
}

static inline int16_t s24_to_s16(const uint8_t *p) {
  int32_t s24 = read_s24_le(p);
  int32_t s16 = s24 >> 8;  // arithmetic right shift preserves sign
  if (s16 > 32767) s16 = 32767;
  if (s16 < -32768) s16 = -32768;
  return static_cast<int16_t>(s16);
}

// ---------------------------------------------------------------------------
// Main
// ---------------------------------------------------------------------------

int main(int argc, char *argv[]) {
  if (argc < 2) {
    printf("用法: %s [-i SSH_KEY] <机器人IP> [录音秒数]\n", argv[0]);
    printf("\n");
    printf("  机器人IP 为 127.0.0.1 或 localhost 时自动使用本机模式（共享内存）\n");
    printf("  否则使用远程模式（TCP）\n");
    printf("\n");
    printf("  可选参数：\n");
    printf("    -i SSH_KEY    SSH 私钥文件路径（机器人需要密钥登录）\n");
    return 1;
  }

  // 解析可选参数 -i SSH_KEY
  const char *ssh_key = nullptr;
  int arg_idx = 1;
  if (argc >= 3 && strcmp(argv[1], "-i") == 0) {
    if (argc < 4) {
      printf("错误: -i 参数需要指定 SSH 私钥文件路径\n");
      return 1;
    }
    ssh_key = argv[2];
    arg_idx = 3;
  }

  signal(SIGINT, signal_handler);
  signal(SIGTERM, signal_handler);

  const char *robot_ip = argv[arg_idx];
  int duration = (argc > arg_idx + 1) ? atoi(argv[arg_idx + 1]) : 5;
  if (duration <= 0) duration = 5;

  // 检测本机/远程模式
  bool local_mode = is_local_robot(robot_ip);

  std::string pulse_server_value;
  MicProfile profile;
  if (local_mode) {
    pulse_server_value = get_local_pulse_server();
    if (!pulse_server_value.empty()) {
      printf("本机模式（共享内存: %s），录音 %d 秒...\n", pulse_server_value.c_str(), duration);
    } else {
      pulse_server_value = "";  // 使用 PulseAudio 默认 socket
      printf("本机模式（默认 socket），录音 %d 秒...\n", duration);
    }
    profile = detect_mic_profile(local_mode, robot_ip, ssh_key);
  } else {
    printf("检测到远程环境，使用 TCP 传输\n");
    if (ssh_key) {
      printf("使用 SSH 密钥: %s\n", ssh_key);
    }
    // 远程模式：通过一次 SSH 同时完成 TCP 模块加载 + 录音源检测（避免重复输入密码）
    std::string remote_source = setup_and_detect_source(robot_ip, ssh_key);
    if (g_stop.load()) {
      printf("\n已取消\n");
      return 0;
    }
    pulse_server_value = "tcp:" + std::string(robot_ip) + ":4713";
    printf("远程模式（TCP: %s:4713），录音 %d 秒...\n", robot_ip, duration);
    profile = parse_mic_profile(remote_source);
  }

  int channels = profile.channels;
  int sample_width = profile.sample_width;
  int bytes_per_frame = channels * sample_width;
  int bits_per_sample = profile.bits_per_sample;

  // ---- Build parec argv ----
  std::string s_channels = std::to_string(channels);
  std::string s_rate = std::to_string(kRate);
  std::string s_device = profile.device_name;

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

    if (!pulse_server_value.empty()) {
      ::setenv("PULSE_SERVER", pulse_server_value.c_str(), 1);
    }
    ::execlp("parec", "parec",
             "--channels", s_channels.c_str(),
             "--rate", s_rate.c_str(),
             "--format", profile.format.c_str(),
             "--device", s_device.c_str(),
             nullptr);
    perror("execlp parec");
    _exit(1);
  }

  // Parent: close write end
  ::close(pipefd[1]);

  // ---- Read PCM data with threaded progress display ----
  int bytes_per_sec = kRate * bytes_per_frame;
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

  size_t n_frames = audio.size() / bytes_per_frame;
  double actual_secs = static_cast<double>(n_frames) / kRate;

  printf("录音 %zu 字节 (%.1f 秒)\n", audio.size(), actual_secs);

  if (n_frames == 0) {
    printf("未录制到数据，请检查：\n");
    printf("  1. 机器人 IP 是否正确\n");
    printf("  2. TCP 录音模块是否已加载\n");
    printf("  3. 是否已安装 pulseaudio-utils: sudo apt install pulseaudio-utils\n");
    return 1;
  }

  // ---- Save raw WAV (8ch or 2ch depending on mic type) ----
  const char *raw_filename = profile.is_builtin ? "raw_8ch.wav" : "raw_2ch.wav";
  write_wav(raw_filename, audio, channels, kRate, bits_per_sample);

  // ---- Per-channel analysis + mono extraction ----
  printf("分析中...\n");

  std::vector<int64_t> ch_sum(channels, 0);
  std::vector<int16_t> ch_peak(channels, 0);
  std::vector<uint8_t> mono;
  mono.reserve(n_frames * 2);  // mono is always 16-bit

  for (size_t i = 0; i < n_frames; ++i) {
    const uint8_t *frame = audio.data() + i * bytes_per_frame;

    if (profile.is_builtin) {
      // Built-in mic: 8ch S16LE, use ch5-6 (index 4,5) mixed with 10x gain
      int16_t samples[8];
      for (int c = 0; c < channels; ++c) {
        samples[c] = read_s16_le(frame + c * sample_width);
        int16_t abs_val = static_cast<int16_t>(std::abs(samples[c]));
        ch_sum[c] += abs_val;
        if (abs_val > ch_peak[c]) ch_peak[c] = abs_val;
      }
      int32_t mixed = (static_cast<int32_t>(samples[4]) + samples[5]) / 2 * 10;
      int16_t mono_sample = clamp_s16(mixed);
      uint8_t sample_buf[2];
      write_s16_le(sample_buf, mono_sample);
      mono.insert(mono.end(), sample_buf, sample_buf + 2);
    } else {
      // External USB mic: 2ch S24LE, mix ch1-2 and convert 24→16 bit
      int32_t samples[2];
      for (int c = 0; c < channels; ++c) {
        const uint8_t *sp = frame + c * sample_width;
        samples[c] = read_s24_le(sp);
        // Convert to 16-bit for stats
        int16_t s16 = s24_to_s16(sp);
        int16_t abs_val = static_cast<int16_t>(std::abs(s16));
        ch_sum[c] += abs_val;
        if (abs_val > ch_peak[c]) ch_peak[c] = abs_val;
      }
      // Average ch1-2, convert to 16-bit
      int32_t mixed = (samples[0] + samples[1]) / 2;
      int32_t s16 = mixed >> 8;
      if (s16 > 32767) s16 = 32767;
      if (s16 < -32768) s16 = -32768;
      int16_t mono_sample = static_cast<int16_t>(s16);
      uint8_t sample_buf[2];
      write_s16_le(sample_buf, mono_sample);
      mono.insert(mono.end(), sample_buf, sample_buf + 2);
    }
  }

  // ---- Save mono WAV ----
  write_wav("mic_mono.wav", mono, 1, kRate, 16);

  // ---- Print per-channel stats ----
  printf("\n各通道音量:\n");
  for (int c = 0; c < channels; ++c) {
    double avg = static_cast<double>(ch_sum[c]) / n_frames;
    const char *tag = (avg > 50) ? "有声" : "静音";
    printf("  通道 %d: 平均 %6.1f  峰值 %5d  %s\n", c + 1, avg, ch_peak[c], tag);
  }

  printf("\n保存: %s, mic_mono.wav\n", raw_filename);
  printf("播放: paplay mic_mono.wav\n");
  return 0;
}
