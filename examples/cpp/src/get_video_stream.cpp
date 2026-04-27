/**
 * @brief Example reader for the RTSP video stream.
 *
 * RTSP URL Format:
 *   rtsp://{ip}:2554/live_{camera_id}
 *
 * Default MP4 output:
 *   /tmp/video_capture.mp4
 *
 * Supported arguments:
 *   --camera_id, --camera: Camera identifier (interactive prompt if not provided)
 *   --robot_ip, --ip: Robot IP address (interactive prompt if not provided)
 *   --output_file, --output: MP4 output path.
 *   --capture_seconds, --duration: stop automatically after the first frame arrives and
 *     this many seconds have elapsed. Set <= 0 to run until Ctrl+C.
 *
 * Interactive Mode:
 *   If --camera or --ip is not provided, the program will prompt for input.
 *   Press Enter to use the default values shown in brackets.
 *
 * Examples:
 *   ros2 run aimdk_examples_cpp get_video_stream
 */

#include <opencv2/core.hpp>
#include <opencv2/core/utils/logger.hpp>
#include <opencv2/videoio.hpp>

#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdint>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace {

constexpr char kRtspUrl[] = "rtsp://{ip}:2554/live_{camera_id}";
constexpr char kDefaultOutputFile[] = "/tmp/video_capture.mp4";
constexpr double kDefaultCaptureSeconds = 5.0;
constexpr int kLogEveryNFrames = 100;
constexpr double kDefaultFallbackFps = 25.0;
constexpr int kOpenTimeoutMs = 5000;
constexpr int kReadTimeoutMs = 5000;
constexpr char kMp4Codec[] = "mp4v";

std::atomic<bool> g_stop_requested{false};

struct Options {
  std::string output_file{kDefaultOutputFile};
  double capture_seconds{kDefaultCaptureSeconds};
  std::string camera_id{""};
  std::string robot_ip{""};
};

std::string format_double(double value) {
  std::ostringstream stream;
  stream << std::fixed << std::setprecision(3) << value;
  return stream.str();
}

void print_usage(const char *program) {
  std::cout
      << "Usage: " << program
      << " [--camera_id CAMERA_ID] [--robot_ip ROBOT_IP]"
      << " [--output_file OUTPUT_FILE] [--capture_seconds CAPTURE_SECONDS]\n\n"
      << "Read decoded frames from the RTSP stream and save MP4.\n\n"
      << "Options:\n"
      << "  -h, --help                  Show this help message and exit.\n"
      << "  --camera_id CAMERA_ID       Camera identifier (e.g., head_monocular_centra).\n"
      << "  --camera CAMERA_ID          Alias of --camera_id.\n"
      << "  --robot_ip ROBOT_IP         Robot IP address (e.g., 10.1.1.100).\n"
      << "  --ip ROBOT_IP               Alias of --robot_ip.\n"
      << "  --output_file OUTPUT_FILE   Output MP4 file path.\n"
      << "  --output OUTPUT_FILE        Alias of --output_file.\n"
      << "  --capture_seconds SEC       Stop automatically after this many\n"
      << "                              seconds. Use <= 0 to run forever.\n"
      << "  --duration SEC              Alias of --capture_seconds.\n\n"
      << "RTSP URL Format: " << kRtspUrl << "\n"
      << "Default MP4 output: " << kDefaultOutputFile << '\n';
}

std::string require_value(int argc, char **argv, int *index,
                          const std::string &option_name) {
  if (*index + 1 >= argc) {
    throw std::invalid_argument("missing value for " + option_name);
  }
  ++(*index);
  return argv[*index];
}

double parse_double(const std::string &text, const std::string &option_name) {
  std::size_t parsed = 0;
  const double value = std::stod(text, &parsed);
  if (parsed != text.size()) {
    throw std::invalid_argument("invalid value for " + option_name + ": " + text);
  }
  return value;
}

std::string prompt_input(const std::string &message, const std::string &default_value) {
  std::cout << "\n" << message << " [" << default_value << "]: ";
  std::string input;
  std::getline(std::cin, input);
  
  if (input.empty()) {
    return default_value;
  }
  
  // Trim whitespace
  size_t start = input.find_first_not_of(" \t\r\n");
  if (start == std::string::npos) {
    return default_value;
  }
  size_t end = input.find_last_not_of(" \t\r\n");
  return input.substr(start, end - start + 1);
}

void print_camera_list() {
  std::cout << std::string(60, '=') << std::endl;
  std::cout << "Available Camera IDs:" << std::endl;
  std::cout << std::string(60, '=') << std::endl;
  std::cout << "  head_monocular_centra  - Head monocular central camera" << std::endl;
  std::cout << "  head_stereo_left       - Head stereo left camera" << std::endl;
  std::cout << "  head_stereo_right      - Head stereo right camera" << std::endl;
  std::cout << "  head_fisheye_1         - Head fisheye camera 1" << std::endl;
  std::cout << "  head_fisheye_2         - Head fisheye camera 2" << std::endl;
  std::cout << "  head_TOF3D             - Head TOF 3D camera" << std::endl;
  std::cout << std::string(60, '=') << std::endl;
  std::cout << "For camera_id corresponding to different robot configurations, please refer to the interface documentation." << std::endl;
  std::cout << std::endl;
}

std::string build_rtsp_url(const std::string &ip, const std::string &camera_id) {
  std::string url = kRtspUrl;
  
  // Replace {ip}
  size_t ip_pos = url.find("{ip}");
  if (ip_pos != std::string::npos) {
    url.replace(ip_pos, 4, ip);
  }
  
  // Replace {camera_id}
  size_t camera_pos = url.find("{camera_id}");
  if (camera_pos != std::string::npos) {
    url.replace(camera_pos, 11, camera_id);
  }
  
  return url;
}

Options parse_args(int argc, char **argv) {
  Options options;

  for (int i = 1; i < argc; ++i) {
    const std::string argument = argv[i];

    if (argument == "-h" || argument == "--help") {
      print_usage(argv[0]);
      std::exit(0);
    }

    if (argument == "--camera_id" || argument == "--camera") {
      options.camera_id = require_value(argc, argv, &i, argument);
      continue;
    }

    if (argument.rfind("--camera_id=", 0) == 0) {
      options.camera_id = argument.substr(std::string("--camera_id=").size());
      continue;
    }

    if (argument.rfind("--camera=", 0) == 0) {
      options.camera_id = argument.substr(std::string("--camera=").size());
      continue;
    }

    if (argument == "--robot_ip" || argument == "--ip") {
      options.robot_ip = require_value(argc, argv, &i, argument);
      continue;
    }

    if (argument.rfind("--robot_ip=", 0) == 0) {
      options.robot_ip = argument.substr(std::string("--robot_ip=").size());
      continue;
    }

    if (argument.rfind("--ip=", 0) == 0) {
      options.robot_ip = argument.substr(std::string("--ip=").size());
      continue;
    }

    if (argument == "--output_file" || argument == "--output") {
      options.output_file = require_value(argc, argv, &i, argument);
      continue;
    }

    if (argument.rfind("--output_file=", 0) == 0) {
      options.output_file = argument.substr(std::string("--output_file=").size());
      continue;
    }

    if (argument.rfind("--output=", 0) == 0) {
      options.output_file = argument.substr(std::string("--output=").size());
      continue;
    }

    if (argument == "--capture_seconds" || argument == "--duration") {
      options.capture_seconds =
          parse_double(require_value(argc, argv, &i, argument), argument);
      continue;
    }

    if (argument.rfind("--capture_seconds=", 0) == 0) {
      options.capture_seconds = parse_double(
          argument.substr(std::string("--capture_seconds=").size()),
          "--capture_seconds");
      continue;
    }

    if (argument.rfind("--duration=", 0) == 0) {
      options.capture_seconds = parse_double(
          argument.substr(std::string("--duration=").size()), "--duration");
      continue;
    }

    throw std::invalid_argument("unknown argument: " + argument);
  }

  // Interactive input for robot_ip if not provided
  if (options.robot_ip.empty()) {
    options.robot_ip = prompt_input("Please enter robot IP address", "");
    if (options.robot_ip.empty()) {
      throw std::invalid_argument("robot_ip must not be empty.");
    }
  }

  // Interactive input for camera_id if not provided
  if (options.camera_id.empty()) {
    print_camera_list();
    options.camera_id = prompt_input("Please enter camera ID", "");
    if (options.camera_id.empty()) {
      throw std::invalid_argument("camera_id must not be empty.");
    }
  }

  if (options.output_file.empty()) {
    throw std::invalid_argument("output_file must not be empty");
  }

  return options;
}

void signal_handler(int) { g_stop_requested.store(true); }

class RtspVideoStreamReader {
public:
  explicit RtspVideoStreamReader(const Options &options)
      : output_file_(options.output_file),
        capture_seconds_(options.capture_seconds),
        camera_id_(options.camera_id),
        robot_ip_(options.robot_ip),
        rtsp_url_(build_rtsp_url(robot_ip_, camera_id_)),
        output_path_(prepare_output_file(output_file_)) {}

  ~RtspVideoStreamReader() {
    release_resources();
    log_summary();
  }

  int run() {
    open_capture();

    while (!g_stop_requested.load()) {
      cv::Mat frame;
      if (!capture_.read(frame) || frame.empty()) {
        throw std::runtime_error("Failed to read frame from the RTSP stream.");
      }

      handle_frame(frame);
      if (should_stop()) {
        std::cout << "Reached capture_seconds=" << format_double(capture_seconds_)
                  << ", shutting down." << std::endl;
        break;
      }
    }

    if (g_stop_requested.load()) {
      std::cout << "Capture interrupted by user." << std::endl;
    }

    return 0;
  }

private:
  static std::filesystem::path prepare_output_file(const std::string &output_file) {
    std::filesystem::path output_path(output_file);
    if (output_path.extension() != ".mp4") {
      throw std::invalid_argument("output_file must use the .mp4 extension.");
    }

    const auto parent = output_path.parent_path();
    if (!parent.empty()) {
      std::error_code ec;
      std::filesystem::create_directories(parent, ec);
      if (ec) {
        throw std::runtime_error("Failed to create directory " +
                                 parent.string() + ": " + ec.message());
      }
    }

    return output_path;
  }

  bool try_open_capture(int api_preference, const std::string &backend_label,
                        bool quiet_opencv_logs) {
    capture_.release();

    const auto previous_log_level = cv::utils::logging::getLogLevel();
    try {
      if (quiet_opencv_logs) {
        cv::utils::logging::setLogLevel(cv::utils::logging::LOG_LEVEL_SILENT);
      }

      const std::vector<int> open_params = {
          cv::CAP_PROP_OPEN_TIMEOUT_MSEC, kOpenTimeoutMs,
          cv::CAP_PROP_READ_TIMEOUT_MSEC, kReadTimeoutMs,
      };
      const bool opened = capture_.open(rtsp_url_, api_preference, open_params);
      cv::utils::logging::setLogLevel(previous_log_level);
      if (!opened || !capture_.isOpened()) {
        capture_.release();
        return false;
      }
    } catch (const cv::Exception &) {
      cv::utils::logging::setLogLevel(previous_log_level);
      capture_.release();
      return false;
    }

    backend_name_ = backend_label;
    try {
      backend_name_ = capture_.getBackendName();
    } catch (const cv::Exception &) {
    }

    std::cout << "Opened RTSP stream: backend=" << backend_name_
              << " open_timeout_ms=" << kOpenTimeoutMs
              << " read_timeout_ms=" << kReadTimeoutMs << std::endl;
    return true;
  }

  void open_capture() {
    if (try_open_capture(cv::CAP_FFMPEG, "FFMPEG", false)) {
      return;
    }

    std::cout << "FFMPEG backend open failed, falling back to the default backend."
              << std::endl;
    if (try_open_capture(cv::CAP_ANY, "DEFAULT", true)) {
      return;
    }

    throw std::runtime_error(std::string("Failed to open RTSP stream: ") +
                             rtsp_url_);
  }

  static bool is_valid_fps(double value) {
    return value > 0.0 && value < 1000.0;
  }

  double resolve_output_fps() const {
    const double stream_fps = capture_.get(cv::CAP_PROP_FPS);
    if (is_valid_fps(stream_fps)) {
      return stream_fps;
    }
    return kDefaultFallbackFps;
  }

  void create_output_writer() {
    output_writer_.open(output_path_.string(),
                        cv::VideoWriter::fourcc(kMp4Codec[0], kMp4Codec[1],
                                                kMp4Codec[2], kMp4Codec[3]),
                        output_fps_, cv::Size(frame_width_, frame_height_));
    if (!output_writer_.isOpened()) {
      output_writer_.release();
      throw std::runtime_error("Failed to create output file: " +
                               output_path_.string());
    }
  }

  void write_frame(const cv::Mat &frame) {
    if (!output_writer_.isOpened()) {
      return;
    }

    try {
      output_writer_.write(frame);
    } catch (const cv::Exception &error) {
      throw std::runtime_error("Failed while writing to " + output_path_.string() +
                               ": " + error.what());
    }

    if (!output_writer_.isOpened()) {
      throw std::runtime_error("Video writer closed unexpectedly during capture.");
    }
  }

  void log_first_frame(const cv::Mat &frame) const {
    std::cout << "First frame received: resolution=" << frame_width_ << 'x'
              << frame_height_ << " channels=" << frame.channels()
              << " payload_bytes=" << frame.total() * frame.elemSize()
              << " stream_fps=" << format_double(stream_fps_)
              << " output_fps=" << format_double(output_fps_)
              << " backend=" << backend_name_ << std::endl;
  }

  void handle_frame(const cv::Mat &frame) {
    const auto frame_bytes =
        static_cast<std::uint64_t>(frame.total() * frame.elemSize());
    ++frame_count_;
    total_frame_bytes_ += frame_bytes;

    if (!first_frame_received_) {
      first_frame_received_ = true;
      first_frame_time_ = std::chrono::steady_clock::now();
      frame_width_ = frame.cols;
      frame_height_ = frame.rows;

      const double stream_fps = capture_.get(cv::CAP_PROP_FPS);
      if (is_valid_fps(stream_fps)) {
        stream_fps_ = stream_fps;
      }
      output_fps_ = resolve_output_fps();
      create_output_writer();

      log_first_frame(frame);
      std::cout << "MP4 output file: " << output_path_.string() << std::endl;
      if (capture_seconds_ > 0) {
        std::cout << "The reader will stop "
                  << format_double(capture_seconds_)
                  << " second(s) after the first frame." << std::endl;
      } else {
        std::cout << "Auto stop disabled. Press Ctrl+C to exit." << std::endl;
      }
    }

    // `frame` is the decoded image read from the RTSP stream.
    // Feed it directly into your vision algorithm here.
    // Example:
    // auto result = your_algorithm(frame);
    write_frame(frame);

    if (kLogEveryNFrames > 0 &&
        frame_count_ % static_cast<std::uint64_t>(kLogEveryNFrames) == 0 &&
        first_frame_received_) {
      const auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                                  std::chrono::steady_clock::now() -
                                  first_frame_time_)
                                  .count();
      std::cout << "Received " << static_cast<unsigned long long>(frame_count_)
                << " frames, total_frame_bytes="
                << static_cast<unsigned long long>(total_frame_bytes_)
                << ", elapsed=" << static_cast<long long>(elapsed_ms) << " ms"
                << std::endl;
    }
  }

  bool should_stop() const {
    if (!first_frame_received_ || capture_seconds_ <= 0) {
      return false;
    }

    const auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now() - first_frame_time_);
    return elapsed.count() >= static_cast<long long>(capture_seconds_ * 1000.0);
  }

  void release_resources() {
    if (capture_.isOpened()) {
      capture_.release();
    }

    if (output_writer_.isOpened()) {
      output_writer_.release();
    }
  }

  void log_summary() {
    if (summary_logged_) {
      return;
    }
    summary_logged_ = true;

    if (!first_frame_received_) {
      std::cerr << "No video frame received before shutdown." << std::endl;
      return;
    }

    const auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                                std::chrono::steady_clock::now() -
                                first_frame_time_)
                                .count();
    std::cout << "Video stream summary: frames="
              << static_cast<unsigned long long>(frame_count_)
              << " total_frame_bytes="
              << static_cast<unsigned long long>(total_frame_bytes_)
              << " elapsed=" << static_cast<long long>(elapsed_ms)
              << " ms resolution=" << frame_width_ << 'x' << frame_height_
              << " stream_fps=" << format_double(stream_fps_)
              << " output_fps=" << format_double(output_fps_)
              << " backend=" << backend_name_ << std::endl;
    std::cout << "Captured MP4 file: " << output_path_.string() << std::endl;
  }

  std::string output_file_;
  double capture_seconds_{kDefaultCaptureSeconds};
  std::string camera_id_;
  std::string robot_ip_;
  std::string rtsp_url_;
  std::filesystem::path output_path_;
  cv::VideoCapture capture_;
  cv::VideoWriter output_writer_;
  bool first_frame_received_{false};
  bool summary_logged_{false};
  std::uint64_t frame_count_{0};
  std::uint64_t total_frame_bytes_{0};
  std::chrono::steady_clock::time_point first_frame_time_{};
  int frame_width_{0};
  int frame_height_{0};
  double stream_fps_{0.0};
  double output_fps_{0.0};
  std::string backend_name_;
};

} // namespace

int main(int argc, char **argv) {
  try {
    std::signal(SIGINT, signal_handler);
    std::signal(SIGTERM, signal_handler);

    const Options options = parse_args(argc, argv);
    RtspVideoStreamReader reader(options);
    return reader.run();
  } catch (const std::exception &error) {
    std::cerr << "Error: " << error.what() << std::endl;
    return 1;
  }
}
