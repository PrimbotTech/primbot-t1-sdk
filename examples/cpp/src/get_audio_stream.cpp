/**
 * @brief Example subscriber for /aima/hal/audio/capture
 *
 * Supported ROS parameters:
 *   - output_file: PCM output path. Leave empty to disable file dump.
 *   - capture_seconds: stop automatically after the first packet arrives and
 *     this many seconds have elapsed. Set <= 0 to run until Ctrl+C.
 *   - log_every_n_messages: print progress every N messages. Set <= 0 to
 *     disable periodic progress logs.
 *   - enable_playback: if True, re-publishes the captured audio to the playback
 *     topic to echo the audio out to the speakers. Default is False.
 *
 * Examples:
 *   # 1. Capture for 5 seconds to a specific file:
 *   ros2 run aimdk_examples_cpp get_audio_stream --ros-args \
 *     -p output_file:=/tmp/audio_capture.pcm -p capture_seconds:=5
 *
 *   # 2. Capture continuously until Ctrl+C is pressed:
 *   ros2 run aimdk_examples_cpp get_audio_stream --ros-args \
 *     -p capture_seconds:=-1
 *
 *   # 3. Echo audio to speakers (loopback) and capture for 10 seconds:
 *   ros2 run aimdk_examples_cpp get_audio_stream --ros-args \
 *     -p enable_playback:=true -p capture_seconds:=10
 */
#include "aimdk_msgs/msg/audio_capture.hpp"
#include "aimdk_msgs/msg/audio_playback.hpp"
#include "rclcpp/rclcpp.hpp"

#include <chrono>
#include <cstdint>
#include <filesystem>
#include <functional>
#include <fstream>
#include <memory>
#include <signal.h>
#include <stdexcept>
#include <string>
#include <thread>
#include <unistd.h>

using namespace std::chrono_literals;

class AudioStreamSubscriber : public rclcpp::Node {
public:
  AudioStreamSubscriber() : Node("get_audio_stream") {
    output_file_ =
        this->declare_parameter<std::string>("output_file", "/tmp/audio_capture.pcm");
    capture_seconds_ = this->declare_parameter<int>("capture_seconds", 5);
    enable_playback_ = this->declare_parameter<bool>("enable_playback", false);

    file_enabled_ = !output_file_.empty();
    if (file_enabled_ && !prepare_output_file()) {
      throw std::runtime_error("failed to open output file: " + output_file_);
    }

    auto qos = rclcpp::QoS(rclcpp::KeepLast(20));
    qos.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);

    if (enable_playback_) {
      pub_ = this->create_publisher<aimdk_msgs::msg::AudioPlayback>(
          "/aima/hal/audio/playback", qos);
      RCLCPP_INFO(this->get_logger(), "Audio playback loopback is ENABLED.");
    }

    sub_ = this->create_subscription<aimdk_msgs::msg::AudioCapture>(
        "/aima/hal/audio/capture", qos,
        std::bind(&AudioStreamSubscriber::on_audio_capture, this,
                  std::placeholders::_1));

    timer_ = this->create_wall_timer(
        200ms, std::bind(&AudioStreamSubscriber::check_auto_stop, this));

    if (file_enabled_) {
      RCLCPP_INFO(this->get_logger(), "PCM output file: %s",
                  output_file_.c_str());
    } else {
      RCLCPP_INFO(this->get_logger(),
                  "PCM output disabled because output_file is empty.");
    }

    if (capture_seconds_ > 0) {
      RCLCPP_INFO(this->get_logger(),
                  "The node will stop %d second(s) after the first packet.",
                  capture_seconds_);
    } else {
      RCLCPP_INFO(this->get_logger(),
                  "Auto stop disabled. Press Ctrl+C to exit.");
    }
  }

  ~AudioStreamSubscriber() override {
    close_output_file();
    log_summary();
  }

  void request_shutdown(int signal) {
    if (stop_requested_) {
      return;
    }
    stop_requested_ = true;
    close_output_file();
    log_summary();
    RCLCPP_INFO(this->get_logger(),
                "Received signal %d, shutting down get_audio_stream...", signal);
    rclcpp::shutdown();
  }

private:
  bool prepare_output_file() {
    std::filesystem::path output_path(output_file_);
    const auto parent = output_path.parent_path();

    if (!parent.empty()) {
      std::error_code ec;
      std::filesystem::create_directories(parent, ec);
      if (ec) {
        RCLCPP_ERROR(this->get_logger(), "Failed to create directory %s: %s",
                     parent.string().c_str(), ec.message().c_str());
        return false;
      }
    }

    output_stream_.open(output_file_,
                        std::ios::binary | std::ios::out | std::ios::trunc);
    if (!output_stream_.is_open()) {
      RCLCPP_ERROR(this->get_logger(), "Failed to open output file: %s",
                   output_file_.c_str());
      return false;
    }

    return true;
  }

  void close_output_file() {
    if (output_stream_.is_open()) {
      output_stream_.close();
    }
  }

  void on_audio_capture(const aimdk_msgs::msg::AudioCapture::SharedPtr msg) {
    const std::size_t payload_bytes = msg->data.data.size();
    ++message_count_;
    total_bytes_ += payload_bytes;

    if (!first_packet_received_) {
      first_packet_received_ = true;
      first_packet_time_ = std::chrono::steady_clock::now();
      last_sample_rate_ = msg->info.sample_rate;
      last_total_channels_ = msg->info.channels;
      last_sample_format_ = msg->info.sample_format;
      last_coding_format_ = msg->info.coding_format;
      log_first_packet(*msg, payload_bytes);
    }

    // Some publishers leave msg.info.size as 0, so use the actual payload size.
    if (!size_mismatch_logged_ && msg->info.size != 0 &&
        msg->info.size != payload_bytes) {
      size_mismatch_logged_ = true;
      RCLCPP_WARN(this->get_logger(),
                  "AudioInfo.size=%u but actual payload is %zu bytes. The "
                  "example will use msg.data.data.size().",
                  msg->info.size, payload_bytes);
    }

    if (file_enabled_ && output_stream_.is_open() && payload_bytes > 0) {
      output_stream_.write(reinterpret_cast<const char *>(msg->data.data.data()),
                           static_cast<std::streamsize>(payload_bytes));
      if (!output_stream_) {
        RCLCPP_ERROR(this->get_logger(), "Failed while writing to %s",
                     output_file_.c_str());
        file_enabled_ = false;
        close_output_file();
      }
    }

    if (pub_) {
      aimdk_msgs::msg::AudioPlayback playback_msg;
      playback_msg.stamps = msg->stamps;
      playback_msg.info = msg->info;
      playback_msg.data = msg->data;
      playback_msg.pkg_name = "get_audio_stream_loopback";
      playback_msg.token_id = "loopback_session";
      pub_->publish(playback_msg);
    }

    if (log_every_n_messages_ > 0 &&
        message_count_ % static_cast<std::uint64_t>(log_every_n_messages_) ==
            0) {
      RCLCPP_INFO(this->get_logger(),
                  "Received %llu packets, total_bytes=%llu, latest_payload=%zu",
                  static_cast<unsigned long long>(message_count_),
                  static_cast<unsigned long long>(total_bytes_), payload_bytes);
    }
  }

  void log_first_packet(const aimdk_msgs::msg::AudioCapture &msg,
                        std::size_t payload_bytes) {
    RCLCPP_INFO(
        this->get_logger(),
        "First packet received: stamp=%d.%09u mic_channels=%u ref_channels=%u "
        "total_channels=%u sample_rate=%u sample_format=%s coding_format=%s "
        "mic_source=%u payload_bytes=%zu",
        msg.stamps.sec, msg.stamps.nanosec,
        static_cast<unsigned int>(msg.mic_channels),
        static_cast<unsigned int>(msg.ref_channels),
        static_cast<unsigned int>(msg.info.channels), msg.info.sample_rate,
        msg.info.sample_format.c_str(),
        msg.info.coding_format.c_str(), msg.mic_source, payload_bytes);
  }

  void check_auto_stop() {
    if (!first_packet_received_ || capture_seconds_ <= 0 || stop_requested_) {
      return;
    }

    const auto elapsed = std::chrono::steady_clock::now() - first_packet_time_;
    if (elapsed < std::chrono::seconds(capture_seconds_)) {
      return;
    }

    stop_requested_ = true;
    close_output_file();
    log_summary();
    RCLCPP_INFO(this->get_logger(),
                "Reached capture_seconds=%d, shutting down.", capture_seconds_);
    rclcpp::shutdown();
  }

  void log_summary() {
    if (summary_logged_) {
      return;
    }
    summary_logged_ = true;

    if (!first_packet_received_) {
      RCLCPP_WARN(this->get_logger(),
                  "No audio packet received before shutdown.");
      return;
    }

    const auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
                             std::chrono::steady_clock::now() -
                             first_packet_time_)
                             .count();

    RCLCPP_INFO(
        this->get_logger(),
        "Audio stream summary: packets=%llu total_bytes=%llu elapsed=%lld ms "
        "sample_rate=%u total_channels=%u sample_format=%s coding_format=%s",
        static_cast<unsigned long long>(message_count_),
        static_cast<unsigned long long>(total_bytes_),
        static_cast<long long>(elapsed), last_sample_rate_,
        static_cast<unsigned int>(last_total_channels_),
        last_sample_format_.c_str(), last_coding_format_.c_str());

    if (file_enabled_ || !output_file_.empty()) {
      RCLCPP_INFO(this->get_logger(), "Captured PCM file: %s",
                  output_file_.c_str());
    }
  }

  std::string output_file_;
  int capture_seconds_{5};
  int log_every_n_messages_{100};
  bool file_enabled_{false};
  bool first_packet_received_{false};
  bool size_mismatch_logged_{false};
  bool stop_requested_{false};
  bool summary_logged_{false};
  bool enable_playback_{false};
  std::uint64_t message_count_{0};
  std::uint64_t total_bytes_{0};
  std::uint32_t last_sample_rate_{0};
  std::uint8_t last_total_channels_{0};
  std::string last_sample_format_;
  std::string last_coding_format_;
  std::chrono::steady_clock::time_point first_packet_time_{};
  std::ofstream output_stream_;
  rclcpp::Subscription<aimdk_msgs::msg::AudioCapture>::SharedPtr sub_;
  rclcpp::Publisher<aimdk_msgs::msg::AudioPlayback>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

std::shared_ptr<rclcpp::Node> g_node = nullptr;

void signal_handler(int signal) {
  if (!rclcpp::ok()) {
    return;
  }
  if (g_node) {
    if (auto node = std::dynamic_pointer_cast<AudioStreamSubscriber>(g_node)) {
      node->request_shutdown(signal);
    } else {
      RCLCPP_INFO(g_node->get_logger(),
                  "Received signal %d, shutting down get_audio_stream...",
                  signal);
      rclcpp::shutdown();
    }
    g_node.reset();
    return;
  }
  rclcpp::shutdown();
}

int main(int argc, char **argv) {
  try {
    rclcpp::init(argc, argv);
    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);
    auto node = std::make_shared<AudioStreamSubscriber>();
    g_node = node;
    rclcpp::spin(node);
    
    // Start a watchdog thread to force exit if cleanup hangs for more than 3s
    std::thread([]() {
      std::this_thread::sleep_for(std::chrono::seconds(3));
      // Use _exit to bypass any remaining cleanup and force terminate
      _exit(0);
    }).detach();

    g_node.reset();
    rclcpp::shutdown();
    return 0;
  } catch (const std::exception &e) {
    RCLCPP_ERROR(rclcpp::get_logger("get_audio_stream"),
                 "Program exited with exception: %s", e.what());
    return 1;
  }
}
