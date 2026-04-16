/**
 * Usage:
 *   ros2 run aimdk_examples_cpp get_audio_stream --ros-args -p output_file:=<path> -p capture_seconds:=<seconds>
 *
 * Notes:
 *   - After the recording duration expires, you MUST type 'y' in the terminal to trigger conversion and playback.
 *
 * Parameters:
 *   - output_file: Path to save the original 16kHz PCM file (Default: /tmp/audio_capture.pcm).
 *   - capture_seconds: Duration of automated recording in seconds (Default: 5s).
 *
 * Example:
 *   ros2 run aimdk_examples_cpp get_audio_stream --ros-args -p capture_seconds:=10
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
#include <string>
#include <thread>
#include <vector>
#include <iostream>

using namespace std::chrono_literals;

class AudioStreamSubscriber : public rclcpp::Node {
public:
  AudioStreamSubscriber() : Node("get_audio_stream") {
    output_file_ = this->declare_parameter<std::string>("output_file", "/tmp/audio_capture.pcm");
    capture_seconds_ = this->declare_parameter<int>("capture_seconds", 5);

    if (!output_file_.empty() && !prepare_output_file()) {
      throw std::runtime_error("failed to open output file: " + output_file_);
    }

    auto qos = rclcpp::QoS(rclcpp::KeepLast(20)).reliable();
    
    // Playback publisher
    pub_ = this->create_publisher<aimdk_msgs::msg::AudioPlayback>("/aima/hal/audio/playback", qos);

    // Capture subscription
    sub_ = this->create_subscription<aimdk_msgs::msg::AudioCapture>(
        "/aima/hal/audio/capture", qos,
        std::bind(&AudioStreamSubscriber::on_audio_capture, this, std::placeholders::_1));

    timer_ = this->create_wall_timer(200ms, std::bind(&AudioStreamSubscriber::check_auto_stop, this));

    RCLCPP_INFO(this->get_logger(), "PCM output file: %s", output_file_.c_str());
  }

  void interactive_session() {
    std::cout << "\n==================================================" << std::endl;
    std::cout << "Capture done. Play back via robot? (y/n): ";
    char choice;
    std::cin >> choice;
    if (choice == 'y' || choice == 'Y') {
      process_and_play();
    }
  }

  bool capture_done_{false};

private:
  void on_audio_capture(const aimdk_msgs::msg::AudioCapture::SharedPtr msg) {
    if (capture_done_) return;

    if (!first_packet_received_) {
      first_packet_received_ = true;
      first_packet_time_ = std::chrono::steady_clock::now();
      last_info_ = msg->info;
      RCLCPP_INFO(this->get_logger(), "Recording started (Channels: %d, Rate: %dHz)...", 
                  static_cast<int>(last_info_.channels), last_info_.sample_rate);
    }

    if (output_stream_.is_open()) {
      output_stream_.write(reinterpret_cast<const char *>(msg->data.data.data()), 
                           msg->data.data.size());
    }
  }

  void process_and_play() {
    std::string playback_file = output_file_;
    size_t last_dot = playback_file.find_last_of(".");
    if (last_dot != std::string::npos) {
        playback_file.insert(last_dot, "_playback_24k");
    } else {
        playback_file += "_playback_24k.pcm";
    }

    RCLCPP_INFO(this->get_logger(), "Optimizing: 16k -> 24k (Linear Interpolation)...");

    std::ifstream fin(output_file_, std::ios::binary);
    std::ofstream fout(playback_file, std::ios::binary);
    
    if (!fin || !fout) {
        RCLCPP_ERROR(this->get_logger(), "Failed to open files for conversion.");
        return;
    }

    int stride = last_info_.channels * 2;
    std::vector<char> buffer(stride * 2);

    while (fin.read(buffer.data(), buffer.size())) {
        // Extract Ch1 from two frames for interpolation
        int16_t s1 = *reinterpret_cast<int16_t*>(&buffer[0]);
        int16_t s2 = *reinterpret_cast<int16_t*>(&buffer[stride]);

        int16_t s_mid = (s1 + s2) / 2;

        // Write 3 samples (16k * 1.5 = 24k)
        fout.write(reinterpret_cast<const char*>(&s1), 2);
        fout.write(reinterpret_cast<const char*>(&s_mid), 2);
        fout.write(reinterpret_cast<const char*>(&s2), 2);
    }
    fin.close();
    fout.close();

    RCLCPP_INFO(this->get_logger(), "Conversion done. Starting stream playback...");
    stream_to_robot(playback_file);
  }

  void stream_to_robot(const std::string& path) {
    while (pub_->get_subscription_count() == 0) {
        if (!rclcpp::ok()) return;
        std::this_thread::sleep_for(100ms);
    }

    std::ifstream f(path, std::ios::binary);
    std::string token = "cpp_play_" + std::to_string(std::time(nullptr));
    
    // 24kHz Mono 100ms = 4800 bytes
    std::vector<char> chunk(4800);
    while (rclcpp::ok()) {
        f.read(chunk.data(), chunk.size());
        auto bytes_read = f.gcount();
        if (bytes_read <= 0) break;

        aimdk_msgs::msg::AudioPlayback msg;
        msg.info.channels = 1;
        msg.info.sample_rate = 24000;
        msg.info.sample_format = "S16_LE";
        msg.info.coding_format = "pcm";
        msg.info.size = static_cast<uint32_t>(bytes_read);
        msg.pkg_name = "get_audio_stream_cpp";
        msg.token_id = token;
        msg.stamps = this->get_clock()->now();
        msg.data.data.assign(chunk.begin(), chunk.begin() + bytes_read);
        
        pub_->publish(msg);
        std::this_thread::sleep_for(100ms);
    }
  }

  void check_auto_stop() {
    if (!first_packet_received_ || capture_done_) return;
    auto elapsed = std::chrono::steady_clock::now() - first_packet_time_;
    if (elapsed >= std::chrono::seconds(capture_seconds_)) {
      capture_done_ = true;
      output_stream_.close();
      RCLCPP_INFO(this->get_logger(), "Capture finished. File saved to %s", output_file_.c_str());
    }
  }

  bool prepare_output_file() {
    std::filesystem::path p(output_file_);
    if (!p.parent_path().empty()) std::filesystem::create_directories(p.parent_path());
    output_stream_.open(output_file_, std::ios::binary | std::ios::trunc);
    return output_stream_.is_open();
  }

  std::string output_file_;
  int capture_seconds_;
  bool first_packet_received_{false};
  aimdk_msgs::msg::AudioInfo last_info_;
  std::ofstream output_stream_;
  std::chrono::steady_clock::time_point first_packet_time_;
  rclcpp::Subscription<aimdk_msgs::msg::AudioCapture>::SharedPtr sub_;
  rclcpp::Publisher<aimdk_msgs::msg::AudioPlayback>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<AudioStreamSubscriber>();
  
  // Spin ROS logic in a separate thread so main can handle std::cin
  std::thread ros_thread([&]() { rclcpp::spin(node); });

  // Main thread monitors capture status
  while (rclcpp::ok()) {
      if (node->capture_done_) {
          node->interactive_session();
          break;
      }
      std::this_thread::sleep_for(100ms);
  }

  if (ros_thread.joinable()) {
      // Small delay before shutdown to ensure final logs are flushed
      std::this_thread::sleep_for(500ms);
      rclcpp::shutdown();
      ros_thread.join();
  }
  return 0;
}
