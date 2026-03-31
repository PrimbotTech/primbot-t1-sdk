/**
 * @brief Example client for /aimdk_5Fmsgs/srv/PlayAudioFile
 *
 * The following ROS parameters can be set via startup arguments:
 * --ros-args -p <name>:=<value>
 *
 * Supported parameters:
 *   - file_name: audio file name only
 *   - file_path: directory containing the audio file
 *
 * Examples:
 *   ros2 run aimdk_examples_cpp play_audio --ros-args -p
 *   file_name:=demo.wav -p file_path:=/tmp
 *
 *   ros2 run aimdk_examples_cpp play_audio --ros-args -p
 *   file_name:=sample.wav -p file_path:=/agibot/software/interaction/bin/cfg
 *
 * Other request fields use built-in defaults and are not configurable from
 * the command line in this demo.
 */
#include "aimdk_msgs/msg/common_request.hpp"
#include "aimdk_msgs/msg/common_state.hpp"
#include "aimdk_msgs/srv/play_audio_file.hpp"
#include "rclcpp/rclcpp.hpp"

#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <exception>
#include <memory>
#include <signal.h>
#include <string>

std::shared_ptr<rclcpp::Node> g_node = nullptr;

void signal_handler(int signal) {
  if (g_node) {
    RCLCPP_INFO(g_node->get_logger(), "Received signal %d, shutting down...",
                signal);
    g_node.reset();
  }
  rclcpp::shutdown();
  exit(signal);
}

class PlayAudioFileClient : public rclcpp::Node {
public:
  PlayAudioFileClient() : Node("play_audio_file_client") {
    file_name_ =
        this->declare_parameter<std::string>("file_name", file_name_);
    file_path_ =
        this->declare_parameter<std::string>("file_path", file_path_);

    client_ = this->create_client<aimdk_msgs::srv::PlayAudioFile>(service_name_);
    RCLCPP_INFO(this->get_logger(), "PlayAudioFile client node created.");
  }

  bool send_request() {
    if (!wait_for_service()) {
      return false;
    }

    auto request = std::make_shared<aimdk_msgs::srv::PlayAudioFile::Request>();
    request->request = aimdk_msgs::msg::CommonRequest();
    request->request.header.stamp = this->now();

    // pkg_name/file_name/file_path locate the audio resource; info describes its format.
    request->file.pkg_name = pkg_name_;
    request->file.file_name = file_name_;
    request->file.file_path = file_path_;

    if (request->file.file_name.empty()) {
      RCLCPP_ERROR(this->get_logger(), "file_name is empty.");
      return false;
    }

    request->file.info.channels = static_cast<uint8_t>(channels_);
    request->file.info.sample_rate = static_cast<uint32_t>(sample_rate_);
    request->file.info.size = static_cast<uint32_t>(size_);
    request->file.info.sample_format = sample_format_;
    request->file.info.coding_format = coding_format_;
    request->file.priority = static_cast<uint32_t>(priority_);
    request->file.priority_weight = static_cast<uint32_t>(priority_weight_);

    RCLCPP_INFO(this->get_logger(),
                "Sending PlayAudioFile request: file=%s, path=%s, pkg=%s, "
                "ch=%d, sr=%d, fmt=%s, coding=%s, priority=%d, weight=%d",
                request->file.file_name.c_str(), request->file.file_path.c_str(),
                request->file.pkg_name.c_str(), channels_, sample_rate_,
                sample_format_.c_str(), coding_format_.c_str(), priority_,
                priority_weight_);

    auto future = client_->async_send_request(request);
    auto retcode = rclcpp::spin_until_future_complete(shared_from_this(), future,
                                                     std::chrono::seconds(5));
    if (retcode != rclcpp::FutureReturnCode::SUCCESS) {
      RCLCPP_ERROR(this->get_logger(),
                   "PlayAudioFile call failed or timed out.");
      return false;
    }

    const auto response = future.get();
    const auto code = response->reponse.header.code;
    const auto status = response->reponse.status.value;
    // Treat code==0 or status==SUCCESS as success.
    const bool ok =
        code == 0 || status == aimdk_msgs::msg::CommonState::SUCCESS;

    if (ok) {
      RCLCPP_INFO(this->get_logger(),
                  "PlayAudioFile accepted. code=%ld status=%d msg=%s", code,
                  status, response->reponse.message.c_str());
      return true;
    }

    RCLCPP_ERROR(this->get_logger(), "PlayAudioFile rejected. code=%ld status=%d "
                                     "msg=%s",
                 code, status, response->reponse.message.c_str());
    return false;
  }

private:
  bool wait_for_service() {
    while (!client_->wait_for_service(std::chrono::seconds(2))) {
      if (!rclcpp::ok()) {
        return false;
      }
      RCLCPP_INFO(this->get_logger(), "Waiting for service: %s",
                  service_name_.c_str());
    }
    RCLCPP_INFO(this->get_logger(), "Service available, ready to send request.");
    return true;
  }

  std::string service_name_ = "/aimdk_5Fmsgs/srv/PlayAudioFile";
  std::string pkg_name_ = "sdk_demo";
  std::string file_name_ = "撒娇.wav";
  std::string file_path_ = "/agibot/software/interaction/bin/cfg";
  std::string sample_format_ = "S16_LE";
  std::string coding_format_ = "wave";
  int channels_ = 1;
  int sample_rate_ = 16000;
  int size_ = 0;
  int priority_ = 6;
  int priority_weight_ = 0;
  rclcpp::Client<aimdk_msgs::srv::PlayAudioFile>::SharedPtr client_;
};

int main(int argc, char *argv[]) {
  try {
    rclcpp::init(argc, argv);
    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);

    g_node = std::make_shared<PlayAudioFileClient>();
    auto client = std::dynamic_pointer_cast<PlayAudioFileClient>(g_node);
    const bool ok = client ? client->send_request() : false;

    g_node.reset();
    rclcpp::shutdown();
    return ok ? 0 : 1;
  } catch (const std::exception &e) {
    RCLCPP_ERROR(rclcpp::get_logger("main"),
                 "Program exited with exception: %s", e.what());
    return 1;
  }
}
