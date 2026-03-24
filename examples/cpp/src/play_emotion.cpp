/**
 * @brief Example client for /aimdk_5Fmsgs/srv/PlayEmotion
 *
 * The following ROS parameters can be set via startup arguments:
 * --ros-args -p <name>:=<value>
 *
 * Supported parameters:
 *   - type: "emotion" or "file"
 *   - emotion_ids: integer array, required when type=emotion
 *   - file_paths: string array, required when type=file
 *
 * Examples:
 *   ros2 run aimdk_examples_cpp play_emotion --ros-args -p
 *   type:=emotion -p emotion_ids:="[90]"
 *
 *   ros2 run aimdk_examples_cpp play_emotion --ros-args -p
 *   type:=file -p file_paths:="[\"/tmp/demo.mp4\"]"
 */
#include "aimdk_msgs/msg/common_request.hpp"
#include "aimdk_msgs/msg/common_state.hpp"
#include "aimdk_msgs/srv/play_emotion.hpp"
#include "rclcpp/rclcpp.hpp"

#include <chrono>
#include <cstdint>
#include <exception>
#include <memory>
#include <signal.h>
#include <string>
#include <vector>

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

class PlayEmotionClient : public rclcpp::Node {
public:
  PlayEmotionClient() : Node("play_emotion_client") {
    type_ = this->declare_parameter<std::string>("type", "emotion");
    emotion_ids_ =
        this->declare_parameter<std::vector<int64_t>>(
            "emotion_ids", std::vector<int64_t>{});
    file_paths_ =
        this->declare_parameter<std::vector<std::string>>(
            "file_paths", std::vector<std::string>{});

    client_ = this->create_client<aimdk_msgs::srv::PlayEmotion>(
        "/aimdk_5Fmsgs/srv/PlayEmotion");
    RCLCPP_INFO(this->get_logger(), "PlayEmotion client node created.");

    while (!client_->wait_for_service(std::chrono::seconds(2))) {
      if (!rclcpp::ok()) {
        return;
      }
      RCLCPP_INFO(this->get_logger(), "Waiting for service...");
    }
    RCLCPP_INFO(this->get_logger(),
                "Service available, ready to send request.");
  }

  bool send_request() {
    try {
      if (!validate_parameters()) {
        return false;
      }

      auto request = std::make_shared<aimdk_msgs::srv::PlayEmotion::Request>();
      request->header = aimdk_msgs::msg::CommonRequest();
      request->type = type_;
      request->priority = priority_;
      request->loop_count = loop_count_;

      if (type_ == "emotion") {
        request->emotion_ids.reserve(emotion_ids_.size());
        for (const auto emotion_id : emotion_ids_) {
          request->emotion_ids.push_back(static_cast<int32_t>(emotion_id));
        }
      } else {
        request->file_paths = file_paths_;
      }

      request->header.header.stamp = this->now();

      RCLCPP_INFO(this->get_logger(),
                  "Sending PlayEmotion request: type=%s emotion_ids=%zu "
                  "file_paths=%zu priority=%d loop_count=%d",
                  request->type.c_str(), request->emotion_ids.size(),
                  request->file_paths.size(), request->priority,
                  request->loop_count);
      if (!request->emotion_ids.empty()) {
        for (const auto emotion_id : request->emotion_ids) {
          RCLCPP_INFO(this->get_logger(), "emotion_id=%d", emotion_id);
        }
      }
      if (!request->file_paths.empty()) {
        for (const auto &file_path : request->file_paths) {
          RCLCPP_INFO(this->get_logger(), "file_path=%s", file_path.c_str());
        }
      }

      auto future = client_->async_send_request(request);
      auto retcode = rclcpp::spin_until_future_complete(
          shared_from_this(), future, std::chrono::seconds(2));
      if (retcode != rclcpp::FutureReturnCode::SUCCESS) {
        RCLCPP_ERROR(this->get_logger(),
                     "Service call failed or timed out.");
        return false;
      }

      const auto response = future.get();
      const auto code = response->header.header.code;
      const auto status = response->header.status.value;

      RCLCPP_INFO(this->get_logger(),
                  "Response: code=%ld status=%d message=%s", code, status,
                  response->header.message.c_str());

      if (code == 0 || status == aimdk_msgs::msg::CommonState::SUCCESS) {
        RCLCPP_INFO(this->get_logger(), "PlayEmotion request accepted.");
        return true;
      }

      RCLCPP_ERROR(this->get_logger(), "PlayEmotion request failed.");
      return false;
    } catch (const std::exception &e) {
      RCLCPP_ERROR(this->get_logger(), "Exception occurred: %s", e.what());
      return false;
    }
  }

private:
  bool validate_parameters() {
    if (type_ != "emotion" && type_ != "file") {
      RCLCPP_ERROR(this->get_logger(),
                   "Invalid parameter 'type': %s. Use 'emotion' or 'file'.",
                   type_.c_str());
      return false;
    }

    if (type_ == "emotion" && emotion_ids_.empty()) {
      RCLCPP_ERROR(this->get_logger(),
                   "Parameter 'emotion_ids' must be set when type=emotion.");
      return false;
    }

    if (type_ == "file" && file_paths_.empty()) {
      RCLCPP_ERROR(this->get_logger(),
                   "Parameter 'file_paths' must be set when type=file.");
      return false;
    }

    return true;
  }

  std::string type_;
  std::vector<int64_t> emotion_ids_;
  std::vector<std::string> file_paths_;
  int32_t priority_ = 0;
  int32_t loop_count_ = 1;
  rclcpp::Client<aimdk_msgs::srv::PlayEmotion>::SharedPtr client_;
};

int main(int argc, char *argv[]) {
  try {
    rclcpp::init(argc, argv);
    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);

    g_node = std::make_shared<PlayEmotionClient>();
    auto client = std::dynamic_pointer_cast<PlayEmotionClient>(g_node);
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
