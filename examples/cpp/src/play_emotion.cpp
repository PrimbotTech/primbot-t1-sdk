/**
 * @brief Example client for /aimdk_5Fmsgs/srv/PlayEmotion
 *
 * Usage Note:
 *   It is recommended to specify the 'type' and ('emotion_ids' or 'file_paths')
 *   based on your requirements.
 *   If no parameters are provided, the script will attempt to play a default emotion.
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
#include "aimdk_msgs/srv/play_emotion.hpp"
#include "aimdk_msgs/msg/common_request.hpp"
#include "aimdk_msgs/msg/common_state.hpp"
#include "rclcpp/rclcpp.hpp"

#include <signal.h>
#include <chrono>
#include <cstdint>
#include <exception>
#include <memory>
#include <string>
#include <vector>

std::shared_ptr<rclcpp::Node> g_node = nullptr;

void signal_handler(int signal)
{
  if (g_node) {
    RCLCPP_INFO(g_node->get_logger(), "Received signal %d, shutting down...", signal);
    g_node.reset();
  }
  rclcpp::shutdown();
  exit(signal);
}

class PlayEmotionClient : public rclcpp::Node
{
 public:
  PlayEmotionClient() : Node("play_emotion_client")
  {
    type_        = this->declare_parameter<std::string>("type", "emotion");
    emotion_ids_ = this->declare_parameter<std::vector<int64_t>>(
      "emotion_ids", std::vector<int64_t>{10});
    file_paths_ = this->declare_parameter<std::vector<std::string>>(
      "file_paths", std::vector<std::string>{});

    client_ = this->create_client<aimdk_msgs::srv::PlayEmotion>(
      "/aimdk_5Fmsgs/srv/PlayEmotion");
    RCLCPP_INFO(this->get_logger(), "PlayEmotion client node created.");

    while (!client_->wait_for_service(std::chrono::seconds(2))) {
      if (!rclcpp::ok())
        return;
      RCLCPP_INFO(this->get_logger(), "Waiting for service...");
    }
  }

  bool send_request()
  {
    try {
      if (!validate_parameters()) return false;

      // 第一步：尝试原始请求 (默认为 ID 10)
      bool ok = call_service(type_, emotion_ids_, file_paths_);

      // 第二步：降级逻辑 (兼容 Q 系列机型)
      // 如果尝试 ID 10 失败，自动尝试保底 ID 3001
      if (!ok && type_ == "emotion" && std::find(emotion_ids_.begin(), emotion_ids_.end(), 10) != emotion_ids_.end()) {
        ok = call_service("emotion", {3001}, {});
      }

      if (!ok) {
        RCLCPP_ERROR(this->get_logger(), "PlayEmotion request failed after all attempts.");
        return false;
      }
      return true;
    } catch (const std::exception &e) {
      RCLCPP_ERROR(this->get_logger(), "Exception occurred: %s", e.what());
      return false;
    }
  }

 private:
  bool call_service(const std::string &type, const std::vector<int64_t> &emotion_ids, const std::vector<std::string> &file_paths)
  {
    auto request                 = std::make_shared<aimdk_msgs::srv::PlayEmotion::Request>();
    request->header.header.stamp = this->now();
    request->type                = type;
    request->priority            = priority_;
    request->loop_count          = loop_count_;

    for (auto id : emotion_ids)
      request->emotion_ids.push_back(static_cast<int32_t>(id));
    request->file_paths = file_paths;

    RCLCPP_INFO(this->get_logger(), "Sending PlayEmotion request: type=%s", type.c_str());

    auto future = client_->async_send_request(request);
    if (rclcpp::spin_until_future_complete(shared_from_this(), future, std::chrono::seconds(2)) != rclcpp::FutureReturnCode::SUCCESS)
      return false;

    auto res    = future.get();
    auto code   = res->header.header.code;
    auto status = res->header.status.value;

    if (code == 0 || status == aimdk_msgs::msg::CommonState::SUCCESS) {
      RCLCPP_INFO(this->get_logger(), "Request accepted (code=%ld).", code);
      return true;
    }
    return false;
  }

  bool validate_parameters()
  {
    if (type_ != "emotion" && type_ != "file") return false;
    if (type_ == "emotion" && emotion_ids_.empty()) return false;
    if (type_ == "file" && file_paths_.empty()) return false;
    return true;
  }

  std::string type_;
  std::vector<int64_t> emotion_ids_;
  std::vector<std::string> file_paths_;
  int32_t priority_   = 0;
  int32_t loop_count_ = 1;
  rclcpp::Client<aimdk_msgs::srv::PlayEmotion>::SharedPtr client_;
};

int main(int argc, char *argv[])
{
  try {
    rclcpp::init(argc, argv);
    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);

    g_node      = std::make_shared<PlayEmotionClient>();
    auto client = std::dynamic_pointer_cast<PlayEmotionClient>(g_node);
    bool ok     = client ? client->send_request() : false;

    g_node.reset();
    rclcpp::shutdown();
    return ok ? 0 : 1;
  } catch (const std::exception &e) {
    RCLCPP_ERROR(rclcpp::get_logger("main"), "Exited with exception: %s", e.what());
    return 1;
  }
}
