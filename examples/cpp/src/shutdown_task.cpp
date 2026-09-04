/*
 @file shutdown_task.cpp
 @brief 主动关机示例脚本

 @description
   通过 ShutdownTask 服务请求机器人执行主动关机操作。
   该接口在 task engine 层实现，需要设置来源标识（source）。

   警告：调用此接口后，系统将进入安全关机流程，依次停止各子系统服务后断电。
   请确保在调用前已保存重要数据。

 @prerequisites
   - Task engine 服务必须正在运行
   - 机器人在安全状态下可执行关机

 @usage
   ros2 run aimdk_examples_cpp shutdown_task

 @example
   ros2 run aimdk_examples_cpp shutdown_task

 @parameters
   - None
 */

#include "aimdk_msgs/msg/common_request.hpp"
#include "aimdk_msgs/msg/common_state.hpp"
#include "aimdk_msgs/msg/token_common.hpp"
#include "aimdk_msgs/srv/shutdown_task.hpp"
#include "rclcpp/rclcpp.hpp"

#include <chrono>
#include <csignal>
#include <memory>
#include <string>
#include <thread>

using namespace std::chrono_literals;

namespace
{

constexpr auto kServiceWaitInterval = 2s;
constexpr auto kServiceCallTimeout  = 5s;
constexpr int kMaxRetryCount = 3;

}  // namespace

std::shared_ptr<rclcpp::Node> g_node = nullptr;

void signal_handler(int signal)
{
  if (g_node) {
    RCLCPP_INFO(g_node->get_logger(), "Received signal %d, shutting down...",
                signal);
    g_node.reset();
  }
  rclcpp::shutdown();
  exit(signal);
}

class ShutdownTaskClient : public rclcpp::Node
{
 public:
  ShutdownTaskClient() : Node("shutdown_task_client")
  {
    client_ = this->create_client<aimdk_msgs::srv::ShutdownTask>(
      "/aimdk_5Fmsgs/srv/ShutdownTask");
    RCLCPP_INFO(this->get_logger(), "ShutdownTask client node created.");

    while (!client_->wait_for_service(kServiceWaitInterval)) {
      if (!rclcpp::ok())
        return;
      RCLCPP_INFO(this->get_logger(), "Waiting for service...");
    }
    RCLCPP_INFO(this->get_logger(), "Service available, ready to send request.");
  }

  bool send_request(const std::string &source = "sdk_example")
  {
    try {
      auto request = std::make_shared<aimdk_msgs::srv::ShutdownTask::Request>();
      request->header.header.stamp = this->now();
      request->token.source = source;
      request->token.token = "";

      RCLCPP_INFO(this->get_logger(),
                  "Sending ShutdownTask request: source=%s", source.c_str());

      auto future = client_->async_send_request(request);

      for (int i = 0; i < kMaxRetryCount; ++i) {
        auto retcode = rclcpp::spin_until_future_complete(
          shared_from_this(), future, kServiceCallTimeout);

        if (retcode == rclcpp::FutureReturnCode::SUCCESS) {
          auto response = future.get();
          if (!response) {
            RCLCPP_ERROR(this->get_logger(), "ShutdownTask returned no response.");
            return false;
          }

          const auto code = response->header.header.code;
          if (code == 0) {
            RCLCPP_INFO(this->get_logger(),
                        "ShutdownTask request sent successfully. "
                        "Robot is shutting down...");
            return true;
          }

          RCLCPP_ERROR(this->get_logger(),
                       "ShutdownTask failed with code: %ld, "
                       "status: %d, reason: %u, message: %s",
                       code,
                       response->header.status.value,
                       response->header.status.reason,
                       response->header.message.c_str());
          return false;
        }

        RCLCPP_INFO(this->get_logger(),
                    "ShutdownTask attempt %d/%d timed out, retrying...",
                    i + 1, kMaxRetryCount);
        std::this_thread::sleep_for(200ms);

        future = client_->async_send_request(request);
      }

      RCLCPP_ERROR(this->get_logger(),
                   "ShutdownTask failed after %d attempts", kMaxRetryCount);
      return false;
    } catch (const std::exception &e) {
      RCLCPP_ERROR(this->get_logger(), "Exception occurred: %s", e.what());
      return false;
    }
  }

 private:
  rclcpp::Client<aimdk_msgs::srv::ShutdownTask>::SharedPtr client_;
};

int main(int argc, char *argv[])
{
  try {
    rclcpp::init(argc, argv);
    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);

    g_node      = std::make_shared<ShutdownTaskClient>();
    auto client = std::dynamic_pointer_cast<ShutdownTaskClient>(g_node);
    bool ok     = client ? client->send_request() : false;

    g_node.reset();
    rclcpp::shutdown();
    return ok ? 0 : 1;
  } catch (const std::exception &e) {
    RCLCPP_ERROR(rclcpp::get_logger("main"), "Exited with exception: %s", e.what());
    return 1;
  }
}
