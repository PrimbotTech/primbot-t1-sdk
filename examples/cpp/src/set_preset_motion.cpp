#include "aimdk_msgs/msg/common_response.hpp"
#include "aimdk_msgs/msg/common_state.hpp"
#include "aimdk_msgs/msg/common_task_response.hpp"
#include "aimdk_msgs/msg/mc_preset_motion.hpp"
#include "aimdk_msgs/msg/request_header.hpp"
#include "aimdk_msgs/srv/set_mc_preset_motion.hpp"
#include "rclcpp/rclcpp.hpp"
#include <chrono>
#include <memory>
#include <signal.h>

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

class PresetMotionClient : public rclcpp::Node {
public:
  PresetMotionClient() : Node("preset_motion_client") {
    const std::chrono::seconds timeout(8);

    client_ = this->create_client<aimdk_msgs::srv::SetMcPresetMotion>(
        "/aimdk_5Fmsgs/srv/SetMcPresetMotion");

    RCLCPP_INFO(this->get_logger(), "SetMcPresetMotion client node created.");

    while (!client_->wait_for_service(std::chrono::seconds(2))) {
      if (!rclcpp::ok()) {
        return;
      }
      RCLCPP_INFO(this->get_logger(), "Service unavailable, waiting...");
    }
    RCLCPP_INFO(this->get_logger(),
                "Service available, ready to send request.");
  }

  bool send_request(int motion_id) {
    try {
      auto request =
          std::make_shared<aimdk_msgs::srv::SetMcPresetMotion::Request>();
      request->header = aimdk_msgs::msg::RequestHeader();

      // motion.value is the preset motion ID.
      aimdk_msgs::msg::McPresetMotion motion;
      motion.value = motion_id;
      request->motion = motion;
      request->interrupt = true;

      RCLCPP_INFO(this->get_logger(),
                  "Sending request to set preset motion: motion=%d",
                  motion_id);

      const std::chrono::milliseconds timeout(2000);
      request->header.stamp = this->now();
      auto future = client_->async_send_request(request);
      auto retcode = rclcpp::spin_until_future_complete(shared_from_this(),
                                                        future, timeout);
      if (retcode != rclcpp::FutureReturnCode::SUCCESS) {
        RCLCPP_ERROR(this->get_logger(), "Service call failed or timed out.");
        return false;
      }

      auto response = future.get();
      auto code = response->response.header.code;
      auto state = response->response.state.value;

      // Treat both SUCCESS and RUNNING as successful requests.
      if (code != 0) {
        RCLCPP_WARN(this->get_logger(),
                    "Failed to set preset motion: code=%ld, state=%d, task_id=%lu",
                    code, state, response->response.task_id);
        return false;
      }

      if (state == aimdk_msgs::msg::CommonState::SUCCESS) {
        RCLCPP_INFO(this->get_logger(), "Preset motion set successfully: %lu",
                    response->response.task_id);
        return true;
      } else if (state == aimdk_msgs::msg::CommonState::RUNNING) {
        RCLCPP_INFO(this->get_logger(), "Preset motion executing: %lu",
                    response->response.task_id);
        return true;
      } else {
        RCLCPP_WARN(this->get_logger(),
                    "Failed to set preset motion: code=%ld, state=%d, task_id=%lu",
                    code, state, response->response.task_id);
        return false;
      }
    } catch (const std::exception &e) {
      RCLCPP_ERROR(this->get_logger(), "Exception occurred: %s", e.what());
      return false;
    }
  }

private:
  rclcpp::Client<aimdk_msgs::srv::SetMcPresetMotion>::SharedPtr client_;
};

int main(int argc, char *argv[]) {
  try {
    rclcpp::init(argc, argv);
    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);

    g_node = std::make_shared<PresetMotionClient>();
    auto client = std::dynamic_pointer_cast<PresetMotionClient>(g_node);

    int motion = 1003;
    std::cout
        << "Enter preset motion ID(default: 1003): ";
    std::cin >> motion;
    if (client) {
      client->send_request(motion);
    }
    g_node.reset();
    rclcpp::shutdown();

    return 0;
  } catch (const std::exception &e) {
    RCLCPP_ERROR(rclcpp::get_logger("main"),
                 "Program exited with exception: %s", e.what());
    return 1;
  }
}
