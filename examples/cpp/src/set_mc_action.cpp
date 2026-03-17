#include "aimdk_msgs/msg/common_response.hpp"
#include "aimdk_msgs/msg/common_request.hpp"
#include "aimdk_msgs/msg/common_state.hpp"
#include "aimdk_msgs/msg/mc_action_command.hpp"
#include "aimdk_msgs/msg/mc_action_status.hpp"
#include "aimdk_msgs/msg/request_header.hpp"
#include "aimdk_msgs/srv/get_mc_action.hpp"
#include "aimdk_msgs/srv/set_mc_action.hpp"
#include "rclcpp/rclcpp.hpp"
#include <chrono>
#include <iostream>
#include <memory>
#include <signal.h>
#include <thread>

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

class SetMcActionClient : public rclcpp::Node {
public:
  SetMcActionClient() : Node("set_mc_action_client") {
    set_client_ = this->create_client<aimdk_msgs::srv::SetMcAction>(
        "/aimdk_5Fmsgs/srv/SetMcAction");
    get_client_ = this->create_client<aimdk_msgs::srv::GetMcAction>(
        "/aimdk_5Fmsgs/srv/GetMcAction");
    RCLCPP_INFO(this->get_logger(), "✅ SetMcAction client node created.");

    wait_for_service(set_client_, "/aimdk_5Fmsgs/srv/SetMcAction");
    wait_for_service(get_client_, "/aimdk_5Fmsgs/srv/GetMcAction");
  }

  bool set_action(int32_t action_value) {
    try {
      auto request = std::make_shared<aimdk_msgs::srv::SetMcAction::Request>();
      request->header.stamp = this->now();
      request->command.action.value = action_value;
      request->command.action_desc = "";

      RCLCPP_INFO(this->get_logger(), "📨 Sending request: action_id=%d",
                  action_value);
   
      auto future = set_client_->async_send_request(request);
      auto retcode = rclcpp::spin_until_future_complete(
          shared_from_this(), future, std::chrono::seconds(2));

      if (retcode != rclcpp::FutureReturnCode::SUCCESS) {
        RCLCPP_ERROR(this->get_logger(), "❌ Service call failed or timed out.");
        return false;
      }

      auto response = future.get();
      if (response->response.status.value == aimdk_msgs::msg::CommonState::SUCCESS) {
        RCLCPP_INFO(this->get_logger(), "✅ SetMcAction request accepted by service.");
        return true;
      }

      RCLCPP_ERROR(this->get_logger(), "❌ Failed to set robot mode: %s",
                   response->response.message.c_str());
      return false;
    } catch (const std::exception &e) {
      RCLCPP_ERROR(this->get_logger(), "Exception occurred: %s", e.what());
      return false;
    }
  }

  bool wait_for_action(
      int32_t expected_action_id,
      std::chrono::seconds timeout = std::chrono::seconds(10),
      std::chrono::milliseconds poll_interval = std::chrono::milliseconds(200)) {
    auto deadline = std::chrono::steady_clock::now() + timeout;

    RCLCPP_INFO(this->get_logger(),
                "⏳ Waiting for target action_id=%d to reach RUNNING state...",
                expected_action_id);

                
    while (rclcpp::ok() && std::chrono::steady_clock::now() < deadline) {
      int32_t current_action_id = 0;
      int32_t current_status = aimdk_msgs::msg::McActionStatus::IDLE;
      if (!get_action_status(current_action_id, current_status)) {
        std::this_thread::sleep_for(poll_interval);
        continue;
      }

      if (current_status == aimdk_msgs::msg::McActionStatus::RUNNING &&
          current_action_id == expected_action_id) {
        RCLCPP_INFO(this->get_logger(),
                    "✅ Target action reached and is running: action_id=%d",
                    expected_action_id);
        return true;
      }

      std::this_thread::sleep_for(poll_interval);
    }

    RCLCPP_ERROR(this->get_logger(),
                 "❌ Timed out waiting for target action_id=%d to reach "
                 "RUNNING state.",
                 expected_action_id);
    return false;
  }

private:
  bool get_action_status(int32_t &action_id, int32_t &status) {
    try {
      auto request = std::make_shared<aimdk_msgs::srv::GetMcAction::Request>();
      request->request = aimdk_msgs::msg::CommonRequest();
      request->request.header.stamp = this->now();

      auto future = get_client_->async_send_request(request);
      auto retcode = rclcpp::spin_until_future_complete(
          shared_from_this(), future, std::chrono::seconds(2));

      if (retcode != rclcpp::FutureReturnCode::SUCCESS) {
        RCLCPP_WARN(this->get_logger(),
                    "⚠️ Get current action request service call failed or timed "
                    "out.");
        return false;
      }

      auto response = future.get();
      action_id = response->info.current_action.value;
      status = response->info.status.value;
      return true;
    } catch (const std::exception &e) {
      RCLCPP_ERROR(this->get_logger(), "Exception occurred: %s", e.what());
      return false;
    }
  }

  template <typename ClientT>
  void wait_for_service(
      const std::shared_ptr<ClientT> &client,
      const std::string &service_name) {
    while (!client->wait_for_service(std::chrono::seconds(2))) {
      if (!rclcpp::ok()) {
        return;
      }
      RCLCPP_INFO(this->get_logger(), "⏳ Service unavailable, waiting: %s",
                  service_name.c_str());
    }
    RCLCPP_INFO(this->get_logger(), "🟢 Service available: %s",
                service_name.c_str());
  }

  rclcpp::Client<aimdk_msgs::srv::SetMcAction>::SharedPtr set_client_;
  rclcpp::Client<aimdk_msgs::srv::GetMcAction>::SharedPtr get_client_;
};

int main(int argc, char *argv[]) {
  try {
    rclcpp::init(argc, argv);
    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);

    int32_t action_id = 0;
    std::cout << "Enter action_id: ";
    std::cin >> action_id;

    g_node = std::make_shared<SetMcActionClient>();
    auto client = std::dynamic_pointer_cast<SetMcActionClient>(g_node);
    if (client) {
      if (!client->set_action(action_id)) {
        g_node.reset();
        rclcpp::shutdown();
        return 1;
      }
      if (!client->wait_for_action(action_id)) {
        g_node.reset();
        rclcpp::shutdown();
        return 1;
      }
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
