/**
 * @brief Example client for /aimdk_5Fmsgs/srv/SetMcAction and
 * /aimdk_5Fmsgs/srv/SetMcMotion
 *
 * The following ROS parameters can be set via startup arguments:
 * --ros-args -p <name>:=<value>
 *
 * Supported parameters:
 *   - type: "action" or "motion", required
 *   - action_desc: string, required when type=action
 *   - motion: string, required when type=motion
 *   - interrupt: bool, optional when type=motion, default=true
 *
 * Examples:
 *   ros2 run aimdk_examples_cpp set_mc_action --ros-args -p type:=action -p
 *   action_desc:=BIPED_STAND_DEFAULT
 *
 *   ros2 run aimdk_examples_cpp set_mc_action --ros-args -p type:=motion -p
 *   motion:=INTRO_POSE6 -p interrupt:=true
 */
#include "aimdk_msgs/msg/common_request.hpp"
#include "aimdk_msgs/msg/common_state.hpp"
#include "aimdk_msgs/msg/mc_action_status.hpp"
#include "aimdk_msgs/msg/mc_motion_type.hpp"
#include "aimdk_msgs/srv/get_mc_action.hpp"
#include "aimdk_msgs/srv/set_mc_action.hpp"
#include "aimdk_msgs/srv/set_mc_motion.hpp"
#include "rclcpp/rclcpp.hpp"

#include <chrono>
#include <cstdint>
#include <memory>
#include <signal.h>
#include <string>
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
    type_ = this->declare_parameter<std::string>("type", "");
    action_desc_ = this->declare_parameter<std::string>("action_desc", "");
    motion_ = this->declare_parameter<std::string>("motion", "");
    interrupt_ = this->declare_parameter<bool>("interrupt", true);

    set_action_client_ = this->create_client<aimdk_msgs::srv::SetMcAction>(
        "/aimdk_5Fmsgs/srv/SetMcAction");
    set_motion_client_ = this->create_client<aimdk_msgs::srv::SetMcMotion>(
        "/aimdk_5Fmsgs/srv/SetMcMotion");
    get_client_ = this->create_client<aimdk_msgs::srv::GetMcAction>(
        "/aimdk_5Fmsgs/srv/GetMcAction");
    RCLCPP_INFO(this->get_logger(),
                "SetMcAction client node created with type=%s action_desc=%s "
                "motion=%s interrupt=%s",
                type_.c_str(), action_desc_.c_str(), motion_.c_str(),
                interrupt_ ? "true" : "false");
  }

  bool execute() {
    if (!validate_parameters()) {
      return false;
    }

    wait_for_services();

    if (type_ == "action") {
      ActionInfo info;
      if (get_action_status(info) && info.action_desc == action_desc_) {
        if (info.status == aimdk_msgs::msg::McActionStatus::RUNNING) {
          RCLCPP_INFO(this->get_logger(),
                      "Target action is already running: action_desc=%s",
                      action_desc_.c_str());
          return true;
        }

        RCLCPP_INFO(this->get_logger(),
                    "Target action is already active with status=%d, waiting "
                    "for RUNNING: action_desc=%s",
                    info.status, action_desc_.c_str());
        return wait_for_action(action_desc_);
      }

      if (!set_action(action_desc_)) {
        return false;
      }
      return wait_for_action(action_desc_);
    }

    if (!set_motion(motion_, interrupt_)) {
      return false;
    }
    return wait_for_motion();
  }

private:
  struct ActionInfo {
    int32_t action_id = 0;
    std::string action_desc;
    int32_t status = aimdk_msgs::msg::McActionStatus::IDLE;
  };

  bool validate_parameters() {
    if (type_.empty()) {
      RCLCPP_ERROR(this->get_logger(),
                   "Parameter 'type' must be set. Use 'action' or 'motion'.");
      return false;
    }

    if (type_ != "action" && type_ != "motion") {
      RCLCPP_ERROR(this->get_logger(),
                   "Invalid parameter 'type': %s. Use 'action' or 'motion'.",
                   type_.c_str());
      return false;
    }

    if (type_ == "action" && action_desc_.empty()) {
      RCLCPP_ERROR(this->get_logger(),
                   "Parameter 'action_desc' must be set when "
                   "type=action.");
      return false;
    }

    if (type_ == "motion" && motion_.empty()) {
      RCLCPP_ERROR(this->get_logger(),
                   "Parameter 'motion' must be set when type=motion.");
      return false;
    }

    return true;
  }

  bool set_action(const std::string &action_desc) {
    try {
      auto request = std::make_shared<aimdk_msgs::srv::SetMcAction::Request>();
      request->header.stamp = this->now();
      request->command.action.value = 0;
      request->command.action_desc = action_desc;

      RCLCPP_INFO(this->get_logger(), "Sending request: action_desc=%s",
                  action_desc.c_str());

      auto future = set_action_client_->async_send_request(request);
      auto retcode = rclcpp::spin_until_future_complete(
          shared_from_this(), future, std::chrono::seconds(2));

      if (retcode != rclcpp::FutureReturnCode::SUCCESS) {
        ActionInfo info;
        if (get_action_status(info) && info.action_desc == action_desc) {
          RCLCPP_WARN(this->get_logger(),
                      "SetMcAction request timed out, but target action is "
                      "already active: action_desc=%s status=%d",
                      info.action_desc.c_str(), info.status);
          return true;
        }

        RCLCPP_ERROR(this->get_logger(), "Service call failed or timed out.");
        return false;
      }

      auto response = future.get();
      if (response->response.status.value ==
          aimdk_msgs::msg::CommonState::SUCCESS) {
        RCLCPP_INFO(this->get_logger(),
                    "SetMcAction request accepted by service.");
        return true;
      }

      RCLCPP_ERROR(this->get_logger(), "Failed to set robot mode: %s",
                   response->response.message.c_str());
      return false;
    } catch (const std::exception &e) {
      RCLCPP_ERROR(this->get_logger(), "Exception occurred: %s", e.what());
      return false;
    }
  }

  bool set_motion(const std::string &motion_name, bool interrupt) {
    try {
      constexpr int kMaxAttempts = 5;
      const auto request_timeout = std::chrono::seconds(1);

      for (int attempt = 1; attempt <= kMaxAttempts; ++attempt) {
        auto request =
            std::make_shared<aimdk_msgs::srv::SetMcMotion::Request>();
        request->header.stamp = this->now();
        request->motion.tag = motion_name;
        request->motion.type.value = aimdk_msgs::msg::McMotionType::MIMIC;
        request->interrupt = interrupt;

        RCLCPP_INFO(this->get_logger(),
                    "Sending SetMcMotion request (%d/%d): motion=%s "
                    "interrupt=%s",
                    attempt, kMaxAttempts, motion_name.c_str(),
                    interrupt ? "true" : "false");

        auto future = set_motion_client_->async_send_request(request);
        auto retcode = rclcpp::spin_until_future_complete(
            shared_from_this(), future, request_timeout);

        if (retcode != rclcpp::FutureReturnCode::SUCCESS) {
          RCLCPP_WARN(this->get_logger(),
                      "SetMcMotion request attempt %d/%d failed or timed out.",
                      attempt, kMaxAttempts);
          continue;
        }

        auto response = future.get();
        const auto code = response->response.header.code;
        const auto status = response->response.status.value;

        if (code == 0 &&
            (status == aimdk_msgs::msg::CommonState::SUCCESS ||
             status == aimdk_msgs::msg::CommonState::RUNNING)) {
          RCLCPP_INFO(this->get_logger(),
                      "SetMcMotion request accepted by service: code=%ld "
                      "status=%d",
                      static_cast<long>(code), status);
          return true;
        }

        RCLCPP_WARN(this->get_logger(),
                    "SetMcMotion request attempt %d/%d was not accepted: "
                    "code=%ld status=%d",
                    attempt, kMaxAttempts, static_cast<long>(code), status);
      }

      RCLCPP_ERROR(this->get_logger(),
                   "Failed to set motion after %d attempts.", kMaxAttempts);
      return false;
    } catch (const std::exception &e) {
      RCLCPP_ERROR(this->get_logger(), "Exception occurred: %s", e.what());
      return false;
    }
  }

  bool wait_for_action(
      const std::string &expected_action_desc,
      std::chrono::seconds timeout = std::chrono::seconds(10),
      std::chrono::milliseconds poll_interval = std::chrono::milliseconds(200)) {
    auto deadline = std::chrono::steady_clock::now() + timeout;

    RCLCPP_INFO(this->get_logger(),
                "Waiting for target action_desc=%s to reach RUNNING state...",
                expected_action_desc.c_str());

    while (rclcpp::ok() && std::chrono::steady_clock::now() < deadline) {
      ActionInfo info;
      if (!get_action_status(info)) {
        std::this_thread::sleep_for(poll_interval);
        continue;
      }

      if (info.status == aimdk_msgs::msg::McActionStatus::RUNNING &&
          info.action_desc == expected_action_desc) {
        RCLCPP_INFO(this->get_logger(),
                    "Target action reached and is running: action_desc=%s",
                    expected_action_desc.c_str());
        return true;
      }

      std::this_thread::sleep_for(poll_interval);
    }

    RCLCPP_ERROR(this->get_logger(),
                 "Timed out waiting for target action_desc=%s to reach "
                 "RUNNING state.",
                 expected_action_desc.c_str());
    return false;
  }

  bool wait_for_motion(
      std::chrono::seconds timeout = std::chrono::seconds(10),
      std::chrono::milliseconds poll_interval = std::chrono::milliseconds(200)) {
    auto deadline = std::chrono::steady_clock::now() + timeout;

    RCLCPP_INFO(this->get_logger(),
                "Waiting for current motion action to reach RUNNING state...");

    while (rclcpp::ok() && std::chrono::steady_clock::now() < deadline) {
      ActionInfo info;
      if (!get_action_status(info)) {
        std::this_thread::sleep_for(poll_interval);
        continue;
      }

      if (info.status == aimdk_msgs::msg::McActionStatus::RUNNING) {
        RCLCPP_INFO(this->get_logger(),
                    "Current motion action is running: action_id=%d "
                    "action_desc=%s status=%d",
                    info.action_id, info.action_desc.c_str(), info.status);
        return true;
      }

      std::this_thread::sleep_for(poll_interval);
    }

    RCLCPP_ERROR(this->get_logger(),
                 "Timed out waiting for current motion action to reach "
                 "RUNNING state.");
    return false;
  }

  bool get_action_status(ActionInfo &info) {
    try {
      auto request = std::make_shared<aimdk_msgs::srv::GetMcAction::Request>();
      request->request = aimdk_msgs::msg::CommonRequest();
      request->request.header.stamp = this->now();

      auto future = get_client_->async_send_request(request);
      auto retcode = rclcpp::spin_until_future_complete(
          shared_from_this(), future, std::chrono::seconds(2));

      if (retcode != rclcpp::FutureReturnCode::SUCCESS) {
        RCLCPP_WARN(this->get_logger(),
                    "Get current action request service call failed or timed "
                    "out.");
        return false;
      }

      auto response = future.get();
      info.action_id = response->info.current_action.value;
      info.action_desc = response->info.action_desc;
      info.status = response->info.status.value;
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
      RCLCPP_INFO(this->get_logger(), "Service unavailable, waiting: %s",
                  service_name.c_str());
    }
    RCLCPP_INFO(this->get_logger(), "Service available: %s",
                service_name.c_str());
  }

  void wait_for_services() {
    wait_for_service(set_action_client_, "/aimdk_5Fmsgs/srv/SetMcAction");
    wait_for_service(set_motion_client_, "/aimdk_5Fmsgs/srv/SetMcMotion");
    wait_for_service(get_client_, "/aimdk_5Fmsgs/srv/GetMcAction");
  }

  std::string type_;
  std::string action_desc_;
  std::string motion_;
  bool interrupt_ = true;

  rclcpp::Client<aimdk_msgs::srv::SetMcAction>::SharedPtr set_action_client_;
  rclcpp::Client<aimdk_msgs::srv::SetMcMotion>::SharedPtr set_motion_client_;
  rclcpp::Client<aimdk_msgs::srv::GetMcAction>::SharedPtr get_client_;
};

int main(int argc, char *argv[]) {
  try {
    rclcpp::init(argc, argv);
    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);

    g_node = std::make_shared<SetMcActionClient>();
    auto client = std::dynamic_pointer_cast<SetMcActionClient>(g_node);
    const bool ok = client ? client->execute() : false;

    g_node.reset();
    rclcpp::shutdown();

    return ok ? 0 : 1;
  } catch (const std::exception &e) {
    RCLCPP_ERROR(rclcpp::get_logger("main"),
                 "Program exited with exception: %s", e.what());
    return 1;
  }
}
