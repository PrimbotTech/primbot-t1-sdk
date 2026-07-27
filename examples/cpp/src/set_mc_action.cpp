/*
 @brief Action Switch Example Script
 
 Description:
   This script demonstrates how to switch robot action states through the
   shared McActionSwitcher state-machine interface.
 
 Prerequisites:
   - Robot motion control service must be running
   - SetMcAction and GetMcAction services must be available
 
 Usage:
   ros2 run aimdk_examples_cpp set_mc_action --ros-args -p type:=<TYPE> \
   [-p action_desc:=<ACTION>] [-p motion:=<MOTION>]
 
 Example:
   # Interactive mode: input target action via terminal
   ros2 run aimdk_examples_cpp set_mc_action --ros-args -p type:=action
 
   # Non-interactive mode: specify action via parameter
   ros2 run aimdk_examples_cpp set_mc_action --ros-args -p type:=action \
   -p action_desc:=QUADRUPED_LOCOMOTION_JUMP

   # Motion mode: switch to the matching biped/quadruped locomotion action first
   ros2 run aimdk_examples_cpp set_mc_action --ros-args -p type:=motion \
   -p motion:=<MOTION>
 
 Parameters:
   - type: "action" or "motion", required
   - action_desc: string, optional; if set, skips interactive input and executes directly
   - motion: string, required when type=motion; BIPED_* and QUAD_* names are
     routed to the corresponding whole-body action automatically
 
 Notes:
   - Single Switch: Switches to the requested target action and waits for completion.
   - Auto Path: McActionSwitcher finds and executes a valid transition path.
   - Recovery: McActionSwitcher handles intermediate and automatic transitions.
   - Retry: McActionSwitcher waits for each requested action to become active.
   - WARNING: Do NOT use QUADRUPED_LOCOMOTION_JUMP for testing — the robot will jump and may cause injury or damage.
 */
#include "aimdk_msgs/msg/common_request.hpp"
#include "aimdk_msgs/msg/common_state.hpp"
#include "aimdk_msgs/msg/mc_action_status.hpp"
#include "aimdk_msgs/srv/get_mc_action.hpp"
#include "aimdk_msgs/srv/set_mc_action.hpp"
#include "aimdk_msgs/srv/set_mc_motion.hpp"
#include "mc_action_switcher.hpp"
#include "rclcpp/rclcpp.hpp"

#include <atomic>
#include <chrono>
#include <cstdint>
#include <memory>
#include <signal.h>
#include <string>
#include <thread>

constexpr double kServiceCallTimeoutSec = 2.0;
constexpr int kMaxRetryCount = 3;

std::shared_ptr<rclcpp::Node> g_node = nullptr;
std::atomic_bool g_shutdown_requested{false};

void signal_handler(int signal) {
  const bool already_requested = g_shutdown_requested.exchange(true);
  if (already_requested) {
    rclcpp::shutdown();
    return;
  }

  if (g_node) {
    RCLCPP_WARN(g_node->get_logger(),
                "Received signal %d, cancelling the current action switch.",
                signal);
  }
}

class SetMcActionClient : public rclcpp::Node {
public:
  SetMcActionClient() : Node("set_mc_action_client") {
    type_ = this->declare_parameter<std::string>("type", "");
    action_desc_ = this->declare_parameter<std::string>("action_desc", "");
    motion_ = this->declare_parameter<std::string>("motion", "");
    set_motion_client_ = this->create_client<aimdk_msgs::srv::SetMcMotion>(
        "/aimdk_5Fmsgs/srv/SetMcMotion");
    get_client_ = this->create_client<aimdk_msgs::srv::GetMcAction>(
        "/aimdk_5Fmsgs/srv/GetMcAction");
    RCLCPP_INFO(this->get_logger(),
                "SetMcAction client node created with type=%s action_desc=%s "
                "motion=%s",
                type_.c_str(), action_desc_.c_str(), motion_.c_str());
  }

  struct ActionInfo {
    std::string action_desc;
    int32_t status = aimdk_msgs::msg::McActionStatus::IDLE;
  };

  bool switch_action(const std::string &target_action) {
    aimdk_examples::McActionSwitcher switcher(shared_from_this());
    aimdk_examples::McActionSwitchOptions options;
    options.source = "set_mc_action";
    options.total_timeout = std::chrono::seconds(30);
    options.should_cancel = []() {
      return g_shutdown_requested.load();
    };

    const auto result = switcher.switch_to(target_action, options);
    if (!result.success) {
      RCLCPP_ERROR(
          this->get_logger(),
          "Failed to switch from %s to %s: %s",
          result.current_action.empty() ? "(unknown)" : result.current_action.c_str(),
          target_action.c_str(), result.message.c_str());
    }
    return result.success;
  }

  bool execute() {
    if (!validate_parameters()) return false;
    wait_for_services();

    if (type_ == "action") {
      ActionInfo current;
      if (!get_action_status(current)) return false;
      RCLCPP_INFO(this->get_logger(), "Current Action is: %s", current.action_desc.c_str());

      std::string target_action;
      if (action_desc_.empty()) {
        std::cout << "Current Action is: " << current.action_desc
                  << ", please input the expected Action according to the "
                     "motion control state machine transition logic in the "
                     "interface documentation. The Action you need to switch: "
                  << std::flush;
        if (!std::getline(std::cin, target_action) || target_action.empty()) return true;
      } else {
        target_action = action_desc_;
      }

      // Execute SetMcAction through the shared state-machine switcher.
      bool ok = switch_action(target_action);
      if (g_shutdown_requested.load()) {
        return false;
      }
      std::cout << (ok ? "Switch succeeded." :
                       "Switch failed, please confirm if the expected Action "
                       "complies with the state machine transition logic")
                << std::endl;

      return true;
    } else if (type_ == "motion") {
      const char *target_action = motion_action_target(motion_);
      if (target_action == nullptr) {
        RCLCPP_ERROR(
            this->get_logger(),
            "Cannot determine the robot form for motion '%s'. Motion names "
            "must start with BIPED_ or QUAD_.",
            motion_.c_str());
        return false;
      }

      RCLCPP_INFO(this->get_logger(),
                  "Switching to %s before executing motion %s.",
                  target_action, motion_.c_str());
      if (!switch_action(target_action)) {
        return false;
      }

      if (g_shutdown_requested.load()) {
        return false;
      }

      if (!set_motion(motion_)) {
        return false;
      }

      return wait_for_motion();
    }

    return false;
  }

  bool get_action_status(ActionInfo &info) {
    try {
      auto request = std::make_shared<aimdk_msgs::srv::GetMcAction::Request>();
      request->request = aimdk_msgs::msg::CommonRequest();
      request->request.header.stamp = this->now();

      auto future = call_service_with_retry<aimdk_msgs::srv::GetMcAction>(
          get_client_, request, "GetMcAction");

      if (!future.valid()) {
        RCLCPP_WARN(this->get_logger(), "Get current action request failed or timed out.");
        return false;
      }

      auto response = future.get();
      info.action_desc = response->info.action_desc;
      info.status = response->info.status.value;
      return true;
    } catch (const std::exception &e) {
      RCLCPP_ERROR(this->get_logger(), "Exception occurred: %s", e.what());
      return false;
    }
  }

private:
  static const char *motion_action_target(const std::string &motion_name) {
    if (motion_name.rfind("QUAD_", 0) == 0 ||
        motion_name.rfind("QUADRUPED_", 0) == 0) {
      return "QUADRUPED_LOCOMOTION_DEFAULT";
    }
    if (motion_name.rfind("BIPED_", 0) == 0) {
      return "BIPED_LOCOMOTION_WBC";
    }
    return nullptr;
  }

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

    if (type_ == "motion" && motion_.empty()) {
      RCLCPP_ERROR(this->get_logger(),
                   "Parameter 'motion' must be set when type=motion.");
      return false;
    }

    return true;
  }

  // Generic retry function for service calls
  template <typename ServiceT>
  typename rclcpp::Client<ServiceT>::SharedFuture
  call_service_with_retry(
      typename rclcpp::Client<ServiceT>::SharedPtr client,
      typename ServiceT::Request::SharedPtr request,
      const std::string &service_name,
      std::chrono::milliseconds timeout = std::chrono::milliseconds(
          static_cast<int>(kServiceCallTimeoutSec * 1000)),
      int max_retries = kMaxRetryCount) {
    for (int i = 0; i < max_retries; ++i) {
      auto future = client->async_send_request(request);
      if (rclcpp::spin_until_future_complete(this->shared_from_this(), future, timeout) ==
          rclcpp::FutureReturnCode::SUCCESS) {
        return future.future.share();
      }
      RCLCPP_INFO(this->get_logger(), "%s attempt %d/%d timed out, retrying...",
                  service_name.c_str(), i + 1, max_retries);
      std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }
    RCLCPP_ERROR(this->get_logger(), "%s failed after %d attempts", service_name.c_str(), max_retries);
    return typename rclcpp::Client<ServiceT>::SharedFuture();
  }

  bool set_motion(const std::string &motion_name) {
    try {
      for (int attempt = 1; attempt <= 5; ++attempt) {
        auto request = std::make_shared<aimdk_msgs::srv::SetMcMotion::Request>();
        request->header.stamp = this->now();
        request->source = "set_mc_action";
        request->motion = motion_name;
        request->type = aimdk_msgs::srv::SetMcMotion::Request::MIMIC_QY;
        request->interrupt = false;

        RCLCPP_INFO(this->get_logger(),
                    "Sending SetMcMotion request (%d/5): motion=%s",
                    attempt, motion_name.c_str());

        auto future = set_motion_client_->async_send_request(request);
        if (rclcpp::spin_until_future_complete(shared_from_this(), future, std::chrono::seconds(1)) !=
            rclcpp::FutureReturnCode::SUCCESS) {
          RCLCPP_WARN(this->get_logger(), "SetMcMotion request attempt %d/5 failed or timed out.", attempt);
          continue;
        }

        auto response = future.get();
        auto code = response->response.header.code;
        auto state = response->response.state.value;

        if (code == 0 && (state == aimdk_msgs::msg::CommonState::SUCCESS ||
                          state == aimdk_msgs::msg::CommonState::RUNNING)) {
          RCLCPP_INFO(this->get_logger(), "SetMcMotion request accepted: code=%ld state=%d",
                      static_cast<long>(code), state);
          return true;
        }

        RCLCPP_WARN(this->get_logger(), "SetMcMotion request attempt %d/5 not accepted: code=%ld state=%d",
                    attempt, static_cast<long>(code), state);
      }

      RCLCPP_ERROR(this->get_logger(), "Failed to set motion after 5 attempts.");
      return false;
    } catch (const std::exception &e) {
      RCLCPP_ERROR(this->get_logger(), "Exception occurred: %s", e.what());
      return false;
    }
  }

  bool wait_for_motion(
      std::chrono::seconds timeout = std::chrono::seconds(10),
      std::chrono::milliseconds poll_interval = std::chrono::milliseconds(200)) {
    auto deadline = std::chrono::steady_clock::now() + timeout;
    RCLCPP_INFO(this->get_logger(), "Waiting for current motion action to reach RUNNING state...");

    while (rclcpp::ok() && std::chrono::steady_clock::now() < deadline) {
      ActionInfo info;
      if (!get_action_status(info)) {
        std::this_thread::sleep_for(poll_interval);
        continue;
      }
      if (info.status == aimdk_msgs::msg::McActionStatus::RUNNING) {
        RCLCPP_INFO(this->get_logger(),
                    "Current motion action is running: action_desc=%s",
                    info.action_desc.c_str());
        return true;
      }
      std::this_thread::sleep_for(poll_interval);
    }

    RCLCPP_ERROR(this->get_logger(), "Timed out waiting for current motion action to reach RUNNING state.");
    return false;
  }

  template <typename ClientT>
  void wait_for_service(const std::shared_ptr<ClientT> &client, const std::string &service_name) {
    while (!client->wait_for_service(std::chrono::seconds(2))) {
      if (!rclcpp::ok()) return;
      RCLCPP_INFO(this->get_logger(), "Service unavailable, waiting: %s", service_name.c_str());
    }
    RCLCPP_INFO(this->get_logger(), "Service available: %s", service_name.c_str());
  }

  void wait_for_services() {
    wait_for_service(get_client_, "/aimdk_5Fmsgs/srv/GetMcAction");
    if (type_ == "motion") {
      wait_for_service(set_motion_client_, "/aimdk_5Fmsgs/srv/SetMcMotion");
    }
  }

  std::string type_;
  std::string action_desc_;
  std::string motion_;
  rclcpp::Client<aimdk_msgs::srv::SetMcMotion>::SharedPtr set_motion_client_;
  rclcpp::Client<aimdk_msgs::srv::GetMcAction>::SharedPtr get_client_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  signal(SIGINT, signal_handler);
  signal(SIGTERM, signal_handler);

  auto node = std::make_shared<SetMcActionClient>();
  g_node = node;

  bool ok = false;
  try {
    ok = node->execute();
  } catch (...) {
    if (node && rclcpp::ok()) {
      RCLCPP_WARN(node->get_logger(),
                  "Interrupted by an exception; shutting down.");
    }
  }

  node.reset();
  g_node.reset();
  if (rclcpp::ok()) rclcpp::shutdown();
  return g_shutdown_requested.load() ? 130 : (ok ? 0 : 1);
}
