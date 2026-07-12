/*
 @brief Example client for /aima/mc/locomotion/velocity.
 
 This script automatically handles the required state machine transitions for safety.
 Locomotion control (walking/running) requires the robot to be in
 QUADRUPED_LOCOMOTION_DEFAULT mode.
 
 Prerequisites auto-handled by this script:
   The script ensures a safe sequential transition path:
   PASSIVE_DEFAULT -> QUADRUPED_STAND_DEFAULT -> QUADRUPED_LOCOMOTION_DEFAULT
   Depending on the initial state, it enters the sequence at the appropriate step.
 
 Flow:
   1. Detect current state and transition to QUADRUPED_LOCOMOTION_DEFAULT sequentially.
   2. Register this node as an authorized input source (priority 80).
   3. Prompt the user for target velocities.
   4. Publish velocity commands for 5 seconds.
   5. Stop the robot by sending zero velocity.
   6. Release the registered input source.
 
 Usage:
   ros2 run aimdk_examples_cpp mc_locomotion_velocity
 */
#include "aimdk_msgs/msg/mc_locomotion_velocity.hpp"
#include "aimdk_msgs/msg/common_request.hpp"
#include "aimdk_msgs/msg/common_response.hpp"
#include "aimdk_msgs/msg/common_state.hpp"
#include "aimdk_msgs/msg/common_task_response.hpp"
#include "aimdk_msgs/msg/mc_input_action.hpp"
#include "aimdk_msgs/msg/message_header.hpp"
#include "aimdk_msgs/msg/mc_action.hpp"
#include "aimdk_msgs/msg/mc_action_command.hpp"
#include "aimdk_msgs/msg/mc_action_status.hpp"
#include "aimdk_msgs/msg/request_header.hpp"
#include "aimdk_msgs/srv/get_current_input_source.hpp"
#include "aimdk_msgs/srv/set_mc_input_source.hpp"
#include "aimdk_msgs/srv/get_mc_action.hpp"
#include "aimdk_msgs/srv/set_mc_action.hpp"

#include "rclcpp/rclcpp.hpp"
#include <cmath>
#include <chrono>
#include <iostream>
#include <memory>
#include <signal.h>
#include <stdexcept>
#include <thread>
#include <unordered_map>
#include <vector>

constexpr double kServiceCallTimeoutSec = 2.0;
constexpr int kMaxRetryCount = 3;

class DirectVelocityControl : public rclcpp::Node {
public:
  DirectVelocityControl() : Node("direct_velocity_control") {
    // Create publisher
    publisher_ = this->create_publisher<aimdk_msgs::msg::McLocomotionVelocity>(
        "/aima/mc/locomotion/velocity", 10);
    // Create service clients
    set_client_ = this->create_client<aimdk_msgs::srv::SetMcInputSource>(
        "/aimdk_5Fmsgs/srv/SetMcInputSource");
    get_client_ = this->create_client<aimdk_msgs::srv::GetCurrentInputSource>(
        "/aimdk_5Fmsgs/srv/GetCurrentInputSource");
    set_action_client_ = this->create_client<aimdk_msgs::srv::SetMcAction>(
        "/aimdk_5Fmsgs/srv/SetMcAction");
    get_action_client_ = this->create_client<aimdk_msgs::srv::GetMcAction>(
        "/aimdk_5Fmsgs/srv/GetMcAction");

    // Minimum speed limits (0 is also OK)
    min_forward_speed_ = 0.2; // m/s
    min_lateral_speed_ = 0.2; // m/s
    min_angular_speed_ = 0.2; // rad/s

    RCLCPP_INFO(this->get_logger(), "Direct velocity control node started.");
  }

  void start_publish() {
    if (timer_ != nullptr) {
      return;
    }
    // Set timer to periodically publish velocity messages (50Hz)
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(20),
        std::bind(&DirectVelocityControl::publish_velocity, this));
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
      auto retcode = rclcpp::spin_until_future_complete(
          this->shared_from_this(), future, timeout);

      if (retcode == rclcpp::FutureReturnCode::SUCCESS) {
        return future.future.share();
      }

      RCLCPP_INFO(this->get_logger(),
                  "%s attempt %d/%d timed out, retrying...",
                  service_name.c_str(), i + 1, max_retries);
      std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }

    RCLCPP_ERROR(this->get_logger(),
                 "%s failed after %d attempts",
                 service_name.c_str(), max_retries);
    return typename rclcpp::Client<ServiceT>::SharedFuture();
  }

  bool register_input_source() {
    if (!wait_for_service(set_client_, "/aimdk_5Fmsgs/srv/SetMcInputSource")) {
      return false;
    }

    // 先清理残留的同名输入源（上次运行可能未正常释放）
    auto del_request =
        std::make_shared<aimdk_msgs::srv::SetMcInputSource::Request>();
    del_request->action.value = 1003;  // INPUTACTION_DELETE
    del_request->input_source.name = "node";
    del_request->request.header.stamp = this->now();
    auto del_future = call_service_with_retry<aimdk_msgs::srv::SetMcInputSource>(
        set_client_, del_request, "SetMcInputSource(DELETE)");
    if (del_future.valid()) {
      auto del_resp = del_future.get();
      if (del_resp->response.header.code == 0) {
        RCLCPP_INFO(this->get_logger(), "Cleaned up leftover input source 'node'");
      }
    }

    auto request =
        std::make_shared<aimdk_msgs::srv::SetMcInputSource::Request>();
    request->action.value = 1001;  // INPUTACTION_ADD
    request->input_source.name = "node";
    request->input_source.priority = 80;
    request->input_source.timeout = 1000;

    request->request.header.stamp = this->now();
    auto future = call_service_with_retry<aimdk_msgs::srv::SetMcInputSource>(
        set_client_, request, "SetMcInputSource(ADD)");
    
    if (!future.valid()) {
      RCLCPP_ERROR(this->get_logger(), "SetMcInputSource(ADD) failed after retries");
      return false;
    }

    auto response = future.get();
    int ret_code = response->response.header.code;
    if (ret_code != 0) {
      RCLCPP_WARN(this->get_logger(),
                  "SetMcInputSource(ADD) returned code=%d", ret_code);
      return false;
    }
    int state = response->response.state.value;
    RCLCPP_INFO(this->get_logger(),
                "Set input source succeeded: state=%d, task_id=%lu", state,
                response->response.task_id);
    return true;
  }

  bool get_current_input_source() {
    if (!wait_for_service(get_client_, "/aimdk_5Fmsgs/srv/GetCurrentInputSource")) {
      return false;
    }

    RCLCPP_INFO(this->get_logger(), "Querying current input source");

    auto request =
        std::make_shared<aimdk_msgs::srv::GetCurrentInputSource::Request>();
    request->request = aimdk_msgs::msg::CommonRequest();
    request->request.header.stamp = this->now();

    auto future = call_service_with_retry<aimdk_msgs::srv::GetCurrentInputSource>(
        get_client_, request, "GetCurrentInputSource");
    
    if (!future.valid()) {
      RCLCPP_WARN(this->get_logger(), "GetCurrentInputSource failed after retries");
      return false;
    }

    auto response = future.get();
    if (response->response.header.code == 0) {
      RCLCPP_INFO(this->get_logger(),
                  "Current input source: name=%s, priority=%d, timeout=%d",
                  response->input_source.name.c_str(),
                  response->input_source.priority,
                  response->input_source.timeout);
      return true;
    }

    RCLCPP_WARN(this->get_logger(), "GetCurrentInputSource returned code=%ld",
                response->response.header.code);
    return false;
  }

  void publish_velocity() {
    auto msg = std::make_unique<aimdk_msgs::msg::McLocomotionVelocity>();
    msg->header = aimdk_msgs::msg::MessageHeader();
    msg->header.stamp = this->now();
    msg->source = "node"; // Set message source
    msg->forward_velocity = forward_velocity_;
    msg->lateral_velocity = lateral_velocity_;
    msg->angular_velocity = angular_velocity_;
    msg->pitch_velocity = 0.0;
    msg->level = 0.0;

    publisher_->publish(std::move(msg));
  }

  void clear_velocity() {
    forward_velocity_ = 0.0;
    lateral_velocity_ = 0.0;
    angular_velocity_ = 0.0;
  }

  bool set_forward(double forward) {
    if (std::abs(forward) < 0.005) {
      forward_velocity_ = 0.0;
      return true;
    } else if (std::abs(forward) < min_forward_speed_) {
      throw std::out_of_range(
          "forward speed must be 0 or have an absolute value of at least 0.2 m/s");
    } else {
      forward_velocity_ = forward;
      return true;
    }
  }

  bool set_lateral(double lateral) {
    if (std::abs(lateral) < 0.005) {
      lateral_velocity_ = 0.0;
      return true;
    } else if (std::abs(lateral) < min_lateral_speed_) {
      throw std::out_of_range(
          "lateral speed must be 0 or have an absolute value of at least 0.2 m/s");
    } else {
      lateral_velocity_ = lateral;
      return true;
    }
  }

  bool set_angular(double angular) {
    if (std::abs(angular) < 0.005) {
      angular_velocity_ = 0.0;
      return true;
    } else if (std::abs(angular) < min_angular_speed_) {
      throw std::out_of_range(
          "angular speed must be 0 or have an absolute value of at least 0.2 rad/s");
    } else {
      angular_velocity_ = angular;
      return true;
    }
  }

  struct ActionInfo {
    std::string action_desc;
    int32_t status = aimdk_msgs::msg::McActionStatus::IDLE;
  };

  bool get_action_status(ActionInfo &info) {
    auto request = std::make_shared<aimdk_msgs::srv::GetMcAction::Request>();
    request->request.header.stamp = this->now();
    
    auto future = call_service_with_retry<aimdk_msgs::srv::GetMcAction>(
        get_action_client_, request, "GetMcAction");
    
    if (!future.valid()) {
      return false;
    }

    auto res = future.get();
    if (!res) return false;
    info.action_desc = res->info.action_desc;
    info.status = res->info.status.value;
    return true;
  }

  bool set_action(const std::string &desc) {
    auto request = std::make_shared<aimdk_msgs::srv::SetMcAction::Request>();
    request->header.stamp = this->now();
    request->source = "node";  // 触发源标识
    request->command.action_desc = desc;
    RCLCPP_INFO(this->get_logger(), "Requesting state switch to: %s",
                desc.c_str());
    
    auto future = call_service_with_retry<aimdk_msgs::srv::SetMcAction>(
        set_action_client_, request, "SetMcAction");
    
    if (!future.valid()) {
      return false;
    }

    auto res = future.get();
    if (res && res->response.status.value == aimdk_msgs::msg::CommonState::SUCCESS) {
      return true;
    }
    
    if (res) {
      RCLCPP_ERROR(this->get_logger(),
                   "SetMcAction failed. code=%ld status=%d msg=%s",
                   res->response.header.code,
                   res->response.status.value,
                   res->response.message.c_str());
    }
    
    return false;
  }

  bool wait_for_action(const std::string &target,
                       std::chrono::seconds timeout = std::chrono::seconds(10)) {
    auto deadline = std::chrono::steady_clock::now() + timeout;
    while (std::chrono::steady_clock::now() < deadline) {
      ActionInfo info;
      // Locomotion modes (walk/run) report status=2 (TRANSITION) instead of
      // RUNNING (100), so accept any non-IDLE status when action_desc matches.
      if (get_action_status(info) && info.action_desc == target &&
          info.status != aimdk_msgs::msg::McActionStatus::IDLE) {
        RCLCPP_INFO(this->get_logger(), "Robot reached state: %s (status=%d)",
                    target.c_str(), info.status);
        return true;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }
    return false;
  }

  bool ensure_ready_state() {
    ActionInfo info;
    bool has_info = get_action_status(info);

    // Safety margin: If initial query fails, poll for up to 5s to recover communication
    if (!has_info) {
      RCLCPP_WARN(this->get_logger(),
                  "Initial action status query failed. Retrying for up to 5 seconds...");
      auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
      while (std::chrono::steady_clock::now() < deadline) {
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        if (get_action_status(info)) {
          has_info = true;
          RCLCPP_INFO(this->get_logger(),
                      "Successfully recovered action status: %s",
                      info.action_desc.c_str());
          break;
        }
      }
      if (!has_info) {
        RCLCPP_ERROR(
            this->get_logger(),
            "Action status remained unavailable after 5 seconds of polling.");
      }
    }

    if (has_info && info.action_desc == "QUADRUPED_LOCOMOTION_DEFAULT" &&
        info.status != aimdk_msgs::msg::McActionStatus::IDLE) {
      return true;
    }

    RCLCPP_INFO(this->get_logger(),
                "Current state is %s. Starting state machine transition sequence...",
                info.action_desc.c_str());

    std::vector<std::string> sequence = {
      "PASSIVE_DEFAULT",
      "QUADRUPED_STAND_DEFAULT",
      "QUADRUPED_LOCOMOTION_DEFAULT"
    };

    size_t start_index = 0;
    if (info.action_desc == "PASSIVE_DEFAULT") {
        start_index = 1;
    } else if (info.action_desc == "QUADRUPED_STAND_DEFAULT") {
        start_index = 2;
    } else if (info.action_desc == "DAMPING_DEFAULT" ) {
        start_index = 0;
    } else {
        start_index = 0;
    }

    for (size_t i = start_index; i < sequence.size(); ++i) {
        if (!set_action(sequence[i]) || !wait_for_action(sequence[i])) {
            return false;
        }
    }

    return true;
  }

  bool release_input_source() {
    auto request =
        std::make_shared<aimdk_msgs::srv::SetMcInputSource::Request>();
    request->action.value = 1003;  // INPUTACTION_DELETE
    request->input_source.name = "node";
    request->request.header.stamp = this->now();

    auto future = call_service_with_retry<aimdk_msgs::srv::SetMcInputSource>(
        set_client_, request, "SetMcInputSource");

    if (!future.valid()) {
      RCLCPP_ERROR(this->get_logger(), "ReleaseInputSource failed after retries");
      return false;
    }

    auto response = future.get();
    int ret_code = response->response.header.code;
    if (ret_code == 0) {
      RCLCPP_INFO(this->get_logger(), "Input source released successfully.");
      return true;
    }

    RCLCPP_WARN(this->get_logger(),
                "SetMcInputSource returned code=%d", ret_code);
    return false;
  }

  void wait_for_services() {
    auto wait = [this](auto &client, const std::string &name) {
      while (!client->wait_for_service(std::chrono::seconds(2))) {
        if (!rclcpp::ok())
          return;
        RCLCPP_INFO(this->get_logger(), "Waiting for service %s...",
                    name.c_str());
      }
    };
    wait(set_client_, "/aimdk_5Fmsgs/srv/SetMcInputSource");
    wait(get_client_, "/aimdk_5Fmsgs/srv/GetCurrentInputSource");
    wait(set_action_client_, "/aimdk_5Fmsgs/srv/SetMcAction");
    wait(get_action_client_, "/aimdk_5Fmsgs/srv/GetMcAction");
  }

private:
  template <typename ClientT>
  bool wait_for_service(const std::shared_ptr<ClientT> &client,
                        const char *service_name) {
    while (!client->wait_for_service(std::chrono::seconds(2))) {
      if (!rclcpp::ok()) {
        return false;
      }
      RCLCPP_INFO(this->get_logger(), "Waiting for service: %s", service_name);
    }
    return true;
  }

  rclcpp::Publisher<aimdk_msgs::msg::McLocomotionVelocity>::SharedPtr
      publisher_;
  rclcpp::Client<aimdk_msgs::srv::SetMcInputSource>::SharedPtr set_client_;
  rclcpp::Client<aimdk_msgs::srv::GetCurrentInputSource>::SharedPtr get_client_;
  rclcpp::Client<aimdk_msgs::srv::SetMcAction>::SharedPtr set_action_client_;
  rclcpp::Client<aimdk_msgs::srv::GetMcAction>::SharedPtr get_action_client_;
  rclcpp::TimerBase::SharedPtr timer_;

  double forward_velocity_;
  double lateral_velocity_;
  double angular_velocity_;

  double min_forward_speed_;
  double min_lateral_speed_;
  double min_angular_speed_;
};

std::shared_ptr<DirectVelocityControl> g_node = nullptr;

void signal_handler(int signal) {
  if (g_node) {
    g_node->clear_velocity();
    RCLCPP_INFO(g_node->get_logger(),
                "Received signal %d, clearing velocity and shutting down...",
                signal);
    g_node.reset();
  }
  rclcpp::shutdown();
  exit(signal);
}

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  signal(SIGINT, signal_handler);
  signal(SIGTERM, signal_handler);

  g_node = std::make_shared<DirectVelocityControl>();
  auto node = g_node;

  node->wait_for_services();

  // Step 1: Ensure the robot is in BIPED_WALK_RUN state
  if (!node->ensure_ready_state()) {
    RCLCPP_ERROR(node->get_logger(), "Failed to prepare robot state for walking.");
    g_node.reset();
    rclcpp::shutdown();
    return 1;
  }

  // Step 2: Register input source
  if (!node->register_input_source()) {
    RCLCPP_ERROR(node->get_logger(),
                 "Input source registration failed, exiting");
    g_node.reset();
    rclcpp::shutdown();
    return 1;
  }

  try {
    // Get and check control values. MC enforces thresholds before movement.
    double forward, lateral, angular;
    std::cout << "Enter forward speed 0 or |v| >= 0.2 m/s: ";
    if (!(std::cin >> forward)) {
      throw std::invalid_argument("forward speed must be a number");
    }
    node->set_forward(forward);

    std::cout << "Enter lateral speed 0 or |v| >= 0.2 m/s: ";
    if (!(std::cin >> lateral)) {
      throw std::invalid_argument("lateral speed must be a number");
    }
    node->set_lateral(lateral);

    std::cout << "Enter angular speed 0 or |v| >= 0.2 rad/s: ";
    if (!(std::cin >> angular)) {
      throw std::invalid_argument("angular speed must be a number");
    }
    node->set_angular(angular);

    RCLCPP_INFO(node->get_logger(),
                "Start publishing velocity for 5 seconds: Forward %.2f m/s, "
                "Lateral %.2f m/s, Angular %.2f rad/s",
                forward, lateral, angular);

    node->start_publish();

    auto start_time = node->now();
    bool queried_after_publish = false;
    while ((node->now() - start_time).seconds() < 5.0) {
      if (!queried_after_publish &&
          (node->now() - start_time).seconds() > 1.0) {
        node->get_current_input_source();
        queried_after_publish = true;
      }
      rclcpp::spin_some(node);
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    node->clear_velocity();
    node->publish_velocity();
    RCLCPP_INFO(node->get_logger(), "5 seconds elapsed; robot stopped");
  } catch (const std::exception &error) {
    RCLCPP_ERROR(node->get_logger(), "Invalid velocity input: %s", error.what());
    node->clear_velocity();
    node->publish_velocity();
    node->release_input_source();
    g_node.reset();
    rclcpp::shutdown();
    return 2;
  }

  // Step 6: Release input source
  node->release_input_source();

  g_node.reset();
  rclcpp::shutdown();
  return 0;
}
