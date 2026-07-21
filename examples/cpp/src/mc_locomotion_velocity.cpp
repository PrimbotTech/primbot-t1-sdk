/*
 @brief Example client for /aima/mc/locomotion/velocity.
 
 This script automatically handles the required state machine transitions for safety.
 Locomotion control (walking/running) requires the robot to be in
 QUADRUPED_LOCOMOTION_DEFAULT mode.
 
 Prerequisites auto-handled by this script:
   The shared MC action switcher handles the initial state.
 
 Flow:
   1. Detect current state and transition to QUADRUPED_LOCOMOTION_DEFAULT.
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
#include "aimdk_msgs/msg/mc_input_action.hpp"
#include "aimdk_msgs/msg/message_header.hpp"
#include "aimdk_msgs/srv/get_current_input_source.hpp"
#include "aimdk_msgs/srv/set_mc_input_source.hpp"
#include "mc_action_switcher.hpp"

#include "rclcpp/rclcpp.hpp"
#include <cmath>
#include <chrono>
#include <iostream>
#include <memory>
#include <signal.h>
#include <stdexcept>
#include <thread>

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

  bool ensure_ready_state() {
    aimdk_examples::McActionSwitcher switcher(this->shared_from_this());
    aimdk_examples::McActionSwitchOptions options;
    options.source = "locomotion_velocity_node";
    options.total_timeout = std::chrono::seconds(30);

    const auto result =
        switcher.switch_to("QUADRUPED_LOCOMOTION_DEFAULT", options);
    if (!result.success) {
      RCLCPP_ERROR(
          this->get_logger(),
          "Failed to switch from %s to QUADRUPED_LOCOMOTION_DEFAULT: %s",
          result.current_action.empty() ? "(unknown)" : result.current_action.c_str(),
          result.message.c_str());
    }
    return result.success;
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
