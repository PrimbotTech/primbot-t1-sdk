/*
 Camera Switch Control Example Script

 Description:
   This script demonstrates how to use the GetCameraSwitch and SetCameraSwitch
   services to query and control the camera stream on/off state.

   Workflow:
     1. Query current camera switch state
     2. Toggle the camera switch (enable if disabled, disable if enabled)
     3. Query again to confirm the change

 Prerequisites:
   - Robot camera service must be running
   - GetCameraSwitch and SetCameraSwitch services must be available

 Usage:
   ros2 run aimdk_examples_cpp get_camera_switch [--enable | --disable]

 Example:
   ros2 run aimdk_examples_cpp get_camera_switch              # Toggle current state
   ros2 run aimdk_examples_cpp get_camera_switch --enable     # Force enable
   ros2 run aimdk_examples_cpp get_camera_switch --disable    # Force disable

 Parameters:
   --enable    Force enable camera stream
   --disable   Force disable camera stream
 */

#include "aimdk_msgs/srv/get_camera_switch.hpp"
#include "aimdk_msgs/srv/set_camera_switch.hpp"
#include "rclcpp/rclcpp.hpp"

#include <cstring>

class CameraSwitchClient : public rclcpp::Node {
public:
  CameraSwitchClient() : Node("get_camera_switch") {
    get_client_ = this->create_client<aimdk_msgs::srv::GetCameraSwitch>(
        "/aimdk_5Fmsgs/srv/GetCameraSwitch");
    set_client_ = this->create_client<aimdk_msgs::srv::SetCameraSwitch>(
        "/aimdk_5Fmsgs/srv/SetCameraSwitch");

    RCLCPP_INFO(this->get_logger(),
                "CameraSwitch client created. Waiting for services...");

    if (!get_client_->wait_for_service(std::chrono::seconds(10))) {
      RCLCPP_ERROR(this->get_logger(),
                   "Service GetCameraSwitch not available after 10s.");
      throw std::runtime_error("GetCameraSwitch unavailable");
    }

    if (!set_client_->wait_for_service(std::chrono::seconds(10))) {
      RCLCPP_ERROR(this->get_logger(),
                   "Service SetCameraSwitch not available after 10s.");
      throw std::runtime_error("SetCameraSwitch unavailable");
    }

    RCLCPP_INFO(this->get_logger(), "All camera switch services are available.");
  }

  std::pair<bool, bool> get_state() {
    auto request = std::make_shared<aimdk_msgs::srv::GetCameraSwitch::Request>();

    auto future = get_client_->async_send_request(request);
    auto status = future.wait_for(std::chrono::seconds(5));

    if (status != std::future_status::ready) {
      RCLCPP_ERROR(this->get_logger(), "GetCameraSwitch timed out.");
      return {false, false};
    }

    auto response = future.get();
    if (!response) {
      RCLCPP_ERROR(this->get_logger(),
                   "GetCameraSwitch returned empty response.");
      return {false, false};
    }

    int64_t code = response->response.header.code;
    int32_t state = response->response.status.value;
    bool enabled = response->camera_enabled;

    RCLCPP_INFO(this->get_logger(),
                "GetCameraSwitch: code=%ld, state=%d, camera_enabled=%s",
                code, state, enabled ? "ON" : "OFF");

    return {true, enabled};
  }

  bool set_state(bool enable) {
    auto request = std::make_shared<aimdk_msgs::srv::SetCameraSwitch::Request>();
    request->camera_enabled = enable;

    RCLCPP_INFO(this->get_logger(),
                "Setting camera switch to %s...", enable ? "ON" : "OFF");

    auto future = set_client_->async_send_request(request);
    auto status = future.wait_for(std::chrono::seconds(5));

    if (status != std::future_status::ready) {
      RCLCPP_ERROR(this->get_logger(), "SetCameraSwitch timed out.");
      return false;
    }

    auto response = future.get();
    if (!response) {
      RCLCPP_ERROR(this->get_logger(),
                   "SetCameraSwitch returned empty response.");
      return false;
    }

    int64_t code = response->response.header.code;
    int32_t state = response->response.status.value;
    std::string message = response->response.message;
    bool enabled = response->camera_enabled;

    RCLCPP_INFO(this->get_logger(),
                "SetCameraSwitch: code=%ld, state=%d, message='%s', camera_enabled=%s",
                code, state, message.c_str(), enabled ? "ON" : "OFF");

    return code == 0;
  }

private:
  rclcpp::Client<aimdk_msgs::srv::GetCameraSwitch>::SharedPtr get_client_;
  rclcpp::Client<aimdk_msgs::srv::SetCameraSwitch>::SharedPtr set_client_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<CameraSwitchClient>();

  bool force_enable = false;
  bool force_disable = false;

  for (int i = 1; i < argc; ++i) {
    if (std::strcmp(argv[i], "--enable") == 0) {
      force_enable = true;
    } else if (std::strcmp(argv[i], "--disable") == 0) {
      force_disable = true;
    }
  }

  // Step 1: Query current state
  RCLCPP_INFO(node->get_logger(), "==================================================");
  RCLCPP_INFO(node->get_logger(), "Step 1: Query current camera switch state");
  auto [ok, current] = node->get_state();
  if (!ok) {
    rclcpp::shutdown();
    return 1;
  }

  // Step 2: Determine target state
  bool target;
  if (force_enable) {
    target = true;
  } else if (force_disable) {
    target = false;
  } else {
    // Toggle
    target = !current;
  }

  if (target == current) {
    RCLCPP_INFO(node->get_logger(),
                "Camera is already %s, no change needed.",
                current ? "ON" : "OFF");
    rclcpp::shutdown();
    return 0;
  }

  // Step 3: Set new state
  RCLCPP_INFO(node->get_logger(), "==================================================");
  RCLCPP_INFO(node->get_logger(), "Step 2: Set camera switch to %s",
              target ? "ON" : "OFF");
  if (!node->set_state(target)) {
    rclcpp::shutdown();
    return 1;
  }

  // Step 4: Verify
  RCLCPP_INFO(node->get_logger(), "==================================================");
  RCLCPP_INFO(node->get_logger(), "Step 3: Verify camera switch state");
  auto [ok2, confirmed] = node->get_state();
  if (!ok2) {
    rclcpp::shutdown();
    return 1;
  }

  if (confirmed == target) {
    RCLCPP_INFO(node->get_logger(), "Camera switch state changed successfully.");
  } else {
    RCLCPP_WARN(node->get_logger(),
                "State mismatch: expected %s, got %s",
                target ? "ON" : "OFF",
                confirmed ? "ON" : "OFF");
  }

  rclcpp::shutdown();
  return 0;
}
