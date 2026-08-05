/*
 HAL Camera Command Example Script

 Description:
   This script demonstrates how to use the HALCameraCommand service to send
   generic camera commands via a KV parameter interface.

   Supported operations:
     - Enable/disable camera stream publishing
     - Set camera frame rate (FPS)
     - Query current camera frame rate

 Prerequisites:
   - Robot camera service must be running
   - HALCameraCommand service must be available

 Usage:
   ros2 run aimdk_examples_cpp hal_camera_command <command> [options]

 Examples:
   ros2 run aimdk_examples_cpp hal_camera_command enable --topic /aima/hal/camera/head_stereo_left/nv12
   ros2 run aimdk_examples_cpp hal_camera_command disable --topic /aima/hal/camera/head_stereo_left/nv12
   ros2 run aimdk_examples_cpp hal_camera_command set-fps --topic /aima/hal/camera/head_stereo_left/nv12 --fps 30
   ros2 run aimdk_examples_cpp hal_camera_command get-fps --topic /aima/hal/camera/head_stereo_left/nv12

 Parameters:
   command    One of: enable, disable, set-fps, get-fps
   --topic    Camera topic path (required)
   --fps      Target frame rate (required for set-fps)
 */

#include "aimdk_msgs/srv/hal_camera_command.hpp"
#include "rclcpp/rclcpp.hpp"

#include <cstring>
#include <string>
#include <vector>

class HALCameraCommandClient : public rclcpp::Node {
public:
  HALCameraCommandClient() : Node("hal_camera_command") {
    client_ = this->create_client<aimdk_msgs::srv::HALCameraCommand>(
        "/aimdk_5Fmsgs/srv/HALCameraCommand");

    RCLCPP_INFO(this->get_logger(),
                "HALCameraCommand client created. Waiting for service: %s",
                "/aimdk_5Fmsgs/srv/HALCameraCommand");

    if (!client_->wait_for_service(std::chrono::seconds(10))) {
      RCLCPP_ERROR(this->get_logger(),
                   "Service not available after 10s. Is the camera service running?");
      throw std::runtime_error("HALCameraCommand unavailable");
    }

    RCLCPP_INFO(this->get_logger(), "Service is available.");
  }

  bool send_command(const std::string& command,
                    const std::vector<std::string>& keys,
                    const std::vector<std::string>& values) {
    auto request = std::make_shared<aimdk_msgs::srv::HALCameraCommand::Request>();
    request->command = command;
    request->param_keys = keys;
    request->param_values = values;

    std::string keys_str = "[";
    std::string values_str = "[";
    for (size_t i = 0; i < keys.size(); ++i) {
      if (i > 0) {
        keys_str += ", ";
        values_str += ", ";
      }
      keys_str += keys[i];
      values_str += values[i];
    }
    keys_str += "]";
    values_str += "]";

    RCLCPP_INFO(this->get_logger(),
                "Sending command: '%s', keys=%s, values=%s",
                command.c_str(), keys_str.c_str(), values_str.c_str());

    auto future = client_->async_send_request(request);
    auto status = future.wait_for(std::chrono::seconds(5));

    if (status != std::future_status::ready) {
      RCLCPP_ERROR(this->get_logger(),
                   "HALCameraCommand timed out after 5000 ms.");
      return false;
    }

    auto response = future.get();
    if (!response) {
      RCLCPP_ERROR(this->get_logger(),
                   "HALCameraCommand returned an empty response.");
      return false;
    }

    int64_t code = response->response.header.code;
    int32_t state = response->response.status.value;
    std::string message = response->response.message;

    RCLCPP_INFO(this->get_logger(),
                "Response: code=%ld, state=%d, message='%s'",
                code, state, message.c_str());

    // Print optional return values
    if (!response->result_keys.empty() && !response->result_values.empty()) {
      RCLCPP_INFO(this->get_logger(), "Result:");
      for (size_t i = 0; i < response->result_keys.size() && i < response->result_values.size(); ++i) {
        RCLCPP_INFO(this->get_logger(), "  %s: %s",
                    response->result_keys[i].c_str(),
                    response->result_values[i].c_str());
      }
    }

    return code == 0;
  }

private:
  rclcpp::Client<aimdk_msgs::srv::HALCameraCommand>::SharedPtr client_;
};

void print_usage() {
  printf("Usage: hal_camera_command <command> [options]\n\n");
  printf("Commands:\n");
  printf("  enable --topic <topic>\n");
  printf("  disable --topic <topic>\n");
  printf("  set-fps --topic <topic> --fps <fps>\n");
  printf("  get-fps --topic <topic>\n\n");
  printf("Examples:\n");
  printf("  hal_camera_command enable --topic /aima/hal/camera/head_stereo_left/nv12\n");
  printf("  hal_camera_command set-fps --topic /aima/hal/camera/head_stereo_left/nv12 --fps 30\n");
}

int main(int argc, char **argv) {
  if (argc < 2) {
    print_usage();
    return 1;
  }

  rclcpp::init(argc, argv);
  auto node = std::make_shared<HALCameraCommandClient>();

  std::string command = argv[1];
  std::string topic;
  std::string fps;

  for (int i = 2; i < argc; ++i) {
    if (std::strcmp(argv[i], "--topic") == 0 && i + 1 < argc) {
      topic = argv[++i];
    } else if (std::strcmp(argv[i], "--fps") == 0 && i + 1 < argc) {
      fps = argv[++i];
    }
  }

  if (topic.empty()) {
    RCLCPP_ERROR(node->get_logger(), "Missing required --topic parameter");
    print_usage();
    rclcpp::shutdown();
    return 1;
  }

  bool ok = false;
  if (command == "enable") {
    ok = node->send_command("stream.enable", {"topic"}, {topic});
  } else if (command == "disable") {
    ok = node->send_command("stream.disable", {"topic"}, {topic});
  } else if (command == "set-fps") {
    if (fps.empty()) {
      RCLCPP_ERROR(node->get_logger(), "Missing required --fps parameter for set-fps");
      print_usage();
      rclcpp::shutdown();
      return 1;
    }
    ok = node->send_command("set.fps", {"topic", "fps"}, {topic, fps});
  } else if (command == "get-fps") {
    ok = node->send_command("get.fps", {"topic"}, {topic});
  } else {
    RCLCPP_ERROR(node->get_logger(), "Unknown command: %s", command.c_str());
    print_usage();
    rclcpp::shutdown();
    return 1;
  }

  rclcpp::shutdown();
  return ok ? 0 : 1;
}
