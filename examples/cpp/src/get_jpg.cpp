/*
 @brief Example client for CaptureJpegImage service.
 
 The following ROS parameters can be set via startup arguments:
 --ros-args -p <name>:=<value>
 
 Supported parameters:
   - service_name: CaptureJpegImage service name
   - camera_id: Camera identifier (interactive prompt if not provided)
        - Available camera_id options:
          - head_stereo_left
          - head_stereo_right
          For details, please refer to the interface documentation.
   - timeout_ms: wait time for a fresh JPEG frame, in milliseconds. Use 0 to
     follow the server default.
   - output_file: local JPEG output path. Leave empty to auto-generate one.
 
 Interactive Mode:
   If camera_id is not provided via -p camera_id:=<value>,
   the program will prompt for input. Press Enter to use the default value.
 
 Examples:
   # Interactive mode (will prompt for camera_id)
   ros2 run aimdk_examples_cpp get_jpg --ros-args \
     -p output_file:=/tmp/camera_capture.jpg
 
 */
#include "aimdk_msgs/msg/common_request.hpp"
#include "aimdk_msgs/msg/common_state.hpp"
#include "aimdk_msgs/srv/capture_jpeg_image.hpp"
#include "aimdk_msgs/srv/set_servo.hpp"
#include "rclcpp/rclcpp.hpp"

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <ctime>
#include <exception>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <memory>
#include <signal.h>
#include <stdexcept>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

namespace {

constexpr char kDefaultServiceName[] =
    "/aima/hal/camera/CaptureJpegImage";
constexpr char kSetServoServiceName[] = "/aimdk_5Fmsgs/srv/SetServo";
constexpr char kDefaultOutputFile[] = "/tmp/camera_capture.jpg";
constexpr int kDefaultRequestTimeoutMs = 5000;
constexpr int kServiceWaitSeconds = 2;
constexpr int kMinCallTimeoutMs = 16000;
constexpr int kMaxRetryCount = 3;

std::shared_ptr<rclcpp::Node> g_node = nullptr;

std::string get_camera_id_from_user() {
  std::cout << "\n" << std::string(60, '=') << std::endl;
  std::cout << "The list of T1 Camera IDs is as follows:" << std::endl;
  std::cout << std::string(60, '=') << std::endl;
  std::cout << "  head_stereo_left       - 头部双目左机" << std::endl;
  std::cout << "  head_stereo_right      - 头部双目右机" << std::endl;
  std::cout << std::string(60, '=') << std::endl;
  std::cout << "For camera_id corresponding to different robot configurations, please refer to the interface documentation." << std::endl;
  std::cout << std::endl;
  
  std::cout << "\nPlease enter camera ID: ";
  std::string camera_id;
  std::getline(std::cin, camera_id);
  
  // Trim whitespace
  size_t start = camera_id.find_first_not_of(" \t\r\n");
  if (start == std::string::npos) {
    std::cout << "\nError: camera_id cannot be empty." << std::endl;
    std::cout << "Exiting...\n" << std::endl;
    std::exit(1);
  }
  size_t end = camera_id.find_last_not_of(" \t\r\n");
  camera_id = camera_id.substr(start, end - start + 1);
  
  std::cout << "\nUsing camera_id: " << camera_id << std::endl;
  std::cout << std::string(60, '=') << "\n" << std::endl;
  
  return camera_id;
}

std::filesystem::path prepare_output_path(const std::string &output_file) {
  std::filesystem::path output_path(
    output_file.empty() ? std::string(kDefaultOutputFile) : output_file);

  const auto extension = output_path.extension().string();
  if (extension != ".jpg" && extension != ".jpeg" && extension != ".JPG" &&
      extension != ".JPEG") {
    throw std::invalid_argument(
        "output_file must use the .jpg or .jpeg extension.");
  }

  const auto parent = output_path.parent_path();
  if (!parent.empty()) {
    std::error_code ec;
    std::filesystem::create_directories(parent, ec);
    if (ec) {
      throw std::runtime_error("Failed to create directory " +
                               parent.string() + ": " + ec.message());
    }
  }

  return output_path;
}

class CaptureJpegClient : public rclcpp::Node {
public:
  CaptureJpegClient(const std::string &camera_id) : Node("get_jpg") {
    service_name_ =
        this->declare_parameter<std::string>("service_name", kDefaultServiceName);
    timeout_ms_ =
        this->declare_parameter<int>("timeout_ms", kDefaultRequestTimeoutMs);
    output_file_ = this->declare_parameter<std::string>("output_file", "");
    camera_id_ = camera_id;  // Use the camera_id passed from user input

    if (service_name_.empty()) {
      throw std::invalid_argument("service_name must not be empty.");
    }

    if (timeout_ms_ < 0) {
      throw std::invalid_argument("timeout_ms must be greater than or equal to 0.");
    }

    output_path_ = prepare_output_path(output_file_);
    client_ = this->create_client<aimdk_msgs::srv::CaptureJpegImage>(
        service_name_);
    servo_client_ = this->create_client<aimdk_msgs::srv::SetServo>(
        kSetServoServiceName);

    RCLCPP_INFO(this->get_logger(),
                "CaptureJpegImage client created. service=%s camera_id=%s "
                "timeout_ms=%d output_file=%s",
                service_name_.c_str(), camera_id_.empty() ? "(default)" : camera_id_.c_str(),
                timeout_ms_,
                output_path_.string().c_str());
  }

  bool capture_once() {
    if (!wait_for_service()) {
      return false;
    }

    auto request =
        std::make_shared<aimdk_msgs::srv::CaptureJpegImage::Request>();
    request->request = aimdk_msgs::msg::CommonRequest();
    request->request.header.stamp = this->now();
    request->camera_id = camera_id_;
    request->timeout_ms = static_cast<std::uint32_t>(timeout_ms_);

    RCLCPP_INFO(this->get_logger(),
                "Sending CaptureJpegImage request: camera_id=%s timeout_ms=%u",
                camera_id_.empty() ? "(default)" : camera_id_.c_str(),
                request->timeout_ms);

    // Retry mechanism: up to 3 attempts
    auto future = client_->async_send_request(request);
    const auto call_timeout = std::chrono::milliseconds(
        std::max(kMinCallTimeoutMs, timeout_ms_ + 1000));
    
    bool completed = false;
    for (int i = 0; i < kMaxRetryCount; ++i) {
      const auto retcode = rclcpp::spin_until_future_complete(
          shared_from_this(), future, call_timeout);

      if (retcode == rclcpp::FutureReturnCode::SUCCESS) {
        completed = true;
        break;
      }

      RCLCPP_INFO(this->get_logger(),
                  "CaptureJpegImage attempt %d/%d timed out, retrying...",
                  i + 1, kMaxRetryCount);
      std::this_thread::sleep_for(std::chrono::milliseconds(200));
      
      // Re-send request for retry
      future = client_->async_send_request(request);
    }

    if (!completed) {
      RCLCPP_ERROR(this->get_logger(),
                   "CaptureJpegImage timed out after %lld ms.",
                   static_cast<long long>(call_timeout.count()));
      return false;
    }

    const auto response = future.get();
    if (!response) {
      RCLCPP_ERROR(this->get_logger(),
                   "CaptureJpegImage returned an empty response.");
      return false;
    }

    return save_response(*response);
  }

  void request_shutdown(int signal) {
    if (stop_requested_) {
      return;
    }

    stop_requested_ = true;
    RCLCPP_INFO(this->get_logger(),
                "Received signal %d, shutting down get_jpg...", signal);
    rclcpp::shutdown();
  }

  bool set_servo(int position, int speed) {
    if (position < 0 || position > 90) {
      RCLCPP_ERROR(this->get_logger(),
                   "Invalid position: %d. Must be 0-90 degrees.", position);
      return false;
    }

    if (speed < 100 || speed > 1000) {
      RCLCPP_ERROR(this->get_logger(),
                   "Invalid speed: %d. Must be 100-1000.", speed);
      return false;
    }

    while (!stop_requested_ &&
           !servo_client_->wait_for_service(std::chrono::seconds(kServiceWaitSeconds))) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(this->get_logger(), "Shutdown while waiting for SetServo service.");
        return false;
      }
      RCLCPP_INFO(this->get_logger(), "Waiting for service: %s", kSetServoServiceName);
    }

    if (stop_requested_ || !rclcpp::ok()) {
      RCLCPP_ERROR(this->get_logger(), "SetServo service not available.");
      return false;
    }

    auto request = std::make_shared<aimdk_msgs::srv::SetServo::Request>();
    request->request = aimdk_msgs::msg::CommonRequest();
    request->request.header.stamp = this->now();
    request->position = position;
    request->speed = speed;

    auto future = servo_client_->async_send_request(request);
      const auto retcode = rclcpp::spin_until_future_complete(
          shared_from_this(), future, std::chrono::seconds(5));

    if (retcode != rclcpp::FutureReturnCode::SUCCESS) {
      RCLCPP_ERROR(this->get_logger(), "SetServo call timed out.");
      return false;
    }

    const auto response = future.get();
    if (!response) {
      RCLCPP_ERROR(this->get_logger(), "SetServo returned empty response.");
      return false;
    }

    const auto code = response->header.header.code;
    if (code != 0) {
      RCLCPP_ERROR(this->get_logger(), "SetServo failed: code=%ld", code);
      return false;
    }

    RCLCPP_INFO(this->get_logger(), "SetServo success. position=%d speed=%d",
                position, speed);
    return true;
  }

private:
  bool wait_for_service() {
    while (!stop_requested_ &&
           !client_->wait_for_service(std::chrono::seconds(kServiceWaitSeconds))) {
      if (!rclcpp::ok()) {
        return false;
      }
      RCLCPP_INFO(this->get_logger(), "Waiting for service: %s",
                  service_name_.c_str());
    }

    if (stop_requested_ || !rclcpp::ok()) {
      return false;
    }

    RCLCPP_INFO(this->get_logger(), "Service available: %s",
                service_name_.c_str());
    return true;
  }

  bool save_response(const aimdk_msgs::srv::CaptureJpegImage::Response &response) {
    const auto code = response.response.header.code;
    const auto status = response.response.status.value;
    
    if (code != 0 && status != aimdk_msgs::msg::CommonState::SUCCESS) {
      RCLCPP_ERROR(this->get_logger(),
                   "CaptureJpegImage failed. code=%ld status=%d msg=%s", code,
                   status, response.response.message.c_str());
      return false;
    }

    const auto &jpeg = response.jpeg;
    const auto &image = jpeg.image;
    if (image.data.empty()) {
      RCLCPP_ERROR(this->get_logger(),
                   "CaptureJpegImage returned empty JPEG data.");
      return false;
    }

    std::ofstream output_stream(
        output_path_, std::ios::binary | std::ios::out | std::ios::trunc);
    if (!output_stream.is_open()) {
      RCLCPP_ERROR(this->get_logger(), "Failed to open output file: %s",
                   output_path_.string().c_str());
      return false;
    }

    output_stream.write(reinterpret_cast<const char *>(image.data.data()),
                        static_cast<std::streamsize>(image.data.size()));
    if (!output_stream) {
      RCLCPP_ERROR(this->get_logger(), "Failed to write JPEG file: %s",
                   output_path_.string().c_str());
      return false;
    }
    output_stream.close();

    RCLCPP_INFO(
        this->get_logger(),
        "JPEG saved: file=%s bytes=%zu format=%s camera_id=%s device=%s "
        "width=%u height=%u framerate=%u exposure=%u gain=%u frame_id=%s",
        output_path_.string().c_str(), image.data.size(), image.format.c_str(),
        jpeg.camera_id.c_str(), jpeg.device.c_str(), jpeg.width, jpeg.height,
        jpeg.framerate, jpeg.exposure, jpeg.gain, image.header.frame_id.c_str());
    return true;
  }

  std::string service_name_;
  std::string camera_id_;
  int timeout_ms_{kDefaultRequestTimeoutMs};
  std::string output_file_;
  std::filesystem::path output_path_;
  bool stop_requested_{false};
  rclcpp::Client<aimdk_msgs::srv::CaptureJpegImage>::SharedPtr client_;
  rclcpp::Client<aimdk_msgs::srv::SetServo>::SharedPtr servo_client_;
};

void signal_handler(int signal) {
  if (!rclcpp::ok()) {
    return;
  }

  if (g_node) {
    if (auto node = std::dynamic_pointer_cast<CaptureJpegClient>(g_node)) {
      node->request_shutdown(signal);
    } else {
      RCLCPP_INFO(g_node->get_logger(),
                  "Received signal %d, shutting down get_jpg...", signal);
      rclcpp::shutdown();
    }
    g_node.reset();
    return;
  }

  rclcpp::shutdown();
}

}  // namespace

std::string get_int_from_user(const std::string &prompt, int min_val, int max_val, int default_val = -1) {
  std::string input;
  while (true) {
    std::cout << prompt;
    std::getline(std::cin, input);
    
    if (input.empty() && default_val != -1) {
      return std::to_string(default_val);
    }
    
    try {
      int value = std::stoi(input);
      if (value >= min_val && value <= max_val) {
        return std::to_string(value);
      }
      std::cout << "Error: Value must be between " << min_val << " and " << max_val << "." << std::endl;
    } catch (const std::exception &) {
      std::cout << "Error: Invalid input. Please enter a number." << std::endl;
    }
  }
}

int main(int argc, char *argv[]) {
  try {
    rclcpp::init(argc, argv);
    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);

    std::cout << "\n" << std::string(60, '=') << std::endl;
    std::cout << "  Camera & Servo Control Menu" << std::endl;
    std::cout << std::string(60, '=') << std::endl;
    std::cout << "\nSelect control mode:" << std::endl;
    std::cout << "  1. Capture JPEG Image" << std::endl;
    std::cout << "  2. Set Servo Position" << std::endl;
    std::cout << "\nEnter your choice (1 or 2): ";
    
    std::string choice;
    std::getline(std::cin, choice);
    
    if (choice == "1") {
      // Camera capture mode
      std::string camera_id = get_camera_id_from_user();
      auto node = std::make_shared<CaptureJpegClient>(camera_id);
      g_node = node;
      const bool ok = node->capture_once();

      g_node.reset();
      rclcpp::shutdown();
      return ok ? 0 : 1;
    } else if (choice == "2") {
      // Servo control mode
      
      auto node = std::make_shared<CaptureJpegClient>("");
      g_node = node;

      int position = std::stoi(get_int_from_user("\nEnter servo position (0-90 degrees): ", 0, 90));
      int speed = std::stoi(get_int_from_user("Enter servo speed (100-1000): ", 100, 1000));
      
      const bool ok = node->set_servo(position, speed);

      g_node.reset();
      rclcpp::shutdown();
      return ok ? 0 : 1;
    } else {
      std::cout << "Invalid choice. Exiting..." << std::endl;
      rclcpp::shutdown();
      return 1;
    }
  } catch (const std::exception &e) {
    RCLCPP_ERROR(rclcpp::get_logger("get_jpg"),
                 "Program exited with exception: %s", e.what());
    return 1;
  }
}
