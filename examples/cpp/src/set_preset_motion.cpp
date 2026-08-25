/*
 @brief Preset Motion Example Script
 
 Description:
   This script demonstrates how to call the SetMcPresetMotion service to play preset
   motions (like waving or handshaking) on the robot. Supports T and Q series robots
   with interactive series and motion selection.
 
 Prerequisites:
   - Robot motion control service must be running
   - McActionSwitcher will transition to BIPED_LOCOMOTION_WBC before the motion
   - SetMcPresetMotion service must be available
 
 Usage:
   ros2 run aimdk_examples_cpp set_preset_motion
 
 Example:
   # Interactive mode (will prompt for series and motion ID)
   ros2 run aimdk_examples_cpp set_preset_motion
 
 Supported Motions:
   T series: 1001=raise, 1002=wave, 1003=handshake, 2001=handheart
   Q series: 3001=wave, 3002=handshake, 3003=bump, 3004=wave_hand
 */
#include "aimdk_msgs/msg/mc_preset_motion.hpp"
#include "aimdk_msgs/srv/set_mc_preset_motion.hpp"
#include "mc_action_switcher.hpp"
#include "rclcpp/rclcpp.hpp"

#include <algorithm>
#include <cctype>
#include <chrono>
#include <iostream>
#include <map>
#include <memory>
#include <signal.h>
#include <string>
#include <thread>

constexpr double kServiceCallTimeoutSec = 2.0;
constexpr int kMaxRetryCount = 3;

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
    preset_client_ = this->create_client<aimdk_msgs::srv::SetMcPresetMotion>(
        "/aimdk_5Fmsgs/srv/SetMcPresetMotion");

    RCLCPP_INFO(this->get_logger(), "SetMcPresetMotion client node created.");
    wait_for_services();
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
      auto result = client->async_send_request(request);
      if (rclcpp::spin_until_future_complete(this->shared_from_this(), result, timeout) ==
          rclcpp::FutureReturnCode::SUCCESS) {
        return result.future.share();
      }
      RCLCPP_INFO(this->get_logger(), "%s attempt %d/%d timed out, retrying...",
                  service_name.c_str(), i + 1, max_retries);
      std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }
    RCLCPP_ERROR(this->get_logger(), "%s failed after %d attempts", service_name.c_str(), max_retries);
    return typename rclcpp::Client<ServiceT>::SharedFuture();
  }

  bool send_request(int motion_id) {
    if (!ensure_ready_state()) {
      RCLCPP_ERROR(this->get_logger(), "Failed to prepare robot state for preset motion.");
      return false;
    }

    try {
      auto request = std::make_shared<aimdk_msgs::srv::SetMcPresetMotion::Request>();
      request->header.stamp = this->now();
      request->source = "sdk_rpc";
      request->motion.value = motion_id;
      request->interrupt = true;

      RCLCPP_INFO(this->get_logger(), "Sending preset motion request: ID=%d", motion_id);

      auto future = call_service_with_retry<aimdk_msgs::srv::SetMcPresetMotion>(
          preset_client_, request, "SetMcPresetMotion");

      if (!future.valid()) {
        RCLCPP_ERROR(this->get_logger(), "Service call failed or timed out.");
        return false;
      }

       auto response = future.get();
       if (response && response->response.header.code == 0) {
         RCLCPP_INFO(this->get_logger(), "Motion request accepted. Task ID: %lu",
                     response->response.task_id);
         return true;
       }
       
       if (response) {
         RCLCPP_ERROR(this->get_logger(),
                      "SetMcPresetMotion failed. code=%ld status=%d reason=%u",
                      response->response.header.code,
                      response->response.state.value,
                      response->response.state.reason);
       }
       
       return false;
    } catch (const std::exception &e) {
      RCLCPP_ERROR(this->get_logger(), "Exception occurred: %s", e.what());
      return false;
    }
  }

  bool send_custom_request(const std::string & ani_path) {
    if (!ensure_ready_state()) {
      RCLCPP_ERROR(this->get_logger(), "Failed to prepare robot state for custom motion.");
      return false;
    }

    try {
      auto request = std::make_shared<aimdk_msgs::srv::SetMcPresetMotion::Request>();
      request->header.stamp = this->now();
      request->source = "sdk_rpc";
      request->motion.value = 0;  // not using built-in enum
      request->interrupt = true;
      request->ani_path = ani_path;  // custom motion file path

      RCLCPP_INFO(this->get_logger(), "Sending custom motion request: path=%s", ani_path.c_str());

      auto future = call_service_with_retry<aimdk_msgs::srv::SetMcPresetMotion>(
          preset_client_, request, "SetMcPresetMotion");

      if (!future.valid()) {
        RCLCPP_ERROR(this->get_logger(), "Service call failed or timed out.");
        return false;
      }

      auto response = future.get();
      if (response && response->response.header.code == 0) {
        RCLCPP_INFO(this->get_logger(), "Custom motion request accepted. Task ID: %lu",
                    response->response.task_id);
        return true;
      }

      if (response) {
        RCLCPP_ERROR(this->get_logger(),
                     "SetMcPresetMotion (custom) failed. code=%ld status=%d reason=%u",
                     response->response.header.code,
                     response->response.state.value,
                     response->response.state.reason);
      }

      return false;
    } catch (const std::exception &e) {
      RCLCPP_ERROR(this->get_logger(), "Exception occurred: %s", e.what());
      return false;
    }
  }

private:
  void wait_for_services() {
    auto wait = [this](auto &client, const std::string &name) {
      while (!client->wait_for_service(std::chrono::seconds(2))) {
        if (!rclcpp::ok())
          return;
        RCLCPP_INFO(this->get_logger(), "Waiting for service %s...",
                    name.c_str());
      }
    };
    wait(preset_client_, "/aimdk_5Fmsgs/srv/SetMcPresetMotion");
  }

  bool ensure_ready_state() {
    aimdk_examples::McActionSwitcher switcher(this->shared_from_this());
    aimdk_examples::McActionSwitchOptions options;
    options.source = "sdk_rpc";
    options.total_timeout = std::chrono::seconds(30);

    const auto result =
        switcher.switch_to("BIPED_LOCOMOTION_WBC", options);
    if (!result.success) {
      RCLCPP_ERROR(
          this->get_logger(),
          "Failed to switch from %s to BIPED_LOCOMOTION_WBC: %s",
          result.current_action.empty() ? "(unknown)" : result.current_action.c_str(),
          result.message.c_str());
    }
    return result.success;
  }

  rclcpp::Client<aimdk_msgs::srv::SetMcPresetMotion>::SharedPtr preset_client_;
};

int main(int argc, char *argv[]) {
  try {
    rclcpp::init(argc, argv);
    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);

    g_node = std::make_shared<PresetMotionClient>();
    auto client = std::dynamic_pointer_cast<PresetMotionClient>(g_node);

    // Choose between built-in preset and custom motion file
    std::cout << "\nOptions:" << std::endl;
    std::cout << "  [1] Play built-in preset motion" << std::endl;
    std::cout << "  [2] Play custom motion file (from teaching)" << std::endl;
    std::cout << "Enter choice (1/2): ";
    std::string choice;
    std::getline(std::cin, choice);
    choice.erase(std::remove_if(choice.begin(), choice.end(), ::isspace), choice.end());

    if (choice == "2") {
        std::cout << "\nEnter custom motion file path "
                  << "(e.g. /robot/userdata/sd/custom_motions/my_motion.csv): ";
        std::string ani_path;
        std::getline(std::cin, ani_path);
        if (ani_path.empty()) {
            std::cerr << "No path provided." << std::endl;
            g_node.reset();
            rclcpp::shutdown();
            return 1;
        }
        if (client) {
            client->send_custom_request(ani_path);
        }
        g_node.reset();
        rclcpp::shutdown();
        return 0;
    }

    // Built-in preset motion flow
    std::cout << "\nPlease refer to the interface documentation for the list "
                 "of supported motions for this model.\n"
              << "If you haven't found the motion list, you can choose "
                 "recommended motions based on the robot model."
              << std::endl;

    std::cout << "\nEnter robot series (Q/T): ";
    std::string robot_series;
    std::getline(std::cin, robot_series);
    robot_series.erase(std::remove_if(robot_series.begin(), robot_series.end(), ::isspace), robot_series.end());
    std::transform(robot_series.begin(), robot_series.end(), robot_series.begin(), ::toupper);

    std::map<int, std::string> motion_map;
    if (robot_series == "T") {
      motion_map = {{1001, "raise"}, {1002, "wave"}, {1003, "handshake"}, {2001, "handheart"}};
    } else if (robot_series == "Q") {
      motion_map = {{3001, "wave"}, {3002, "handshake"}, {3003, "bump"}, {3004, "wave_hand"}};
    } else {
      std::cerr << "Unknown series. Please enter 'Q' or 'T'." << std::endl;
      return 1;
    }

    std::cout << "\nAvailable Preset Motions:" << std::endl;
    for (const auto &kv : motion_map) {
      std::cout << "  " << kv.first << ": " << kv.second << std::endl;
    }
    std::cout << "\nEnter preset motion ID: ";
    int motion_id = 0;
    if (!(std::cin >> motion_id)) return 0;
    if (motion_map.find(motion_id) == motion_map.end()) {
      std::cerr << "Invalid motion ID selected." << std::endl;
      return 1;
    }

    if (client) client->send_request(motion_id);
    g_node.reset();
    rclcpp::shutdown();
    return 0;
  } catch (const std::exception &e) {
    RCLCPP_ERROR(rclcpp::get_logger("main"), "Exited with exception: %s", e.what());
    return 1;
  }
}
