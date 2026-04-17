/**
 * @brief Example client for /aimdk_5Fmsgs/srv/SetMcPresetMotion.
 *
 * This script automatically handles the required state machine transitions for safety.
 * Playing preset motions (like waving or handshaking) requires the robot to be in
 * BIPED_WHOLE_BODY_CTRL mode.
 *
 * Prerequisites auto-handled by this script:
 *   The script ensures a safe sequential transition path:
 *   PASSIVE_DEFAULT -> BIPED_STAND_DEFAULT -> BIPED_WALK_RUN -> BIPED_WHOLE_BODY_CTRL.
 *   Depending on the initial state, it enters the sequence at the appropriate step.
 *   Finally, execute the requested preset motion.
 */
#include "aimdk_msgs/msg/common_request.hpp"
#include "aimdk_msgs/msg/common_response.hpp"
#include "aimdk_msgs/msg/common_state.hpp"
#include "aimdk_msgs/msg/common_task_response.hpp"
#include "aimdk_msgs/msg/mc_action_status.hpp"
#include "aimdk_msgs/msg/mc_preset_motion.hpp"
#include "aimdk_msgs/msg/request_header.hpp"
#include "aimdk_msgs/srv/get_mc_action.hpp"
#include "aimdk_msgs/srv/set_mc_action.hpp"
#include "aimdk_msgs/srv/set_mc_preset_motion.hpp"
#include <string>
#include <iostream>
#include <map>
#include <vector>
#include <algorithm>
#include <cctype>
#include "rclcpp/rclcpp.hpp"

#include <chrono>
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

class PresetMotionClient : public rclcpp::Node {
public:
  PresetMotionClient() : Node("preset_motion_client") {
    preset_client_ = this->create_client<aimdk_msgs::srv::SetMcPresetMotion>(
        "/aimdk_5Fmsgs/srv/SetMcPresetMotion");
    set_action_client_ = this->create_client<aimdk_msgs::srv::SetMcAction>(
        "/aimdk_5Fmsgs/srv/SetMcAction");
    get_action_client_ = this->create_client<aimdk_msgs::srv::GetMcAction>(
        "/aimdk_5Fmsgs/srv/GetMcAction");

    RCLCPP_INFO(this->get_logger(), "SetMcPresetMotion client node created.");
    wait_for_services();
  }

  bool send_request(int motion_id) {
    if (!ensure_ready_state()) {
      RCLCPP_ERROR(this->get_logger(),
                   "Failed to prepare robot state for preset motion.");
      return false;
    }

    try {
      auto request =
          std::make_shared<aimdk_msgs::srv::SetMcPresetMotion::Request>();
      request->header.stamp = this->now();
      request->motion.value = motion_id;
      request->interrupt = true;

      RCLCPP_INFO(this->get_logger(),
                  "Sending preset motion request: ID=%d", motion_id);

      auto future = preset_client_->async_send_request(request);
      auto retcode = rclcpp::spin_until_future_complete(
          shared_from_this(), future, std::chrono::seconds(2));

      if (retcode != rclcpp::FutureReturnCode::SUCCESS) {
        RCLCPP_ERROR(this->get_logger(), "Service call failed or timed out.");
        return false;
      }

      auto response = future.get();
      if (response && response->response.header.code == 0) {
        RCLCPP_INFO(this->get_logger(), "Motion request accepted. Task ID: %lu",
                    response->response.task_id);
        return true;
      }
      return false;
    } catch (const std::exception &e) {
      RCLCPP_ERROR(this->get_logger(), "Exception occurred: %s", e.what());
      return false;
    }
  }

private:
  struct ActionInfo {
    std::string action_desc;
    int32_t status = aimdk_msgs::msg::McActionStatus::IDLE;
  };

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
    wait(set_action_client_, "/aimdk_5Fmsgs/srv/SetMcAction");
    wait(get_action_client_, "/aimdk_5Fmsgs/srv/GetMcAction");
  }

  bool get_action_status(ActionInfo &info) {
    auto request = std::make_shared<aimdk_msgs::srv::GetMcAction::Request>();
    request->request.header.stamp = this->now();
    auto future = get_action_client_->async_send_request(request);
    if (rclcpp::spin_until_future_complete(shared_from_this(), future,
                                           std::chrono::seconds(2)) !=
        rclcpp::FutureReturnCode::SUCCESS)
      return false;

    auto res = future.get();
    info.action_desc = res->info.action_desc;
    info.status = res->info.status.value;
    return true;
  }

  bool set_action(const std::string &desc) {
    auto request = std::make_shared<aimdk_msgs::srv::SetMcAction::Request>();
    request->header.stamp = this->now();
    request->command.action_desc = desc;
    RCLCPP_INFO(this->get_logger(), "Requesting state switch to: %s",
                desc.c_str());
    auto future = set_action_client_->async_send_request(request);
    if (rclcpp::spin_until_future_complete(shared_from_this(), future,
                                           std::chrono::seconds(2)) !=
        rclcpp::FutureReturnCode::SUCCESS)
      return false;

    auto res = future.get();
    return res && res->response.status.value ==
                      aimdk_msgs::msg::CommonState::SUCCESS;
  }

  bool wait_for_action(const std::string &target,
                       std::chrono::seconds timeout = std::chrono::seconds(10)) {
    auto deadline = std::chrono::steady_clock::now() + timeout;
    while (std::chrono::steady_clock::now() < deadline) {
      ActionInfo info;
      if (get_action_status(info) && info.action_desc == target &&
          info.status == aimdk_msgs::msg::McActionStatus::RUNNING) {
        RCLCPP_INFO(this->get_logger(), "Robot reached state: %s",
                    target.c_str());
        return true;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }
    return false;
  }

  bool ensure_ready_state() {
    ActionInfo info;
    
    auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
    bool status_ok = false;
    
    // Retry querying the status up to 5 seconds
    while (std::chrono::steady_clock::now() < deadline) {
      if (get_action_status(info)) {
        status_ok = true;
        break;
      }
      RCLCPP_WARN(this->get_logger(), "Current action state is unavailable, retrying...");
      std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }

    if (!status_ok) {
      RCLCPP_ERROR(this->get_logger(), "Failed to get valid action state after 5 seconds. Aborting for safety.");
      return false;
    }

    if (info.action_desc == "BIPED_WHOLE_BODY_CTRL" &&
        info.status == aimdk_msgs::msg::McActionStatus::RUNNING) {
      return true;
    }

    RCLCPP_INFO(this->get_logger(),
                "Current state is %s. Starting state machine transition sequence...",
                info.action_desc.c_str());

    std::vector<std::string> sequence = {
      "PASSIVE_DEFAULT",
      "BIPED_STAND_DEFAULT",
      "BIPED_WALK_RUN",
      "BIPED_WHOLE_BODY_CTRL"
    };

    size_t start_index = 0;
    if (info.action_desc == "PASSIVE_DEFAULT") {
        start_index = 1;
    } else if (info.action_desc == "BIPED_STAND_DEFAULT") {
        start_index = 2;
    } else if (info.action_desc == "BIPED_WALK_RUN") {
        start_index = 3;
    } else if (info.action_desc == "DAMPING_DEFAULT" || info.action_desc == "STORE_DEFAULT") {
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

  rclcpp::Client<aimdk_msgs::srv::SetMcPresetMotion>::SharedPtr preset_client_;
  rclcpp::Client<aimdk_msgs::srv::SetMcAction>::SharedPtr set_action_client_;
  rclcpp::Client<aimdk_msgs::srv::GetMcAction>::SharedPtr get_action_client_;
};

int main(int argc, char *argv[]) {
  try {
    rclcpp::init(argc, argv);
    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);

    g_node = std::make_shared<PresetMotionClient>();
    auto client = std::dynamic_pointer_cast<PresetMotionClient>(g_node);

    // Prompt user to refer to documentation
    std::cout << "\nPlease refer to the interface documentation for the list of supported motions for this model." << std::endl;
    // Ask for robot series (Q or T)
    std::cout << "\nEnter robot series (Q/T): ";
    std::string robot_series;
    std::getline(std::cin, robot_series);
    // Trim whitespace and convert to uppercase
    robot_series.erase(std::remove_if(robot_series.begin(), robot_series.end(), ::isspace), robot_series.end());
    std::transform(robot_series.begin(), robot_series.end(), robot_series.begin(), ::toupper);

    std::map<int, std::string> motion_map;
    if (robot_series == "T") {
        motion_map = {{1001, "raise"}, {1002, "wave"}, {1003, "handshake"}, {1004, "airkiss"}};
    } else if (robot_series == "Q") {
        motion_map = {{3001, "wave"}, {3002, "handshake"}, {3003, "bump"}, {3004, "wave_hand"}};
    } else {
        std::cerr << "Unknown series. Please enter 'Q' or 'T'." << std::endl;
        return 1;
    }

    // Display available motions
    std::cout << "\nAvailable Preset Motions:" << std::endl;
    for (const auto &kv : motion_map) {
        std::cout << "  " << kv.first << ": " << kv.second << std::endl;
    }
    std::cout << "\nEnter preset motion ID: ";
    int motion_id = 0;
    if (!(std::cin >> motion_id)) return 0;
    // Validate selection
    if (motion_map.find(motion_id) == motion_map.end()) {
        std::cerr << "Invalid motion ID selected." << std::endl;
        return 1;
    }
    
    if (client) {
      client->send_request(motion_id);
    }
    g_node.reset();
    rclcpp::shutdown();
    return 0;
  } catch (const std::exception &e) {
    RCLCPP_ERROR(rclcpp::get_logger("main"), "Exited with exception: %s",
                 e.what());
    return 1;
  }
}
