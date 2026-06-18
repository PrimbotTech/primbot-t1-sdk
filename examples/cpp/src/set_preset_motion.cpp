/**
 * @brief Preset Motion Example Script
 *
 * Description:
 *   This script demonstrates how to call the SetMcPresetMotion service to play preset
 *   motions (like waving or handshaking) on the robot. Supports T and Q series robots
 *   with interactive series and motion selection.
 *
 * Prerequisites:
 *   - Robot motion control service must be running
 *   - Robot must support BIPED_LOCOMOTION_WBC action state
 *   - SetMcPresetMotion service must be available
 *
 * Usage:
 *   ros2 run aimdk_examples_cpp set_preset_motion
 *
 * Example:
 *   # Interactive mode (will prompt for series and motion ID)
 *   ros2 run aimdk_examples_cpp set_preset_motion
 *
 * Supported Motions:
 *   T series: 1001=raise, 1002=wave, 1003=handshake, 2001=handheart
 *   Q series: 3001=wave, 3002=handshake, 3003=bump, 3004=wave_hand
 */
#include "aimdk_msgs/msg/common_request.hpp"
#include "aimdk_msgs/msg/common_state.hpp"
#include "aimdk_msgs/msg/mc_action_status.hpp"
#include "aimdk_msgs/msg/mc_preset_motion.hpp"
#include "aimdk_msgs/srv/get_mc_action.hpp"
#include "aimdk_msgs/srv/set_mc_action.hpp"
#include "aimdk_msgs/srv/set_mc_preset_motion.hpp"
#include "rclcpp/rclcpp.hpp"

#include <algorithm>
#include <cctype>
#include <chrono>
#include <deque>
#include <map>
#include <memory>
#include <signal.h>
#include <string>
#include <thread>
#include <unordered_map>
#include <unordered_set>
#include <vector>

constexpr double kServiceCallTimeoutSec = 2.0;
constexpr int kMaxRetryCount = 3;

// CommonState reason 字段对应的中文描述
const std::unordered_map<uint32_t, std::string> kReasonDescriptions = {
    {0, "无错误"},
    {1, "开箱状态中"},
    {2, "开机自检中"},
    {3, "关机状态中"},
    {4, "当前形态不支持"},
    {5, "低电量限制"},
    {6, "正在充电中"},
    {7, "动作不在白名单"},
    {8, "HDS故障"},
    {9, "当前模式不支持"},
    {10, "前方有障碍物"},
    {11, "后方有障碍物"},
    {12, "左方有障碍物"},
    {13, "右方有障碍物"},
    {14, "上方有障碍物"},
    {15, "其他任务正在运行"},
    {16, "机器人已经是目标状态"}
};

std::string GetReasonDescription(uint32_t reason) {
  auto it = kReasonDescriptions.find(reason);
  if (it != kReasonDescriptions.end()) {
    return it->second;
  }
  return "未知原因(" + std::to_string(reason) + ")";
}

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

// T1状态机有向图（与 set_mc_action.cpp 保持一致）
// 定义状态间的合法转换边，navigate_to_action 使用 BFS 自动寻路
// 所有四足目标状态统一经过 QUADRUPED_LOCOMOTION_DEFAULT 后跳转
// 所有双足目标状态统一经过 BIPED_LOCOMOTION_WBC 后跳转
const std::unordered_map<std::string, std::vector<std::string>> ACTION_GRAPH = {
    {"PASSIVE_DEFAULT",
     {"QUADRUPED_STAND_DEFAULT", "QUADRUPED_GET_DOWN_DEFAULT",
      "QUADRUPED_SIT_DOWN_DEFAULT", "QUADRUPED_RECOVERY",
      "BIPED_RECOVERY", "DAMPING_DEFAULT"}},
    {"QUADRUPED_STAND_DEFAULT",
     {"QUADRUPED_LOCOMOTION_DEFAULT", "QUADRUPED_GET_DOWN_DEFAULT",
      "QUADRUPED_SIT_DOWN_DEFAULT", "QUADRUPED_LOCOMOTION_TERRAIN",
      "QUADRUPED_LOCOMOTION_RUN"}},
    {"QUADRUPED_LOCOMOTION_DEFAULT",
     {"QUADRUPED_LOCOMOTION_TERRAIN", "QUADRUPED_LOCOMOTION_RUN",
      "QUADRUPED_LOCOMOTION_BIONIC", "QUADRUPED_LOCOMOTION_PRIDE",
      "QUADRUPED_LOCOMOTION_PLEASURE", "QUADRUPED_LOCOMOTION_JUMP",
      "QUADRUPED_LOCOMOTION_HANDSHAKE", "QUADRUPED_LOCOMOTION_STRETCH",
      "QUADRUPED_LOCOMOTION_DANCE", "QUADRUPED_LOCOMOTION_FRONTFLIP",
      "QUADRUPED_LOCOMOTION_BACKFLIP", "QUADRUPED_TO_BIPED",
      "QUADRUPED_TO_BIPED_ROTATE", "QUADRUPED_TO_BIPED_FRONTFLIP",
      "QUADRUPED_STAND_DEFAULT"}},
    {"BIPED_LOCOMOTION_WBC",
     {"BIPED_LOCOMOTION_DEFAULT", "BIPED_LOCOMOTION_TERRAIN",
      "BIPED_LOCOMOTION_RUN", "BIPED_TO_QUADRUPED",
      "BIPED_TO_QUADRUPED_ROTATE", "BIPED_TO_QUADRUPED_FRONTFLIP",
      "BIPED_LOCOMOTION_ROTATE_LOCAL", "BIPED_LOCOMOTION_BACKBEND",
      "BIPED_LOCOMOTION_MOONWALK"}},
    {"BIPED_LOCOMOTION_DEFAULT",
     {"BIPED_LOCOMOTION_WBC", "BIPED_LOCOMOTION_TERRAIN",
      "BIPED_LOCOMOTION_RUN", "BIPED_LOCOMOTION_ROTATE_LOCAL",
      "BIPED_LOCOMOTION_BACKBEND", "BIPED_LOCOMOTION_MOONWALK"}},
    // ── 四足趴下/坐下 → 回到 四足位控站立 ──
    {"QUADRUPED_GET_DOWN_DEFAULT", {"QUADRUPED_STAND_DEFAULT"}},
    {"QUADRUPED_SIT_DOWN_DEFAULT", {"QUADRUPED_STAND_DEFAULT"}},
    // ── 四足技能/基础运动 → 回到 LOCOMOTION ──
    {"QUADRUPED_LOCOMOTION_TERRAIN", {"QUADRUPED_LOCOMOTION_DEFAULT"}},
    {"QUADRUPED_LOCOMOTION_RUN", {"QUADRUPED_LOCOMOTION_DEFAULT"}},
    // ── 双足基础运动 → 回到 WBC ──
    {"BIPED_LOCOMOTION_TERRAIN", {"BIPED_LOCOMOTION_WBC"}},
    {"BIPED_LOCOMOTION_RUN", {"BIPED_LOCOMOTION_WBC"}},
    // ── 双足技能运动 → 回到 WBC ──
    {"BIPED_LOCOMOTION_ROTATE_LOCAL", {"BIPED_LOCOMOTION_WBC"}},
    {"BIPED_LOCOMOTION_BACKBEND", {"BIPED_LOCOMOTION_WBC"}},
    {"BIPED_LOCOMOTION_MOONWALK", {"BIPED_LOCOMOTION_WBC"}},

    // ── 自动切换边（系统自动完成，navigate_to_action 会确认状态而非重设） ──
    {"BIPED_RECOVERY", {"BIPED_LOCOMOTION_WBC"}},
    {"QUADRUPED_RECOVERY", {"QUADRUPED_LOCOMOTION_DEFAULT"}},
    {"QUADRUPED_TO_BIPED", {"BIPED_LOCOMOTION_WBC"}},
    {"QUADRUPED_TO_BIPED_ROTATE", {"BIPED_LOCOMOTION_WBC"}},
    {"QUADRUPED_TO_BIPED_FRONTFLIP", {"BIPED_LOCOMOTION_WBC"}},
    {"BIPED_TO_QUADRUPED", {"QUADRUPED_LOCOMOTION_DEFAULT"}},
    {"BIPED_TO_QUADRUPED_ROTATE", {"QUADRUPED_LOCOMOTION_DEFAULT"}},
    {"BIPED_TO_QUADRUPED_FRONTFLIP", {"QUADRUPED_LOCOMOTION_DEFAULT"}},
    {"QUADRUPED_LOCOMOTION_BIONIC", {"QUADRUPED_LOCOMOTION_DEFAULT"}},
    {"QUADRUPED_LOCOMOTION_PRIDE", {"QUADRUPED_LOCOMOTION_DEFAULT"}},
    {"QUADRUPED_LOCOMOTION_PLEASURE", {"QUADRUPED_LOCOMOTION_DEFAULT"}},
    {"QUADRUPED_LOCOMOTION_JUMP", {"QUADRUPED_LOCOMOTION_DEFAULT"}},
    {"QUADRUPED_LOCOMOTION_HANDSHAKE", {"QUADRUPED_LOCOMOTION_DEFAULT"}},
    {"QUADRUPED_LOCOMOTION_STRETCH", {"QUADRUPED_LOCOMOTION_DEFAULT"}},
    {"QUADRUPED_LOCOMOTION_DANCE", {"QUADRUPED_LOCOMOTION_DEFAULT"}},
    {"QUADRUPED_LOCOMOTION_FRONTFLIP", {"QUADRUPED_LOCOMOTION_DEFAULT"}},
    {"QUADRUPED_LOCOMOTION_BACKFLIP", {"QUADRUPED_LOCOMOTION_DEFAULT"}},
};

// 需要先恢复到 PASSIVE_DEFAULT 的状态（这些状态无法直接跳转到其他路径）
const std::unordered_map<std::string, std::string> RECOVERY_TO_PASSIVE = {
    {"DAMPING_DEFAULT", "PASSIVE_DEFAULT"},
};

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
        return future;
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
      if (response->response.header.code == 0) {
        RCLCPP_INFO(this->get_logger(), "Motion request accepted. Task ID: %lu", response->response.task_id);
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

    auto future = call_service_with_retry<aimdk_msgs::srv::GetMcAction>(
        get_action_client_, request, "GetMcAction");

    if (!future.valid()) return false;

    auto res = future.get();
    info.action_desc = res->info.action_desc;
    info.status = res->info.status.value;
    return true;
  }

  bool set_action(const std::string &desc) {
    auto request = std::make_shared<aimdk_msgs::srv::SetMcAction::Request>();
    request->header.stamp = this->now();
    request->source = "node";  // 触发源标识
    request->command.action_desc = desc;
    RCLCPP_INFO(this->get_logger(), "Requesting state switch to: %s", desc.c_str());

    auto future = call_service_with_retry<aimdk_msgs::srv::SetMcAction>(
        set_action_client_, request, "SetMcAction");

    if (!future.valid()) return false;

    auto res = future.get();
    if (res->response.status.value == aimdk_msgs::msg::CommonState::SUCCESS) {
      return true;
    }
    
    // 获取失败原因
    uint32_t reason = res->response.status.reason;
    if (reason > 0) {
      std::string reason_desc = GetReasonDescription(reason);
      RCLCPP_WARN(this->get_logger(), "SetMcAction rejected: reason=%u - %s", reason, reason_desc.c_str());
    }
    
    return false;
  }

  bool wait_for_action(const std::string &target,
                       std::chrono::seconds timeout = std::chrono::seconds(10)) {
    auto deadline = std::chrono::steady_clock::now() + timeout;
    while (std::chrono::steady_clock::now() < deadline) {
      ActionInfo info;
      if (get_action_status(info) && info.action_desc == target &&
          info.status == aimdk_msgs::msg::McActionStatus::RUNNING) {
        RCLCPP_INFO(this->get_logger(), "Robot successfully reached state: %s",
                    target.c_str());
        return true;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }
    RCLCPP_ERROR(this->get_logger(), "Timeout waiting for state: %s",
                 target.c_str());
    return false;
  }

  std::vector<std::string> find_path_bfs(const std::string &start,
                                          const std::string &target) {
    if (start == target) {
      return {start};
    }

    std::unordered_set<std::string> visited = {start};
    std::deque<std::pair<std::string, std::vector<std::string>>> queue;
    queue.push_back({start, {start}});

    while (!queue.empty()) {
      auto [current, path] = queue.front();
      queue.pop_front();

      auto it = ACTION_GRAPH.find(current);
      if (it == ACTION_GRAPH.end()) {
        continue;
      }

      for (const auto &neighbor : it->second) {
        if (neighbor == target) {
          auto result = path;
          result.push_back(neighbor);
          return result;
        }
        if (visited.find(neighbor) == visited.end()) {
          visited.insert(neighbor);
          auto new_path = path;
          new_path.push_back(neighbor);
          queue.push_back({neighbor, new_path});
        }
      }
    }

    return {};
  }

  bool navigate_to_action(const std::string &target_action,
                           const std::string &current_desc,
                           int32_t current_status) {
    if (current_desc == target_action &&
        current_status == aimdk_msgs::msg::McActionStatus::RUNNING) {
      RCLCPP_INFO(this->get_logger(), "Already at target action: %s", target_action.c_str());
      return true;
    }

    // 处理需要先恢复到 PASSIVE_DEFAULT 的状态
    std::string actual_current = current_desc;
    auto recovery_it = RECOVERY_TO_PASSIVE.find(current_desc);
    if (recovery_it != RECOVERY_TO_PASSIVE.end()) {
      const std::string &recovery_target = recovery_it->second;
      RCLCPP_INFO(this->get_logger(), "Recovery: switching from %s to %s...",
                  current_desc.c_str(), recovery_target.c_str());
      if (!set_action(recovery_target) || !wait_for_action(recovery_target)) return false;
      actual_current = recovery_target;
    }

    // 特殊路径：如果从 PASSIVE_DEFAULT 开始，强制使用固定路径
    std::vector<std::string> path;
    if (actual_current == "PASSIVE_DEFAULT" && target_action == "BIPED_LOCOMOTION_WBC") {
      RCLCPP_INFO(this->get_logger(), "Using fixed path from PASSIVE_DEFAULT to BIPED_LOCOMOTION_WBC");
      path = {
        "PASSIVE_DEFAULT",
        "QUADRUPED_STAND_DEFAULT",
        "QUADRUPED_LOCOMOTION_DEFAULT",
        "QUADRUPED_TO_BIPED",
        "BIPED_LOCOMOTION_WBC"
      };
    } else {
      path = find_path_bfs(actual_current, target_action);
    }

    if (path.empty()) {
      RCLCPP_INFO(this->get_logger(), "No path found for %s -> %s, attempting direct transition...",
                  actual_current.c_str(), target_action.c_str());
      if (!set_action(target_action)) return false;
      return wait_for_action(target_action, std::chrono::seconds(5));
    }

    for (size_t i = 1; i < path.size(); ++i) {
      const std::string &target = path[i];
      RCLCPP_INFO(this->get_logger(), "Path step %zu/%zu: Switching to %s...",
                  i, path.size() - 1, target.c_str());
      // 重试机制：机器人可能还在运动中，等待稳定后再试
      for (int attempt = 1; attempt <= 5; ++attempt) {
        if (set_action(target)) {
          if (wait_for_action(target)) break;
          return false;
        }
        if (attempt < 5) {
          RCLCPP_INFO(this->get_logger(), "Step %zu attempt %d/5 failed, waiting for robot to stabilize...", i, attempt);
          std::this_thread::sleep_for(std::chrono::seconds(2));
        } else {
          return false;
        }
      }
      // 等待物理动作稳定后再执行下一步
      std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    return true;
  }

  bool ensure_ready_state() {
    ActionInfo info;
    auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);

    // Retry querying the status up to 5 seconds
    while (std::chrono::steady_clock::now() < deadline) {
      if (get_action_status(info))
        return navigate_to_action("BIPED_LOCOMOTION_WBC", info.action_desc, info.status);
      RCLCPP_WARN(this->get_logger(), "Current action state is unavailable, retrying...");
      std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }

    RCLCPP_ERROR(this->get_logger(),
                 "Failed to get valid action state after 5 seconds. Aborting for safety.");
    return false;
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

    g_node = std::make_shared<PresetMotionClient>();
    auto client = std::dynamic_pointer_cast<PresetMotionClient>(g_node);
    if (client) client->send_request(motion_id);
    g_node.reset();
    rclcpp::shutdown();
    return 0;
  } catch (const std::exception &e) {
    RCLCPP_ERROR(rclcpp::get_logger("main"), "Exited with exception: %s", e.what());
    return 1;
  }
}
