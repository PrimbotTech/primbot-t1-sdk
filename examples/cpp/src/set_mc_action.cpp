/**
 * @brief Action Switch Example Script
 *
 * Description:
 *   This script demonstrates how to call the SetMcAction service to switch robot
 *   action states. Uses BFS on ACTION_GRAPH to automatically navigate the state
 *   machine transition path from the current action to the target action.
 *
 * Prerequisites:
 *   - Robot motion control service must be running
 *   - SetMcAction and GetMcAction services must be available
 *
 * Usage:
 *   ros2 run aimdk_examples_cpp set_mc_action --ros-args -p type:=action \
 *   [-p action_desc:=<ACTION>]
 *
 * Example:
 *   # Interactive mode: input target action via terminal
 *   ros2 run aimdk_examples_cpp set_mc_action --ros-args -p type:=action
 *
 *   # Non-interactive mode: specify action via parameter
 *   ros2 run aimdk_examples_cpp set_mc_action --ros-args -p type:=action \
 *   -p action_desc:=QUADRUPED_LOCOMOTION_JUMP
 *
 * Parameters:
 *   - type: "action" or "motion", required (motion currently disabled)
 *   - action_desc: string, optional; if set, skips interactive input and executes directly
 *   - motion: string, required when type=motion
 *   - interrupt: bool, optional when type=motion, default=true
 *
 * Notes:
 *   - Single Switch: Switches to target action, holds for 2s, then auto-returns to QUADRUPED_LOCOMOTION_DEFAULT.
 *   - Auto Path: BFS finds shortest transition path on ACTION_GRAPH, skipping redundant steps.
 *   - Recovery: Handles DAMPING_DEFAULT by recovering to PASSIVE_DEFAULT first.
 *   - Retry: Each path step retries up to 5 times if the robot is still moving.
 *   - Ctrl+C Safety: During execution, Ctrl+C navigates back to QUADRUPED_LOCOMOTION_DEFAULT before exit.
 *   - WARNING: Do NOT use QUADRUPED_LOCOMOTION_JUMP for testing — the robot will jump and may cause injury or damage.
 */
#include "aimdk_msgs/msg/common_request.hpp"
#include "aimdk_msgs/msg/common_state.hpp"
#include "aimdk_msgs/msg/mc_action_status.hpp"
#include "aimdk_msgs/srv/get_mc_action.hpp"
#include "aimdk_msgs/srv/set_mc_action.hpp"
#include "aimdk_msgs/srv/set_mc_motion.hpp"
#include "rclcpp/rclcpp.hpp"

#include <chrono>
#include <cstdint>
#include <deque>
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
    {9, "当前模式不支持"}
};

std::string GetReasonDescription(uint32_t reason) {
  auto it = kReasonDescriptions.find(reason);
  if (it != kReasonDescriptions.end()) {
    return it->second;
  }
  return "未知原因(" + std::to_string(reason) + ")";
}

std::shared_ptr<rclcpp::Node> g_node = nullptr;
bool g_shutdown_requested = false;

void signal_handler(int signal) {
  if (g_node) {
    RCLCPP_INFO(g_node->get_logger(),
                "Received signal %d, will navigate back before shutdown.",
                signal);
  }
  g_shutdown_requested = true;
}

// T1狗形状态机有向图（基于 qd1_t1d5/action_ruler.yaml 及状态机流程图）
// 定义状态间的合法转换边，navigate_to_action 使用 BFS 自动寻路
// 所有四足目标状态统一经过 QUADRUPED_LOCOMOTION_DEFAULT 后跳转
// 所有双足目标状态统一经过 BIPED_LOCOMOTION_WBC 后跳转
const std::unordered_map<std::string, std::vector<std::string>> ACTION_GRAPH = {
    {"PASSIVE_DEFAULT",
     {"QUADRUPED_STAND_DEFAULT", "QUADRUPED_GET_DOWN_DEFAULT",
      "QUADRUPED_SIT_DOWN_DEFAULT", "QUADRUPED_RECOVERY",
      "DAMPING_DEFAULT"}},
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

  struct ActionInfo {
    int32_t action_id = 0;
    std::string action_desc;
    int32_t status = aimdk_msgs::msg::McActionStatus::IDLE;
  };

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

      // Execute SetMcAction (single switch)
      bool ok = navigate_to_action(target_action, current.action_desc, current.status) &&
                wait_for_action(target_action, std::chrono::seconds(5));
      if (ok) {
        RCLCPP_INFO(this->get_logger(), "Target action %s reached, holding for 2s...", target_action.c_str());
        std::this_thread::sleep_for(std::chrono::seconds(2));
      }
      std::cout << (ok ? "Switch succeeded." :
                       "Switch failed, please confirm if the expected Action "
                       "complies with the state machine transition logic")
                << std::endl;

      // Switch completed, navigate back to QUADRUPED_LOCOMOTION_DEFAULT
      if (!get_action_status(current)) return true;
      RCLCPP_INFO(this->get_logger(),
                  "Switch done, navigating back to QUADRUPED_LOCOMOTION_DEFAULT, from %s...",
                  current.action_desc.c_str());
      if (navigate_to_action("QUADRUPED_LOCOMOTION_DEFAULT", current.action_desc, current.status))
        wait_for_action("QUADRUPED_LOCOMOTION_DEFAULT", std::chrono::seconds(5));
      return true;
    } else {
      // Motion mode is currently disabled
      RCLCPP_WARN(this->get_logger(), "Motion mode is currently disabled.");
      return false;

      // Optimized logic for 'motion' type: Ensure robot is in QUADRUPED_LOCOMOTION_DEFAULT
      ActionInfo current;
      if (!get_action_status(current)) return false;

      if (current.action_desc == "QUADRUPED_LOCOMOTION_DEFAULT" &&
          current.status == aimdk_msgs::msg::McActionStatus::RUNNING) {
        RCLCPP_INFO(this->get_logger(),
                    "Robot already in QUADRUPED_LOCOMOTION_DEFAULT. Proceeding to motion...");
      } else {
        RCLCPP_INFO(this->get_logger(),
                    "Current state is %s. Starting state machine transition sequence...",
                    current.action_desc.c_str());
        std::vector<std::string> sequence = {
            "PASSIVE_DEFAULT", "BIPED_STAND_DEFAULT", "QUADRUPED_LOCOMOTION_DEFAULT"};
        static const std::unordered_map<std::string, size_t> kStartIndex = {
            {"PASSIVE_DEFAULT", 1}, {"BIPED_STAND_DEFAULT", 2}};
        size_t start_index = (kStartIndex.count(current.action_desc) ? kStartIndex.at(current.action_desc) : 0);
        for (size_t i = start_index; i < sequence.size(); ++i) {
          if (!set_action(sequence[i]) || !wait_for_action(sequence[i])) return false;
        }
      }

      // Execute final target motion
      if (!set_motion(motion_, interrupt_)) return false;
      return wait_for_motion();
    }
  }

  // 执行路径中的每个步骤
  bool execute_path(const std::vector<std::string> &path) {
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

  bool navigate_to_action(const std::string &target_action,
                          const std::string &current_desc, int32_t current_status) {
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
    if (actual_current == "PASSIVE_DEFAULT" && target_action == "BIPED_LOCOMOTION_WBC") {
      RCLCPP_INFO(this->get_logger(), "Using fixed path from PASSIVE_DEFAULT to BIPED_LOCOMOTION_WBC");
      std::vector<std::string> path = {
          "PASSIVE_DEFAULT",
          "QUADRUPED_STAND_DEFAULT",
          "QUADRUPED_LOCOMOTION_DEFAULT",
          "QUADRUPED_TO_BIPED",
          "BIPED_LOCOMOTION_WBC"
      };
      return execute_path(path);
    }

    auto path = find_path_bfs(actual_current, target_action);

    if (path.empty()) {
      // 图中找不到路径：尝试直接转换（由服务端校验合法性）
      RCLCPP_INFO(this->get_logger(), "No path found for %s -> %s, attempting direct transition...",
                  actual_current.c_str(), target_action.c_str());
      if (!set_action(target_action)) return false;
      return wait_for_action(target_action, std::chrono::seconds(5));
    }

    return execute_path(path);
  }

  bool wait_for_action(
      const std::string &expected_action_desc,
      std::chrono::seconds timeout = std::chrono::seconds(10),
      std::chrono::milliseconds poll_interval = std::chrono::milliseconds(200)) {
    auto deadline = std::chrono::steady_clock::now() + timeout;
    RCLCPP_INFO(this->get_logger(), "Waiting for target action_desc=%s to reach RUNNING state...",
                expected_action_desc.c_str());

    while (rclcpp::ok() && std::chrono::steady_clock::now() < deadline) {
      ActionInfo info;
      if (!get_action_status(info)) {
        std::this_thread::sleep_for(poll_interval);
        continue;
      }
      if (info.status == aimdk_msgs::msg::McActionStatus::RUNNING &&
          info.action_desc == expected_action_desc) {
        RCLCPP_INFO(this->get_logger(), "Target action reached and is running: action_desc=%s",
                    expected_action_desc.c_str());
        return true;
      }
      std::this_thread::sleep_for(poll_interval);
    }

    RCLCPP_ERROR(this->get_logger(),
                 "Timed out waiting for target action_desc=%s to reach RUNNING state.",
                 expected_action_desc.c_str());
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
      info.action_id = response->info.current_action.value;
      info.action_desc = response->info.action_desc;
      info.status = response->info.status.value;
      return true;
    } catch (const std::exception &e) {
      RCLCPP_ERROR(this->get_logger(), "Exception occurred: %s", e.what());
      return false;
    }
  }

private:
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
        return future;
      }
      RCLCPP_INFO(this->get_logger(), "%s attempt %d/%d timed out, retrying...",
                  service_name.c_str(), i + 1, max_retries);
      std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }
    RCLCPP_ERROR(this->get_logger(), "%s failed after %d attempts", service_name.c_str(), max_retries);
    return typename rclcpp::Client<ServiceT>::SharedFuture();
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

  bool set_action(const std::string &action_desc) {
    try {
      auto request = std::make_shared<aimdk_msgs::srv::SetMcAction::Request>();
      request->header.stamp = this->now();
      request->source = "node";  // 触发源标识
      request->command.action.value = 0;
      request->command.action_desc = action_desc;

      RCLCPP_INFO(this->get_logger(), "Sending request: action_desc=%s", action_desc.c_str());

      auto future = call_service_with_retry<aimdk_msgs::srv::SetMcAction>(
          set_action_client_, request, "SetMcAction");

      // 超时后检查是否实际已到达目标状态
      if (!future.valid()) {
        ActionInfo info;
        if (get_action_status(info) && info.action_desc == action_desc) {
          RCLCPP_WARN(this->get_logger(),
                      "SetMcAction timed out, but target action is already active: %s status=%d",
                      info.action_desc.c_str(), info.status);
          return true;
        }
        return false;
      }

      auto response = future.get();
      if (response->response.status.value == aimdk_msgs::msg::CommonState::SUCCESS) {
        RCLCPP_INFO(this->get_logger(), "SetMcAction request accepted by service.");
        return true;
      }
      
      // 获取失败原因
      uint32_t reason = response->response.status.reason;
      if (reason > 0) {
        std::string reason_desc = GetReasonDescription(reason);
        RCLCPP_WARN(this->get_logger(), "SetMcAction rejected: reason=%u - %s", 
                    reason, reason_desc.c_str());
      }

      RCLCPP_ERROR(this->get_logger(), "Failed to set robot mode: %s", response->response.message.c_str());
      return false;
    } catch (const std::exception &e) {
      RCLCPP_ERROR(this->get_logger(), "Exception occurred: %s", e.what());
      return false;
    }
  }

  bool set_motion(const std::string &motion_name, bool interrupt) {
    try {
      for (int attempt = 1; attempt <= 5; ++attempt) {
        auto request = std::make_shared<aimdk_msgs::srv::SetMcMotion::Request>();
        request->header.stamp = this->now();
        request->motion = motion_name;
        request->type = aimdk_msgs::srv::SetMcMotion::Request::MIMIC_QY;
        request->interrupt = interrupt;

        RCLCPP_INFO(this->get_logger(), "Sending SetMcMotion request (%d/5): motion=%s interrupt=%s",
                    attempt, motion_name.c_str(), interrupt ? "true" : "false");

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
        RCLCPP_INFO(this->get_logger(), "Current motion action is running: action_id=%d action_desc=%s",
                    info.action_id, info.action_desc.c_str());
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
    if (type_ == "action") {
      wait_for_service(set_action_client_, "/aimdk_5Fmsgs/srv/SetMcAction");
    } else if (type_ == "motion") {
      wait_for_service(set_motion_client_, "/aimdk_5Fmsgs/srv/SetMcMotion");
    }
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
  rclcpp::init(argc, argv);
  signal(SIGINT, signal_handler);
  signal(SIGTERM, signal_handler);

  auto node = std::make_shared<SetMcActionClient>();
  g_node = node;

  try {
    bool ok = node->execute();
    node.reset();
    g_node.reset();
    if (rclcpp::ok()) rclcpp::shutdown();
    return ok ? 0 : 1;
  } catch (...) {
    // Ctrl+C or other interrupt: navigate back to QUADRUPED_LOCOMOTION_DEFAULT
    if (node && rclcpp::ok()) {
      SetMcActionClient::ActionInfo current;
      if (node->get_action_status(current)) {
        RCLCPP_INFO(node->get_logger(), "Ctrl+C received, navigating back to QUADRUPED_LOCOMOTION_DEFAULT from %s...",
                    current.action_desc.c_str());
        if (node->navigate_to_action("QUADRUPED_LOCOMOTION_DEFAULT", current.action_desc, current.status))
          node->wait_for_action("QUADRUPED_LOCOMOTION_DEFAULT", std::chrono::seconds(5));
      }
    }
    node.reset();
    g_node.reset();
    if (rclcpp::ok()) rclcpp::shutdown();
    return 0;
  }
}
