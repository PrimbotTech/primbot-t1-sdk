// Copyright 2026 PrimeBot Team
//
// Header-only T1 MC action switcher. It mirrors the action graph in
// qd1_t2d0/action_ruler.yaml, plans a route from the current action, and
// waits for every requested MC runner transition before continuing.

#ifndef AIMDK_EXAMPLES_CPP_MC_ACTION_SWITCHER_HPP_
#define AIMDK_EXAMPLES_CPP_MC_ACTION_SWITCHER_HPP_

#include "aimdk_msgs/msg/common_request.hpp"
#include "aimdk_msgs/msg/common_state.hpp"
#include "aimdk_msgs/msg/mc_action_command.hpp"
#include "aimdk_msgs/msg/mc_action_status.hpp"
#include "aimdk_msgs/srv/get_mc_action.hpp"
#include "aimdk_msgs/srv/set_mc_action.hpp"
#include "rclcpp/rclcpp.hpp"

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <deque>
#include <functional>
#include <memory>
#include <string>
#include <thread>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

namespace aimdk_examples
{

struct McActionSwitchOptions
{
  std::string source = "sdk_node";
  std::chrono::milliseconds service_wait_timeout{5000};
  std::chrono::milliseconds service_call_timeout{3000};
  std::chrono::milliseconds poll_interval{500};
  std::chrono::milliseconds minimum_set_action_interval{500};
  std::chrono::milliseconds total_timeout{20000};
  std::function<bool()> should_cancel;
};

enum class McActionSwitchError
{
  kNone,
  kInvalidArgument,
  kServiceUnavailable,
  kGetActionFailed,
  kTargetNotActive,
  kNoTransitionPath,
  kSetActionFailed,
  kTimeout,
  kShutdown,
};

struct McActionSwitchResult
{
  bool success = false;
  McActionSwitchError error = McActionSwitchError::kNone;
  std::string current_action;
  std::string target_action;
  std::string message;
};

// Switches to target_action synchronously. The target and all intermediate
// action names use the action_desc strings from qd1_t2d0/action_ruler.yaml.
class McActionSwitcher
{
 public:
  explicit McActionSwitcher(
      const rclcpp::Node::SharedPtr & node,
      const std::string & set_action_service =
          "/aimdk_5Fmsgs/srv/SetMcAction",
      const std::string & get_action_service =
          "/aimdk_5Fmsgs/srv/GetMcAction")
  : node_(node),
    set_action_service_(set_action_service),
    get_action_service_(get_action_service)
  {
    if (node_) {
      set_action_client_ =
          node_->create_client<aimdk_msgs::srv::SetMcAction>(set_action_service_);
      get_action_client_ =
          node_->create_client<aimdk_msgs::srv::GetMcAction>(get_action_service_);
    }
  }

  McActionSwitchResult switch_to(
      const std::string & target_action,
      const McActionSwitchOptions & options = McActionSwitchOptions())
  {
    McActionSwitchResult result;
    result.target_action = target_action;

    if (!node_ || target_action.empty()) {
      return fail(result, McActionSwitchError::kInvalidArgument,
                  "Node and target_action must be provided.");
    }
    if (is_cancelled(options)) {
      return fail(result, McActionSwitchError::kShutdown,
                  "Action switch cancelled.");
    }
    if (!wait_for_services(options, result)) {
      return result;
    }

    ActionState current;
    if (!get_current_action(options, current)) {
      if (is_cancelled(options)) {
        return fail(result, McActionSwitchError::kShutdown,
                    "Action switch cancelled while reading the current action.");
      }
      return fail(result, McActionSwitchError::kGetActionFailed,
                  "GetMcAction did not return the current action.");
    }
    result.current_action = current.action_desc;
    if (current.action_desc == target_action) {
      if (is_active(current)) {
        result.success = true;
        result.message = "Robot is already in the requested action.";
        return result;
      }
      return fail(result, McActionSwitchError::kTargetNotActive,
                  "MC reports the target action as IDLE; refusing to leave " +
                      target_action + " through a fallback transition.");
    }

    const std::vector<PlannedTransition> path =
        plan_path(current.action_desc, target_action);
    if (path.empty()) {
      return fail(result, McActionSwitchError::kNoTransitionPath,
                  "No T1 action_ruler path from " + current.action_desc +
                      " to " + target_action + ".");
    }

    const auto deadline = std::chrono::steady_clock::now() + options.total_timeout;
    auto next_request_time = std::chrono::steady_clock::now();
    for (const auto & transition : path) {
      if (!rclcpp::ok()) {
        return fail(result, McActionSwitchError::kShutdown,
                    "ROS shutdown while switching actions.");
      }
      if (std::chrono::steady_clock::now() >= deadline) {
        return fail(result, McActionSwitchError::kTimeout,
                    "Timed out while switching actions.");
      }

      std::this_thread::sleep_until(next_request_time);
      if (std::chrono::steady_clock::now() >= deadline) {
        return fail(result, McActionSwitchError::kTimeout,
                    "Timed out while waiting to request the next action.");
      }
      if (!request_action(transition.command, options)) {
        if (is_cancelled(options)) {
          return fail(result, McActionSwitchError::kShutdown,
                      "Action switch cancelled while requesting " +
                          transition.command + ".");
        }
        return fail(result, McActionSwitchError::kSetActionFailed,
                    "SetMcAction rejected " + transition.command + ".");
      }
      next_request_time = std::chrono::steady_clock::now() +
                          options.minimum_set_action_interval;

      ActionState reached;
      if (!wait_for_action(transition.expected_action, options, deadline,
                           reached)) {
        if (!rclcpp::ok()) {
          return fail(result, McActionSwitchError::kShutdown,
                      "ROS shutdown while waiting for " +
                          transition.expected_action + ".");
        }
        if (is_cancelled(options)) {
          return fail(result, McActionSwitchError::kShutdown,
                      "Action switch cancelled while waiting for " +
                          transition.expected_action + ".");
        }
        return fail(result, McActionSwitchError::kTimeout,
                    "Timed out waiting for " + transition.expected_action +
                        " after requesting " + transition.command + ".");
      }
      current = reached;
      result.current_action = current.action_desc;
    }

    if (current.action_desc != target_action) {
      return fail(result, McActionSwitchError::kTimeout,
                  "MC did not reach the requested target action.");
    }
    result.success = true;
    result.message = "Robot reached the requested action.";
    return result;
  }

 private:
  static bool is_cancelled(const McActionSwitchOptions & options)
  {
    return options.should_cancel && options.should_cancel();
  }

  struct ActionState
  {
    std::string action_desc;
    int32_t status = aimdk_msgs::msg::McActionStatus::IDLE;
  };

  struct PlannedTransition
  {
    std::string command;
    std::string expected_action;
  };

  static McActionSwitchResult & fail(
      McActionSwitchResult & result, McActionSwitchError error,
      const std::string & message)
  {
    result.success = false;
    result.error = error;
    result.message = message;
    return result;
  }

  static bool is_active(const ActionState & action)
  {
    return action.status != aimdk_msgs::msg::McActionStatus::IDLE;
  }

  static const std::unordered_map<std::string, std::vector<std::string>> &
  action_rules()
  {
    static const std::unordered_map<std::string, std::vector<std::string>> rules = {
        {"PASSIVE_DEFAULT", {"QUADRUPED_STAND_DEFAULT",
                              "QUADRUPED_SIT_DOWN_DEFAULT",
                              "QUADRUPED_GET_DOWN_DEFAULT",
                              "QUADRUPED_RECOVERY"}},
        {"DAMPING_DEFAULT", {"PASSIVE_DEFAULT"}},
        {"QUAD_LIE_DOWN", {"PASSIVE_DEFAULT"}},
        {"QUADRUPED_STAND_DEFAULT", {"QUADRUPED_LOCOMOTION_DEFAULT",
                                      "QUADRUPED_LOCOMOTION_TERRAIN",
                                      "QUADRUPED_LOCOMOTION_RUN",
                                      "QUADRUPED_GET_DOWN_DEFAULT",
                                      "QUADRUPED_SIT_DOWN_DEFAULT",
                                      "QUADRUPED_LOCOMOTION_BIONIC",
                                      "QUADRUPED_LOCOMOTION_PRIDE",
                                      "QUADRUPED_LOCOMOTION_PLEASURE",
                                      "QUADRUPED_LOCOMOTION_JUMP",
                                      "QUADRUPED_LOCOMOTION_HANDSHAKE",
                                      "QUADRUPED_LOCOMOTION_STRETCH",
                                      "QUADRUPED_LOCOMOTION_DANCE",
                                      "QUADRUPED_PUSH_UP"}},
        {"QUADRUPED_GET_DOWN_DEFAULT", {"QUADRUPED_STAND_DEFAULT",
                                        "QUADRUPED_SIT_DOWN_DEFAULT"}},
        {"QUADRUPED_SIT_DOWN_DEFAULT", {"QUADRUPED_GET_DOWN_DEFAULT",
                                        "QUADRUPED_STAND_DEFAULT"}},
        {"QUADRUPED_LOCOMOTION_DEFAULT", {"QUADRUPED_LOCOMOTION_TERRAIN",
                                           "QUADRUPED_LOCOMOTION_BIONIC",
                                           "QUADRUPED_LOCOMOTION_RUN",
                                           "QUADRUPED_TO_BIPED",
                                           "QUADRUPED_TO_BIPED_ROTATE",
                                           "QUADRUPED_TO_BIPED_FRONTFLIP",
                                           "QUADRUPED_LOCOMOTION_FRONTFLIP",
                                           "QUADRUPED_LOCOMOTION_BACKFLIP",
                                           "QUADRUPED_LOCOMOTION_STEP",
                                           "QUADRUPED_LOCOMOTION_PRIDE",
                                           "QUADRUPED_LOCOMOTION_HANDSHAKE",
                                           "QUADRUPED_LOCOMOTION_STRETCH",
                                           "QUADRUPED_LOCOMOTION_DANCE",
                                           "QUADRUPED_PUSH_UP",
                                           "QUADRUPED_STAND_DEFAULT",
                                           "QUADRUPED_GET_DOWN_DEFAULT",
                                           "QUADRUPED_SIT_DOWN_DEFAULT"}},
        {"QUADRUPED_LOCOMOTION_STEP", {"QUADRUPED_LOCOMOTION_DEFAULT"}},
        {"QUADRUPED_LOCOMOTION_RUN", {"QUADRUPED_LOCOMOTION_DEFAULT",
                                      "QUADRUPED_LOCOMOTION_TERRAIN",
                                      "QUADRUPED_LOCOMOTION_BIONIC",
                                      "QUADRUPED_TO_BIPED",
                                      "QUADRUPED_TO_BIPED_ROTATE",
                                      "QUADRUPED_TO_BIPED_FRONTFLIP",
                                      "QUADRUPED_LOCOMOTION_FRONTFLIP",
                                      "QUADRUPED_LOCOMOTION_BACKFLIP",
                                      "QUADRUPED_LOCOMOTION_STEP",
                                      "QUADRUPED_LOCOMOTION_PRIDE",
                                      "QUADRUPED_LOCOMOTION_HANDSHAKE",
                                      "QUADRUPED_LOCOMOTION_STRETCH",
                                      "QUADRUPED_LOCOMOTION_DANCE",
                                      "QUADRUPED_PUSH_UP",
                                      "QUADRUPED_STAND_DEFAULT",
                                      "QUADRUPED_GET_DOWN_DEFAULT",
                                      "QUADRUPED_SIT_DOWN_DEFAULT"}},
        {"QUADRUPED_LOCOMOTION_TERRAIN", {"QUADRUPED_LOCOMOTION_DEFAULT",
                                          "QUADRUPED_LOCOMOTION_BIONIC",
                                          "QUADRUPED_LOCOMOTION_RUN",
                                          "QUADRUPED_TO_BIPED",
                                          "QUADRUPED_TO_BIPED_ROTATE",
                                          "QUADRUPED_TO_BIPED_FRONTFLIP",
                                          "QUADRUPED_LOCOMOTION_FRONTFLIP",
                                          "QUADRUPED_LOCOMOTION_BACKFLIP",
                                          "QUADRUPED_LOCOMOTION_STEP",
                                          "QUADRUPED_LOCOMOTION_PRIDE",
                                          "QUADRUPED_LOCOMOTION_HANDSHAKE",
                                          "QUADRUPED_LOCOMOTION_STRETCH",
                                          "QUADRUPED_LOCOMOTION_DANCE",
                                          "QUADRUPED_PUSH_UP",
                                          "QUADRUPED_STAND_DEFAULT",
                                          "QUADRUPED_GET_DOWN_DEFAULT",
                                          "QUADRUPED_SIT_DOWN_DEFAULT"}},
        {"QUADRUPED_LOCOMOTION_BIONIC", {"QUADRUPED_LOCOMOTION_DEFAULT",
                                         "QUADRUPED_LOCOMOTION_TERRAIN",
                                         "QUADRUPED_LOCOMOTION_RUN"}},
        {"QUADRUPED_LOCOMOTION_PRIDE", {"QUADRUPED_LOCOMOTION_DEFAULT",
                                        "QUADRUPED_LOCOMOTION_PLEASURE"}},
        {"QUADRUPED_PUSH_UP", {"QUADRUPED_LOCOMOTION_DEFAULT",
                                "QUADRUPED_STAND_DEFAULT"}},
        {"QUADRUPED_LOCOMOTION_PLEASURE", {"QUADRUPED_LOCOMOTION_DEFAULT",
                                            "QUADRUPED_LOCOMOTION_PRIDE"}},
        {"QUADRUPED_LOCOMOTION_JUMP", {"QUADRUPED_LOCOMOTION_DEFAULT",
                                        "QUADRUPED_STAND_DEFAULT"}},
        {"QUADRUPED_LOCOMOTION_HANDSHAKE", {"QUADRUPED_LOCOMOTION_DEFAULT",
                                             "QUADRUPED_STAND_DEFAULT"}},
        {"QUADRUPED_RECOVERY", {"QUADRUPED_STAND_DEFAULT",
                                 "QUADRUPED_LOCOMOTION_DEFAULT",
                                 "QUADRUPED_LOCOMOTION_TERRAIN",
                                 "QUADRUPED_LOCOMOTION_RUN"}},
        {"QUADRUPED_LOCOMOTION_STRETCH", {"QUADRUPED_LOCOMOTION_DEFAULT",
                                           "QUADRUPED_STAND_DEFAULT"}},
        {"QUADRUPED_LOCOMOTION_DANCE", {"QUADRUPED_LOCOMOTION_DEFAULT",
                                         "QUADRUPED_STAND_DEFAULT"}},
        {"BIPED_STAND_DEFAULT", {"BIPED_LOCOMOTION_WBC"}},
        {"BIPED_LOCOMOTION_WBC", { "BIPED_TO_QUADRUPED",
                                   "BIPED_TO_QUADRUPED_ROTATE",
                                   "BIPED_TO_QUADRUPED_FRONTFLIP",
                                   "BIPED_LOCOMOTION_TERRAIN",
                                   "BIPED_LOCOMOTION_RUN",
                                   "BIPED_LOCOMOTION_DEFAULT",
                                   "BIPED_LOCOMOTION_ROTATE_LOCAL",
                                   "BIPED_LOCOMOTION_BACKBEND", "BIPED_YUZURU",
                                   "BIPED_SPLIT", "BIPED_LOCOMOTION_MOONWALK",
                                   "BIPED_LOCOMOTION_DANCE",
                                   "BIPED_LOCOMOTION_ANIMATION", "BIPED_RECORD_UPPER",
                                   "BIPED_CUSTOM_UPPER"}},
        {"BIPED_RECORD_UPPER", {"BIPED_LOCOMOTION_WBC"}},
        {"BIPED_LOCOMOTION_ANIMATION", { "BIPED_TO_QUADRUPED",
                                         "BIPED_TO_QUADRUPED_ROTATE",
                                         "BIPED_TO_QUADRUPED_FRONTFLIP",
                                         "BIPED_LOCOMOTION_TERRAIN",
                                         "BIPED_LOCOMOTION_RUN",
                                         "BIPED_LOCOMOTION_DEFAULT",
                                         "BIPED_LOCOMOTION_ROTATE_LOCAL",
                                         "BIPED_LOCOMOTION_BACKBEND", "BIPED_YUZURU",
                                         "BIPED_SPLIT", "BIPED_LOCOMOTION_MOONWALK",
                                         "BIPED_LOCOMOTION_WBC"}},
        {"BIPED_LOCOMOTION_RUN", {"BIPED_LOCOMOTION_WBC",
                                   "BIPED_TO_QUADRUPED_ROTATE"}},
        {"BIPED_LOCOMOTION_TERRAIN", {"BIPED_LOCOMOTION_WBC"}},
        {"BIPED_YUZURU", {"BIPED_LOCOMOTION_WBC"}},
        {"BIPED_SPLIT", {"BIPED_LOCOMOTION_WBC"}},
        {"BIPED_LOCOMOTION_DEFAULT", {"BIPED_LOCOMOTION_TERRAIN",
                                       "BIPED_LOCOMOTION_WBC",
                                       "BIPED_LOCOMOTION_RUN"}},
        {"QUADRUPED_TO_BIPED", {"BIPED_LOCOMOTION_WBC",
                                 "BIPED_LOCOMOTION_TERRAIN"}},
        {"QUADRUPED_TO_BIPED_ROTATE", {"BIPED_LOCOMOTION_WBC",
                                        "BIPED_LOCOMOTION_TERRAIN"}},
        {"BIPED_TO_QUADRUPED", {"QUADRUPED_STAND_DEFAULT",
                                 "QUADRUPED_LOCOMOTION_DEFAULT",
                                 "QUADRUPED_LOCOMOTION_TERRAIN"}},
        {"BIPED_TO_QUADRUPED_FRONTFLIP", {"QUADRUPED_LOCOMOTION_DEFAULT"}},
        {"BIPED_TO_QUADRUPED_ROTATE", {"QUADRUPED_LOCOMOTION_DEFAULT"}},
        {"QUADRUPED_LOCOMOTION_FRONTFLIP", {"QUADRUPED_LOCOMOTION_DEFAULT"}},
        {"QUADRUPED_LOCOMOTION_BACKFLIP", {"QUADRUPED_LOCOMOTION_DEFAULT"}},
        {"QUADRUPED_TO_BIPED_FRONTFLIP", {"BIPED_LOCOMOTION_WBC"}},
        {"BIPED_LOCOMOTION_ROTATE_LOCAL", {"BIPED_LOCOMOTION_WBC"}},
        {"BIPED_LOCOMOTION_BACKBEND", {"BIPED_LOCOMOTION_WBC"}},
        {"BIPED_LOCOMOTION_MOONWALK", {"BIPED_LOCOMOTION_WBC"}},
        {"BIPED_LOCOMOTION_DANCE", {"BIPED_LOCOMOTION_WBC"}},
        {"BIPED_CUSTOM_UPPER", {"BIPED_LOCOMOTION_WBC"}},
    };
    return rules;
  }

  static const std::unordered_set<std::string> & skip_actions()
  {
    static const std::unordered_set<std::string> actions = {
        "PASSIVE_DEFAULT",
        "BIPED_LOCOMOTION_WBC",
        "BIPED_LOCOMOTION_ANIMATION",
        "QUADRUPED_LOCOMOTION_DEFAULT",
        "BIPED_STAND_DEFAULT",
        "QUADRUPED_RECOVERY",
        "QUADRUPED_TO_BIPED_FRONTFLIP",
        "BIPED_TO_QUADRUPED_FRONTFLIP",
        "QUADRUPED_LOCOMOTION_FRONTFLIP",
        "QUADRUPED_LOCOMOTION_BACKFLIP",
        "QUADRUPED_GET_DOWN_DEFAULT",
        "QUADRUPED_SIT_DOWN_DEFAULT",
        "BIPED_TO_QUADRUPED",
        "QUADRUPED_TO_BIPED",
        "QUADRUPED_LOCOMOTION_TERRAIN",
        "QUADRUPED_LOCOMOTION_RUN",
    };
    return actions;
  }

  static const std::unordered_map<std::string, std::string> &
  automatic_next_actions()
  {
    // These commands start a transition runner. MC reports the corresponding
    // bridge action after the runner completes, so do not send the bridge
    // action again before observing it through GetMcAction.
    static const std::unordered_map<std::string, std::string> actions = {
        {"BIPED_STAND_DEFAULT", "BIPED_LOCOMOTION_WBC"},
        {"QUADRUPED_TO_BIPED", "BIPED_LOCOMOTION_WBC"},
        {"QUADRUPED_TO_BIPED_ROTATE", "BIPED_LOCOMOTION_WBC"},
        {"QUADRUPED_TO_BIPED_FRONTFLIP", "BIPED_LOCOMOTION_WBC"},
        {"BIPED_TO_QUADRUPED", "QUADRUPED_LOCOMOTION_DEFAULT"},
        {"BIPED_TO_QUADRUPED_FRONTFLIP", "QUADRUPED_LOCOMOTION_DEFAULT"},
        {"BIPED_TO_QUADRUPED_ROTATE", "QUADRUPED_LOCOMOTION_DEFAULT"},
        {"QUADRUPED_LOCOMOTION_FRONTFLIP", "QUADRUPED_LOCOMOTION_DEFAULT"},
        {"QUADRUPED_LOCOMOTION_BACKFLIP", "QUADRUPED_LOCOMOTION_DEFAULT"},
    };
    return actions;
  }

  static bool is_bridge_action(const std::string & action)
  {
    return action == "QUADRUPED_LOCOMOTION_DEFAULT" ||
           action == "BIPED_LOCOMOTION_WBC";
  }

  static std::vector<PlannedTransition> plan_path(
      const std::string & current_action, const std::string & target_action)
  {
    if (current_action == "DAMPING_DEFAULT" ||
        current_action == "QUAD_LIE_DOWN") {
      if (target_action == "PASSIVE_DEFAULT") {
        return {{"PASSIVE_DEFAULT", "PASSIVE_DEFAULT"}};
      }
      auto path = plan_path("PASSIVE_DEFAULT", target_action);
      if (!path.empty()) {
        path.insert(path.begin(), {"PASSIVE_DEFAULT", "PASSIVE_DEFAULT"});
      }
      return path;
    }

    struct Previous
    {
      std::string action;
      PlannedTransition transition;
    };

    std::deque<std::string> pending;
    std::unordered_set<std::string> visited;
    std::unordered_map<std::string, Previous> previous;
    pending.push_back(current_action);
    visited.insert(current_action);

    while (!pending.empty()) {
      const std::string from = pending.front();
      pending.pop_front();

      std::vector<std::string> candidates;
      const auto rules_it = action_rules().find(from);
      if (rules_it != action_rules().end()) {
        candidates = rules_it->second;
      }

      // These are the action_ruler allow_all_switch_to destinations.
      candidates.push_back("PASSIVE_DEFAULT");
      candidates.push_back("DAMPING_DEFAULT");
      candidates.push_back("QUAD_LIE_DOWN");

      for (const auto & candidate : candidates) {
        PlannedTransition transition;
        std::string arrival;
        if (candidate == target_action) {
          // A skip action is valid when it is the caller's target. In that
          // case SetMcAction must be sent and the target itself must be seen.
          transition = {candidate, candidate};
          arrival = candidate;
        } else {
          const auto automatic_it = automatic_next_actions().find(candidate);
          if (automatic_it != automatic_next_actions().end()) {
            transition = {candidate, automatic_it->second};
            arrival = automatic_it->second;
          } else if (is_bridge_action(candidate) ||
                     skip_actions().count(candidate) == 0U) {
            transition = {candidate, candidate};
            arrival = candidate;
          } else {
            continue;
          }
        }

        if (!visited.insert(arrival).second) {
          continue;
        }
        previous.emplace(arrival, Previous{from, transition});
        if (arrival == target_action) {
          std::vector<PlannedTransition> path;
          std::string cursor = target_action;
          while (cursor != current_action) {
            const auto previous_it = previous.find(cursor);
            if (previous_it == previous.end()) {
              return {};
            }
            path.push_back(previous_it->second.transition);
            cursor = previous_it->second.action;
          }
          std::reverse(path.begin(), path.end());
          return path;
        }
        pending.push_back(arrival);
      }
    }
    return {};
  }

  bool wait_for_services(const McActionSwitchOptions & options,
                         McActionSwitchResult & result) const
  {
    if (is_cancelled(options)) {
      fail(result, McActionSwitchError::kShutdown,
           "Action switch cancelled while waiting for services.");
      return false;
    }
    if (!set_action_client_->wait_for_service(options.service_wait_timeout)) {
      fail(result, McActionSwitchError::kServiceUnavailable,
           "SetMcAction service is unavailable: " + set_action_service_);
      return false;
    }
    if (is_cancelled(options)) {
      fail(result, McActionSwitchError::kShutdown,
           "Action switch cancelled while waiting for services.");
      return false;
    }
    if (!get_action_client_->wait_for_service(options.service_wait_timeout)) {
      fail(result, McActionSwitchError::kServiceUnavailable,
           "GetMcAction service is unavailable: " + get_action_service_);
      return false;
    }
    return true;
  }

  bool get_current_action(const McActionSwitchOptions & options,
                          ActionState & action) const
  {
    constexpr int max_retries = 3;
    for (int attempt = 1; attempt <= max_retries; ++attempt) {
      if (is_cancelled(options)) {
        return false;
      }
      auto request =
          std::make_shared<aimdk_msgs::srv::GetMcAction::Request>();
      request->request = aimdk_msgs::msg::CommonRequest();
      request->request.header.stamp = node_->now();

      auto future = get_action_client_->async_send_request(request);
      const auto return_code = rclcpp::spin_until_future_complete(
          node_, future, options.service_wait_timeout);
      if (return_code != rclcpp::FutureReturnCode::SUCCESS) {
        RCLCPP_WARN(
            node_->get_logger(),
            "GetMcAction attempt %d/%d timed out or was interrupted.",
            attempt, max_retries);
      } else {
        const auto response = future.get();
        if (!response || response->header.code != 0) {
          return false;
        }
        action.action_desc = response->info.action_desc;
        action.status = response->info.status.value;
        return true;
      }

      if (attempt < max_retries) {
        std::this_thread::sleep_for(options.poll_interval);
      }
    }
    return false;
  }

  bool request_action(const std::string & action_desc,
                      const McActionSwitchOptions & options) const
  {
    if (is_cancelled(options)) {
      return false;
    }
    auto request = std::make_shared<aimdk_msgs::srv::SetMcAction::Request>();
    request->header.stamp = node_->now();
    request->source = options.source;
    request->command = aimdk_msgs::msg::McActionCommand();
    request->command.action_desc = action_desc;

    RCLCPP_INFO(node_->get_logger(), "Requesting MC action: %s",
                action_desc.c_str());
    auto future = set_action_client_->async_send_request(request);
    if (rclcpp::spin_until_future_complete(node_, future,
                                           options.service_call_timeout) !=
        rclcpp::FutureReturnCode::SUCCESS) {
      return false;
    }
    const auto response = future.get();
    return response &&
           response->response.status.value == aimdk_msgs::msg::CommonState::SUCCESS;
  }

  bool wait_for_action(
      const std::string & expected_action,
      const McActionSwitchOptions & options,
      const std::chrono::steady_clock::time_point & deadline,
      ActionState & reached) const
  {
    while (rclcpp::ok() && !is_cancelled(options) &&
           std::chrono::steady_clock::now() < deadline) {
      ActionState current;
      if (get_current_action(options, current) &&
          current.action_desc == expected_action && is_active(current)) {
        RCLCPP_INFO(node_->get_logger(), "MC reached action: %s (status=%d)",
                    expected_action.c_str(), current.status);
        reached = current;
        return true;
      }

      const auto now = std::chrono::steady_clock::now();
      if (now >= deadline) {
        break;
      }
      std::this_thread::sleep_for(
          std::min(options.poll_interval,
                   std::chrono::duration_cast<std::chrono::milliseconds>(
                       deadline - now)));
    }
    return false;
  }

  rclcpp::Node::SharedPtr node_;
  std::string set_action_service_;
  std::string get_action_service_;
  rclcpp::Client<aimdk_msgs::srv::SetMcAction>::SharedPtr set_action_client_;
  rclcpp::Client<aimdk_msgs::srv::GetMcAction>::SharedPtr get_action_client_;
};

}  // namespace aimdk_examples

#endif  // AIMDK_EXAMPLES_CPP_MC_ACTION_SWITCHER_HPP_
