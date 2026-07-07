/**
 * T1 Custom Upper Control Example
 *
 * Description:
 *   Demonstrates how to publish upper-body joint commands to
 *   /aima/mc/custom/joint/command with aimdk_msgs/msg/McCustomJointCommand.
 *
 * Prerequisites:
 *   - MC must stay running. Do not disable the robot motion control module.
 *   - Robot must be in a safe environment for motion testing.
 *   - State machine will auto-transition to BIPED_CUSTOM_UPPER before publishing commands.
 *
 * Usage:
 *   ros2 run aimdk_examples_cpp upper_body_control
 */

#include "aimdk_msgs/msg/common_request.hpp"
#include "aimdk_msgs/msg/common_state.hpp"
#include "aimdk_msgs/msg/joint_command.hpp"
#include "aimdk_msgs/msg/joint_state_array.hpp"
#include "aimdk_msgs/msg/mc_action_command.hpp"
#include "aimdk_msgs/msg/mc_action_status.hpp"
#include "aimdk_msgs/msg/mc_custom_joint_command.hpp"
#include "aimdk_msgs/msg/request_header.hpp"
#include "aimdk_msgs/srv/get_mc_action.hpp"
#include "aimdk_msgs/srv/set_mc_action.hpp"
#include "rclcpp/rclcpp.hpp"

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <unordered_map>
#include <utility>
#include <vector>

namespace {

constexpr double kPublishRateHz = 1000.0;
constexpr double kDefaultStiffness = 20.0;
constexpr double kDefaultDamping = 3.0;
constexpr double kPi = 3.14159265358979323846;

const std::array<std::string, 6> kJointNames = {
    "FL_HIP_ROLL_Joint",
    "FL_HIP_PITCH_Joint",
    "FL_KNEE_Joint",
    "FR_HIP_ROLL_Joint",
    "FR_HIP_PITCH_Joint",
    "FR_KNEE_Joint",
};

const std::array<double, 6> kDefaultStandPositions = {
    -0.0031864201078528787,
    1.5550110086252016,
    0.5175245036476512,
    0.003206134017743055,
    1.554158788692746,
    0.5169196042010172,
};

std::atomic_bool g_stop{false};

void signal_handler(int)
{
  g_stop.store(true);
}

double smooth_step(double t)
{
  t = std::clamp(t, 0.0, 1.0);
  return 0.5 - 0.5 * std::cos(kPi * t);
}

}  // namespace

class CustomUpperControlNode : public rclcpp::Node
{
 public:
  CustomUpperControlNode() : Node("custom_upper_control")
  {
    auto qos = rclcpp::QoS(rclcpp::KeepLast(10));
    qos.best_effort();
    qos.durability_volatile();

    command_pub_ =
        this->create_publisher<aimdk_msgs::msg::McCustomJointCommand>(
            "/aima/mc/custom/joint/command", qos);

    set_action_client_ = this->create_client<aimdk_msgs::srv::SetMcAction>(
        "/aimdk_5Fmsgs/srv/SetMcAction");
    get_action_client_ = this->create_client<aimdk_msgs::srv::GetMcAction>(
        "/aimdk_5Fmsgs/srv/GetMcAction");

    initialize_command_message();

    RCLCPP_INFO(
        this->get_logger(),
        "custom_upper_control started. Make sure MC is running. The robot "
        "will auto-transition to BIPED_CUSTOM_UPPER before publishing commands.");
  }

  void start_subscription()
  {
    auto qos = rclcpp::QoS(rclcpp::KeepLast(10));
    qos.best_effort();
    qos.durability_volatile();

    state_sub_ = this->create_subscription<aimdk_msgs::msg::JointStateArray>(
        "/aima/hal/joint/state", qos,
        std::bind(&CustomUpperControlNode::on_joint_state, this,
                  std::placeholders::_1));
  }

  bool has_initial_state() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return initial_state_ready_;
  }

  std::string get_action_desc()
  {
    auto request = std::make_shared<aimdk_msgs::srv::GetMcAction::Request>();
    request->request = aimdk_msgs::msg::CommonRequest();
    request->request.header.stamp = this->now();

    auto future = get_action_client_->async_send_request(request);
    if (rclcpp::spin_until_future_complete(this->shared_from_this(), future,
                                           std::chrono::seconds(2)) !=
        rclcpp::FutureReturnCode::SUCCESS) {
      return "";
    }

    auto response = future.get();
    if (!response) {
      return "";
    }
    if (response->info.status.value != aimdk_msgs::msg::McActionStatus::RUNNING) {
      return "";
    }
    return response->info.action_desc;
  }

  bool set_action(const std::string &action_desc)
  {
    auto request = std::make_shared<aimdk_msgs::srv::SetMcAction::Request>();
    request->header.stamp = this->now();
    request->source = "node";
    request->command = aimdk_msgs::msg::McActionCommand();
    request->command.action_desc = action_desc;

    RCLCPP_INFO(this->get_logger(), "Requesting state switch to: %s",
                action_desc.c_str());

    auto future = set_action_client_->async_send_request(request);
    if (rclcpp::spin_until_future_complete(this->shared_from_this(), future,
                                           std::chrono::seconds(2)) !=
        rclcpp::FutureReturnCode::SUCCESS) {
      return false;
    }
    return future.get() != nullptr;
  }

  bool wait_for_action(const std::string &action_desc,
                       double timeout_sec = 20.0)
  {
    const auto deadline =
        std::chrono::steady_clock::now() +
        std::chrono::duration_cast<std::chrono::steady_clock::duration>(
            std::chrono::duration<double>(timeout_sec));

    while (rclcpp::ok() && !g_stop.load() &&
           std::chrono::steady_clock::now() < deadline) {
      if (get_action_desc() == action_desc) {
        RCLCPP_INFO(this->get_logger(), "Robot reached state: %s",
                    action_desc.c_str());
        return true;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }

    RCLCPP_ERROR(this->get_logger(), "Timeout waiting for state: %s",
                 action_desc.c_str());
    return false;
  }

  bool switch_to_custom_upper()
  {
    if (!set_action_client_->wait_for_service(std::chrono::seconds(5))) {
      RCLCPP_ERROR(this->get_logger(), "SetMcAction service is not available.");
      return false;
    }
    if (!get_action_client_->wait_for_service(std::chrono::seconds(5))) {
      RCLCPP_ERROR(this->get_logger(), "GetMcAction service is not available.");
      return false;
    }

    std::string current_action = get_action_desc();
    if (current_action == "BIPED_CUSTOM_UPPER") {
      return true;
    }

    RCLCPP_INFO(this->get_logger(),
                "Current state is %s. Switching to BIPED_CUSTOM_UPPER.",
                current_action.empty() ? "(unknown)" : current_action.c_str());

    static const std::vector<std::string> sequence = {
        "PASSIVE_DEFAULT",
        "QUADRUPED_LOCOMOTION_DEFAULT",
        "BIPED_LOCOMOTION_WBC",
        "BIPED_CUSTOM_UPPER",
    };

    size_t start_index = 0;
    if (current_action == "PASSIVE_DEFAULT") {
      start_index = 1;
    } else if (current_action == "QUADRUPED_LOCOMOTION_DEFAULT") {
      start_index = 2;
    } else if (current_action == "BIPED_LOCOMOTION_WBC") {
      start_index = 3;
    } else if (current_action == "DAMPING_DEFAULT" ||
               current_action == "STORE_DEFAULT") {
      start_index = 0;
    } else {
      start_index = 3;
    }

    for (size_t i = start_index; i < sequence.size(); ++i) {
      const std::string &action_desc = sequence[i];
      const std::string &wait_desc =
          (action_desc == "QUADRUPED_TO_BIPED") ? "BIPED_LOCOMOTION_WBC"
                                                 : action_desc;
      if (!set_action(action_desc) || !wait_for_action(wait_desc)) {
        return false;
      }
      std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    return true;
  }

  // ── Demo logic ──

  bool run_demo()
  {
    const auto initial = initial_positions();
    const auto stand = default_stand_positions();

    RCLCPP_INFO(this->get_logger(),
                "Moving from current posture to default upper posture.");
    if (!publish_interpolation(initial, stand, 2.0)) {
      return false;
    }

    const auto wave_start = wave_pose(stand, 0.0);
    RCLCPP_INFO(this->get_logger(), "Extending right upper limb smoothly.");
    if (!publish_interpolation(stand, wave_start, 2.0)) {
      return false;
    }

    RCLCPP_INFO(this->get_logger(),
                "Running a small upper-body demo motion.");
    if (!publish_wave_motion(stand, 4.0)) {
      return false;
    }

    RCLCPP_INFO(this->get_logger(),
                "Returning to default upper posture.");
    if (!publish_interpolation(current_demo_pose_, stand, 2.0)) {
      return false;
    }

    RCLCPP_INFO(this->get_logger(),
                "Holding default upper posture before exit.");
    return publish_hold(stand, 1.0);
  }

  void publish_default_for_shutdown()
  {
    publish_hold(default_stand_positions(), 1.0);
  }

 private:
  using Pose = std::map<std::string, double>;

  void on_joint_state(const aimdk_msgs::msg::JointStateArray::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (initial_state_ready_) {
      return;
    }

    std::unordered_map<std::string, double> state_by_name;
    for (const auto &joint : msg->joints) {
      state_by_name[joint.name] = joint.position;
    }

    Pose captured;
    for (const auto &name : kJointNames) {
      auto it = state_by_name.find(name);
      if (it == state_by_name.end()) {
        return;
      }
      captured[name] = it->second;
    }

    initial_positions_ = std::move(captured);
    initial_state_ready_ = true;
    RCLCPP_INFO(this->get_logger(),
                "Received initial state for all custom upper joints.");
  }

  Pose initial_positions() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return initial_positions_;
  }

  Pose default_stand_positions() const
  {
    Pose pose;
    for (size_t i = 0; i < kJointNames.size(); ++i) {
      pose[kJointNames[i]] = kDefaultStandPositions[i];
    }
    return pose;
  }

  Pose interpolate_pose(const Pose &from, const Pose &to, double alpha) const
  {
    Pose pose;
    const double s = smooth_step(alpha);
    for (const auto &name : kJointNames) {
      const double start = from.at(name);
      const double target = to.at(name);
      pose[name] = start + (target - start) * s;
    }
    return pose;
  }

  Pose wave_pose(const Pose &base, double t) const
  {
    Pose pose = base;
    pose["FR_HIP_ROLL_Joint"] = -0.18;
    pose["FR_HIP_PITCH_Joint"] = 1.9;
    pose["FR_KNEE_Joint"] = 0.80 + 0.18 * std::sin(2.0 * kPi * 1.5 * t);
    return pose;
  }

  bool publish_interpolation(const Pose &from, const Pose &to,
                             double duration_s)
  {
    const int total_steps =
        std::max(1, static_cast<int>(duration_s * kPublishRateHz));
    auto next_tick = std::chrono::steady_clock::now();
    for (int step = 0; rclcpp::ok() && !g_stop.load() && step <= total_steps;
         ++step) {
      const double alpha =
          static_cast<double>(step) / static_cast<double>(total_steps);
      current_demo_pose_ = interpolate_pose(from, to, alpha);
      publish_pose(current_demo_pose_);
      next_tick = sleep_to_next_tick(next_tick);
    }
    return rclcpp::ok() && !g_stop.load();
  }

  bool publish_wave_motion(const Pose &base, double duration_s)
  {
    const int total_steps =
        std::max(1, static_cast<int>(duration_s * kPublishRateHz));
    auto next_tick = std::chrono::steady_clock::now();
    for (int step = 0; rclcpp::ok() && !g_stop.load() && step <= total_steps;
         ++step) {
      const double t =
          static_cast<double>(step) / static_cast<double>(total_steps);
      current_demo_pose_ = wave_pose(base, t);
      publish_pose(current_demo_pose_);
      next_tick = sleep_to_next_tick(next_tick);
    }
    return rclcpp::ok() && !g_stop.load();
  }

  bool publish_hold(const Pose &pose, double duration_s)
  {
    const int total_steps =
        std::max(1, static_cast<int>(duration_s * kPublishRateHz));
    auto next_tick = std::chrono::steady_clock::now();
    for (int step = 0; rclcpp::ok() && !g_stop.load() && step < total_steps;
         ++step) {
      publish_pose(pose);
      next_tick = sleep_to_next_tick(next_tick);
    }
    return rclcpp::ok();
  }

  std::chrono::steady_clock::time_point sleep_to_next_tick(
      std::chrono::steady_clock::time_point next_tick)
  {
    const auto now = std::chrono::steady_clock::now();
    if (next_tick > now) {
      std::this_thread::sleep_until(next_tick);
    }
    return next_tick +
           std::chrono::duration_cast<std::chrono::steady_clock::duration>(
               std::chrono::duration<double>(1.0 / kPublishRateHz));
  }

  void initialize_command_message()
  {
    command_msg_.joints.resize(kJointNames.size());

    for (size_t i = 0; i < kJointNames.size(); ++i) {
      auto &joint = command_msg_.joints[i];
      joint.name = kJointNames[i];
      joint.position = 0.0;
      joint.velocity = 0.0;
      joint.effort = 0.0;
      joint.stiffness = kDefaultStiffness;
      joint.damping = kDefaultDamping;
    }
  }

  void publish_pose(const Pose &pose)
  {
    command_msg_.header.stamp = this->now();
    command_msg_.header.sequence = sequence_++;

    for (size_t i = 0; i < kJointNames.size(); ++i) {
      command_msg_.joints[i].position = pose.at(kJointNames[i]);
    }

    command_pub_->publish(command_msg_);
  }

  rclcpp::Publisher<aimdk_msgs::msg::McCustomJointCommand>::SharedPtr command_pub_;
  rclcpp::Subscription<aimdk_msgs::msg::JointStateArray>::SharedPtr state_sub_;
  rclcpp::Client<aimdk_msgs::srv::SetMcAction>::SharedPtr set_action_client_;
  rclcpp::Client<aimdk_msgs::srv::GetMcAction>::SharedPtr get_action_client_;
  aimdk_msgs::msg::McCustomJointCommand command_msg_;

  mutable std::mutex mutex_;
  bool initial_state_ready_ = false;
  Pose initial_positions_;
  Pose current_demo_pose_;
  uint32_t sequence_ = 0;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  std::signal(SIGINT, signal_handler);
  std::signal(SIGTERM, signal_handler);

  auto node = std::make_shared<CustomUpperControlNode>();
  int ret = 0;

  try {
    if (!node->switch_to_custom_upper()) {
      ret = 1;
    } else {
      std::this_thread::sleep_for(std::chrono::seconds(1));
      node->start_subscription();

      const auto deadline =
          std::chrono::steady_clock::now() + std::chrono::seconds(10);
      while (rclcpp::ok() && !g_stop.load() && !node->has_initial_state() &&
             std::chrono::steady_clock::now() < deadline) {
        rclcpp::spin_some(node);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
      }

      if (!node->has_initial_state()) {
        RCLCPP_ERROR(
            node->get_logger(),
            "Timed out waiting for /aima/hal/joint/state with all custom "
            "upper joints.");
        ret = 1;
      } else if (!g_stop.load()) {
        if (!node->run_demo()) {
          ret = g_stop.load() ? 0 : 1;
        }
      }

      if (rclcpp::ok() && node->has_initial_state()) {
        node->publish_default_for_shutdown();
      }
    }
  } catch (const std::exception &e) {
    RCLCPP_ERROR(node->get_logger(), "Exception: %s", e.what());
    ret = 1;
  }

  node.reset();
  if (rclcpp::ok()) {
    rclcpp::shutdown();
  }
  return ret;
}
