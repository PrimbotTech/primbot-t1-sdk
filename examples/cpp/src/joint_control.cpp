// 注意：在使用本脚本前，需首先关闭机器人本体运控模块，否则可能出现不可预期的异常行为！
// Prerequisites:
//     1. Build the SDK: `colcon build`
//     2. Source environment: `source install/setup.bash`
//
// Before running the following command, place the robot flat on the ground.
// The following example lifts the left arm and right leg.

// Example:
//     ros2 run aimdk_examples_cpp joint_control --ros-args \
//       -p joint_names:="['FL_HIP_PITCH_Joint']" \
//       -p target_positions:="[2.0]" \
//       -p stiffness:="[80.0]" \
//       -p damping:="[0.5]" \
//       -p default_stiffness:=80.0 \
//       -p default_damping:=0.5
//
// Notes:
//   - joint_names and target_positions must have the same length.
//   - stiffness and damping are optional. If omitted, default_stiffness and
//     default_damping are used for every joint.
//   - The node waits for /aima/hal/joint/state, then uses Ruckig to publish
//     trajectory points to /aima/hal/joint/command until the target is reached.
#include <ruckig/ruckig.hpp>
#include "aimdk_msgs/msg/joint_command_array.hpp"
#include "aimdk_msgs/msg/joint_state_array.hpp"
#include "rclcpp/rclcpp.hpp"

#include <signal.h>
#include <algorithm>
#include <chrono>
#include <cstdint>
#include <functional>
#include <iomanip>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <vector>

using DynamicRuckig = ruckig::Ruckig<ruckig::DynamicDOFs>;
using DynamicInput  = ruckig::InputParameter<ruckig::DynamicDOFs>;
using DynamicOutput = ruckig::OutputParameter<ruckig::DynamicDOFs>;
using SteadyClock   = std::chrono::steady_clock;

std::shared_ptr<rclcpp::Node> g_node = nullptr;

void signal_handler(int signal)
{
  if (g_node) {
    RCLCPP_INFO(g_node->get_logger(), "Received signal %d, shutting down joint_control...", signal);
    g_node.reset();
  }
  rclcpp::shutdown();
  exit(signal);
}

class JointControlNode : public rclcpp::Node
{
 public:
  JointControlNode() : Node("joint_control")
  {
    const std::vector<std::string> empty_joint_names;
    const std::vector<double> empty_doubles;
    joint_names_ =
      this->declare_parameter<std::vector<std::string>>("joint_names", empty_joint_names);
    target_positions_ =
      this->declare_parameter<std::vector<double>>("target_positions", empty_doubles);
    stiffness_ = this->declare_parameter<std::vector<double>>("stiffness", empty_doubles);
    damping_ =
      this->declare_parameter<std::vector<double>>("damping", empty_doubles);
    default_stiffness_ =
      this->declare_parameter<double>("default_stiffness", 20.0);
    default_damping_ =
      this->declare_parameter<double>("default_damping", 2.0);
    max_velocity_ = this->declare_parameter<double>("max_velocity", 3.0);
    max_acceleration_ =
      this->declare_parameter<double>("max_acceleration", 10.0);
    max_jerk_ = this->declare_parameter<double>("max_jerk", 25.0);
    control_period_s_ =
      this->declare_parameter<double>("control_period_s", 0.002);
    state_timeout_s_ =
      this->declare_parameter<double>("state_timeout_s", 10.0);
    publish_full_command_ =
      this->declare_parameter<bool>("publish_full_command", true);

    validate_parameters();

    dofs_ = joint_names_.size();
    fill_optional_gains();

    ruckig_ = std::make_unique<DynamicRuckig>(dofs_, control_period_s_);
    input_  = std::make_unique<DynamicInput>(dofs_);
    output_ = std::make_unique<DynamicOutput>(dofs_);

    input_->max_velocity         = std::vector<double>(dofs_, max_velocity_);
    input_->max_acceleration     = std::vector<double>(dofs_, max_acceleration_);
    input_->max_jerk             = std::vector<double>(dofs_, max_jerk_);
    input_->current_acceleration = std::vector<double>(dofs_, 0.0);
    input_->target_velocity      = std::vector<double>(dofs_, 0.0);
    input_->target_acceleration  = std::vector<double>(dofs_, 0.0);

    auto qos = rclcpp::QoS(rclcpp::KeepLast(10));
    qos.best_effort();
    qos.durability_volatile();

    command_pub_ = this->create_publisher<aimdk_msgs::msg::JointCommandArray>(
      "/aima/hal/joint/command", qos);
    state_sub_ = this->create_subscription<aimdk_msgs::msg::JointStateArray>(
      "/aima/hal/joint/state", qos,
      std::bind(&JointControlNode::on_joint_state, this, std::placeholders::_1));
    // Timer to print joint state every 10 seconds
    print_timer_ = this->create_wall_timer(
      std::chrono::seconds(10),
      std::bind(&JointControlNode::print_joint_state, this));

    wait_started_at_     = SteadyClock::now();
    state_timeout_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(100),
      std::bind(&JointControlNode::check_state_timeout, this));

    RCLCPP_INFO(this->get_logger(),
                "Waiting for /aima/hal/joint/state, joints=%s, targets=%s, "
                "control_period_s=%.3f, state_timeout_s=%.3f, "
                "publish_full_command=%s",
                format_strings(joint_names_).c_str(), format_doubles(target_positions_).c_str(), control_period_s_, state_timeout_s_, publish_full_command_ ? "true" : "false");
  }

 private:
  void validate_parameters()
  {
    if (joint_names_.empty()) {
      throw std::runtime_error("Parameter 'joint_names' must not be empty.");
    }
    if (target_positions_.empty()) {
      throw std::runtime_error(
        "Parameter 'target_positions' must not be empty.");
    }
    if (joint_names_.size() != target_positions_.size()) {
      throw std::runtime_error(
        "Parameter 'joint_names' and 'target_positions' must have the same "
        "length.");
    }
    if (!stiffness_.empty() && stiffness_.size() != joint_names_.size()) {
      throw std::runtime_error(
        "Parameter 'stiffness' must be empty or match joint_names length.");
    }
    if (!damping_.empty() && damping_.size() != joint_names_.size()) {
      throw std::runtime_error(
        "Parameter 'damping' must be empty or match joint_names length.");
    }
    if (control_period_s_ <= 0.0) {
      throw std::runtime_error(
        "Parameter 'control_period_s' must be greater than zero.");
    }
    if (state_timeout_s_ <= 0.0) {
      throw std::runtime_error(
        "Parameter 'state_timeout_s' must be greater than zero.");
    }
    if (max_velocity_ <= 0.0 || max_acceleration_ <= 0.0 || max_jerk_ <= 0.0) {
      throw std::runtime_error(
        "Motion limits 'max_velocity', 'max_acceleration', and 'max_jerk' "
        "must be greater than zero.");
    }
  }

  void fill_optional_gains()
  {
    if (stiffness_.empty()) {
      stiffness_ = std::vector<double>(dofs_, default_stiffness_);
    }
    if (damping_.empty()) {
      damping_ = std::vector<double>(dofs_, default_damping_);
    }
  }

  void on_joint_state(const aimdk_msgs::msg::JointStateArray::SharedPtr msg)
  {
    latest_state_ = msg;
    if (trajectory_started_ || finished_) {
      return;
    }

    start_trajectory();
  }

  void print_joint_state()
  {
    if (!latest_state_) {
      RCLCPP_WARN(this->get_logger(), "No joint state received yet – waiting for /aima/hal/joint/state.");
      return;
    }
    std::vector<std::string> parts;
    for (const auto &joint : latest_state_->joints) {
      std::ostringstream oss;
      oss << joint.name << ": pos=" << std::fixed << std::setprecision(3) << joint.position
          << ", vel=" << joint.velocity;
      parts.push_back(oss.str());
    }
    std::string msg = "Current joint state (every 10 s): ";
    for (size_t i = 0; i < parts.size(); ++i) {
      if (i > 0) msg += ", ";
      msg += parts[i];
    }
    RCLCPP_INFO(this->get_logger(), "%s", msg.c_str());
  }

  void start_trajectory()
  {
    if (!latest_state_) {
      return;
    }

    std::unordered_map<std::string, size_t> state_index_by_name;
    state_index_by_name.reserve(latest_state_->joints.size());
    for (size_t i = 0; i < latest_state_->joints.size(); ++i) {
      state_index_by_name[latest_state_->joints[i].name] = i;
    }

    std::vector<std::string> missing_state_joints;
    std::vector<double> current_positions(dofs_, 0.0);
    std::vector<double> current_velocities(dofs_, 0.0);
    target_full_indices_.assign(dofs_, 0);

    for (size_t i = 0; i < dofs_; ++i) {
      const auto state_it = state_index_by_name.find(joint_names_[i]);
      if (state_it == state_index_by_name.end()) {
        missing_state_joints.push_back(joint_names_[i]);
        continue;
      }

      const auto &joint_state = latest_state_->joints[state_it->second];
      target_full_indices_[i] = state_it->second;
      current_positions[i]    = joint_state.position;
      current_velocities[i]   = joint_state.velocity;
    }

    if (!missing_state_joints.empty()) {
      RCLCPP_ERROR(this->get_logger(), "Missing joints in /aima/hal/joint/state: %s", format_strings(missing_state_joints).c_str());
      shutdown_with_error();
      return;
    }

    input_->current_position     = current_positions;
    input_->current_velocity     = current_velocities;
    input_->current_acceleration = std::vector<double>(dofs_, 0.0);
    input_->target_position      = target_positions_;
    input_->target_velocity      = std::vector<double>(dofs_, 0.0);
    input_->target_acceleration  = std::vector<double>(dofs_, 0.0);

    control_started_at_ = SteadyClock::now();
    trajectory_started_ = true;

    if (state_timeout_timer_) {
      state_timeout_timer_->cancel();
    }

    control_timer_ = this->create_wall_timer(
      to_timer_period(control_period_s_),
      std::bind(&JointControlNode::on_control_timer, this));

    RCLCPP_INFO(this->get_logger(),
                "Trajectory initialized from current=%s to target=%s, "
                "stiffness=%s, damping=%s",
                format_doubles(current_positions).c_str(), format_doubles(target_positions_).c_str(), format_doubles(stiffness_).c_str(), format_doubles(damping_).c_str());
  }

  void on_control_timer()
  {
    if (finished_) {
      return;
    }

    const auto cycle_start = SteadyClock::now();
    const auto result      = ruckig_->update(*input_, *output_);

    if (result != ruckig::Result::Working && result != ruckig::Result::Finished) {
      RCLCPP_ERROR(this->get_logger(), "Ruckig trajectory planning failed with result=%d", static_cast<int>(result));
      shutdown_with_error();
      return;
    }

    publish_command(output_->new_position);
    output_->pass_to_input(*input_);

    const auto cycle_end = SteadyClock::now();
    const double cycle_cost_s =
      std::chrono::duration<double>(cycle_end - cycle_start).count();
    ++cycle_count_;
    total_cycle_cost_s_ += cycle_cost_s;
    max_cycle_cost_s_ = std::max(max_cycle_cost_s_, cycle_cost_s);
    if (cycle_cost_s > control_period_s_) {
      const double overrun_s = cycle_cost_s - control_period_s_;
      ++overrun_count_;
      total_overrun_s_ += overrun_s;
      max_overrun_s_ = std::max(max_overrun_s_, overrun_s);
    }

    if (result == ruckig::Result::Finished) {
      publish_command(target_positions_);
      log_stats_and_shutdown();
    }
  }

  void publish_command(const std::vector<double> &positions)
  {
    aimdk_msgs::msg::JointCommandArray cmd;
    cmd.header.stamp    = this->now();
    cmd.header.sequence = sequence_++;
    if (latest_state_) {
      cmd.header.frame_id   = latest_state_->header.frame_id;
      cmd.header.meas_stamp = latest_state_->header.meas_stamp;
    }

    if (publish_full_command_ && latest_state_ && target_full_indices_.size() == dofs_) {
      cmd.joints.resize(latest_state_->joints.size());
      for (size_t i = 0; i < latest_state_->joints.size(); ++i) {
        auto &joint             = cmd.joints[i];
        const auto &state_joint = latest_state_->joints[i];
        joint.name              = state_joint.name;
        joint.position          = state_joint.position;
        joint.velocity          = 0.0;
        joint.effort            = 0.0;
        joint.stiffness         = default_stiffness_;
        joint.damping           = default_damping_;
      }

      for (size_t i = 0; i < dofs_; ++i) {
        const size_t full_index = target_full_indices_[i];
        if (full_index >= cmd.joints.size()) {
          continue;
        }

        auto &joint     = cmd.joints[full_index];
        joint.position  = positions[i];
        joint.stiffness = stiffness_[i];
        joint.damping   = damping_[i];
      }
    } else {
      cmd.joints.resize(dofs_);
      for (size_t i = 0; i < dofs_; ++i) {
        auto &joint     = cmd.joints[i];
        joint.name      = joint_names_[i];
        joint.position  = positions[i];
        joint.velocity  = 0.0;
        joint.effort    = 0.0;
        joint.stiffness = stiffness_[i];
        joint.damping   = damping_[i];
      }
    }

    command_pub_->publish(cmd);
  }

  void check_state_timeout()
  {
    if (trajectory_started_ || finished_) {
      return;
    }

    const double wait_s =
      std::chrono::duration<double>(SteadyClock::now() - wait_started_at_)
        .count();
    if (wait_s > state_timeout_s_) {
      RCLCPP_ERROR(this->get_logger(), "Timed out after %.3f s waiting for /aima/hal/joint/state", wait_s);
      shutdown_with_error();
    }
  }

  void shutdown_with_error()
  {
    if (finished_) {
      return;
    }

    finished_ = true;
    if (control_timer_) {
      control_timer_->cancel();
    }
    if (state_timeout_timer_) {
      state_timeout_timer_->cancel();
    }
    rclcpp::shutdown();
  }

  void log_stats_and_shutdown()
  {
    if (finished_) {
      return;
    }

    finished_ = true;
    if (control_timer_) {
      control_timer_->cancel();
    }
    if (state_timeout_timer_) {
      state_timeout_timer_->cancel();
    }

    const double runtime_s =
      std::chrono::duration<double>(SteadyClock::now() - control_started_at_)
        .count();
    const double actual_hz =
      runtime_s > 0.0 ? static_cast<double>(cycle_count_) / runtime_s : 0.0;
    const double avg_cycle_ms =
      cycle_count_ > 0
        ? (total_cycle_cost_s_ / static_cast<double>(cycle_count_)) *
            1000.0
        : 0.0;
    const double avg_overrun_ms =
      overrun_count_ > 0
        ? (total_overrun_s_ / static_cast<double>(overrun_count_)) * 1000.0
        : 0.0;
    const double trajectory_duration_s =
      output_ ? output_->trajectory.get_duration() : 0.0;

    RCLCPP_INFO(
      this->get_logger(),
      "Trajectory completed. planned_duration=%.6f s, runtime=%.6f s, "
      "cycles=%zu, actual_hz=%.2f, avg_cycle_ms=%.3f, max_cycle_ms=%.3f, "
      "overruns=%zu, avg_overrun_ms=%.3f, max_overrun_ms=%.3f",
      trajectory_duration_s, runtime_s, cycle_count_, actual_hz, avg_cycle_ms,
      max_cycle_cost_s_ * 1000.0, overrun_count_, avg_overrun_ms,
      max_overrun_s_ * 1000.0);
    rclcpp::shutdown();
  }

  static std::chrono::nanoseconds to_timer_period(double seconds)
  {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(seconds));
  }

  static std::string format_strings(const std::vector<std::string> &values)
  {
    std::ostringstream oss;
    oss << "[";
    for (size_t i = 0; i < values.size(); ++i) {
      if (i > 0) {
        oss << ", ";
      }
      oss << values[i];
    }
    oss << "]";
    return oss.str();
  }

  static std::string format_doubles(const std::vector<double> &values)
  {
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(6) << "[";
    for (size_t i = 0; i < values.size(); ++i) {
      if (i > 0) {
        oss << ", ";
      }
      oss << values[i];
    }
    oss << "]";
    return oss.str();
  }

  std::vector<std::string> joint_names_;
  std::vector<double> target_positions_;
  std::vector<double> stiffness_;
  std::vector<double> damping_;

  double default_stiffness_{20.0};
  double default_damping_{2.0};
  double max_velocity_{3.0};
  double max_acceleration_{10.0};
  double max_jerk_{25.0};
  double control_period_s_{0.002};
  double state_timeout_s_{5.0};
  bool publish_full_command_{true};
  size_t dofs_{0};
  std::vector<size_t> target_full_indices_;

  bool trajectory_started_{false};
  bool finished_{false};
  uint32_t sequence_{0};

  size_t cycle_count_{0};
  size_t overrun_count_{0};
  double total_cycle_cost_s_{0.0};
  double max_cycle_cost_s_{0.0};
  double total_overrun_s_{0.0};
  double max_overrun_s_{0.0};

  SteadyClock::time_point wait_started_at_;
  SteadyClock::time_point control_started_at_;

  std::unique_ptr<DynamicRuckig> ruckig_;
  std::unique_ptr<DynamicInput> input_;
  std::unique_ptr<DynamicOutput> output_;

  aimdk_msgs::msg::JointStateArray::SharedPtr latest_state_;
  rclcpp::Publisher<aimdk_msgs::msg::JointCommandArray>::SharedPtr command_pub_;
  rclcpp::Subscription<aimdk_msgs::msg::JointStateArray>::SharedPtr state_sub_;
  rclcpp::TimerBase::SharedPtr state_timeout_timer_;
  rclcpp::TimerBase::SharedPtr control_timer_;
  rclcpp::TimerBase::SharedPtr print_timer_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  signal(SIGINT, signal_handler);
  signal(SIGTERM, signal_handler);

  try {
    auto node = std::make_shared<JointControlNode>();
    g_node    = node;
    rclcpp::spin(node);
    g_node.reset();
  } catch (const std::exception &e) {
    RCLCPP_ERROR(rclcpp::get_logger("joint_control"), "Failed to start joint_control: %s", e.what());
    if (rclcpp::ok()) {
      rclcpp::shutdown();
    }
    return 1;
  }

  if (rclcpp::ok()) {
    rclcpp::shutdown();
  }
  return 0;
}
