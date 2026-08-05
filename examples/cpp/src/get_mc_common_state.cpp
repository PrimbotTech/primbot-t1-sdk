/*
 MC Common State Monitor Example Script

 Description:
   This script demonstrates how to subscribe to the robot's MC common state topic.
   It displays real-time motion control status including input source, action info,
   form state, FSM state, posture, motion status, and more.

 Prerequisites:
   - Robot MC service must be running

 Usage:
   ros2 run aimdk_examples_cpp get_mc_common_state

 Example:
   ros2 run aimdk_examples_cpp get_mc_common_state

 Parameters:
   - None
 */

#include "aimdk_msgs/msg/mc_common_state.hpp"
#include "rclcpp/rclcpp.hpp"

#include <iomanip>
#include <sstream>
#include <unordered_map>

class McCommonStateEcho : public rclcpp::Node {
public:
  McCommonStateEcho() : Node("get_mc_common_state") {
    auto qos = rclcpp::SensorDataQoS();
    sub_ = this->create_subscription<aimdk_msgs::msg::McCommonState>(
        "/aima/mc/base/state", qos,
        std::bind(&McCommonStateEcho::callback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "Subscribing MC common state topic: %s",
                "/aima/mc/base/state");
  }

private:
  static const char* form_state_str(uint8_t val) {
    static const std::unordered_map<uint8_t, const char*> map = {
        {0, "UNKNOWN"}, {1, "QUADRUPEDAL"}, {2, "BIPEDAL"}};
    auto it = map.find(val);
    return it != map.end() ? it->second : "UNKNOWN";
  }

  static const char* fsm_state_str(uint32_t val) {
    static const std::unordered_map<uint32_t, const char*> map = {
        {0, "UNKNOWN"}, {1, "STARTING"}, {2, "STABLE"},
        {3, "MOVING"},   {4, "SAFE"},    {5, "SPECIAL"},
        {6, "TEST"}};
    auto it = map.find(val);
    return it != map.end() ? it->second : "UNKNOWN";
  }

  static const char* player_state_str(uint32_t val) {
    static const std::unordered_map<uint32_t, const char*> map = {
        {0, "IDLE"},
        {1, "PRE_PLAYING"},
        {2, "PLAYING"},
        {3, "INTERRUPTING"},
        {4, "ERROR"}};
    auto it = map.find(val);
    return it != map.end() ? it->second : "UNKNOWN";
  }

  static const char* motion_type_str(int32_t val) {
    static const std::unordered_map<int32_t, const char*> map = {
        {0, "NONE"},
        {1, "ANIMATION"},
        {2, "MIMIC"},
        {3, "FOUNDATION"},
        {4, "MIMIC_OPENSOURCE"}};
    auto it = map.find(val);
    return it != map.end() ? it->second : "NONE";
  }

  static const char* intr_type_str(int32_t val) {
    static const std::unordered_map<int32_t, const char*> map = {
        {0, "INTR_NO"}, {1, "INTR_FREE"}, {2, "INTR_BREAKPOINT"}};
    auto it = map.find(val);
    return it != map.end() ? it->second : "INTR_NO";
  }

  void callback(const aimdk_msgs::msg::McCommonState::SharedPtr msg) {
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(4);

    oss << "======== McCommonState ========\n";

    // Input source
    oss << "  [InputSource]\n"
        << "    name:          " << msg->input_source.name << "\n"
        << "    priority:      " << msg->input_source.priority << "\n"
        << "    timeout:       " << msg->input_source.timeout << " ms\n";

    // Action info
    oss << "  [ActionInfo]\n"
        << "    action:        " << msg->action_info.current_action.value << "\n"
        << "    desc:          " << msg->action_info.action_desc << "\n"
        << "    status:        " << msg->action_info.status.value << "\n"
        << "    switch_ready:  " << (msg->action_info.switch_ready ? "true" : "false") << "\n";

    // Form state
    oss << "  [FormState]\n"
        << "    form:          " << form_state_str(msg->form_state.current_form) << "\n";

    // FSM state
    oss << "  [FsmState]\n"
        << "    state:         " << fsm_state_str(msg->fsm_state.current_state) << "\n";

    // Posture
    oss << "  [Posture]\n"
        << "    roll:          " << msg->posture.roll << " rad\n"
        << "    pitch:         " << msg->posture.pitch << " rad\n"
        << "    yaw:           " << msg->posture.yaw << " rad\n";

    // Motion flags
    oss << "  is_moving:       " << (msg->is_moving ? "true" : "false") << "\n"
        << "  is_fallen:       " << (msg->is_fallen ? "true" : "false") << "\n";

    // Motion status
    oss << "  [MotionStatus]\n"
        << "    player:        " << player_state_str(msg->motion_status.player_state.value) << "\n"
        << "    motion:        " << msg->motion_status.motion << "\n"
        << "    type:          " << motion_type_str(msg->motion_status.type.value) << "\n"
        << "    intr_type:     " << intr_type_str(msg->motion_status.intr_info.intr_type.value) << "\n"
        << "    steps:         " << msg->motion_status.current_steps << "\n";

    // Lift state
    oss << "  [LiftState]\n"
        << "    is_lifted:     " << (msg->lift_state.is_lifted ? "true" : "false") << "\n"
        << "    probability:   " << msg->lift_state.probability << "\n"
        << "    threshold:     " << msg->lift_state.threshold << "\n";

    // Task info
    oss << "  [TaskInfo]\n"
        << "    task_id:       " << msg->task_info.task_id << "\n";

    RCLCPP_INFO(this->get_logger(), "%s", oss.str().c_str());
  }

  rclcpp::Subscription<aimdk_msgs::msg::McCommonState>::SharedPtr sub_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<McCommonStateEcho>());
  rclcpp::shutdown();
  return 0;
}
