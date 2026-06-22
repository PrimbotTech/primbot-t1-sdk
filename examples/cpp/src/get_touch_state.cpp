#include "aimdk_msgs/msg/touch_state.hpp"
#include "rclcpp/rclcpp.hpp"

#include <iomanip>
#include <sstream>

class TouchStateEcho : public rclcpp::Node {
public:
  TouchStateEcho() : Node("get_touch_state") {
    auto qos = rclcpp::SensorDataQoS();
    sub_ = this->create_subscription<aimdk_msgs::msg::TouchState>(
        "/aima/hal/touch/state", qos,
        std::bind(&TouchStateEcho::callback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "Subscribing touch topic: %s",
                "/aima/hal/touch/state");
  }

private:
  void callback(const aimdk_msgs::msg::TouchState::SharedPtr msg) {
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(6);
    // event_type is the primary field of interest.
    oss << "TouchState received\n"
        << "  frame_id: " << msg->header.frame_id << "\n"
        << "  sequence: " << msg->header.sequence << "\n"
        << "  stamp:    " << rclcpp::Time(msg->header.stamp).seconds() << " s\n"
        << "  meas:     " << rclcpp::Time(msg->header.meas_stamp).seconds() << " s\n"
        << "  sensor:   " << static_cast<int>(msg->sensor_id) << "\n"
        << "  event:    " << static_cast<int>(msg->event_type);

    // Throttle logs to avoid flooding the console.
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500, "%s",
                         oss.str().c_str());
  }
  rclcpp::Subscription<aimdk_msgs::msg::TouchState>::SharedPtr sub_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TouchStateEcho>());
  rclcpp::shutdown();
  return 0;
}
