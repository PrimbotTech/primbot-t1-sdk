/*
 ASR Result Monitor Example Script

 Description:
   This script demonstrates how to subscribe to the robot's ASR (Automatic Speech Recognition) result topic.
   It displays real-time speech-to-text results including recognized text, confidence, and event context.

 Prerequisites:
   - Robot ASR service must be running
   - Microphone hardware must be operational

 Usage:
   ros2 run aimdk_examples_cpp get_asr_result

 Example:
   ros2 run aimdk_examples_cpp get_asr_result

 Parameters:
   - None
 */

#include "aimdk_msgs/msg/asr_result.hpp"
#include "rclcpp/rclcpp.hpp"

#include <iomanip>
#include <sstream>

class AsrResultEcho : public rclcpp::Node {
public:
  AsrResultEcho() : Node("get_asr_result") {
    auto qos = rclcpp::SensorDataQoS();
    sub_ = this->create_subscription<aimdk_msgs::msg::AsrResult>(
        "/aima/agent/asr_result", qos,
        std::bind(&AsrResultEcho::callback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "Subscribing ASR topic: %s",
                "/aima/agent/asr_result");
  }

private:
  void callback(const aimdk_msgs::msg::AsrResult::SharedPtr msg) {
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(6);
    oss << "AsrResult received\n"
        << "  frame_id:   " << msg->header.frame_id << "\n"
        << "  sequence:   " << msg->header.sequence << "\n"
        << "  stamp:      " << rclcpp::Time(msg->header.stamp).seconds() << " s\n"
        << "  text:       " << msg->text << "\n"
        << "  event_id:   " << msg->event_id << "\n"
        << "  is_final:   " << (msg->is_final ? "true" : "false") << "\n"
        << "  confidence: " << msg->confidence << "\n"
        << "  language:   " << msg->language;

    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500, "%s",
                         oss.str().c_str());
  }
  rclcpp::Subscription<aimdk_msgs::msg::AsrResult>::SharedPtr sub_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AsrResultEcho>());
  rclcpp::shutdown();
  return 0;
}
