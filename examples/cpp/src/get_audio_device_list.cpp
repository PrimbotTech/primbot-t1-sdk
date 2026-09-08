/*
 Audio Device List Monitor Example Script

 Description:
   This script demonstrates how to subscribe to the robot's audio device list topic.
   It displays available input (microphone) and output (speaker) devices with their properties.

 Prerequisites:
   - Robot audio service must be running
   - Audio devices must be properly configured

 Usage:
   ros2 run aimdk_examples_cpp get_audio_device_list

 Example:
   ros2 run aimdk_examples_cpp get_audio_device_list

 Parameters:
   - None
 */

#include "aimdk_msgs/msg/audio_device_info_list.hpp"
#include "rclcpp/rclcpp.hpp"

#include <iomanip>
#include <sstream>

class AudioDeviceListEcho : public rclcpp::Node {
public:
  AudioDeviceListEcho() : Node("get_audio_device_list") {
    auto qos = rclcpp::SensorDataQoS();
    sub_ = this->create_subscription<aimdk_msgs::msg::AudioDeviceInfoList>(
        "/aima/hal/audio/audio_device_list", qos,
        std::bind(&AudioDeviceListEcho::callback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "Subscribing audio device list topic: %s",
                "/aima/hal/audio/audio_device_list");
  }

private:
  void callback(const aimdk_msgs::msg::AudioDeviceInfoList::SharedPtr msg) {
    std::ostringstream oss;
    oss << "AudioDeviceInfoList received\n";

    // Print input devices (microphones)
    oss << "  Input devices (" << msg->input_list.size() << "):\n";
    for (size_t i = 0; i < msg->input_list.size(); ++i) {
      const auto &dev = msg->input_list[i];
      oss << "    [" << i << "] id=" << dev.id
          << " addr=" << dev.device_addr
          << " type=" << dev.device_type
          << " name=" << dev.device_name
          << " selected=" << (dev.is_sel ? "yes" : "no") << "\n";
    }

    // Print output devices (speakers)
    oss << "  Output devices (" << msg->output_list.size() << "):\n";
    for (size_t i = 0; i < msg->output_list.size(); ++i) {
      const auto &dev = msg->output_list[i];
      oss << "    [" << i << "] id=" << dev.id
          << " addr=" << dev.device_addr
          << " type=" << dev.device_type
          << " name=" << dev.device_name
          << " selected=" << (dev.is_sel ? "yes" : "no") << "\n";
    }

    RCLCPP_INFO(this->get_logger(), "%s", oss.str().c_str());
  }
  rclcpp::Subscription<aimdk_msgs::msg::AudioDeviceInfoList>::SharedPtr sub_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AudioDeviceListEcho>());
  rclcpp::shutdown();
  return 0;
}
