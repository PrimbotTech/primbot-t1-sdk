#include "aimdk_msgs/msg/joint_state_array.hpp"
#include "rclcpp/rclcpp.hpp"

#include <functional>
#include <iomanip>
#include <memory>
#include <signal.h>
#include <sstream>
#include <string>

std::shared_ptr<rclcpp::Node> g_node = nullptr;

void signal_handler(int signal) {
  if (g_node) {
    RCLCPP_INFO(g_node->get_logger(),
                "Received signal %d, shutting down get_joint_state...", signal);
    g_node.reset();
  }
  rclcpp::shutdown();
  exit(signal);
}

class JointStateEcho : public rclcpp::Node {
public:
  JointStateEcho() : Node("get_joint_state") {
    joint_name_ = this->declare_parameter<std::string>("joint_name", "");
    print_period_ms_ = this->declare_parameter<int>("print_period_ms", 500);
    if (print_period_ms_ <= 0) {
      print_period_ms_ = 500;
    }

    auto qos = rclcpp::QoS(rclcpp::KeepLast(10));
    qos.best_effort();
    qos.durability_volatile();

    sub_ = this->create_subscription<aimdk_msgs::msg::JointStateArray>(
        "/aima/hal/joint/state", qos,
        std::bind(&JointStateEcho::callback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(),
                "Subscribing joint state topic: /aima/hal/joint/state, "
                "joint_name='%s', print_period_ms=%d",
                joint_name_.c_str(), print_period_ms_);
  }

private:
  void callback(const aimdk_msgs::msg::JointStateArray::SharedPtr msg) {
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(6);
    oss << "JointStateArray received\n"
        << "  frame_id: " << msg->header.frame_id << "\n"
        << "  sequence: " << msg->header.sequence << "\n"
        << "  stamp:    " << rclcpp::Time(msg->header.stamp).seconds() << " s\n"
        << "  meas:     " << rclcpp::Time(msg->header.meas_stamp).seconds()
        << " s\n";

    if (joint_name_.empty()) {
      oss << "  joints:\n";
      for (const auto &joint : msg->joints) {
        append_joint_line(oss, joint);
      }
      RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(),
                           print_period_ms_, "%s", oss.str().c_str());
      return;
    }

    for (const auto &joint : msg->joints) {
      if (joint.name == joint_name_) {
        oss << "  joints:\n";
        append_joint_line(oss, joint);
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(),
                             print_period_ms_, "%s", oss.str().c_str());
        return;
      }
    }

    RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), print_period_ms_,
        "Joint '%s' not found in /aima/hal/joint/state", joint_name_.c_str());
  }

  static void append_joint_line(
      std::ostringstream &oss, const aimdk_msgs::msg::JointState &joint) {
    oss << "    - name=" << joint.name << ", position=" << joint.position
        << ", velocity=" << joint.velocity << ", effort=" << joint.effort
        << ", coil_temp=" << static_cast<int>(joint.coil_temp)
        << ", motor_temp=" << static_cast<int>(joint.motor_temp)
        << ", motor_vol=" << static_cast<int>(joint.motor_vol) << "\n";
  }

  std::string joint_name_;
  int print_period_ms_{500};
  rclcpp::Subscription<aimdk_msgs::msg::JointStateArray>::SharedPtr sub_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  signal(SIGINT, signal_handler);
  signal(SIGTERM, signal_handler);

  auto node = std::make_shared<JointStateEcho>();
  g_node = node;
  rclcpp::spin(node);
  g_node.reset();
  rclcpp::shutdown();
  return 0;
}
