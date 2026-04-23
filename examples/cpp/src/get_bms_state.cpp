#include "aimdk_msgs/msg/bms.hpp"
#include "rclcpp/rclcpp.hpp"

#include <iomanip>
#include <sstream>

class BmsStateEcho : public rclcpp::Node {
public:
  BmsStateEcho() : Node("get_bms_state") {
    auto qos = rclcpp::SensorDataQoS();
    sub_ = this->create_subscription<aimdk_msgs::msg::Bms>(
        "/aima/hal/bms/state", qos,
        std::bind(&BmsStateEcho::callback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "Subscribing bms topic: %s",
                "/aima/hal/bms/state");
  }

private:
  void callback(const aimdk_msgs::msg::Bms::SharedPtr msg) {
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(3);
    
    oss << "BmsState received\n";
    
    oss << "  manufacturer:                 " << msg->manufacturer << "\n"
        << "  serial_number:                " << msg->serial_number << "\n"
        << "  protocol_version:             " << msg->protocol_version << "\n"
        << "  hardware_version:             " << msg->hardware_version << "\n"
        << "  software_version:             " << msg->software_version << "\n";
    
    oss << "  power_status:                 " << msg->power_status << "\n";
    
    oss << "  bms_status:                   " << msg->bms_status << "\n"
        << "    bit 0 (充电):               " << ((msg->bms_status & 0x01) ? "是" : "否") << "\n"
        << "    bit 1 (充电过流):           " << ((msg->bms_status & 0x02) ? "是" : "否") << "\n"
        << "    bit 2 (放电):               " << ((msg->bms_status & 0x04) ? "是" : "否") << "\n"
        << "    bit 3 (放电过流):           " << ((msg->bms_status & 0x08) ? "是" : "否") << "\n"
        << "    bit 4 (电池短路):           " << ((msg->bms_status & 0x10) ? "是" : "否") << "\n"
        << "    bit 20 (充满状态):          " << ((msg->bms_status & (1 << 20)) ? "是" : "否") << "\n"
        << "    bit 23 (电池包异常):        " << ((msg->bms_status & (1 << 23)) ? "是" : "否") << "\n";
    
    oss << "  balance_line_resistance:      " << msg->balance_line_resistance << " mOhm\n"
        << "  voltage:                      " << (msg->voltage / 1000.0) << " V\n"
        << "  current:                      " << (msg->current / 1000.0) << " A\n"
        << "  power:                        " << (msg->power / 1000.0) << " W\n"
        << "  temperature:                  " << (msg->temperature / 10.0) << " °C\n"
        << "  remaining_capacity:           " << msg->remaining_capacity << " mAh\n"
        << "  remaining_capacity_pct:       " << static_cast<int>(msg->remaining_capacity_percentage) << " %\n"
        << "  cycle_count:                  " << msg->cycle_count << "\n"
        << "  cycle_total_capacity:         " << msg->cycle_total_capacity << " mAh\n";

    RCLCPP_INFO(this->get_logger(), "%s", oss.str().c_str());
  }
  rclcpp::Subscription<aimdk_msgs::msg::Bms>::SharedPtr sub_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<BmsStateEcho>());
  rclcpp::shutdown();
  return 0;
}
