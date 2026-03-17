#include "aimdk_msgs/msg/pmu_state.hpp"
#include "rclcpp/rclcpp.hpp"

#include <iomanip>
#include <sstream>

class PmuStateEcho : public rclcpp::Node {
public:
  PmuStateEcho() : Node("get_pmu_state") {
    auto qos = rclcpp::SensorDataQoS();
    sub_ = this->create_subscription<aimdk_msgs::msg::PmuState>(
        "/aima/hal/pmu/state", qos,
        std::bind(&PmuStateEcho::callback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "Subscribing pmu topic: %s",
                "/aima/hal/pmu/state");
  }

private:
  void callback(const aimdk_msgs::msg::PmuState::SharedPtr msg) {
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(3);
    oss << "PmuState received\n";

    oss << "  pmu_protocol_version: " << msg->pmu_protocol_version << "\n"
        << "  pmu_hardware_version: " << msg->pmu_hardware_version << "\n"
        << "  pmu_software_version: " << msg->pmu_software_version << "\n"
        << "  pmu_bool_status: " << msg->pmu_bool_status << "\n";

    oss << "  soc_5v_voltage:       " << msg->soc_5v_voltage << " V\n"
        << "  sys_5v_voltage:       " << msg->sys_5v_voltage << " V\n"
        << "  sys_12v_voltage:      " << msg->sys_12v_voltage << " V\n"
        << "  fore_leg_48v_voltage: " << msg->fore_leg_48v_voltage << " V\n"
        << "  hind_leg_48v_voltage: " << msg->hind_leg_48v_voltage << " V\n"
        << "  ext_12v_voltage:      " << msg->ext_12v_voltage << " V\n"
        << "  ext_24v_voltage:      " << msg->ext_24v_voltage << " V\n"
        << "  soc_5v_current:       " << msg->soc_5v_current << " A\n"
        << "  sys_5v_current:       " << msg->sys_5v_current << " A\n"
        << "  sys_12v_current:      " << msg->sys_12v_current << " A\n"
        << "  fore_leg_48v_current: " << msg->fore_leg_48v_current << " A\n"
        << "  hind_leg_48v_current: " << msg->hind_leg_48v_current << " A\n"
        << "  ext_12v_current:      " << msg->ext_12v_current << " A\n"
        << "  ext_24v_current:      " << msg->ext_24v_current << " A\n";

    oss << "  bms_manufacturer:                 " << msg->bms_manufacturer << "\n"
        << "  bms_serial_number:                " << msg->bms_serial_number << "\n"
        << "  bms_protocol_version:             " << msg->bms_protocol_version
        << "\n"
        << "  bms_hardware_version:             " << msg->bms_hardware_version
        << "\n"
        << "  bms_software_version:             " << msg->bms_software_version
        << "\n"
        << "  bms_status_bits:                 " << msg->bms_status_bits
        << "\n";

    oss << "  battery_balance_line_resistance:  "
        << msg->battery_balance_line_resistance << " mOhm\n"
        << "  battery_pack_voltage:             " << msg->battery_pack_voltage
        << " V\n"
        << "  battery_current:                  " << msg->battery_current
        << " A\n"
        << "  battery_output_power:             " << msg->battery_output_power
        << " W\n"
        << "  battery_temperature:              " << msg->battery_temperature
        << " °C\n"
        << "  battery_remaining_capacity:       "
        << msg->battery_remaining_capacity << " mAh\n"
        << "  battery_remaining_capacity_pct:   "
        << static_cast<int>(msg->battery_remaining_capacity_percentage)
        << " %\n"
        << "  battery_cycle_count:              " << msg->battery_cycle_count
        << "\n"
        << "  battery_cycle_total_capacity:     "
        << msg->battery_cycle_total_capacity << " Ah\n";

    RCLCPP_INFO(this->get_logger(), "%s", oss.str().c_str());
  }
  rclcpp::Subscription<aimdk_msgs::msg::PmuState>::SharedPtr sub_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PmuStateEcho>());
  rclcpp::shutdown();
  return 0;
}
