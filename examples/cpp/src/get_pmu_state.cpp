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
    // Group output by power-supply and battery fields.
    oss << "PmuState received\n";

    oss << "  pmu_protocol_version: " << msg->pmu_protocol_version << "\n"
        << "  pmu_hardware_version: " << msg->pmu_hardware_version << "\n"
        << "  pmu_software_version: " << msg->pmu_software_version << "\n"
        << "  pmu_bool_status: " << msg->pmu_bool_status << "\n";

    oss << "  vcc_3v3_voltage:      " << msg->vcc_3v3_voltage << " V\n"
        << "  vcc_1v8_voltage:      " << msg->vcc_1v8_voltage << " V\n"
        << "  vcc_1v2_voltage:      " << msg->vcc_1v2_voltage << " V\n"
        << "  nfc_5v_voltage:       " << msg->nfc_5v_voltage << " V\n"
        << "  soc_5v_voltage:       " << msg->soc_5v_voltage << " V\n"
        << "  sys_5v_voltage:       " << msg->sys_5v_voltage << " V\n"
        << "  sys_12v_voltage:      " << msg->sys_12v_voltage << " V\n"
        << "  orin_12v_voltage:     " << msg->orin_12v_voltage << " V\n"
        << "  ext_12v_voltage:      " << msg->ext_12v_voltage << " V\n"
        << "  ext_24v_voltage:      " << msg->ext_24v_voltage << " V\n"
        << "  arm_48v_voltage:      " << msg->arm_48v_voltage << " V\n"
        << "  leg_48v_voltage:      " << msg->leg_48v_voltage << " V\n"
        << "  soc_5v_current:       " << msg->soc_5v_current << " A\n"
        << "  sys_5v_current:       " << msg->sys_5v_current << " A\n"
        << "  sys_12v_current:      " << msg->sys_12v_current << " A\n"
        << "  orin_12v_current:     " << msg->orin_12v_current << " A\n"
        << "  ext_12v_current:      " << msg->ext_12v_current << " A\n"
        << "  ext_24v_current:      " << msg->ext_24v_current << " A\n"
        << "  arm_48v_current:      " << msg->arm_48v_current << " A\n"
        << "  leg_48v_current:      " << msg->leg_48v_current << " A\n";

    oss << "  bms_manufacturer:                 " << msg->bms_manufacturer << "\n"
        << "  bms_serial_number:                " << msg->bms_serial_number << "\n"
        << "  bms_protocol_version:             " << msg->bms_protocol_version
        << "\n"
        << "  bms_hardware_version:             " << msg->bms_hardware_version
        << "\n"
        << "  bms_software_version:             " << msg->bms_software_version
        << "\n"
        << "  bms_status:                       " << msg->bms_status << "\n";

    oss << "  bms_balance_line_resistance:      "
        << msg->bms_balance_line_resistance << " mOhm\n"
        << "  bms_voltage:                      " << msg->bms_voltage << " V\n"
        << "  bms_current:                      " << msg->bms_current << " A\n"
        << "  bms_power:                        " << msg->bms_power << " W\n"
        << "  bms_temperature:                  " << msg->bms_temperature
        << " °C\n"
        << "  bms_remaining_capacity:           "
        << msg->bms_remaining_capacity << " mAh\n"
        << "  bms_remaining_capacity_pct:       "
        << static_cast<int>(msg->bms_remaining_capacity_percentage) << " %\n"
        << "  bms_cycle_count:                  " << msg->bms_cycle_count
        << "\n"
        << "  bms_cycle_total_capacity:         "
        << msg->bms_cycle_total_capacity << " Ah\n";

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
