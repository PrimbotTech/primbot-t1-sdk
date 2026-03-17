#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from aimdk_msgs.msg import PmuState


class PmuStateEcho(Node):
    def __init__(self):
        super().__init__("get_pmu_state")

        self.subscription = self.create_subscription(
            PmuState,
            "/aima/hal/pmu/state",
            self.callback,
            qos_profile_sensor_data,
        )

        self.get_logger().info("Subscribing pmu topic: /aima/hal/pmu/state")

    @staticmethod
    def _fmt3(value) -> str:
        return f"{float(value):.3f}"

    def callback(self, msg: PmuState):
        lines = [
            "PmuState received",
            f"  pmu_protocol_version: {msg.pmu_protocol_version}",
            f"  pmu_hardware_version: {msg.pmu_hardware_version}",
            f"  pmu_software_version: {msg.pmu_software_version}",
            f"  pmu_bool_status: {msg.pmu_bool_status}",
            f"  soc_5v_voltage:       {self._fmt3(msg.soc_5v_voltage)} V",
            f"  sys_5v_voltage:       {self._fmt3(msg.sys_5v_voltage)} V",
            f"  sys_12v_voltage:      {self._fmt3(msg.sys_12v_voltage)} V",
            f"  fore_leg_48v_voltage: {self._fmt3(msg.fore_leg_48v_voltage)} V",
            f"  hind_leg_48v_voltage: {self._fmt3(msg.hind_leg_48v_voltage)} V",
            f"  ext_12v_voltage:      {self._fmt3(msg.ext_12v_voltage)} V",
            f"  ext_24v_voltage:      {self._fmt3(msg.ext_24v_voltage)} V",
            f"  soc_5v_current:       {self._fmt3(msg.soc_5v_current)} A",
            f"  sys_5v_current:       {self._fmt3(msg.sys_5v_current)} A",
            f"  sys_12v_current:      {self._fmt3(msg.sys_12v_current)} A",
            f"  fore_leg_48v_current: {self._fmt3(msg.fore_leg_48v_current)} A",
            f"  hind_leg_48v_current: {self._fmt3(msg.hind_leg_48v_current)} A",
            f"  ext_12v_current:      {self._fmt3(msg.ext_12v_current)} A",
            f"  ext_24v_current:      {self._fmt3(msg.ext_24v_current)} A",
            f"  bms_manufacturer:                 {msg.bms_manufacturer}",
            f"  bms_serial_number:                {msg.bms_serial_number}",
            f"  bms_protocol_version:             {msg.bms_protocol_version}",
            f"  bms_hardware_version:             {msg.bms_hardware_version}",
            f"  bms_software_version:             {msg.bms_software_version}",
            f"  bms_status_bits:                  {msg.bms_status_bits}",
            (
                "  battery_balance_line_resistance:  "
                f"{msg.battery_balance_line_resistance} mOhm"
            ),
            f"  battery_pack_voltage:             {self._fmt3(msg.battery_pack_voltage)} V",
            f"  battery_current:                  {self._fmt3(msg.battery_current)} A",
            f"  battery_output_power:             {self._fmt3(msg.battery_output_power)} W",
            f"  battery_temperature:              {self._fmt3(msg.battery_temperature)} degC",
            f"  battery_remaining_capacity:       {msg.battery_remaining_capacity} mAh",
            (
                "  battery_remaining_capacity_pct:   "
                f"{int(msg.battery_remaining_capacity_percentage)} %"
            ),
            f"  battery_cycle_count:              {msg.battery_cycle_count}",
            f"  battery_cycle_total_capacity:     {msg.battery_cycle_total_capacity} Ah",
        ]

        self.get_logger().info("\n".join(lines))


def main(args=None):
    rclpy.init(args=args)
    node = PmuStateEcho()
    try:
        rclpy.spin(node)
        return 0
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
