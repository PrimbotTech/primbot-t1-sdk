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
        # Group output by power-supply and battery fields.
        lines = [
            "PmuState received",
            f"  pmu_protocol_version: {msg.pmu_protocol_version}",
            f"  pmu_hardware_version: {msg.pmu_hardware_version}",
            f"  pmu_software_version: {msg.pmu_software_version}",
            f"  pmu_bool_status: {msg.pmu_bool_status}",
            f"  soc_5v_voltage:       {self._fmt3(msg.soc_5v_voltage)} V",
            f"  sys_5v_voltage:       {self._fmt3(msg.sys_5v_voltage)} V",
            f"  sys_12v_voltage:      {self._fmt3(msg.sys_12v_voltage)} V",
            f"  orin_12v_voltage:     {self._fmt3(msg.orin_12v_voltage)} V",
            f"  ext_12v_voltage:      {self._fmt3(msg.ext_12v_voltage)} V",
            f"  ext_24v_voltage:      {self._fmt3(msg.ext_24v_voltage)} V",
            f"  arm_48v_voltage:      {self._fmt3(msg.arm_48v_voltage)} V",
            f"  leg_48v_voltage:      {self._fmt3(msg.leg_48v_voltage)} V",
            f"  soc_5v_current:       {self._fmt3(msg.soc_5v_current)} A",
            f"  sys_5v_current:       {self._fmt3(msg.sys_5v_current)} A",
            f"  sys_12v_current:      {self._fmt3(msg.sys_12v_current)} A",
            f"  orin_12v_current:     {self._fmt3(msg.orin_12v_current)} A",
            f"  ext_12v_current:      {self._fmt3(msg.ext_12v_current)} A",
            f"  ext_24v_current:      {self._fmt3(msg.ext_24v_current)} A",
            f"  arm_48v_current:      {self._fmt3(msg.arm_48v_current)} A",
            f"  leg_48v_current:      {self._fmt3(msg.leg_48v_current)} A",
            f"  bms_manufacturer:                 {msg.bms_manufacturer}",
            f"  bms_serial_number:                {msg.bms_serial_number}",
            f"  bms_protocol_version:             {msg.bms_protocol_version}",
            f"  bms_hardware_version:             {msg.bms_hardware_version}",
            f"  bms_software_version:             {msg.bms_software_version}",
            f"  bms_status:                       {msg.bms_status}",
            f"  bms_balance_line_resistance:      {msg.bms_balance_line_resistance} mOhm",
            f"  bms_voltage:                      {self._fmt3(msg.bms_voltage)} V",
            f"  bms_current:                      {self._fmt3(msg.bms_current)} A",
            f"  bms_power:                        {self._fmt3(msg.bms_power)} W",
            f"  bms_temperature:                  {msg.bms_temperature} degC",
            f"  bms_remaining_capacity:           {msg.bms_remaining_capacity} mAh",
            f"  bms_remaining_capacity_percentage:{int(msg.bms_remaining_capacity_percentage)} %",
            f"  bms_cycle_count:                  {msg.bms_cycle_count}",
            f"  bms_cycle_total_capacity:         {msg.bms_cycle_total_capacity} Ah",
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
