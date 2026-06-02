#!/usr/bin/env python3

"""BMS State Monitor Example Script

Description:
  This script subscribes to the BMS (Battery Management System) state topic and
  continuously prints battery information including voltage, current, temperature,
  and charge level.

Prerequisites:
  - BMS hardware must be connected and publishing state data
  - /aima/hal/bms/state topic must be available

Usage:
  python3 get_bms_state.py

Example:
  python3 get_bms_state.py
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from aimdk_msgs.msg import Bms


class BmsStateEcho(Node):
    def __init__(self):
        super().__init__("get_bms_state")

        self.subscription = self.create_subscription(
            Bms,
            "/aima/hal/bms/state",
            self.callback,
            qos_profile_sensor_data,
        )

        self.get_logger().info("Subscribing bms topic: /aima/hal/bms/state")

    @staticmethod
    def _fmt3(value) -> str:
        return f"{float(value):.3f}"

    def callback(self, msg: Bms):
        lines = [
            "BmsState received",
            f"  manufacturer:                 {msg.manufacturer}",
            f"  serial_number:                {msg.serial_number}",
            f"  protocol_version:             {msg.protocol_version}",
            f"  hardware_version:             {msg.hardware_version}",
            f"  software_version:             {msg.software_version}",
            f"  power_request:                {msg.power_request}",
            f"  status:                       {msg.status}",
            f"  balance_line_resistance:      {msg.balance_line_resistance} mOhm",
            f"  voltage:                      {self._fmt3(msg.voltage / 1000.0)} V",
            f"  current:                      {self._fmt3(msg.current / 1000.0)} A",
            f"  power:                        {self._fmt3(msg.power / 1000.0)} W",
            f"  temperature:                  {self._fmt3(msg.temperature)} degC",
            f"  remaining_capacity:           {msg.remaining_capacity} mAh",
            f"  remaining_capacity_percentage:{int(msg.remaining_capacity_percentage)} %",
            f"  cycle_count:                  {msg.cycle_count}",
            f"  cycle_total_capacity:         {self._fmt3(msg.cycle_total_capacity / 1000.0)} Ah",
        ]

        self.get_logger().info("\n".join(lines))


def main(args=None):
    rclpy.init(args=args)
    node = BmsStateEcho()
    try:
        rclpy.spin(node)
        return 0
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
