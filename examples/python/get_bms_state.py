#!/usr/bin/env python3

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
    node = BmsStateEcho()
    try:
        rclpy.spin(node)
        return 0
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
