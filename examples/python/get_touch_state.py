#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from aimdk_msgs.msg import TouchState


class TouchStateEcho(Node):
    def __init__(self):
        super().__init__("get_touch_state")

        self.subscription = self.create_subscription(
            TouchState,
            "/aima/hal/touch/state",
            self.callback,
            qos_profile_sensor_data,
        )

        self.get_logger().info("Subscribing touch topic: /aima/hal/touch/state")

    @staticmethod
    def _to_seconds(stamp) -> float:
        return float(stamp.sec) + float(stamp.nanosec) / 1_000_000_000.0

    def callback(self, msg: TouchState):
        # event_type is the primary field of interest.
        lines = [
            "TouchState received",
            f"  frame_id: {msg.header.frame_id}",
            f"  sequence: {msg.header.sequence}",
            f"  stamp:    {self._to_seconds(msg.header.stamp):.6f} s",
            f"  meas:     {self._to_seconds(msg.header.meas_stamp):.6f} s",
            f"  event:    {int(msg.event_type)}",
        ]

        self.get_logger().info("\n".join(lines))


def main(args=None):
    rclpy.init(args=args)
    node = TouchStateEcho()
    try:
        rclpy.spin(node)
        return 0
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
