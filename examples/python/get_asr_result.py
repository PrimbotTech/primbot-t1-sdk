#!/usr/bin/env python3

"""
ASR Result Monitor Example Script

Description:
  This script demonstrates how to subscribe to the robot's ASR (Automatic Speech Recognition) result topic.
  It displays real-time speech-to-text results including recognized text, confidence, and event context.

Prerequisites:
  - Robot ASR service must be running
  - Microphone hardware must be operational

Usage:
  python3 get_asr_result.py

Example:
  python3 get_asr_result.py

Parameters:
  - None
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from aimdk_msgs.msg import AsrResult


class AsrResultEcho(Node):
    def __init__(self):
        super().__init__("get_asr_result")

        self.subscription = self.create_subscription(
            AsrResult,
            "/aima/agent/asr_result",
            self.callback,
            qos_profile_sensor_data,
        )

        self.get_logger().info("Subscribing ASR topic: /aima/agent/asr_result")

    @staticmethod
    def _to_seconds(stamp) -> float:
        return float(stamp.sec) + float(stamp.nanosec) / 1_000_000_000.0

    def callback(self, msg: AsrResult):
        lines = [
            "AsrResult received",
            f"  frame_id:   {msg.header.frame_id}",
            f"  sequence:   {msg.header.sequence}",
            f"  stamp:      {self._to_seconds(msg.header.stamp):.6f} s",
            f"  text:       {msg.text}",
            f"  event_id:   {msg.event_id}",
            f"  is_final:   {msg.is_final}",
            f"  confidence: {msg.confidence:.4f}",
            f"  language:   {msg.language}",
        ]

        self.get_logger().info("\n".join(lines))


def main(args=None):
    rclpy.init(args=args)
    node = AsrResultEcho()
    try:
        rclpy.spin(node)
        return 0
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
