#!/usr/bin/env python3

"""
Audio Device List Monitor Example Script

Description:
  This script demonstrates how to subscribe to the robot's audio device list topic.
  It displays available input (microphone) and output (speaker) devices with their properties.

Prerequisites:
  - Robot audio service must be running
  - Audio devices must be properly configured

Usage:
  python3 get_audio_device_list.py

Example:
  python3 get_audio_device_list.py

Parameters:
  - None
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from aimdk_msgs.msg import AudioDeviceInfoList


class AudioDeviceListEcho(Node):
    def __init__(self):
        super().__init__("get_audio_device_list")

        self.subscription = self.create_subscription(
            AudioDeviceInfoList,
            "/aima/hal/audio/audio_device_list",
            self.callback,
            qos_profile_sensor_data,
        )

        self.get_logger().info(
            "Subscribing audio device list topic: /aima/hal/audio/audio_device_list"
        )

    def callback(self, msg: AudioDeviceInfoList):
        lines = ["AudioDeviceInfoList received"]

        # Print input devices (microphones)
        lines.append(f"  Input devices ({len(msg.input_list)}):")
        for i, dev in enumerate(msg.input_list):
            selected = "yes" if dev.is_sel else "no"
            lines.append(
                f"    [{i}] id={dev.id} "
                f"addr={dev.device_addr} "
                f"type={dev.device_type} "
                f"name={dev.device_name} "
                f"selected={selected}"
            )

        # Print output devices (speakers)
        lines.append(f"  Output devices ({len(msg.output_list)}):")
        for i, dev in enumerate(msg.output_list):
            selected = "yes" if dev.is_sel else "no"
            lines.append(
                f"    [{i}] id={dev.id} "
                f"addr={dev.device_addr} "
                f"type={dev.device_type} "
                f"name={dev.device_name} "
                f"selected={selected}"
            )

        self.get_logger().info("\n".join(lines))


def main(args=None):
    rclpy.init(args=args)
    node = AudioDeviceListEcho()
    try:
        rclpy.spin(node)
        return 0
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
