#!/usr/bin/env python3

"""
HAL Camera Command Example Script

Description:
  This script demonstrates how to use the HALCameraCommand service to send
  generic camera commands via a KV parameter interface.

  Supported operations:
    - Enable/disable camera stream publishing
    - Set camera frame rate (FPS)
    - Query current camera frame rate

Prerequisites:
  - Robot camera service must be running
  - HALCameraCommand service must be available

Usage:
  python3 hal_camera_command.py <command> [options]

Examples:
  python3 hal_camera_command.py enable --topic /aima/hal/camera/head_monocular_centra/nv12
  python3 hal_camera_command.py disable --topic /aima/hal/camera/head_monocular_centra/nv12
  python3 hal_camera_command.py set-fps --topic /aima/hal/camera/head_monocular_centra/nv12 --fps 30
  python3 hal_camera_command.py get-fps --topic /aima/hal/camera/head_monocular_centra/nv12

Parameters:
  command    One of: enable, disable, set-fps, get-fps
  --topic    Camera topic path (required)
  --fps      Target frame rate (required for set-fps)
"""

import argparse
import sys

import rclpy
from rclpy.node import Node

from aimdk_msgs.srv import HALCameraCommand


SERVICE_NAME = "/aimdk_5Fmsgs/srv/HALCameraCommand"
CALL_TIMEOUT_SEC = 5.0
WAIT_SERVICE_SEC = 10.0


class HALCameraCommandClient(Node):
    def __init__(self):
        super().__init__("hal_camera_command")

        self.client = self.create_client(HALCameraCommand, SERVICE_NAME)

        self.get_logger().info(
            f"HALCameraCommand client created. Waiting for service: {SERVICE_NAME}"
        )

        if not self.client.wait_for_service(timeout_sec=WAIT_SERVICE_SEC):
            self.get_logger().error(
                f"Service {SERVICE_NAME} not available after {WAIT_SERVICE_SEC}s. "
                "Is the camera service running?"
            )
            raise SystemExit(1)

        self.get_logger().info(f"Service {SERVICE_NAME} is available.")

    def send_command(self, command: str, keys: list, values: list) -> bool:
        """Send a HALCameraCommand request."""
        request = HALCameraCommand.Request()
        request.command = command
        request.param_keys = keys
        request.param_values = values

        self.get_logger().info(
            f"Sending command: '{command}', keys={keys}, values={values}"
        )

        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=CALL_TIMEOUT_SEC)

        if not future.done():
            self.get_logger().error(
                f"HALCameraCommand timed out after {int(CALL_TIMEOUT_SEC * 1000)} ms."
            )
            return False

        response = future.result()
        if response is None:
            self.get_logger().error("HALCameraCommand returned an empty response.")
            return False

        code = response.response.header.code
        state = response.response.status.value
        message = response.response.message

        self.get_logger().info(
            f"Response: code={code}, state={state}, message='{message}'"
        )

        # Print optional return values
        if response.result_keys and response.result_values:
            self.get_logger().info("Result:")
            for k, v in zip(response.result_keys, response.result_values):
                self.get_logger().info(f"  {k}: {v}")

        return code == 0


def main(args=None):
    parser = argparse.ArgumentParser(description="HAL Camera Command Example")
    subparsers = parser.add_subparsers(dest="command", required=True)

    # enable
    p_enable = subparsers.add_parser("enable", help="Enable camera stream")
    p_enable.add_argument("--topic", required=True, help="Camera topic path")

    # disable
    p_disable = subparsers.add_parser("disable", help="Disable camera stream")
    p_disable.add_argument("--topic", required=True, help="Camera topic path")

    # set-fps
    p_fps = subparsers.add_parser("set-fps", help="Set camera frame rate")
    p_fps.add_argument("--topic", required=True, help="Camera topic path")
    p_fps.add_argument("--fps", required=True, type=int, help="Target FPS")

    # get-fps
    p_get = subparsers.add_parser("get-fps", help="Query camera frame rate")
    p_get.add_argument("--topic", required=True, help="Camera topic path")

    parsed = parser.parse_args(args if args else sys.argv[1:])

    rclpy.init()
    node = HALCameraCommandClient()

    try:
        if parsed.command == "enable":
            ok = node.send_command(
                "stream.enable", ["topic"], [parsed.topic]
            )
        elif parsed.command == "disable":
            ok = node.send_command(
                "stream.disable", ["topic"], [parsed.topic]
            )
        elif parsed.command == "set-fps":
            ok = node.send_command(
                "set.fps", ["topic", "fps"], [parsed.topic, str(parsed.fps)]
            )
        elif parsed.command == "get-fps":
            ok = node.send_command(
                "get.fps", ["topic"], [parsed.topic]
            )
        else:
            node.get_logger().error(f"Unknown command: {parsed.command}")
            return 1

        return 0 if ok else 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
