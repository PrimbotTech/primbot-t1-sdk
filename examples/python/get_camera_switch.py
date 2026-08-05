#!/usr/bin/env python3

"""
Camera Switch Control Example Script

Description:
  This script demonstrates how to use the GetCameraSwitch and SetCameraSwitch
  services to query and control the camera stream on/off state.

  Workflow:
    1. Query current camera switch state
    2. Toggle the camera switch (enable if disabled, disable if enabled)
    3. Query again to confirm the change

Prerequisites:
  - Robot camera service must be running
  - GetCameraSwitch and SetCameraSwitch services must be available

Usage:
  python3 get_camera_switch.py [--enable | --disable]

Example:
  python3 get_camera_switch.py              # Toggle current state
  python3 get_camera_switch.py --enable     # Force enable
  python3 get_camera_switch.py --disable    # Force disable

Parameters:
  --enable    Force enable camera stream
  --disable   Force disable camera stream
"""

import argparse
import sys

import rclpy
from rclpy.node import Node

from aimdk_msgs.srv import GetCameraSwitch, SetCameraSwitch


GET_SERVICE_NAME = "/aimdk_5Fmsgs/srv/GetCameraSwitch"
SET_SERVICE_NAME = "/aimdk_5Fmsgs/srv/SetCameraSwitch"
CALL_TIMEOUT_SEC = 5.0
WAIT_SERVICE_SEC = 10.0


class CameraSwitchClient(Node):
    def __init__(self):
        super().__init__("get_camera_switch")

        self.get_client = self.create_client(GetCameraSwitch, GET_SERVICE_NAME)
        self.set_client = self.create_client(SetCameraSwitch, SET_SERVICE_NAME)

        self.get_logger().info(
            f"CameraSwitch client created. Waiting for services..."
        )

        if not self.get_client.wait_for_service(timeout_sec=WAIT_SERVICE_SEC):
            self.get_logger().error(
                f"Service {GET_SERVICE_NAME} not available after {WAIT_SERVICE_SEC}s."
            )
            raise SystemExit(1)

        if not self.set_client.wait_for_service(timeout_sec=WAIT_SERVICE_SEC):
            self.get_logger().error(
                f"Service {SET_SERVICE_NAME} not available after {WAIT_SERVICE_SEC}s."
            )
            raise SystemExit(1)

        self.get_logger().info("All camera switch services are available.")

    def get_state(self):
        """Query current camera switch state. Returns (success, camera_enabled)."""
        request = GetCameraSwitch.Request()

        future = self.get_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=CALL_TIMEOUT_SEC)

        if not future.done():
            self.get_logger().error("GetCameraSwitch timed out.")
            return False, None

        response = future.result()
        if response is None:
            self.get_logger().error("GetCameraSwitch returned empty response.")
            return False, None

        code = response.response.header.code
        state = response.response.status.value
        enabled = response.camera_enabled

        self.get_logger().info(
            f"GetCameraSwitch: code={code}, state={state}, "
            f"camera_enabled={'ON' if enabled else 'OFF'}"
        )
        return True, enabled

    def set_state(self, enable: bool):
        """Set camera switch state. Returns success."""
        request = SetCameraSwitch.Request()
        request.camera_enabled = enable

        self.get_logger().info(
            f"Setting camera switch to {'ON' if enable else 'OFF'}..."
        )

        future = self.set_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=CALL_TIMEOUT_SEC)

        if not future.done():
            self.get_logger().error("SetCameraSwitch timed out.")
            return False

        response = future.result()
        if response is None:
            self.get_logger().error("SetCameraSwitch returned empty response.")
            return False

        code = response.response.header.code
        state = response.response.status.value
        message = response.response.message
        enabled = response.camera_enabled

        self.get_logger().info(
            f"SetCameraSwitch: code={code}, state={state}, "
            f"message='{message}', camera_enabled={'ON' if enabled else 'OFF'}"
        )
        return code == 0


def main(args=None):
    parser = argparse.ArgumentParser(description="Camera Switch Control Example")
    group = parser.add_mutually_exclusive_group()
    group.add_argument(
        "--enable", action="store_true", help="Force enable camera stream"
    )
    group.add_argument(
        "--disable", action="store_true", help="Force disable camera stream"
    )
    parsed = parser.parse_args(args if args else sys.argv[1:])

    rclpy.init()
    node = CameraSwitchClient()

    try:
        # Step 1: 查询当前状态
        self.get_logger().info("=" * 50)
        self.get_logger().info("Step 1: Query current camera switch state")
        ok, current = node.get_state()
        if not ok:
            return 1

        # Step 2: 决定目标状态
        if parsed.enable:
            target = True
        elif parsed.disable:
            target = False
        else:
            # 默认 toggle
            target = not current

        if target == current:
            node.get_logger().info(
                f"Camera is already {'ON' if current else 'OFF'}, no change needed."
            )
            return 0

        # Step 3: 设置新状态
        node.get_logger().info("=" * 50)
        node.get_logger().info(
            f"Step 2: Set camera switch to {'ON' if target else 'OFF'}"
        )
        if not node.set_state(target):
            return 1

        # Step 4: 再次查询确认
        node.get_logger().info("=" * 50)
        node.get_logger().info("Step 3: Verify camera switch state")
        ok, confirmed = node.get_state()
        if not ok:
            return 1

        if confirmed == target:
            node.get_logger().info("Camera switch state changed successfully.")
        else:
            node.get_logger().warn(
                f"State mismatch: expected {'ON' if target else 'OFF'}, "
                f"got {'ON' if confirmed else 'OFF'}"
            )

        return 0
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
