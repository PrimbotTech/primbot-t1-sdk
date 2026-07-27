#!/usr/bin/env python3

"""
Preset Motion Example Script

Description:
  This script demonstrates how to call the SetMcPresetMotion service to play preset
  motions (like waving or handshaking) on the robot. Supports T and Q series robots
  with interactive series and motion selection.

Prerequisites:
  - Robot motion control service must be running
  - McActionSwitcher will transition to BIPED_LOCOMOTION_WBC before the motion
  - SetMcPresetMotion service must be available

Usage:
  python3 set_preset_motion.py

Example:
  # Interactive mode (will prompt for series and motion ID)
  python3 set_preset_motion.py

Supported Motions:
  T series: 1001=raise, 1002=wave, 1003=handshake, 2001=handheart
  Q series: 3001=wave, 3002=handshake, 3003=bump, 3004=wave_hand
"""

import sys
import time

import rclpy
from rclpy.node import Node

from aimdk_msgs.msg import McPresetMotion, RequestHeader
from aimdk_msgs.srv import SetMcPresetMotion

from common.mc_action_switcher import McActionSwitchOptions, McActionSwitcher

SERVICE_CALL_TIMEOUT_SEC = 2.0
MAX_RETRY_COUNT = 3

class SetMcPresetMotionClient(Node):
    def __init__(self):
        super().__init__('preset_motion_client')
        
        self.preset_client = self.create_client(
            SetMcPresetMotion, '/aimdk_5Fmsgs/srv/SetMcPresetMotion')
            
        self.get_logger().info('SetMcPresetMotion client node created.')
        self.wait_for_services()

    def wait_for_services(self):
        while not self.preset_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().info(
                'Waiting for service /aimdk_5Fmsgs/srv/SetMcPresetMotion...'
            )
        self.get_logger().info('SetMcPresetMotion service is available.')

    def call_service_with_retry(self, client, request, service_name: str, timeout_sec=None, max_retries=None):
        timeout_sec = timeout_sec or SERVICE_CALL_TIMEOUT_SEC
        max_retries = max_retries or MAX_RETRY_COUNT

        for i in range(max_retries):
            future = client.call_async(request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=timeout_sec)
            if future.done():
                return future
            self.get_logger().info(f'{service_name} attempt {i+1}/{max_retries} timed out, retrying...')
            time.sleep(0.2)

        self.get_logger().error(f'{service_name} failed after {max_retries} attempts')
        return None

    def ensure_ready_state(self) -> bool:
        switcher = McActionSwitcher(self)
        options = McActionSwitchOptions(
            source="preset_motion",
            total_timeout=30.0,
        )
        result = switcher.switch_to("BIPED_LOCOMOTION_WBC", options)
        if not result.success:
            current_action = result.current_action or "(unknown)"
            self.get_logger().error(
                "Failed to switch from "
                f"{current_action} to BIPED_LOCOMOTION_WBC: {result.message}"
            )
        return result.success

    def send_motion_request(self, motion_id: int) -> bool:
        if not self.ensure_ready_state():
            self.get_logger().error('Failed to prepare robot state for preset motion.')
            return False

        request = SetMcPresetMotion.Request()
        request.header = RequestHeader()
        request.header.stamp = self.get_clock().now().to_msg()
        request.source = 'preset_motion'
        request.motion = McPresetMotion()
        request.motion.value = motion_id
        request.interrupt = True

        self.get_logger().info(f'Sending preset motion request: ID={motion_id}')

        future = self.call_service_with_retry(self.preset_client, request, "SetMcPresetMotion")
        if future is None or future.result() is None:
            return False
        res = future.result()
        if res and res.response.header.code == 0:
            self.get_logger().info(f'Motion request accepted. Task ID: {res.response.task_id}')
            return True
        
        if res:
            self.get_logger().error(
                f'SetMcPresetMotion failed. '
                f'code={res.response.header.code} status={res.response.state.value} '
                f'reason={res.response.state.reason}'
            )
            return False


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        print("\nPlease refer to the interface documentation for the list of supported motions for this model.")
        print("If you haven't found the motion list, you can choose recommended motions based on the robot model.")
        robot_series = input("\nEnter robot series (Q/T): ").strip().upper()
        motion_map = {
            "T": {1001: "raise", 1002: "wave", 1003: "handshake", 2001: "handheart"},
            "Q": {3001: "wave", 3002: "handshake", 3003: "bump", 3004: "wave_hand"},
        }.get(robot_series)
        if not motion_map:
            print("Unknown series. Please enter 'Q' or 'T'.")
            sys.exit(1)
        print("\nAvailable Preset Motions:")
        for k, v in motion_map.items():
            print(f"  {k}: {v}")
        motion_id = int(input("\nEnter preset motion ID: "))
        if motion_id not in motion_map:
            print(f"Invalid motion ID '{motion_id}'. Please choose from the list above.")
            sys.exit(1)
        node = SetMcPresetMotionClient()
        node.send_motion_request(motion_id)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Error: {e}")
    finally:
        if node: node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
