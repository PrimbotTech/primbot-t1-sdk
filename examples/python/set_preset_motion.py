#!/usr/bin/env python3

"""
Example client for /aimdk_5Fmsgs/srv/SetMcPresetMotion.

This script automatically handles the required state machine transitions for safety.
Playing preset motions (like waving or handshaking) requires the robot to be in
BIPED_WHOLE_BODY_CTRL mode.

Prerequisites auto-handled by this script:
  1. Switch to PASSIVE_DEFAULT (ensure a clean starting point).
  2. Switch to BIPED_STAND_DEFAULT (enter standing posture).
  3. Switch to BIPED_WHOLE_BODY_CTRL (enable whole body control mode).
  4. Finally, execute the requested preset motion.
"""

import time
import rclpy
import rclpy.logging
from rclpy.node import Node

from aimdk_msgs.srv import GetMcAction, SetMcAction, SetMcPresetMotion
from aimdk_msgs.msg import (
    CommonRequest, CommonState, McAction, McActionCommand, 
    McActionStatus, McPresetMotion, RequestHeader
)
import sys

class SetMcPresetMotionClient(Node):
    def __init__(self):
        super().__init__('preset_motion_client')
        
        self.preset_client = self.create_client(
            SetMcPresetMotion, '/aimdk_5Fmsgs/srv/SetMcPresetMotion')
        self.set_action_client = self.create_client(
            SetMcAction, '/aimdk_5Fmsgs/srv/SetMcAction')
        self.get_action_client = self.create_client(
            GetMcAction, '/aimdk_5Fmsgs/srv/GetMcAction')
            
        self.get_logger().info('SetMcPresetMotion client node created.')
        self.wait_for_services()

    def wait_for_services(self):
        clients = [
            (self.preset_client, '/aimdk_5Fmsgs/srv/SetMcPresetMotion'),
            (self.set_action_client, '/aimdk_5Fmsgs/srv/SetMcAction'),
            (self.get_action_client, '/aimdk_5Fmsgs/srv/GetMcAction')
        ]
        for client, name in clients:
            while not client.wait_for_service(timeout_sec=2.0):
                self.get_logger().info(f'Waiting for service {name}...')
        self.get_logger().info('All required services are available.')

    def get_action_status(self):
        try:
            request = GetMcAction.Request()
            request.request = CommonRequest()
            request.request.header.stamp = self.get_clock().now().to_msg()
            future = self.get_action_client.call_async(request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
            if not future.done(): return None, None, None
            res = future.result()
            if res is None: return None, None, None
            return res.info.current_action.value, res.info.action_desc, res.info.status.value
        except Exception as e:
            self.get_logger().error(f'Error getting action status: {e}')
            return None, None, None

    def set_action(self, action_desc: str) -> bool:
        try:
            request = SetMcAction.Request()
            request.header = RequestHeader()
            request.header.stamp = self.get_clock().now().to_msg()
            request.command = McActionCommand()
            request.command.action = McAction()
            request.command.action_desc = action_desc
            
            self.get_logger().info(f'Requesting state switch to: {action_desc}')
            future = self.set_action_client.call_async(request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
            
            if not future.done(): return False
            res = future.result()
            return res is not None and res.response.status.value == CommonState.SUCCESS
        except Exception as e:
            self.get_logger().error(f'Error calling SetMcAction: {e}')
            return False

    def wait_for_action(self, target_desc: str, timeout_sec: float = 10.0) -> bool:
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline:
            _, desc, status = self.get_action_status()
            if desc == target_desc and status == McActionStatus.RUNNING:
                self.get_logger().info(f'Robot successfully reached state: {target_desc}')
                return True
            time.sleep(0.5)
        self.get_logger().error(f'Timeout waiting for state: {target_desc}')
        return False

    def ensure_ready_state(self) -> bool:
        _, desc, status = self.get_action_status()
        if desc == 'BIPED_WHOLE_BODY_CTRL' and status == McActionStatus.RUNNING:
            return True

        self.get_logger().info('Current state is not BIPED_WHOLE_BODY_CTRL. Starting safety transition sequence...')
        
        # Step 1: Switch to PASSIVE_DEFAULT
        if desc != 'PASSIVE_DEFAULT':
            if not self.set_action('PASSIVE_DEFAULT') or not self.wait_for_action('PASSIVE_DEFAULT'):
                return False
        
        # Step 2: Switch to BIPED_STAND_DEFAULT
        if not self.set_action('BIPED_STAND_DEFAULT') or not self.wait_for_action('BIPED_STAND_DEFAULT'):
            return False
            
        # Step 3: Switch to BIPED_WHOLE_BODY_CTRL
        if not self.set_action('BIPED_WHOLE_BODY_CTRL') or not self.wait_for_action('BIPED_WHOLE_BODY_CTRL'):
            return False
            
        return True

    def send_motion_request(self, motion_id: int) -> bool:
        if not self.ensure_ready_state():
            self.get_logger().error('Failed to prepare robot state for preset motion.')
            return False

        request = SetMcPresetMotion.Request()
        request.header = RequestHeader()
        request.header.stamp = self.get_clock().now().to_msg()
        request.motion = McPresetMotion()
        request.motion.value = motion_id
        request.interrupt = True

        self.get_logger().info(f'Sending preset motion request: ID={motion_id}')
        future = self.preset_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)

        if not future.done(): return False
        res = future.result()
        if res and res.response.header.code == 0:
            self.get_logger().info(f'Motion request accepted. Task ID: {res.response.task_id}')
            return True
        return False


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        # Prompt user to refer to documentation
        print("\nPlease refer to the interface documentation for the list of supported motions for this model.")
        print("If you haven't found the motion list, you can choose recommended motions based on the robot model.")
        # Determine robot series (Q or T). In a real scenario this could be obtained from a parameter or config.
        robot_series = input("\nEnter robot series (Q/T): ").strip().upper()
        if robot_series == "T":
            motion_map = {1001: "raise", 1002: "wave", 1003: "handshake", 1004: "airkiss"}
        elif robot_series == "Q":
            motion_map = {3001: "wave", 3002: "handshake", 3003: "bump", 3004: "wave_hand"}
        else:
            print("Unknown series. Please enter 'Q' or 'T'.")
            sys.exit(1)
        print("\nAvailable Preset Motions:")
        for k, v in motion_map.items():
            print(f"  {k}: {v}")
        motion_id = int(input("\nEnter preset motion ID: "))
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
