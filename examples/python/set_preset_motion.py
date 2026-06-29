#!/usr/bin/env python3

"""
Preset Motion Example Script

Description:
  This script demonstrates how to call the SetMcPresetMotion service to play preset
  motions (like waving or handshaking) on the robot. Supports T and Q series robots
  with interactive series and motion selection.

Prerequisites:
  - Robot motion control service must be running
  - Robot must support BIPED_LOCOMOTION_WBC action state
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
from collections import deque

import rclpy
import rclpy.logging
from rclpy.node import Node

from aimdk_msgs.srv import GetMcAction, SetMcAction, SetMcPresetMotion
from aimdk_msgs.msg import CommonRequest, CommonState, McAction, McActionCommand, McActionStatus, McPresetMotion, RequestHeader

SERVICE_CALL_TIMEOUT_SEC = 2.0
MAX_RETRY_COUNT = 3

# T1状态机有向图（与 set_mc_action.py 保持一致）
ACTION_GRAPH = {
    'PASSIVE_DEFAULT': [
        'QUADRUPED_STAND_DEFAULT',
        'QUADRUPED_GET_DOWN_DEFAULT',
        'QUADRUPED_SIT_DOWN_DEFAULT',
        'QUADRUPED_RECOVERY',
        'BIPED_RECOVERY',
        'DAMPING_DEFAULT',
    ],
    'QUADRUPED_STAND_DEFAULT': [
        'QUADRUPED_LOCOMOTION_DEFAULT',
        'QUADRUPED_GET_DOWN_DEFAULT',
        'QUADRUPED_SIT_DOWN_DEFAULT',
        'QUADRUPED_LOCOMOTION_TERRAIN',
        'QUADRUPED_LOCOMOTION_RUN',
    ],
    'QUADRUPED_LOCOMOTION_DEFAULT': [
        'QUADRUPED_LOCOMOTION_TERRAIN',
        'QUADRUPED_LOCOMOTION_RUN',
        'QUADRUPED_LOCOMOTION_BIONIC',
        'QUADRUPED_LOCOMOTION_PRIDE',
        'QUADRUPED_LOCOMOTION_PLEASURE',
        'QUADRUPED_LOCOMOTION_JUMP',
        'QUADRUPED_LOCOMOTION_HANDSHAKE',
        'QUADRUPED_LOCOMOTION_STRETCH',
        'QUADRUPED_LOCOMOTION_DANCE',
        'QUADRUPED_LOCOMOTION_FRONTFLIP',
        'QUADRUPED_LOCOMOTION_BACKFLIP',
        'QUADRUPED_TO_BIPED',
        'QUADRUPED_TO_BIPED_ROTATE',
        'QUADRUPED_TO_BIPED_FRONTFLIP',
        'QUADRUPED_STAND_DEFAULT',
    ],
    'BIPED_LOCOMOTION_WBC': [
        'BIPED_LOCOMOTION_DEFAULT',
        'BIPED_LOCOMOTION_TERRAIN',
        'BIPED_LOCOMOTION_RUN',
        'BIPED_TO_QUADRUPED',
        'BIPED_TO_QUADRUPED_ROTATE',
        'BIPED_TO_QUADRUPED_FRONTFLIP',
        'BIPED_LOCOMOTION_ROTATE_LOCAL',
        'BIPED_LOCOMOTION_BACKBEND',
        'BIPED_LOCOMOTION_MOONWALK',
    ],
    'BIPED_LOCOMOTION_DEFAULT': [
        'BIPED_LOCOMOTION_WBC',
        'BIPED_LOCOMOTION_TERRAIN',
        'BIPED_LOCOMOTION_RUN',
        'BIPED_LOCOMOTION_ROTATE_LOCAL',
        'BIPED_LOCOMOTION_BACKBEND',
        'BIPED_LOCOMOTION_MOONWALK',
    ],
     # ── 四足趴下/坐下 → 回到 四足位控站立 ──
    'QUADRUPED_GET_DOWN_DEFAULT': ['QUADRUPED_STAND_DEFAULT'],
    'QUADRUPED_SIT_DOWN_DEFAULT': ['QUADRUPED_STAND_DEFAULT'],
    # ── 四足技能/基础运动 → 回到 LOCOMOTION ──
    'QUADRUPED_LOCOMOTION_TERRAIN': ['QUADRUPED_LOCOMOTION_DEFAULT'],
    'QUADRUPED_LOCOMOTION_RUN': ['QUADRUPED_LOCOMOTION_DEFAULT'],
    # ── 双足基础运动 → 回到 WBC ──
    'BIPED_LOCOMOTION_TERRAIN': ['BIPED_LOCOMOTION_WBC'],
    'BIPED_LOCOMOTION_RUN': ['BIPED_LOCOMOTION_WBC'],
    # ── 双足技能运动 → 回到 WBC ──
    'BIPED_LOCOMOTION_ROTATE_LOCAL': ['BIPED_LOCOMOTION_WBC'],
    'BIPED_LOCOMOTION_BACKBEND': ['BIPED_LOCOMOTION_WBC'],
    'BIPED_LOCOMOTION_MOONWALK': ['BIPED_LOCOMOTION_WBC'],
    # ── 自动切换边（系统自动完成，navigate_to_action 会确认状态而非重设） ──
    'BIPED_RECOVERY': ['BIPED_LOCOMOTION_WBC'],
    'QUADRUPED_RECOVERY': ['QUADRUPED_LOCOMOTION_DEFAULT'],
    'QUADRUPED_TO_BIPED': ['BIPED_LOCOMOTION_WBC'],
    'QUADRUPED_TO_BIPED_ROTATE': ['BIPED_LOCOMOTION_WBC'],
    'QUADRUPED_TO_BIPED_FRONTFLIP': ['BIPED_LOCOMOTION_WBC'],
    'BIPED_TO_QUADRUPED': ['QUADRUPED_LOCOMOTION_DEFAULT'],
    'BIPED_TO_QUADRUPED_ROTATE': ['QUADRUPED_LOCOMOTION_DEFAULT'],
    'BIPED_TO_QUADRUPED_FRONTFLIP': ['QUADRUPED_LOCOMOTION_DEFAULT'],
    'QUADRUPED_LOCOMOTION_BIONIC': ['QUADRUPED_LOCOMOTION_DEFAULT'],
    'QUADRUPED_LOCOMOTION_PRIDE': ['QUADRUPED_LOCOMOTION_DEFAULT'],
    'QUADRUPED_LOCOMOTION_PLEASURE': ['QUADRUPED_LOCOMOTION_DEFAULT'],
    'QUADRUPED_LOCOMOTION_JUMP': ['QUADRUPED_LOCOMOTION_DEFAULT'],
    'QUADRUPED_LOCOMOTION_HANDSHAKE': ['QUADRUPED_LOCOMOTION_DEFAULT'],
    'QUADRUPED_LOCOMOTION_STRETCH': ['QUADRUPED_LOCOMOTION_DEFAULT'],
    'QUADRUPED_LOCOMOTION_DANCE': ['QUADRUPED_LOCOMOTION_DEFAULT'],
    'QUADRUPED_LOCOMOTION_FRONTFLIP': ['QUADRUPED_LOCOMOTION_DEFAULT'],
    'QUADRUPED_LOCOMOTION_BACKFLIP': ['QUADRUPED_LOCOMOTION_DEFAULT'],
}

# 需要先恢复到 PASSIVE_DEFAULT 的状态（这些状态无法直接跳转到其他路径）
RECOVERY_TO_PASSIVE = {
    'DAMPING_DEFAULT': 'PASSIVE_DEFAULT',
}

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

    def get_action_status(self):
        try:
            request = GetMcAction.Request()
            request.request = CommonRequest()
            request.request.header.stamp = self.get_clock().now().to_msg()

            future = self.call_service_with_retry(self.get_action_client, request, "GetMcAction")
            if future is None or future.result() is None:
                return None, None, None
            res = future.result()
            return res.info.current_action.value, res.info.action_desc, res.info.status.value
        except Exception as e:
            self.get_logger().error(f'Error getting action status: {e}')
            return None, None, None

    def set_action(self, action_desc: str) -> bool:
        try:
            request = SetMcAction.Request()
            request.header = RequestHeader()
            request.header.stamp = self.get_clock().now().to_msg()
            request.source = "node"  # 触发源标识
            request.command = McActionCommand()
            request.command.action = McAction()
            request.command.action_desc = action_desc

            self.get_logger().info(f'Requesting state switch to: {action_desc}')

            future = self.call_service_with_retry(self.set_action_client, request, "SetMcAction")
            if future is None or future.result() is None:
                return False
            
            result = future.result()
            return result and result.response.status.value == CommonState.SUCCESS
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

    def _find_path_bfs(self, start: str, target: str) -> list:
        if start == target:
            return [start]

        visited = {start}
        queue = deque([(start, [start])])

        while queue:
            current, path = queue.popleft()
            for neighbor in ACTION_GRAPH.get(current, []):
                if neighbor == target:
                    return path + [neighbor]
                if neighbor not in visited:
                    visited.add(neighbor)
                    queue.append((neighbor, path + [neighbor]))

        return []

    def navigate_to_action(self, target_action: str, current_desc: str, current_status: int) -> bool:
        if current_desc == target_action and current_status == McActionStatus.RUNNING:
            self.get_logger().info(f'Already at target action: {target_action}')
            return True

        # 处理需要先恢复到 PASSIVE_DEFAULT 的状态
        actual_current = current_desc
        if current_desc in RECOVERY_TO_PASSIVE:
            recovery_target = RECOVERY_TO_PASSIVE[current_desc]
            self.get_logger().info(f'Recovery: switching from {current_desc} to {recovery_target}...')
            if not self.set_action(recovery_target) or not self.wait_for_action(recovery_target):
                return False
            actual_current = recovery_target

        # 特殊路径：如果从 PASSIVE_DEFAULT 开始，强制使用固定路径
        if actual_current == 'PASSIVE_DEFAULT' and target_action == 'BIPED_LOCOMOTION_WBC':
            self.get_logger().info('Using fixed path from PASSIVE_DEFAULT to BIPED_LOCOMOTION_WBC')
            path = [
                'PASSIVE_DEFAULT',
                'QUADRUPED_STAND_DEFAULT',
                'QUADRUPED_LOCOMOTION_DEFAULT',
                'QUADRUPED_TO_BIPED',
                'BIPED_LOCOMOTION_WBC'
            ]
        else:
            path = self._find_path_bfs(actual_current, target_action)

        if not path:
            self.get_logger().info(f'No path found for {actual_current} -> {target_action}, attempting direct transition...')
            if not self.set_action(target_action):
                return False
            return self.wait_for_action(target_action, timeout_sec=5.0)

        for i in range(1, len(path)):
            target = path[i]
            self.get_logger().info(f'Path step {i}/{len(path) - 1}: Switching to {target}...')
            for attempt in range(1, 6):
                if self.set_action(target):
                    if self.wait_for_action(target):
                        break
                    return False
                if attempt < 5:
                    self.get_logger().info(f'Step {i} attempt {attempt}/5 failed, waiting for robot to stabilize...')
                    time.sleep(2.0)
                else:
                    return False
            time.sleep(1.0)

        return True

    def ensure_ready_state(self) -> bool:
        # Retry querying the status up to 5 seconds if it returns None
        deadline = time.monotonic() + 5.0
        desc = None
        while time.monotonic() < deadline:
            _, desc, status = self.get_action_status()
            if desc is not None:
                return self.navigate_to_action('BIPED_LOCOMOTION_WBC', desc, status)
            self.get_logger().warning('Current action state is None, retrying...')
            time.sleep(0.5)

        self.get_logger().error('Failed to get valid action state after 5 seconds. Aborting for safety.')
        return False

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
