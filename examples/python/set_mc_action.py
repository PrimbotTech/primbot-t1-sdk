#!/usr/bin/env python3

"""
Action Switch Example Script

Description:
  This script demonstrates how to call the SetMcAction service to switch robot action
  states through the shared McActionSwitcher state-machine interface.

Prerequisites:
  - Robot motion control service must be running
  - SetMcAction and GetMcAction services must be available

Usage:
  python3 set_mc_action.py --ros-args -p type:=action [-p action_desc:=<ACTION>]

Example:
  # Interactive mode: input target action via terminal
  python3 set_mc_action.py --ros-args -p type:=action

  # Non-interactive mode: specify action via parameter
  python3 set_mc_action.py --ros-args -p type:=action -p \
  action_desc:=QUADRUPED_LOCOMOTION_JUMP

  # Motion mode: switch to the matching biped/quadruped locomotion action first
  python3 set_mc_action.py --ros-args -p type:=motion -p motion:=<MOTION>

Parameters:
  - type: "action" or "motion", required
  - action_desc: string, optional; if set, skips interactive input and executes directly
  - motion: string, required when type=motion; BIPED_* and QUAD_* names are
    routed to the corresponding whole-body action automatically

Notes:
  - Single Switch: Switches to the requested target action and waits for completion.
  - Auto Path: McActionSwitcher finds and executes a valid transition path.
  - Recovery: McActionSwitcher handles intermediate and automatic transitions.
  - Retry: McActionSwitcher waits for each requested action to become active.
  - Ctrl+C: Interrupts the current action switch and exits without switching to a default action.
  - WARNING: Do NOT use QUADRUPED_LOCOMOTION_JUMP for testing — the robot will jump and may cause injury or damage.
"""

import time

import rclpy
import rclpy.logging
from rclpy.node import Node

from aimdk_msgs.msg import CommonRequest, CommonState, McActionStatus, RequestHeader
from aimdk_msgs.srv import GetMcAction, SetMcMotion

from common.mc_action_switcher import McActionSwitchOptions, McActionSwitcher

SERVICE_CALL_TIMEOUT_SEC = 2.0
MAX_RETRY_COUNT = 3

class SetMcActionClient(Node):
    def __init__(self):
        super().__init__('set_mc_action_client')
        self.type = self.declare_parameter('type', '').value
        self.action_desc = self.declare_parameter('action_desc', '').value
        self.motion = self.declare_parameter('motion', '').value

        self.set_motion_client = self.create_client(
            SetMcMotion, '/aimdk_5Fmsgs/srv/SetMcMotion'
        )
        self.get_client = self.create_client(
            GetMcAction, '/aimdk_5Fmsgs/srv/GetMcAction'
        )
        self.get_logger().info(
            'SetMcAction client node created with '
            f'type={self.type} action_desc={self.action_desc} '
            f'motion={self.motion}'
        )

    def call_service_with_retry(self, client, request, service_name: str, timeout_sec=None, max_retries=None):
        """带重试机制的服务调用"""
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

    def execute(self) -> bool:
        if not self.validate_parameters():
            return False
        self.wait_for_services()

        if self.type == 'action':
            _, current_desc, current_status = self.get_action_status()
            self.get_logger().info(f'Current Action is: {current_desc}')

            try:
                target_action = (
                    self.action_desc or input(
                        f"Current Action is: {current_desc}, please input the expected Action "
                        "according to the motion control state machine transition logic in the "
                        "interface documentation. The Action you need to switch: "
                    ).strip()
                )
            except EOFError:
                return True
            if not target_action:
                return True

            ok = self.switch_action(target_action)
            print("Switch succeeded." if ok else
                  "Switch failed, please confirm if the expected Action complies with the state machine transition logic")
            return True
        else:
            target_action = self._motion_action_target(self.motion)
            if target_action is None:
                self.get_logger().error(
                    f"Cannot determine the robot form for motion '{self.motion}'. "
                    "Motion names must start with BIPED_ or QUAD_."
                )
                return False

            self.get_logger().info(
                f"Switching to {target_action} before executing motion {self.motion}."
            )
            if not self.switch_action(target_action):
                return False
            if not self.set_motion(self.motion):
                return False
            return self.wait_for_motion()


    def validate_parameters(self) -> bool:
        if not self.type:
            self.get_logger().error(
                "Parameter 'type' must be set. Use 'action' or 'motion'."
            )
            return False

        if self.type not in ('action', 'motion'):
            self.get_logger().error(
                f"Invalid parameter 'type': {self.type}. "
                "Use 'action' or 'motion'."
            )
            return False

        if self.type == 'motion' and not self.motion:
            self.get_logger().error(
                "Parameter 'motion' must be set when type=motion."
            )
            return False

        return True

    def wait_for_service(self, client, service_name: str):
        while not client.wait_for_service(timeout_sec=2.0):
            if not rclpy.ok():
                return
            self.get_logger().info(f'Service unavailable, waiting: {service_name}')
        self.get_logger().info(f'Service available: {service_name}')

    def wait_for_services(self):
        """Check only the necessary services based on the operation type."""
        self.wait_for_service(self.get_client, '/aimdk_5Fmsgs/srv/GetMcAction')
        if self.type == 'motion':
            self.wait_for_service(self.set_motion_client, '/aimdk_5Fmsgs/srv/SetMcMotion')

    def switch_action(self, target_action: str) -> bool:
        switcher = McActionSwitcher(self)
        options = McActionSwitchOptions(
            source="set_mc_action",
            total_timeout=30.0,
        )
        result = switcher.switch_to(target_action, options)
        if not result.success:
            current_action = result.current_action or "(unknown)"
            self.get_logger().error(
                f"Failed to switch from {current_action} to {target_action}: "
                f"{result.message}"
            )
        return result.success

    def _motion_action_target(self, motion_name: str):
        if motion_name.startswith(('QUAD_', 'QUADRUPED_')):
            return 'QUADRUPED_LOCOMOTION_DEFAULT'
        if motion_name.startswith('BIPED_'):
            return 'BIPED_LOCOMOTION_WBC'
        return None

    def set_motion(self, motion_name: str) -> bool:
        try:
            for attempt in range(1, 6):
                request = SetMcMotion.Request()
                request.header = RequestHeader()
                request.header.stamp = self.get_clock().now().to_msg()
                request.source = 'set_mc_action'
                request.motion = motion_name
                request.type = SetMcMotion.Request.MIMIC_QY
                request.interrupt = False

                self.get_logger().info(
                    f'Sending SetMcMotion request ({attempt}/5): motion={motion_name}'
                )

                future = self.set_motion_client.call_async(request)
                rclpy.spin_until_future_complete(self, future, timeout_sec=1.0)

                if not future.done() or future.result() is None:
                    self.get_logger().warning(f'SetMcMotion request attempt {attempt}/5 failed or timed out.')
                    continue

                response = future.result()
                code = response.response.header.code
                state = response.response.state.value
                task_id = response.response.task_id

                if code == 0 and state in (CommonState.SUCCESS, CommonState.RUNNING):
                    self.get_logger().info(f'SetMcMotion request accepted: code={code} state={state} task_id={task_id}')
                    return True

                self.get_logger().warning(f'SetMcMotion request attempt {attempt}/5 not accepted: code={code} state={state} task_id={task_id}')

            self.get_logger().error('Failed to set motion after 5 attempts.')
            return False
        except Exception as e:
            self.get_logger().error(f'Exception occurred: {e}')
            return False

    def wait_for_motion(self, timeout_sec: float = 10.0, poll_interval_sec: float = 0.2) -> bool:
        deadline = time.monotonic() + timeout_sec
        self.get_logger().info('Waiting for current motion action to reach RUNNING state...')

        while rclpy.ok() and time.monotonic() < deadline:
            action_id, action_desc, status = self.get_action_status()
            if status == McActionStatus.RUNNING:
                self.get_logger().info(f'Current motion action is running: action_id={action_id} action_desc={action_desc}')
                return True
            time.sleep(poll_interval_sec)

        self.get_logger().error('Timed out waiting for current motion action to reach RUNNING state.')
        return False

    def get_action_status(self):
        try:
            request = GetMcAction.Request()
            request.request = CommonRequest()
            request.request.header.stamp = self.get_clock().now().to_msg()

            future = self.call_service_with_retry(self.get_client, request, "GetMcAction")
            if future is None or future.result() is None:
                self.get_logger().warning('Get current action request failed or timed out.')
                return None, None, None

            response = future.result()
            return response.info.current_action.value, response.info.action_desc, response.info.status.value
        except Exception as e:
            self.get_logger().error(f'Exception occurred: {e}')
            return None, None, None


def main(args=None):
    rclpy.init(args=args)

    node = None
    try:
        node = SetMcActionClient()
        ok = node.execute()
        node.destroy_node()
        node = None
        if rclpy.ok():
            rclpy.shutdown()
        return 0 if ok else 1
    except KeyboardInterrupt:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        return 0
    except Exception as e:
        rclpy.logging.get_logger('main').error(f'Program exited with exception: {e}')
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        return 1


if __name__ == '__main__':
    raise SystemExit(main())
