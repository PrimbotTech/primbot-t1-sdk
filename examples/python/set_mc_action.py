#!/usr/bin/env python3

"""
Example client for /aimdk_5Fmsgs/srv/SetMcAction and /aimdk_5Fmsgs/srv/SetMcMotion

The following ROS parameters can be set via startup arguments:
--ros-args -p <name>:=<value>

Supported parameters:
  - type: "action" or "motion", required
  - action_desc: string, ignored in interactive action mode
  - motion: string, required when type=motion
  - interrupt: bool, optional when type=motion, default=true

Examples:
  # Starts interactive state machine navigator
  python3 examples/python/set_mc_action.py --ros-args -p type:=action
  
  # Executes specific motion with auto-transition to WALK_RUN
  python3 examples/python/set_mc_action.py --ros-args -p type:=motion -p \
  motion:=INTRO_POSE6 -p interrupt:=true

Notes:
  - Interactive Action Mode: Allows manual state machine navigation via terminal.
  - Automatic Motion Mode: seq auto-path (PASSIVE -> STAND -> WALK_RUN).
  - Smart Transition: Automatically skips redundant steps based on current robot pose.
  - On-demand Check: Only waits for services relevant to the selected mode.
"""

import time

import rclpy
import rclpy.logging
from rclpy.node import Node

from aimdk_msgs.msg import CommonRequest, CommonState, McAction, McActionCommand, McActionStatus, RequestHeader
from aimdk_msgs.srv import GetMcAction, SetMcAction, SetMcMotion


class SetMcActionClient(Node):
    def __init__(self):
        super().__init__('set_mc_action_client')
        self.type = self.declare_parameter('type', '').value
        self.action_desc = self.declare_parameter('action_desc', '').value
        self.motion = self.declare_parameter('motion', '').value
        self.interrupt = self.declare_parameter('interrupt', True).value

        self.set_action_client = self.create_client(
            SetMcAction, '/aimdk_5Fmsgs/srv/SetMcAction'
        )
        self.set_motion_client = self.create_client(
            SetMcMotion, '/aimdk_5Fmsgs/srv/SetMcMotion'
        )
        self.get_client = self.create_client(
            GetMcAction, '/aimdk_5Fmsgs/srv/GetMcAction'
        )
        self.get_logger().info(
            'SetMcAction client node created with '
            f'type={self.type} action_desc={self.action_desc} '
            f'motion={self.motion} interrupt={self.interrupt}'
        )

    def execute(self) -> bool:
        if not self.validate_parameters():
            return False

        self.wait_for_services()

        if self.type == 'action':
            try:
                while rclpy.ok():
                    current_id, current_desc, current_status = self.get_action_status()
                    self.get_logger().info(f'Current Action is: {current_desc}')
                    
                    try:
                        target_action = input(
                            f"Current Action is: {current_desc}, please input the expected Action "
                            "according to the motion control state machine transition logic in the "
                            "interface documentation. The Action you need to switch: "
                        ).strip()
                    except EOFError:
                        break

                    if not target_action:
                        continue

                    # Execute SetMcAction
                    if self.set_action(target_action):
                        # Poll for success within 5 seconds
                        if self.wait_for_action(target_action, timeout_sec=5.0):
                            print("Switch succeeded, would you like to continue switching? (y/n): ", end='', flush=True)
                            choice = input().strip().lower()
                            if choice != 'y':
                                break
                        else:
                            print("Switch failed, please confirm if the expected Action complies with the state machine transition logic")
                    else:
                        print("Switch failed, please confirm if the expected Action complies with the state machine transition logic")
                return True
            except KeyboardInterrupt:
                return True
        else:
            # Optimized logic for 'motion' type: Ensure robot is in BIPED_WALK_RUN
            current_id, current_desc, current_status = self.get_action_status()
            
            if current_desc == 'BIPED_WALK_RUN' and current_status == McActionStatus.RUNNING:
                self.get_logger().info('Robot already in BIPED_WALK_RUN. Proceeding to motion...')
            else:
                self.get_logger().info(f'Current state is {current_desc}. Starting state machine transition sequence...')
                sequence = [
                    'PASSIVE_DEFAULT',
                    'BIPED_STAND_DEFAULT',
                    'BIPED_WALK_RUN'
                ]
                
                # Determine starting point in the sequence to skip redundant steps
                start_index = 0
                if current_desc == 'PASSIVE_DEFAULT':
                    start_index = 1
                elif current_desc == 'BIPED_STAND_DEFAULT':
                    start_index = 2
                
                # Execute the required sequence of states
                for i in range(start_index, len(sequence)):
                    target = sequence[i]
                    self.get_logger().info(f'Pre-requisite: Switching to {target}...')
                    if not self.set_action(target) or not self.wait_for_action(target):
                        return False

            # Execute final target motion
            if not self.set_motion(self.motion, self.interrupt):
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
        # GetMcAction is required for both types to query the current state
        self.wait_for_service(self.get_client, '/aimdk_5Fmsgs/srv/GetMcAction')

        if self.type == 'action':
            self.wait_for_service(self.set_action_client, '/aimdk_5Fmsgs/srv/SetMcAction')
        elif self.type == 'motion':
            self.wait_for_service(self.set_motion_client, '/aimdk_5Fmsgs/srv/SetMcMotion')

    def set_action(self, action_desc: str) -> bool:
        try:
            request = SetMcAction.Request()
            request.header = RequestHeader()

            command = McActionCommand()
            command.action = McAction()
            command.action.value = 0
            command.action_desc = action_desc
            request.command = command

            self.get_logger().info(f'Sending request: action_desc={action_desc}')
            request.header.stamp = self.get_clock().now().to_msg()
            future = self.set_action_client.call_async(request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)

            if not future.done():
                current_action_id, current_action_desc, current_status = self.get_action_status()
                if current_action_desc == action_desc:
                    self.get_logger().warning(
                        'SetMcAction request timed out, but target action is '
                        f'already active: action_desc={current_action_desc} '
                        f'status={current_status}'
                    )
                    return True

                self.get_logger().error('Service call failed or timed out.')
                return False

            response = future.result()
            if response is None:
                current_action_id, current_action_desc, current_status = self.get_action_status()
                if current_action_desc == action_desc:
                    self.get_logger().warning(
                        'SetMcAction request timed out, but target action is '
                        f'already active: action_desc={current_action_desc} '
                        f'status={current_status}'
                    )
                    return True

                self.get_logger().error('Service call failed or timed out.')
                return False

            if response.response.status.value == CommonState.SUCCESS:
                self.get_logger().info('SetMcAction request accepted by service.')
                return True

            self.get_logger().error(
                f'Failed to set robot mode: {response.response.message}'
            )
            return False
        except Exception as e:
            self.get_logger().error(f'Exception occurred: {e}')
            return False

    def set_motion(self, motion_name: str, interrupt: bool) -> bool:
        try:
            max_attempts = 5
            request_timeout_sec = 1.0

            for attempt in range(1, max_attempts + 1):
                request = SetMcMotion.Request()
                request.header = RequestHeader()
                request.header.stamp = self.get_clock().now().to_msg()
                request.motion = motion_name
                request.type = SetMcMotion.Request.MIMIC_QY
                request.interrupt = interrupt

                self.get_logger().info(
                    'Sending SetMcMotion request '
                    f'({attempt}/{max_attempts}): motion={motion_name} '
                    f'interrupt={interrupt}'
                )

                future = self.set_motion_client.call_async(request)
                rclpy.spin_until_future_complete(
                    self, future, timeout_sec=request_timeout_sec
                )

                if not future.done():
                    self.get_logger().warning(
                        'SetMcMotion request attempt '
                        f'{attempt}/{max_attempts} failed or timed out.'
                    )
                    continue

                response = future.result()
                if response is None:
                    self.get_logger().warning(
                        'SetMcMotion request attempt '
                        f'{attempt}/{max_attempts} failed or timed out.'
                    )
                    continue

                code = response.response.header.code
                state = response.response.state.value
                task_id = response.response.task_id

                if code == 0 and state in (
                    CommonState.SUCCESS,
                    CommonState.RUNNING,
                ):
                    self.get_logger().info(
                        'SetMcMotion request accepted by service: '
                        f'code={code} state={state} task_id={task_id}'
                    )
                    return True

                self.get_logger().warning(
                    'SetMcMotion request attempt '
                    f'{attempt}/{max_attempts} was not accepted: '
                    f'code={code} status={status} task_id={task_id}'
                )

            self.get_logger().error(
                f'Failed to set motion after {max_attempts} attempts.'
            )
            return False
        except Exception as e:
            self.get_logger().error(f'Exception occurred: {e}')
            return False

    def wait_for_action(
        self,
        expected_action_desc: str,
        timeout_sec: float = 10.0,
        poll_interval_sec: float = 0.2,
    ) -> bool:
        deadline = time.monotonic() + timeout_sec

        self.get_logger().info(
            'Waiting for target action_desc='
            f'{expected_action_desc} to reach RUNNING state...'
        )

        while rclpy.ok() and time.monotonic() < deadline:
            action_id, action_desc, status = self.get_action_status()
            if action_desc is None or status is None:
                time.sleep(poll_interval_sec)
                continue

            if (
                action_desc == expected_action_desc
                and status == McActionStatus.RUNNING
            ):
                self.get_logger().info(
                    'Target action reached and is running: '
                    f'action_desc={expected_action_desc}'
                )
                return True

            time.sleep(poll_interval_sec)

        self.get_logger().error(
            'Timed out waiting for target action_desc='
            f'{expected_action_desc} to reach RUNNING state.'
        )
        return False

    def wait_for_motion(
        self,
        timeout_sec: float = 10.0,
        poll_interval_sec: float = 0.2,
    ) -> bool:
        deadline = time.monotonic() + timeout_sec

        self.get_logger().info(
            'Waiting for current motion action to reach RUNNING state...'
        )

        while rclpy.ok() and time.monotonic() < deadline:
            action_id, action_desc, status = self.get_action_status()
            if action_desc is None or status is None:
                time.sleep(poll_interval_sec)
                continue

            if status == McActionStatus.RUNNING:
                self.get_logger().info(
                    'Current motion action is running: '
                    f'action_id={action_id} action_desc={action_desc} '
                    f'status={status}'
                )
                return True

            time.sleep(poll_interval_sec)

        self.get_logger().error(
            'Timed out waiting for current motion action to reach RUNNING state.'
        )
        return False

    def get_action_status(self):
        try:
            request = GetMcAction.Request()
            request.request = CommonRequest()
            request.request.header.stamp = self.get_clock().now().to_msg()

            future = self.get_client.call_async(request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)

            if not future.done():
                self.get_logger().warning(
                    'Get current action request service call failed or timed out.'
                )
                return None, None, None

            response = future.result()
            if response is None:
                self.get_logger().warning(
                    'Get current action request service call failed or timed out.'
                )
                return None, None, None

            return (
                response.info.current_action.value,
                response.info.action_desc,
                response.info.status.value,
            )
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
            node = None
        if rclpy.ok():
            rclpy.shutdown()
        return 0
    except Exception as e:
        rclpy.logging.get_logger('main').error(
            f'Program exited with exception: {e}'
        )
        if node is not None:
            node.destroy_node()
            node = None
        if rclpy.ok():
            rclpy.shutdown()
        return 1


if __name__ == '__main__':
    raise SystemExit(main())
