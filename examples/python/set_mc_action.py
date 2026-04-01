#!/usr/bin/env python3

"""
Example client for /aimdk_5Fmsgs/srv/SetMcAction and /aimdk_5Fmsgs/srv/SetMcMotion

The following ROS parameters can be set via startup arguments:
--ros-args -p <name>:=<value>

Supported parameters:
  - type: "action" or "motion", required
  - action_desc: string, required when type=action
  - motion: string, required when type=motion
  - interrupt: bool, optional when type=motion, default=true

Examples:
  python3 examples/python/set_mc_action.py --ros-args -p type:=action -p \
  action_desc:=BIPED_STAND_DEFAULT

  python3 examples/python/set_mc_action.py --ros-args -p type:=motion -p \
  motion:=INTRO_POSE6 -p interrupt:=true
"""

import time

import rclpy
import rclpy.logging
from rclpy.node import Node

from aimdk_msgs.msg import CommonRequest, CommonState, McAction, McActionCommand, McActionStatus, RequestHeader, McMotionType
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
            action_id, action_desc, status = self.get_action_status()
            if action_desc == self.action_desc:
                if status == McActionStatus.RUNNING:
                    self.get_logger().info(
                        'Target action is already running: '
                        f'action_desc={self.action_desc}'
                    )
                    return True

                self.get_logger().info(
                    'Target action is already active with '
                    f'status={status}, waiting for RUNNING: '
                    f'action_desc={self.action_desc}'
                )
                return self.wait_for_action(self.action_desc)

            if not self.set_action(self.action_desc):
                return False
            return self.wait_for_action(self.action_desc)

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

        if self.type == 'action' and not self.action_desc:
            self.get_logger().error(
                "Parameter 'action_desc' must be set when type=action."
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
        self.wait_for_service(self.set_action_client, '/aimdk_5Fmsgs/srv/SetMcAction')
        self.wait_for_service(self.set_motion_client, '/aimdk_5Fmsgs/srv/SetMcMotion')
        self.wait_for_service(self.get_client, '/aimdk_5Fmsgs/srv/GetMcAction')

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
                request.motion.tag = motion_name
                request.motion.type.value = McMotionType.MIMIC
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
                status = response.response.status.value
                task_id = response.response.task_id

                if code == 0 and status in (
                    CommonState.SUCCESS,
                    CommonState.RUNNING,
                ):
                    self.get_logger().info(
                        'SetMcMotion request accepted by service: '
                        f'code={code} status={status} task_id={task_id}'
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
