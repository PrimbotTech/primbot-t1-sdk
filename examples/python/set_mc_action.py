#!/usr/bin/env python3

import time

import rclpy
import rclpy.logging
from rclpy.node import Node

from aimdk_msgs.msg import (
    CommonRequest,
    CommonState,
    McAction,
    McActionCommand,
    McActionStatus,
    RequestHeader,
)
from aimdk_msgs.srv import GetMcAction, SetMcAction


class SetMcActionClient(Node):
    def __init__(self):
        super().__init__('set_mc_action_client')
        self.set_client = self.create_client(
            SetMcAction, '/aimdk_5Fmsgs/srv/SetMcAction'
        )
        self.get_client = self.create_client(
            GetMcAction, '/aimdk_5Fmsgs/srv/GetMcAction'
        )
        self.get_logger().info('✅ SetMcAction client node created.')

        self.wait_for_service(self.set_client, '/aimdk_5Fmsgs/srv/SetMcAction')
        self.wait_for_service(self.get_client, '/aimdk_5Fmsgs/srv/GetMcAction')

    def wait_for_service(self, client, service_name: str):
        while not client.wait_for_service(timeout_sec=2.0):
            if not rclpy.ok():
                return
            self.get_logger().info(f'⏳ Service unavailable, waiting: {service_name}')
        self.get_logger().info(f'🟢 Service available: {service_name}')

    def set_action(self, action_id: int) -> bool:
        try:
            request = SetMcAction.Request()
            request.header = RequestHeader()

            command = McActionCommand()
            command.action = McAction()
            command.action.value = action_id
            command.action_desc = ''
            request.command = command

            self.get_logger().info(f'📨 Sending request: action_id={action_id}')
            request.header.stamp = self.get_clock().now().to_msg()
            future = self.set_client.call_async(request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)

            if not future.done():
                self.get_logger().error('❌ Service call failed or timed out.')
                return False

            response = future.result()
            if response is None:
                self.get_logger().error('❌ Service call failed or timed out.')
                return False

            if response.response.status.value == CommonState.SUCCESS:
                self.get_logger().info('✅ SetMcAction request accepted by service.')
                return True

            self.get_logger().error(
                f'❌ Failed to set robot mode: {response.response.message}'
            )
            return False
        except Exception as e:
            self.get_logger().error(f'Exception occurred: {e}')
            return False

    def wait_for_action(
        self,
        expected_action_id: int,
        timeout_sec: float = 15.0,
        poll_interval_sec: float = 0.2,
    ) -> bool:
        deadline = time.monotonic() + timeout_sec

        self.get_logger().info(
            f'⏳ Waiting for target action_id={expected_action_id} to reach RUNNING state...'
        )

        while rclpy.ok() and time.monotonic() < deadline:
            action_id, status = self.get_action_status()
            if action_id is None or status is None:
                time.sleep(poll_interval_sec)
                continue

            if (
                action_id == expected_action_id
                and status == McActionStatus.RUNNING
            ):
                self.get_logger().info(
                    f'✅ Target action reached and is running: action_id={expected_action_id}'
                )
                return True

            time.sleep(poll_interval_sec)

        self.get_logger().error(
            f'❌ Timed out waiting for target action_id={expected_action_id} '
            'to reach RUNNING state.'
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
                    '⚠️ Get current action request service call failed or timed out.'
                )
                return None, None

            response = future.result()
            if response is None:
                self.get_logger().warning(
                    '⚠️ Get current action request service call failed or timed out.'
                )
                return None, None

            return response.info.current_action.value, response.info.status.value
        except Exception as e:
            self.get_logger().error(f'Exception occurred: {e}')
            return None, None


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        action_id = 0
        print('Enter action_id: ', end='')
        try:
            action_id = int(input())
        except Exception:
            pass

        node = SetMcActionClient()
        if not node.set_action(action_id):
            node.destroy_node()
            node = None
            if rclpy.ok():
                rclpy.shutdown()
            return 1

        if not node.wait_for_action(action_id):
            node.destroy_node()
            node = None
            if rclpy.ok():
                rclpy.shutdown()
            return 1

        node.destroy_node()
        node = None
        if rclpy.ok():
            rclpy.shutdown()
        return 0
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
