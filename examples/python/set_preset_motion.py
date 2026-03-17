#!/usr/bin/env python3

import rclpy
import rclpy.logging
from rclpy.node import Node

from aimdk_msgs.srv import SetMcPresetMotion
from aimdk_msgs.msg import McPresetMotion, RequestHeader, CommonState


class SetMcPresetMotionClient(Node):
    def __init__(self):
        super().__init__('preset_motion_client')
        self.client = self.create_client(
            SetMcPresetMotion, '/aimdk_5Fmsgs/srv/SetMcPresetMotion')
        self.get_logger().info('✅ SetMcPresetMotion client node created.')

        # Wait for the service to become available
        while not self.client.wait_for_service(timeout_sec=2.0):
            self.get_logger().info('⏳ Service unavailable, waiting...')

        self.get_logger().info('🟢 Service available, ready to send request.')

    def send_request(self, motion_id: int) -> bool:
        request = SetMcPresetMotion.Request()
        request.header = RequestHeader()

        motion = McPresetMotion()

        motion.value = motion_id

        request.motion = motion
        request.interrupt = True

        self.get_logger().info(
            f'📨 Sending request to set preset motion: motion={motion_id}')

        request.header.stamp = self.get_clock().now().to_msg()
        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)

        if not future.done():
            self.get_logger().error('❌ Service call failed or timed out.')
            return False

        response = future.result()
        if response is None:
            self.get_logger().error('❌ Service call failed or timed out.')
            return False

        code = response.response.header.code
        state = response.response.state.value

        if code != 0:
            self.get_logger().warning(
                f'❌ Failed to set preset motion: code={code}, state={state}, task_id={response.response.task_id}')
            return False

        if state == CommonState.SUCCESS:
            self.get_logger().info(
                f'✅ Preset motion set successfully: {response.response.task_id}')
            return True
        elif state == CommonState.RUNNING:
            self.get_logger().info(
                f'⏳ Preset motion executing: {response.response.task_id}')
            return True
        else:
            self.get_logger().warning(
                f'❌ Failed to set preset motion: code={code}, state={state}, task_id={response.response.task_id}')
            return False


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:    # TODO: Change input validation for motion ID
        motion = int(input(
            "Enter preset motion ID (1001-raise，1002-wave，1003-handshake，1004-airkiss): "))

        node = SetMcPresetMotionClient()
        node.send_request(motion)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        rclpy.logging.get_logger('main').error(
            f'Program exited with exception: {e}')

    if node:
        node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()
