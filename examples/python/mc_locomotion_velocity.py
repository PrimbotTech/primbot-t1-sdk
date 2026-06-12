#!/usr/bin/env python3

"""
Example client for /aima/mc/locomotion/velocity.

This script automatically handles the required state machine transitions for safety.
Locomotion control (walking/running) requires the robot to be in QUADRUPED_LOCOMOTION_DEFAULT mode.

Prerequisites auto-handled by this script:
  The script ensures a safe sequential transition path:
  PASSIVE_DEFAULT -> QUADRUPED_STAND_DEFAULT -> QUADRUPED_LOCOMOTION_DEFAULT
  Depending on the initial state, it enters the sequence at the appropriate step.

Flow:
  1. Detect current state and transition to QUADRUPED_LOCOMOTION_DEFAULT sequentially.
  2. Register this node as an authorized input source (priority 80).
  3. Prompt the user for target velocities.
  4. Publish velocity commands for 5 seconds.
  5. Stop the robot by sending zero velocity.
  6. Release the registered input source.

Usage:
  python3 examples/python/mc_locomotion_velocity.py
"""

import signal
import time

import rclpy
from rclpy.node import Node

from aimdk_msgs.msg import (
    CommonRequest, CommonState, McAction, McActionCommand, 
    McActionStatus, McLocomotionVelocity, MessageHeader, RequestHeader
)
from aimdk_msgs.srv import GetCurrentInputSource, SetMcInputSource, GetMcAction, SetMcAction

# CommonState reason 字段对应的中文描述
REASON_DESCRIPTIONS = {
    0: '无错误',
    1: '开箱状态中',
    2: '开机自检中',
    3: '关机状态中',
    4: '当前形态不支持',
    5: '低电量限制',
    6: '正在充电中',
    7: '动作不在白名单',
    8: 'HDS故障',
    9: '当前模式不支持',
    10: '前方有障碍物',
    11: '后方有障碍物',
    12: '左方有障碍物',
    13: '右方有障碍物',
    14: '上方有障碍物'
}

def get_reason_description(reason: int) -> str:
    """获取失败原因的中文描述"""
    return REASON_DESCRIPTIONS.get(reason, f'未知原因({reason})')

SERVICE_CALL_TIMEOUT_SEC = 2.0
MAX_RETRY_COUNT = 3


class DirectVelocityControl(Node):
    def __init__(self):
        super().__init__("direct_velocity_control")

        self.publisher = self.create_publisher(
            McLocomotionVelocity, "/aima/mc/locomotion/velocity", 10
        )
        self.set_client = self.create_client(
            SetMcInputSource, "/aimdk_5Fmsgs/srv/SetMcInputSource"
        )
        self.get_client = self.create_client(
            GetCurrentInputSource, "/aimdk_5Fmsgs/srv/GetCurrentInputSource"
        )
        self.set_action_client = self.create_client(
            SetMcAction, "/aimdk_5Fmsgs/srv/SetMcAction"
        )
        self.get_action_client = self.create_client(
            GetMcAction, "/aimdk_5Fmsgs/srv/GetMcAction"
        )

        self.forward_velocity = 0.0
        self.lateral_velocity = 0.0
        self.angular_velocity = 0.0

        self.max_forward_speed = 2.0
        self.max_lateral_speed = 1.0
        self.max_angular_speed = 2.5

        self.min_forward_speed = 0.1
        self.min_lateral_speed = 0.3
        self.min_angular_speed = 0.8

        self.timer = None

        self.get_logger().info("Direct velocity control node started.")

    def wait_for_service(self, client, service_name: str) -> bool:
        while not client.wait_for_service(timeout_sec=2.0):
            if not rclpy.ok():
                return False
            self.get_logger().info(f"Waiting for service: {service_name}")
        return True

    def wait_for_services(self) -> bool:
        clients = [
            (self.publisher, "/aima/mc/locomotion/velocity"),
            (self.set_client, "/aimdk_5Fmsgs/srv/SetMcInputSource"),
            (self.get_client, "/aimdk_5Fmsgs/srv/GetCurrentInputSource"),
            (self.set_action_client, "/aimdk_5Fmsgs/srv/SetMcAction"),
            (self.get_action_client, "/aimdk_5Fmsgs/srv/GetMcAction")
        ]
        for client, name in clients:
            if hasattr(client, 'wait_for_service'):
                if not self.wait_for_service(client, name):
                    return False
        return True

    def call_service_with_retry(self, client, request, service_name: str, timeout_sec=None, max_retries=None):
        if timeout_sec is None:
            timeout_sec = SERVICE_CALL_TIMEOUT_SEC
        if max_retries is None:
            max_retries = MAX_RETRY_COUNT
            
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
            
            future = self.call_service_with_retry(
                self.get_action_client, request, "GetMcAction"
            )
            if future is None:
                return None, None, None
                
            res = future.result()
            if res is None:
                return None, None, None
                
            return res.info.current_action.value, res.info.action_desc, res.info.status.value
        except Exception as e:
            self.get_logger().error(f"Error getting action status: {e}")
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
            
            self.get_logger().info(f"Requesting state switch to: {action_desc}")
            
            future = self.call_service_with_retry(
                self.set_action_client, request, "SetMcAction"
            )
            if future is None:
                return False
                
            res = future.result()
            if res is not None and res.response.status.value == CommonState.SUCCESS:
                return True
            
            # 获取失败原因
            if res is not None:
                reason = getattr(res.response.status, 'reason', 0)
                if reason > 0:
                    reason_desc = get_reason_description(reason)
                    self.get_logger().warning(
                        f'SetMcAction rejected: reason={reason} - {reason_desc}'
                    )
            
            return False
        except Exception as e:
            self.get_logger().error(f"Error calling SetMcAction: {e}")
            return False

    def wait_for_action(self, target_desc: str, timeout_sec: float = 10.0) -> bool:
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline:
            _, desc, status = self.get_action_status()
            # Locomotion modes (walk/run) report status=2 (TRANSITION) instead of
            # RUNNING (100), so accept any non-IDLE status when action_desc matches.
            if desc == target_desc and status != McActionStatus.IDLE:
                self.get_logger().info(f"Robot successfully reached state: {target_desc} (status={status})")
                return True
            time.sleep(0.5)
        self.get_logger().error(f"Timeout waiting for state: {target_desc}")
        return False

    def ensure_ready_state(self) -> bool:
        _, desc, status = self.get_action_status()
        
        # Safety margin: If initial state is None, poll for up to 5s to recover communication
        if desc is None:
            self.get_logger().warning("Initial action status is None. Retrying for up to 5 seconds...")
            deadline = time.monotonic() + 5.0
            while time.monotonic() < deadline:
                time.sleep(0.5)
                _, desc, status = self.get_action_status()
                if desc is not None:
                    self.get_logger().info(f"Successfully recovered action status: {desc}")
                    break
            
            if desc is None:
                self.get_logger().error("Action status remained None after 5 seconds of polling.")

        if desc == 'QUADRUPED_LOCOMOTION_DEFAULT' and status != McActionStatus.IDLE:
            return True

        self.get_logger().info(f"Current state is {desc}. Starting state machine transition sequence...")
        
        # Define the target sequence of states for walking
        sequence = [
            'PASSIVE_DEFAULT',
            'QUADRUPED_STAND_DEFAULT',
            'QUADRUPED_LOCOMOTION_DEFAULT'
        ]
        
        # Determine starting point in the sequence
        start_index = 0
        if desc == 'PASSIVE_DEFAULT':
            start_index = 1
        elif desc == 'QUADRUPED_STAND_DEFAULT':
            start_index = 2
        elif desc in ['DAMPING_DEFAULT']:
            start_index = 0
        else:
            start_index = 0
            
        # Execute the sequence from the determined start point
        for i in range(start_index, len(sequence)):
            target = sequence[i]
            if not self.set_action(target) or not self.wait_for_action(target):
                return False
                
        return True

    def start_publish(self):
        if self.timer is None:
            self.timer = self.create_timer(0.02, self.publish_velocity)

    def register_input_source(self) -> bool:
        # Register the input source before publishing velocity.
        if not self.wait_for_service(
            self.set_client, "/aimdk_5Fmsgs/srv/SetMcInputSource"
        ):
            return False

        # 先清理残留的同名输入源（上次运行可能未正常释放）
        del_request = SetMcInputSource.Request()
        del_request.action.value = 1003  # INPUTACTION_DELETE
        del_request.input_source.name = "node"
        del_request.request.header.stamp = self.get_clock().now().to_msg()
        del_future = self.call_service_with_retry(
            self.set_client, del_request, "SetMcInputSource(DELETE)"
        )
        if del_future is not None:
            try:
                del_resp = del_future.result()
                if del_resp.response.header.code == 0:
                    self.get_logger().info("Cleaned up leftover input source 'node'")
            except Exception:
                pass

        request = SetMcInputSource.Request()
        request.action.value = 1001  # INPUTACTION_ADD
        request.input_source.name = "node"
        request.input_source.priority = 80
        request.input_source.timeout = 1000
        request.request.header.stamp = self.get_clock().now().to_msg()

        future = self.call_service_with_retry(
            self.set_client, request, "SetMcInputSource(ADD)"
        )
        if future is None:
            return False

        try:
            response = future.result()
        except Exception as exc:
            self.get_logger().error(f"SetMcInputSource(ADD) failed: {exc}")
            return False

        ret_code = response.response.header.code
        state = response.response.state.value
        task_id = response.response.task_id

        if ret_code == 0:
            self.get_logger().info(
                "Set input source succeeded: "
                f"code={ret_code}, state={state}, task_id={task_id}"
            )
            return True

        self.get_logger().warning(
            "SetMcInputSource(ADD) returned "
            f"code={ret_code}, state={state}, task_id={task_id}"
        )
        return False

    def get_current_input_source(self) -> bool:
        if not self.wait_for_service(
            self.get_client, "/aimdk_5Fmsgs/srv/GetCurrentInputSource"
        ):
            return False

        self.get_logger().info("Querying current input source")

        request = GetCurrentInputSource.Request()
        request.request = CommonRequest()
        request.request.header.stamp = self.get_clock().now().to_msg()

        future = self.call_service_with_retry(
            self.get_client, request, "GetCurrentInputSource"
        )
        if future is None:
            return False

        try:
            response = future.result()
        except Exception as exc:
            self.get_logger().warning(f"GetCurrentInputSource failed: {exc}")
            return False

        ret_code = response.response.header.code
        if ret_code == 0:
            self.get_logger().info(
                "Current input source: "
                f"name={response.input_source.name}, "
                f"priority={response.input_source.priority}, "
                f"timeout={response.input_source.timeout}"
            )
            return True

        self.get_logger().warning(
            f"GetCurrentInputSource returned code={ret_code}"
        )
        return False

    def publish_velocity(self):
        msg = McLocomotionVelocity()
        msg.header = MessageHeader()
        msg.header.stamp = self.get_clock().now().to_msg()
        # source must match the registered input-source name.
        msg.source = "node"
        msg.forward_velocity = self.forward_velocity
        msg.lateral_velocity = self.lateral_velocity
        msg.angular_velocity = self.angular_velocity
        msg.pitch_velocity = 0.0
        msg.level = 0.0

        self.publisher.publish(msg)

    def clear_velocity(self):
        self.forward_velocity = 0.0
        self.lateral_velocity = 0.0
        self.angular_velocity = 0.0

    def set_forward(self, forward: float) -> bool:
        if abs(forward) < 0.005:
            self.forward_velocity = 0.0
            return True
        if abs(forward) > self.max_forward_speed or abs(forward) < self.min_forward_speed:
            self.get_logger().error("Input forward value out of range, exiting")
            return False
        self.forward_velocity = forward
        return True

    def set_lateral(self, lateral: float) -> bool:
        if abs(lateral) < 0.005:
            self.lateral_velocity = 0.0
            return True
        if abs(lateral) > self.max_lateral_speed or abs(lateral) < self.min_lateral_speed:
            self.get_logger().error("Input lateral value out of range, exiting")
            return False
        self.lateral_velocity = lateral
        return True

    def set_angular(self, angular: float) -> bool:
        if abs(angular) < 0.005:
            self.angular_velocity = 0.0
            return True
        if abs(angular) > self.max_angular_speed or abs(angular) < self.min_angular_speed:
            self.get_logger().error("Input angular value out of range, exiting")
            return False
        self.angular_velocity = angular
        return True

    def release_input_source(self) -> bool:
        request = SetMcInputSource.Request()
        request.action.value = 1003
        request.input_source.name = "node"
        request.request.header.stamp = self.get_clock().now().to_msg()

        future = self.call_service_with_retry(
            self.set_client, request, "SetMcInputSource"
        )
        if future is None:
            return False

        try:
            response = future.result()
        except Exception as exc:
            self.get_logger().error(f"SetMcInputSource failed: {exc}")
            return False

        ret_code = response.response.header.code

        if ret_code == 0:
            self.get_logger().info("Input source released successfully.")
            return True

        self.get_logger().warning(f"SetMcInputSource returned code={ret_code}")
        return False

def main(args=None):
    rclpy.init(args=args)
    # init 之后立即覆盖 ROS2 的 SIGINT 处理器
    # 阻止 Ctrl+C 触发 rclpy.shutdown()，保持上下文有效以释放输入源
    def _sigint_handler(sig, frame):
        raise KeyboardInterrupt
    signal.signal(signal.SIGINT, _sigint_handler)

    node = DirectVelocityControl()
    input_source_registered = False

    try:
        if not node.wait_for_services():
            return 1

        # Step 1: Ensure the robot is in QUADRUPED_LOCOMOTION_DEFAULT state
        if not node.ensure_ready_state():
            node.get_logger().error("Failed to prepare robot state for walking.")
            return 1

        # Step 2: Register input source
        if not node.register_input_source():
            node.get_logger().error("Input source registration failed, exiting")
            return 1
        input_source_registered = True

        # Input speed must be 0, or have an absolute value at least the minimum threshold.
        try:
            print("\nEnter control velocities (forward and lateral in m/s, angular in rad/s):")
            forward = float(input("Enter forward speed 0 or +/- (0.1 ~ 2.0) m/s: "))
            lateral = float(input("Enter lateral speed 0 or +/- (0.3 ~ 1.0) m/s: "))
            angular = float(input("Enter angular speed 0 or +/- (0.8 ~ 2.5) rad/s: "))
        except ValueError as exc:
            node.get_logger().error(f"Invalid input: {exc}")
            return 2

        if not node.set_forward(forward):
            return 2
        if not node.set_lateral(lateral):
            return 2
        if not node.set_angular(angular):
            return 2

        node.get_logger().info(
            "Start publishing velocity for 5 seconds: "
            f"Forward {forward:.2f} m/s, "
            f"Lateral {lateral:.2f} m/s, "
            f"Angular {angular:.2f} rad/s"
        )

        node.start_publish()

        start_time = node.get_clock().now()
        queried_after_publish = False
        while (node.get_clock().now() - start_time).nanoseconds / 1e9 < 5.0:
            elapsed = (node.get_clock().now() - start_time).nanoseconds / 1e9
            if not queried_after_publish and elapsed > 1.0:
                node.get_current_input_source()
                queried_after_publish = True

            rclpy.spin_once(node, timeout_sec=0.1)
            time.sleep(0.001)

        node.clear_velocity()
        # Ensure zero velocity is published
        node.publish_velocity()
        node.get_logger().info("5 seconds elapsed; robot stopped")
        node.release_input_source()
        input_source_registered = False
        return 0
    finally:
        if input_source_registered:
            node.get_logger().info("Releasing input source on exit...")
            node.release_input_source()
        if node.timer is not None:
            node.timer.cancel()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
