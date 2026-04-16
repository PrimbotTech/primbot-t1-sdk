#!/usr/bin/env python3

import time

import rclpy
from rclpy.node import Node

from aimdk_msgs.msg import CommonRequest, McLocomotionVelocity, MessageHeader
from aimdk_msgs.srv import GetCurrentInputSource, SetMcInputSource


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

    def start_publish(self):
        if self.timer is None:
            self.timer = self.create_timer(0.02, self.publish_velocity)

    def register_input_source(self) -> bool:
        # Register the input source before publishing velocity.
        if not self.wait_for_service(
            self.set_client, "/aimdk_5Fmsgs/srv/SetMcInputSource"
        ):
            return False

        request = SetMcInputSource.Request()
        request.action.value = 1001
        request.input_source.name = "node"
        request.input_source.priority = 80
        request.input_source.timeout = 1000
        request.request.header.stamp = self.get_clock().now().to_msg()

        future = self.set_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)

        if not future.done():
            self.get_logger().error("SetMcInputSource failed or timed out")
            return False

        try:
            response = future.result()
        except Exception as exc:
            self.get_logger().error(f"SetMcInputSource failed: {exc}")
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
            "SetMcInputSource returned "
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

        future = self.get_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)

        if not future.done():
            self.get_logger().warning("GetCurrentInputSource timed out")
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


def main(args=None):
    rclpy.init(args=args)
    node = DirectVelocityControl()

    try:
        node.get_current_input_source()

        if not node.register_input_source():
            node.get_logger().error("Input source registration failed, exiting")
            return 1

        # Input speed must be 0, or have an absolute value at least the minimum threshold.
        try:
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
        node.publish_velocity()
        node.get_logger().info("5 seconds elapsed; robot stopped")
        return 0
    finally:
        if node.timer is not None:
            node.timer.cancel()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
