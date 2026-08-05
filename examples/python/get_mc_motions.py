#!/usr/bin/env python3

"""
Get MC Motions Example Script

Description:
  This script demonstrates how to call the GetMcMotions service to retrieve
  the list of all available motions registered in the MC module.
  Each motion includes its name, type, and interrupt information.

Prerequisites:
  - Robot MC service must be running
  - GetMcMotions service must be available

Usage:
  python3 get_mc_motions.py

Example:
  python3 get_mc_motions.py

Parameters:
  - None
"""

import rclpy
from rclpy.node import Node

from aimdk_msgs.srv import GetMcMotions


# ---------- 枚举映射 ----------

MOTION_TYPE_MAP = {
    0: "NONE",
    1: "ANIMATION",
    2: "MIMIC",
    3: "FOUNDATION",
    4: "MIMIC_OPENSOURCE",
}

INTR_TYPE_MAP = {0: "INTR_NO", 1: "INTR_FREE", 2: "INTR_BREAKPOINT"}

SERVICE_NAME = "/aimdk_5Fmsgs/srv/GetMcMotions"
CALL_TIMEOUT_SEC = 5.0


class GetMcMotionsClient(Node):
    def __init__(self):
        super().__init__("get_mc_motions")

        self.client = self.create_client(GetMcMotions, SERVICE_NAME)

        self.get_logger().info(
            f"GetMcMotions client created. Waiting for service: {SERVICE_NAME}"
        )

        if not self.client.wait_for_service(timeout_sec=10.0):
            self.get_logger().error(
                f"Service {SERVICE_NAME} not available after 10s. Is MC running?"
            )
            raise SystemExit(1)

        self.get_logger().info(f"Service {SERVICE_NAME} is available.")

    def call(self) -> bool:
        request = GetMcMotions.Request()
        # RequestHeader 使用默认值即可
        request.header.stamp.sec = 0
        request.header.stamp.nanosec = 0

        self.get_logger().info("Sending GetMcMotions request...")

        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=CALL_TIMEOUT_SEC)

        if not future.done():
            self.get_logger().error(
                f"GetMcMotions timed out after {int(CALL_TIMEOUT_SEC * 1000)} ms."
            )
            return False

        response = future.result()
        if response is None:
            self.get_logger().error("GetMcMotions returned an empty response.")
            return False

        # --- 打印结果 ---
        code = response.response.header.code
        state = response.response.state.value
        message = response.response.message
        motions = response.motion

        self.get_logger().info(
            f"Response: code={code}, state={state}, message='{message}'"
        )
        self.get_logger().info(f"Total motions: {len(motions)}")
        self.get_logger().info("-" * 70)
        self.get_logger().info(
            f"{'#':<4} {'Name':<35} {'Type':<18} {'IntrType':<16} {'Breakpoints'}"
        )
        self.get_logger().info("-" * 70)

        for i, m in enumerate(motions):
            type_str = MOTION_TYPE_MAP.get(m.type.value, str(m.type.value))
            intr_str = INTR_TYPE_MAP.get(
                m.intr_info.intr_type.value, str(m.intr_info.intr_type.value)
            )
            breakpoints = (
                str(list(m.intr_info.breakpoint_steps))
                if m.intr_info.breakpoint_steps
                else "[]"
            )
            self.get_logger().info(
                f"{i:<4} {m.tag:<35} {type_str:<18} {intr_str:<16} {breakpoints}"
            )

        return True


def main(args=None):
    rclpy.init(args=args)
    node = GetMcMotionsClient()
    try:
        success = node.call()
        return 0 if success else 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
