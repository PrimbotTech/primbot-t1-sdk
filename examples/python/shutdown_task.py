#!/usr/bin/env python3

"""
主动关机示例脚本

Description:
  通过 ShutdownTask 服务请求机器人执行主动关机操作。
  该接口在 task engine 层实现，需要设置来源标识（source）。

  警告：调用此接口后，系统将进入安全关机流程，依次停止各子系统服务后断电。
  请确保在调用前已保存重要数据。

Prerequisites:
  - Task engine 服务必须正在运行
  - 机器人在安全状态下可执行关机

Usage:
  python3 shutdown_task.py

Example:
  python3 shutdown_task.py

Parameters:
  - None
"""

import time

import rclpy
import rclpy.logging
from rclpy.node import Node

from aimdk_msgs.msg import CommonRequest, TokenCommon
from aimdk_msgs.srv import ShutdownTask

SERVICE_WAIT_SEC = 2.0
SERVICE_CALL_TIMEOUT_SEC = 5.0
MAX_RETRY_COUNT = 3


class ShutdownTaskClient(Node):
    def __init__(self):
        super().__init__("shutdown_task_client")
        self.client = self.create_client(
            ShutdownTask, "/aimdk_5Fmsgs/srv/ShutdownTask"
        )
        self.get_logger().info("ShutdownTask client node created.")

        while not self.client.wait_for_service(timeout_sec=SERVICE_WAIT_SEC):
            if not rclpy.ok():
                return
            self.get_logger().info("Waiting for service...")
        self.get_logger().info("Service available, ready to send request.")

    def send_request(self, source: str = "sdk_example") -> bool:
        request = ShutdownTask.Request()
        request.header = CommonRequest()
        request.header.header.stamp = self.get_clock().now().to_msg()
        request.token = TokenCommon()
        request.token.source = source
        request.token.token = ""

        self.get_logger().info(
            f"Sending ShutdownTask request: source={source}"
        )

        for i in range(MAX_RETRY_COUNT):
            future = self.client.call_async(request)
            rclpy.spin_until_future_complete(
                self, future, timeout_sec=SERVICE_CALL_TIMEOUT_SEC
            )

            if future.done():
                break

            self.get_logger().info(f"trying ... [{i}]")
            time.sleep(0.2)

        if not future.done():
            self.get_logger().error(
                f"ShutdownTask timed out after "
                f"{int(SERVICE_CALL_TIMEOUT_SEC * 1000)} ms."
            )
            return False

        try:
            response = future.result()
        except Exception as error:  # noqa: BLE001
            self.get_logger().error(f"ShutdownTask failed: {error}")
            return False

        if response is None:
            self.get_logger().error("ShutdownTask returned no response.")
            return False

        code = response.header.header.code
        if code == 0:
            self.get_logger().info(
                "ShutdownTask request sent successfully. "
                "Robot is shutting down..."
            )
            return True

        self.get_logger().error(
            f"ShutdownTask failed with code: {code} "
            f"status: {response.header.status.value} "
            f"reason: {response.header.status.reason} "
            f"message: {response.header.message}"
        )
        return False


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = ShutdownTaskClient()
        ok = node.send_request()
        return 0 if ok else 1
    except Exception as error:  # noqa: BLE001
        rclpy.logging.get_logger("main").error(
            f"Program exited with exception: {error}"
        )
        return 1
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
