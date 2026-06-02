#!/usr/bin/env python3

"""LED Lights Control Example Script

Description:
  This script demonstrates how to control the robot's LED strip lights using the
  LedStripCommand service. Supports setting LED strip mode, color (RGB), and brightness.

Prerequisites:
  - LedStripCommand service must be available
  - LED hardware must be connected

Usage:
  python3 play_lights.py

Example:
  python3 play_lights.py
"""

import rclpy
import time
import rclpy.logging
from rclpy.node import Node

from aimdk_msgs.msg import CommonRequest
from aimdk_msgs.srv import LedStripCommand


class PlayLightsClient(Node):
    def __init__(self):
        super().__init__("play_lights_client")
        self.client = self.create_client(
            LedStripCommand, "/aimdk_5Fmsgs/srv/LedStripCommand"
        )
        self.get_logger().info("LedStripCommand client node created.")

        while not self.client.wait_for_service(timeout_sec=2.0):
            self.get_logger().info("Service unavailable, waiting...")

        self.get_logger().info("Service available, ready to send request.")

    def send_request(
        self,
        led_strip_mode: int,
        r: int,
        g: int,
        b: int,
        period: int,
    ) -> bool:
        try:
            request = LedStripCommand.Request()
            request.request = CommonRequest()
            request.request.header.stamp = self.get_clock().now().to_msg()
            request.led_strip_mode = led_strip_mode
            request.r = r
            request.g = g
            request.b = b
            request.period = period

            self.get_logger().info(
                "Sending LedStripCommand request: "
                f"led_strip_mode={request.led_strip_mode}, "
                f"r={request.r}, g={request.g}, b={request.b}, "
                f"period={request.period}"
            )

            for i in range(3):
                future = self.client.call_async(request)
                rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)

                if future.done():
                    break

                self.get_logger().info(f'trying ... [{i}]')
                time.sleep(0.2)

            if not future.done():
                self.get_logger().error("LedStripCommand service timeout.")
                return False

            response = future.result()
            if response is None:
                self.get_logger().error("LedStripCommand service call failed.")
                return False

            code = response.header.header.code  
            status_value = response.header.status.value 
            # 打印响应信息
            self.get_logger().info(
                f"Response: code={code}, status_value={status_value}"
            )

            if code == 0 and status_value == 1:  # 判断是否成功
                self.get_logger().info("LedStripCommand request accepted.")
                return True

            self.get_logger().error("LedStripCommand request failed.")
            return False
        except Exception as error:  # noqa: BLE001
            self.get_logger().error(f"Exception occurred: {error}")
            return False


def read_int(prompt: str, default: int) -> int:
    text = input(prompt).strip()
    if text == "":
        return default
    return int(text)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        led_strip_mode = read_int(
            f"Enter led_strip_mode (default {LedStripCommand.Request.LED_WHITE_ON}): ",
            LedStripCommand.Request.LED_WHITE_ON,
        )
        r = 0
        g = 0
        b = 0
        period = 0

        if led_strip_mode == LedStripCommand.Request.LED_CUSTOM:
            r = read_int("Enter r (default 0): ", 0)
            g = read_int("Enter g (default 0): ", 0)
            b = read_int("Enter b (default 255): ", 255)
            period = read_int("Enter period(ms, default 1000): ", 1000)

        node = PlayLightsClient()
        ok = node.send_request(led_strip_mode, r, g, b, period)
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        return 0 if ok else 1
    except Exception as error:  # noqa: BLE001
        rclpy.logging.get_logger("main").error(
            f"Program exited with exception: {error}"
        )
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
