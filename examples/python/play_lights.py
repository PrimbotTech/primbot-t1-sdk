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
from aimdk_msgs.srv import SetNeckLight

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
    9: '当前模式不支持'
}

def get_reason_description(reason: int) -> str:
    """获取失败原因的中文描述"""
    return REASON_DESCRIPTIONS.get(reason, f'未知原因({reason})')


class PlayLightsClient(Node):
    def __init__(self):
        super().__init__("play_lights_client")
        self.LedStripCommand_client = self.create_client(
            LedStripCommand, "/aimdk_5Fmsgs/srv/LedStripCommand"
        )
        self.get_logger().info("LedStripCommand client node created.")

        self.SetNeckLight_client = self.create_client(
            SetNeckLight, "/aimdk_5Fmsgs/srv/SetNeckLight"
        )
        self.get_logger().info("SetNeckLight client node created.")

        # 等待 LED 灯带服务可用
        while not self.LedStripCommand_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().info("Waiting for LedStripCommand service...")
        
        # 等待颈部灯光服务可用
        while not self.SetNeckLight_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().info("Waiting for SetNeckLight service...")

        self.get_logger().info("All services available, ready to send request.")

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
                future = self.LedStripCommand_client.call_async(request)
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
                        
            # 获取失败原因
            reason = getattr(response.header.status, 'reason', 0)
            if reason > 0:
                reason_desc = get_reason_description(reason)
                self.get_logger().warning(
                    f"LedStripCommand rejected: reason={reason} - {reason_desc}"
                )

            self.get_logger().error("LedStripCommand request failed.")
            return False
        except Exception as error:  # noqa: BLE001
            self.get_logger().error(f"Exception occurred: {error}")
            return False

    # 设置颈部灯光
    # 参数:
    #   enable: 开关 (True=开启, False=关闭)
    #   brightness: 亮度 (0-100 百分比)
    # 返回: bool - 请求是否成功
    def set_neck_light(self, enable: bool, brightness: int) -> bool:
        try:
            # 创建服务请求对象
            request = SetNeckLight.Request()
            request.request = CommonRequest()
            request.request.header.stamp = self.get_clock().now().to_msg()
            request.enable = enable
            request.brightness = brightness

            if enable:
                self.get_logger().info(
                    f"Sending SetNeckLight request: enable={enable}, brightness={brightness}%"
                )
            else:
                self.get_logger().info(
                    f"Sending SetNeckLight request: enable={enable}"
                )

            # 重试机制：最多重试 3 次
            for i in range(3):
                future = self.SetNeckLight_client.call_async(request)
                rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)

                if future.done():
                    break

                self.get_logger().info(f'SetNeckLight trying ... [{i}]')
                time.sleep(0.2)

            if not future.done():
                self.get_logger().error("SetNeckLight service timeout.")
                return False

            response = future.result()
            if response is None:
                self.get_logger().error("SetNeckLight service call failed.")
                return False

            # SetNeckLight 响应结构:
            # response.header 是 CommonResponse 类型
            #   ├── header (ResponseHeader)
            #   │     └── code (int64)  ← code 在这里！
            #   ├── status (CommonState)
            #   └── message (string)
            code = response.header.header.code
            self.get_logger().info(f"SetNeckLight Response: code={code}")

            if code == 0:  # 判断是否成功
                self.get_logger().info("SetNeckLight request accepted.")
                return True
            
            # 获取失败原因
            reason = getattr(response.header.status, 'reason', 0)
            if reason > 0:
                reason_desc = get_reason_description(reason)
                self.get_logger().warning(
                    f"SetNeckLight rejected: reason={reason} - {reason_desc}"
                )

            self.get_logger().error(f"SetNeckLight request failed with code {code}")
            return False
        except Exception as error:  # noqa: BLE001
            self.get_logger().error(f"Exception in set_neck_light: {error}")
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
        # Print menu
        print("\n" + "="*60)
        print("  LED Lights Control Menu")
        print("="*60)
        print("\nSelect control mode:")
        print("  1. LED Strip Control")
        print("  2. Neck Light Control")
        print("="*60)
        
        choice = read_int("\nEnter choice (1-2, default 1): ", 1)
        
        node = PlayLightsClient()
        
        if choice == 1:
            # LED strip control
            led_strip_mode = read_int(
                f"\nEnter led_strip_mode (default {LedStripCommand.Request.LED_WHITE_ON}): ",
                LedStripCommand.Request.LED_WHITE_ON,
            )
            r = 0
            g = 0
            b = 0
            period = 0

            if led_strip_mode == LedStripCommand.Request.LED_CUSTOM:
                r = read_int("  Enter r (0-255, default 0): ", 0)
                g = read_int("  Enter g (0-255, default 0): ", 0)
                b = read_int("  Enter b (0-255, default 255): ", 255)
                period = read_int("  Enter period(ms, default 1000): ", 1000)

            ok = node.send_request(led_strip_mode, r, g, b, period)
            if ok:
                print("\n✓ LED strip control request sent")
            else:
                print("\n✗ LED strip control request failed")
                
        elif choice == 2:
            # Neck light control
            print("\n--- Neck Light Control ---")
            enable_input = input("  Enable (true/false, default true): ").strip().lower()
            
            # Handle user input
            if enable_input == "" or enable_input == "true" or enable_input == "1":
                enable = True
            elif enable_input == "false" or enable_input == "0":
                enable = False
            else:
                print("\n✗ Invalid input, please use true/false")
                if node is not None:
                    node.destroy_node()
                if rclpy.ok():
                    rclpy.shutdown()
                return 1
            
            # Only prompt for brightness when light is enabled
            brightness = 50
            if enable:
                brightness = read_int("  Brightness (0-100, default 50): ", 50)
                
                # Validate brightness range
                if brightness < 0 or brightness > 100:
                    print("\n✗ Brightness must be in range 0-100")
                    if node is not None:
                        node.destroy_node()
                    if rclpy.ok():
                        rclpy.shutdown()
                    return 1
            
            ok = node.set_neck_light(enable, brightness)
            if ok:
                if enable:
                    print(f"\n✓ Neck light enabled, brightness {brightness}%")
                else:
                    print("\n✓ Neck light disabled")
            else:
                print("\n✗ Neck light control request failed")
        else:
            print("\n✗ 无效的选项")
        
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        return 0
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
