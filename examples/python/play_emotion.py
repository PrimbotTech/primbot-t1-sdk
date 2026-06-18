#!/usr/bin/env python3

"""Emotion Playback Example Script

Description:
  This script demonstrates how to play robot emotions using the PlayEmotion service.
  Supports both emotion ID mode and file path mode. Includes automatic fallback logic:
  if the default T-series emotion ID (90) fails, it automatically tries the Q-series
  fallback ID (3003).

Prerequisites:
  - PlayEmotion service must be available
  - Emotion assets must be deployed on the robot

Usage:
  python3 play_emotion.py --ros-args -p type:=<type> -p emotion_ids:=<ids>

Parameters:
  - type: "emotion" or "file" (Default: emotion)
  - emotion_ids: Integer array of emotion IDs (Default: [90] for T-series)
  - file_paths: String array of emotion file paths (used when type=file)

Example:
  python3 play_emotion.py --ros-args -p type:=emotion -p emotion_ids:=[90]
"""

import rclpy
import time

import rclpy.logging
from rclpy.node import Node
from rclpy.parameter import Parameter

from aimdk_msgs.msg import CommonRequest, CommonState

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
    14: '上方有障碍物',
    15: '其他任务正在运行',
    16: '机器人已经是目标状态'
}

def get_reason_description(reason: int) -> str:
    """获取失败原因的中文描述"""
    return REASON_DESCRIPTIONS.get(reason, f'未知原因({reason})')
from aimdk_msgs.srv import PlayEmotion


class PlayEmotionClient(Node):
    def __init__(self):
        super().__init__("play_emotion_client")
        self.declare_parameter("type", "emotion")
        self.declare_parameter("emotion_ids", [90])
        self.declare_parameter("file_paths", Parameter.Type.STRING_ARRAY)

        self.type = self.get_parameter("type").value
        self.emotion_ids = self.get_parameter_or(
            "emotion_ids",
            Parameter("emotion_ids", Parameter.Type.INTEGER_ARRAY, [90]),
        ).value
        self.file_paths = self.get_parameter_or(
            "file_paths",
            Parameter("file_paths", Parameter.Type.STRING_ARRAY, []),
        ).value
        self.priority = 0
        self.loop_count = 1

        self.client = self.create_client(PlayEmotion, "/aimdk_5Fmsgs/srv/PlayEmotion")
        self.get_logger().info("PlayEmotion client node created.")

        while not self.client.wait_for_service(timeout_sec=2.0):
            self.get_logger().info("Waiting for service...")
        self.get_logger().info("Service available, ready to send request.")

    def send_request(self) -> bool:
        try:
            if not self.validate_parameters():
                return False

            # 一套代码兼容不同机型，T系列默认ID=90，Q系列默认ID=3003
            # 第一步：尝试原始请求（默认 ID 为 90）
            ok = self._call_service(self.type, list(self.emotion_ids), list(self.file_paths))

            # 第二步：降级逻辑
            # 如果是播放表情模式，且尝试 ID 10 失败，则自动尝试播放保底 ID 3003
            if not ok and self.type == "emotion" and 90 in self.emotion_ids:
                # 尝试播放保底表情 3003
                ok = self._call_service("emotion", [3003], [])

            if not ok:
                self.get_logger().error("PlayEmotion request failed after all attempts.")
                return False

            return True
        except Exception as error:  # noqa: BLE001
            self.get_logger().error(f"Exception occurred: {error}")
            return False

    def _call_service(self, msg_type, emotion_ids, file_paths) -> bool:
        request = PlayEmotion.Request()
        request.header = CommonRequest()
        request.header.header.stamp = self.get_clock().now().to_msg()
        request.type = msg_type
        request.priority = self.priority
        request.loop_count = self.loop_count
        request.emotion_ids = emotion_ids
        request.file_paths = file_paths

        self.get_logger().info(
            f"Sending PlayEmotion request: type={msg_type}, "
            f"emotion_ids={emotion_ids}, file_paths={file_paths}"
        )

        for i in range(3):
            future = self.client.call_async(request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)

            if future.done():
                break

            self.get_logger().info(f'trying ... [{i}]')
            time.sleep(0.2)

        if not future.done():
            return False

        response = future.result()
        if response is None:
            return False

        code = response.header.header.code
        status = response.header.status.value
        if code == 0 or status == CommonState.SUCCESS:
            self.get_logger().info(f"Request accepted (code={code}, status={status}).")
            return True

        # 获取失败原因
        reason = getattr(response.header.status, 'reason', 0)
        if reason > 0:
            reason_desc = get_reason_description(reason)
            self.get_logger().warning(
                f"Request rejected: reason={reason} - {reason_desc}"
            )
        
        self.get_logger().warning(f"Request rejected by service (code={code}, status={status}).")
        return False

    def validate_parameters(self) -> bool:
        if self.type not in ("emotion", "file"):
            self.get_logger().error(
                f"Invalid parameter 'type': {self.type}. Use 'emotion' or 'file'."
            )
            return False

        if self.type == "emotion" and not self.emotion_ids:
            self.get_logger().error(
                "Parameter 'emotion_ids' must be set when type=emotion."
            )
            return False

        if self.type == "file" and not self.file_paths:
            self.get_logger().error(
                "Parameter 'file_paths' must be set when type=file."
            )
            return False

        return True


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = PlayEmotionClient()
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
