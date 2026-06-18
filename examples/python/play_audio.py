#!/usr/bin/env python3

"""Audio Playback Example Script

Description:
  This script demonstrates how to play audio files on the robot using the PlayAudioFile
  service. Supports specifying audio file name and directory path. Default sample rate is 24kHz.

Prerequisites:
  - PlayAudioFile service must be available
  - Audio file must exist on the robot at the specified path
  - Audio file must be 24kHz, 16-bit PCM, mono WAV format

Usage:
  python3 play_audio.py --ros-args -p file_name:=<name> -p file_path:=<path>

Parameters:
  - file_name: Audio file name (Default: 小星星.wav)
  - file_path: Directory containing the audio file

Example:
  python3 play_audio.py --ros-args -p file_name:=小星星.wav -p file_path:=/robot/software/aimrt_agent/bin/cfg/t1/audio
"""

import rclpy
import rclpy.logging
import time
from rclpy.node import Node

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
from aimdk_msgs.srv import PlayAudioFile


class PlayAudioFileClient(Node):
    def __init__(self):
        super().__init__("play_audio_file_client")
        self.file_name = self.declare_parameter("file_name", "小星星.wav").value
        self.file_path = self.declare_parameter("file_path", "/robot/software/aimrt_agent/bin/cfg/t1/audio").value

        self.service_name = "/aimdk_5Fmsgs/srv/PlayAudioFile"
        self.pkg_name = "sdk_demo"
        self.sample_format = "S16_LE"
        self.coding_format = "wave"
        self.channels = 1
        self.sample_rate = 24000
        self.size = 0
        self.priority = 6
        self.priority_weight = 0

        self.client = self.create_client(PlayAudioFile, self.service_name)
        self.get_logger().info("PlayAudioFile client node created.")

    def wait_for_service(self) -> bool:
        while not self.client.wait_for_service(timeout_sec=2.0):
            if not rclpy.ok():
                return False
            self.get_logger().info(f"Waiting for service: {self.service_name}")
        self.get_logger().info("Service available, ready to send request.")
        return True

    def send_request(self) -> bool:
        if not self.wait_for_service():
            return False

        try:
            request = PlayAudioFile.Request()
            request.request = CommonRequest()
            request.request.header.stamp = self.get_clock().now().to_msg()

            # pkg_name/file_name/file_path locate the audio resource; info describes its format.
            request.file.pkg_name = self.pkg_name
            request.file.file_name = self.file_name
            request.file.file_path = self.file_path

            if not request.file.file_name:
                self.get_logger().error("file_name is empty.")
                return False

            request.file.info.channels = self.channels
            request.file.info.sample_rate = self.sample_rate
            request.file.info.size = self.size
            request.file.info.sample_format = self.sample_format
            request.file.info.coding_format = self.coding_format
            request.file.priority = self.priority
            request.file.priority_weight = self.priority_weight

            self.get_logger().info(
                "Sending PlayAudioFile request: "
                f"file={request.file.file_name}, "
                f"path={request.file.file_path}, "
                f"pkg={request.file.pkg_name}, "
                f"ch={self.channels}, "
                f"sr={self.sample_rate}, "
                f"fmt={self.sample_format}, "
                f"coding={self.coding_format}, "
                f"priority={self.priority}, "
                f"weight={self.priority_weight}"
            )

            for i in range(3):
                future = self.client.call_async(request)
                rclpy.spin_until_future_complete(self, future, timeout_sec=5)

                if future.done():
                    break

                self.get_logger().info(f'trying ... [{i}]')
                time.sleep(0.2)
            
            if not future.done():
                self.get_logger().error("PlayAudioFile call failed or timed out.")
                return False

            response = future.result()
            if response is None:
                self.get_logger().error("PlayAudioFile call failed or timed out.")
                return False

            code = response.response.header.code
            status = response.response.status.value
            # Treat code==0 or status==SUCCESS as success.
            ok = code == 0 or status == CommonState.SUCCESS

            if ok:
                self.get_logger().info(
                    "PlayAudioFile accepted. "
                    f"code={code} status={status} msg={response.response.message}"
                )
                return True
            
            # 获取失败原因
            reason = getattr(response.response.status, 'reason', 0)
            if reason > 0:
                reason_desc = get_reason_description(reason)
                self.get_logger().warning(
                    f"PlayAudioFile rejected: reason={reason} - {reason_desc}"
                )

            self.get_logger().error(
                "PlayAudioFile rejected. "
                f"code={code} status={status} msg={response.response.message}"
            )
            return False
        except Exception as error:  # noqa: BLE001
            self.get_logger().error(f"Exception occurred: {error}")
            return False


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = PlayAudioFileClient()
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
