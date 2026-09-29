#!/usr/bin/env python3

"""
Audio File Playback Example Script

Description:
  This script demonstrates how to play audio files on the robot using the PlayAudioFile service.
  Default sample rate is 24kHz. Supports various audio formats with configurable parameters.

Prerequisites:
  - Robot audio service must be running
  - Audio file must exist in the specified path
  - Audio file must be 24kHz, 16-bit, mono WAV format
  - Audio output device must be working properly

Usage:
  python3 play_audio_from_robot.py --ros-args -p file_name:=<filename> -p file_path:=<directory> -p media_role:=<role>

Example:
  python3 play_audio_from_robot.py --ros-args -p file_name:=小星星.wav -p file_path:=/robot/software/aimrt_agent/bin/cfg/t1/audio -p media_role:=Music

Parameters:
  - file_name: Audio file name to play (default: 小星星.wav)
  - file_path: Directory containing the audio file (default: /robot/software/aimrt_agent/bin/cfg/t1/audio)
  - media_role: PulseAudio media.role (default: Music)
    Alert(系统告警,优先级100) | Notification(系统提示音,优先级80)
    Communication(对话语音,优先级60) | Music(音乐/媒体,优先级40)
    非 Music 角色播放时会自动降低(duck)音乐音量
"""

import rclpy
import rclpy.logging
import time
from rclpy.node import Node

from aimdk_msgs.msg import CommonRequest, CommonState
from aimdk_msgs.srv import PlayAudioFile


class PlayAudioFileClient(Node):
    def __init__(self):
        super().__init__("play_audio_from_robot_client")
        self.file_name = self.declare_parameter("file_name", "小星星.wav").value
        self.file_path = self.declare_parameter("file_path", "/robot/software/aimrt_agent/bin/cfg/t1/audio").value
        self.media_role = self.declare_parameter("media_role", "Music").value

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
            request.file.media_role = self.media_role

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

            self.get_logger().error(
                f"PlayAudioFile failed. "
                f"code={code} status={status} "
                f"msg={response.response.message}"
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
