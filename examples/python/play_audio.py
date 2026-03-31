#!/usr/bin/env python3

"""
ROS parameters:
  file_name: audio file name.
  file_path: directory containing the audio file.

Example:
  python3 examples/python/play_audio.py --ros-args \
    -p file_name:=demo.wav -p file_path:=/tmp
"""

import rclpy
import rclpy.logging
from rclpy.node import Node

from aimdk_msgs.msg import CommonRequest, CommonState
from aimdk_msgs.srv import PlayAudioFile


class PlayAudioFileClient(Node):
    def __init__(self):
        super().__init__("play_audio_file_client")
        self.file_name = self.declare_parameter("file_name", "撒娇.wav").value
        self.file_path = self.declare_parameter("file_path", "/agibot/software/interaction/bin/cfg").value

        self.service_name = "/aimdk_5Fmsgs/srv/PlayAudioFile"
        self.pkg_name = "sdk_demo"
        self.sample_format = "S16_LE"
        self.coding_format = "wave"
        self.channels = 1
        self.sample_rate = 16000
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

            future = self.client.call_async(request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
            if not future.done():
                self.get_logger().error("PlayAudioFile call failed or timed out.")
                return False

            response = future.result()
            if response is None:
                self.get_logger().error("PlayAudioFile call failed or timed out.")
                return False

            code = response.reponse.header.code
            status = response.reponse.status.value
            # Treat code==0 or status==SUCCESS as success.
            ok = code == 0 or status == CommonState.SUCCESS

            if ok:
                self.get_logger().info(
                    "PlayAudioFile accepted. "
                    f"code={code} status={status} msg={response.reponse.message}"
                )
                return True

            self.get_logger().error(
                "PlayAudioFile rejected. "
                f"code={code} status={status} msg={response.reponse.message}"
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
