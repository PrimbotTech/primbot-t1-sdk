#!/usr/bin/env python3

import rclpy
import rclpy.logging
from rclpy.node import Node
from rclpy.parameter import Parameter

from aimdk_msgs.msg import CommonRequest, CommonState
from aimdk_msgs.srv import PlayEmotion


class PlayEmotionClient(Node):
    def __init__(self):
        super().__init__("play_emotion_client")
        self.declare_parameter("type", "emotion")
        self.declare_parameter("emotion_ids", Parameter.Type.INTEGER_ARRAY)
        self.declare_parameter("file_paths", Parameter.Type.STRING_ARRAY)

        self.type = self.get_parameter("type").value
        self.emotion_ids = self.get_parameter_or(
            "emotion_ids",
            Parameter("emotion_ids", Parameter.Type.INTEGER_ARRAY, []),
        ).value
        self.file_paths = self.get_parameter_or(
            "file_paths",
            Parameter("file_paths", Parameter.Type.STRING_ARRAY, []),
        ).value
        self.priority = 0
        self.loop_count = 1

        self.client = self.create_client(PlayEmotion, "/aimdk_5Fmsgs/srv/PlayEmotion")
        self.get_logger().info("✅ PlayEmotion client node created.")

        while not self.client.wait_for_service(timeout_sec=2.0):
            self.get_logger().info("⏳ Waiting for service...")
        self.get_logger().info("🟢 Service available, ready to send request.")

    def send_request(self) -> bool:
        try:
            if not self.validate_parameters():
                return False

            request = PlayEmotion.Request()
            request.header = CommonRequest()
            request.header.header.stamp = self.get_clock().now().to_msg()
            request.type = self.type
            request.priority = self.priority
            request.loop_count = self.loop_count

            if self.type == "emotion":
                request.emotion_ids = list(self.emotion_ids)
            else:
                request.file_paths = list(self.file_paths)

            self.get_logger().info(
                "📨 Sending PlayEmotion request: "
                f"type={request.type}, "
                f"emotion_ids={list(request.emotion_ids)}, "
                f"file_paths={list(request.file_paths)}, "
                f"priority={request.priority}, "
                f"loop_count={request.loop_count}"
            )
            for emotion_id in request.emotion_ids:
                self.get_logger().info(f"emotion_id={emotion_id}")
            for file_path in request.file_paths:
                self.get_logger().info(f"file_path={file_path}")

            future = self.client.call_async(request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
            if not future.done():
                self.get_logger().error("❌ Service call failed or timed out.")
                return False

            response = future.result()
            if response is None:
                self.get_logger().error("❌ Service call failed or timed out.")
                return False

            code = response.header.header.code
            status = response.header.status.value
            self.get_logger().info(
                "Response: "
                f"code={code}, "
                f"status={status}, "
                f"message={response.header.message}"
            )

            if code == 0 or status == CommonState.SUCCESS:
                self.get_logger().info("✅ PlayEmotion request accepted.")
                return True

            self.get_logger().error("❌ PlayEmotion request failed.")
            return False
        except Exception as error:  # noqa: BLE001
            self.get_logger().error(f"❌ Exception occurred: {error}")
            return False

    def validate_parameters(self) -> bool:
        if self.type not in ("emotion", "file"):
            self.get_logger().error(
                f"❌ Invalid parameter 'type': {self.type}. Use 'emotion' or 'file'."
            )
            return False

        if self.type == "emotion" and not self.emotion_ids:
            self.get_logger().error(
                "❌ Parameter 'emotion_ids' must be set when type=emotion."
            )
            return False

        if self.type == "file" and not self.file_paths:
            self.get_logger().error(
                "❌ Parameter 'file_paths' must be set when type=file."
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
            f"❌ Program exited with exception: {error}"
        )
        return 1
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
