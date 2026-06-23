#!/usr/bin/env python3

"""
Volume Control Demo Example Script

Description:
  This script demonstrates how to control robot audio volume and mute settings using TTS services.
  Includes volume adjustment, mute toggle, and TTS playback demonstration.
  Note: If a volume setting step fails, the script will continue to the next step. Please check the logs for any errors.
  The original volume is recorded at startup and automatically restored when the demo finishes or Ctrl+C is pressed.

Prerequisites:
  - Robot TTS service must be running
  - Audio output device must be working properly
  - Volume and mute services must be available

Usage:
  python3 volume_control.py

Example:
  python3 volume_control.py

Parameters:
  - None 
"""

import signal
import time

import rclpy
import rclpy.logging
from rclpy.node import Node

from aimdk_msgs.msg import CommonRequest, CommonState, TtsPriorityLevel
from aimdk_msgs.srv import GetMute, GetVolume, PlayTts, SetMute, SetVolume

TTS_TEXT = (
    "大家好，我是启元机器人。现在为你演示音频控制示例。"
    "接下来，我会先持续播报一段介绍内容，在播报过程中，系统会依次执行设置音量、查询音量、设置静音和查询静音等操作。"
    "你将听到默认音量、音量调小、音量调大、开启静音以及取消静音这几个阶段的变化。"
    "如果整段流程能够顺利完成，就说明启元的TTS播放、音量控制和静音控制链路都已经正常工作。"
    "感谢你体验启元机器人的音频控制能力。"
)

STEP_INTERVAL_SEC = 5.0
SERVICE_WAIT_SEC = 2.0
SERVICE_CALL_TIMEOUT_SEC = 5.0


class VolumeControlClient(Node):
    def __init__(self):
        super().__init__("volume_control_client")
        self.play_tts_service = "/aimdk_5Fmsgs/srv/PlayTts"
        self.set_volume_service = "/aimdk_5Fmsgs/srv/SetVolume"
        self.get_volume_service = "/aimdk_5Fmsgs/srv/GetVolume"
        self.set_mute_service = "/aimdk_5Fmsgs/srv/SetMute"
        self.get_mute_service = "/aimdk_5Fmsgs/srv/GetMute"
        self.tts_domain = "volume_control_demo"
        self.tts_trace_id = "volume_control_demo_trace"

        self.play_tts_client = self.create_client(PlayTts, self.play_tts_service)
        self.set_volume_client = self.create_client(
            SetVolume, self.set_volume_service
        )
        self.get_volume_client = self.create_client(
            GetVolume, self.get_volume_service
        )
        self.set_mute_client = self.create_client(SetMute, self.set_mute_service)
        self.get_mute_client = self.create_client(GetMute, self.get_mute_service)

        self.original_volume_ = None

        self.get_logger().info("Volume control demo node created.")

    def wait_for_services(self) -> bool:
        return (
            self._wait_for_service(self.play_tts_client, self.play_tts_service)
            and self._wait_for_service(
                self.set_volume_client, self.set_volume_service
            )
            and self._wait_for_service(
                self.get_volume_client, self.get_volume_service
            )
            and self._wait_for_service(self.set_mute_client, self.set_mute_service)
            and self._wait_for_service(self.get_mute_client, self.get_mute_service)
        )

    def _wait_for_service(self, client, service_name: str) -> bool:
        while not client.wait_for_service(timeout_sec=SERVICE_WAIT_SEC):
            if not rclpy.ok():
                return False
            self.get_logger().info(f"Waiting for service: {service_name}")

        self.get_logger().info(f"Service available: {service_name}")
        return True

    def _call_service(self, client, request, service_name: str):
        for i in range(3):
            future = client.call_async(request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=SERVICE_CALL_TIMEOUT_SEC)

            if future.done():
                break

            self.get_logger().info(f'trying ... [{i}]')
            time.sleep(0.2)

        if not future.done():
            self.get_logger().error(
                f"{service_name} timed out after "
                f"{int(SERVICE_CALL_TIMEOUT_SEC * 1000)} ms."
            )
            return None

        try:
            response = future.result()
        except Exception as error:  # noqa: BLE001
            self.get_logger().error(f"{service_name} failed: {error}")
            return None

        if response is None:
            self.get_logger().error(f"{service_name} returned no response.")
            return None

        return response

    def _build_common_request(self) -> CommonRequest:
        request = CommonRequest()
        request.header.stamp = self.get_clock().now().to_msg()
        return request

    def play_tts(self) -> bool:
        request = PlayTts.Request()
        request.header = self._build_common_request()
        # domain and trace_id identify this TTS request.
        request.tts_req.text = TTS_TEXT
        request.tts_req.domain = self.tts_domain
        request.tts_req.trace_id = self.tts_trace_id
        request.tts_req.is_interrupted = True
        request.tts_req.priority_weight = 0
        request.tts_req.priority_level.value = TtsPriorityLevel.INTERACTION_L6

        self.get_logger().info(
            f"Sending PlayTts request. domain={request.tts_req.domain} "
            f"trace_id={request.tts_req.trace_id}"
        )
        self.get_logger().info(f"TTS text: {request.tts_req.text}")

        response = self._call_service(
            self.play_tts_client, request, "PlayTts"
        )
        if response is None:
            return False

        self.get_logger().info(
            "PlayTts response: "
            f"code={response.header.header.code} "
            f"status={response.header.status.value} "
            f"is_success={int(response.tts_resp.is_success)} "
            f"error_message={response.tts_resp.error_message}"
        )

         if not response.tts_resp.is_success:
             self.get_logger().error(
                 f"PlayTts failed. "
                 f"code={response.header.header.code} status={response.header.status.value} "
                 f"msg={response.header.message}"
             )

        return response.tts_resp.is_success

    def execute_volume_step(self, target_volume: int, action_message: str) -> bool:
        self.get_logger().info(
            f"Running volume step. target={target_volume} action={action_message}"
        )

        # Call GetVolume after SetVolume to verify the result.
        set_request = SetVolume.Request()
        set_request.request = self._build_common_request()
        set_request.audio_volume = target_volume

        set_response = self._call_service(
            self.set_volume_client, set_request, "SetVolume"
        )
        if set_response is None:
            return False

        self.get_logger().info(
            "SetVolume response: "
            f"code={set_response.response.header.code} "
            f"status={set_response.response.status.value} "
            f"audio_volume={set_response.audio_volume}"
        )

        if set_response.response.header.code != 0 or set_response.response.status.value != CommonState.SUCCESS:
            self.get_logger().error(
                f"SetVolume failed. "
                f"code={set_response.response.header.code} "
                f"status={set_response.response.status.value} "
                f"reason={set_response.response.status.reason}"
            )
            return False

        get_request = GetVolume.Request()
        get_request.request = self._build_common_request()

        get_response = self._call_service(
            self.get_volume_client, get_request, "GetVolume"
        )
        if get_response is None:
            return False

        self.get_logger().info(
            "GetVolume response: "
            f"code={get_response.response.header.code} "
            f"status={get_response.response.status.value} "
            f"audio_volume={get_response.audio_volume}"
        )

        if get_response.response.header.code != 0 or get_response.response.status.value != CommonState.SUCCESS:
            self.get_logger().error(
                f"GetVolume failed. "
                f"code={get_response.response.header.code} "
                f"status={get_response.response.status.value} "
                f"reason={get_response.response.status.reason}"
            )
            return False

        if get_response.audio_volume != target_volume:
            self.get_logger().error(
                "Volume verification failed. "
                f"target={target_volume} actual={get_response.audio_volume}"
            )
            return False

        self.get_logger().info(
            f"{action_message}，确认当前音量={get_response.audio_volume}"
        )
        return True

    def execute_mute_step(self, target_mute: bool, action_message: str) -> bool:
        self.get_logger().info(
            f"Running mute step. target={int(target_mute)} action={action_message}"
        )

        # Call GetMute after SetMute to verify the result.
        set_request = SetMute.Request()
        set_request.request = self._build_common_request()
        set_request.is_mute = target_mute

        set_response = self._call_service(
            self.set_mute_client, set_request, "SetMute"
        )
        if set_response is None:
            return False

        self.get_logger().info(
            "SetMute response: "
            f"code={set_response.response.header.code} "
            f"status={set_response.response.status.value} "
            f"is_mute={int(set_response.is_mute)} "
            "(final result is verified by GetMute)"
        )

        if set_response.response.header.code != 0 or set_response.response.status.value != CommonState.SUCCESS:
            self.get_logger().error(
                f"SetMute failed. "
                f"code={set_response.response.header.code} "
                f"status={set_response.response.status.value} "
                f"reason={set_response.response.status.reason}"
            )
            return False

        get_request = GetMute.Request()
        get_request.request = self._build_common_request()

        get_response = self._call_service(self.get_mute_client, get_request, "GetMute")
        if get_response is None:
            return False

        self.get_logger().info(
            "GetMute response: "
            f"code={get_response.response.header.code} "
            f"status={get_response.response.status.value} "
            f"is_mute={int(get_response.is_mute)}"
        )

        if get_response.response.header.code != 0 or get_response.response.status.value != CommonState.SUCCESS:
            self.get_logger().error(
                f"GetMute failed. "
                f"code={get_response.response.header.code} "
                f"status={get_response.response.status.value} "
                f"reason={get_response.response.status.reason}"
            )
            return False

        if get_response.is_mute != target_mute:
            self.get_logger().error(
                "Mute verification failed. "
                f"target={int(target_mute)} actual={int(get_response.is_mute)}"
            )
            return False

        self.get_logger().info(
            f"{action_message}，确认当前静音状态={int(get_response.is_mute)}"
        )
        return True

    def run_demo(self) -> bool:
        self.get_logger().info("Volume control demo starts automatically.")
        self.get_logger().info(
            "Volume note: on this device a larger value means a smaller actual volume."
        )

        self.original_volume_ = None
        get_init_request = GetVolume.Request()
        get_init_request.request = self._build_common_request()
        get_init_response = self._call_service(
            self.get_volume_client, get_init_request, "GetVolume"
        )
        if get_init_response is not None:
            self.original_volume_ = get_init_response.audio_volume
            self.get_logger().info(f"Initial volume: {self.original_volume_}")
        else:
            self.get_logger().warning("Failed to get initial volume, will skip restoration.")

        if not self.play_tts():
            self.get_logger().error("PlayTts failed.")
            return False

        steps = [
            ("设为默认音量", lambda: self.execute_volume_step(30, "设为默认音量")),
            ("音量调大", lambda: self.execute_volume_step(40, "音量调大")),
            ("音量调小", lambda: self.execute_volume_step(20, "音量调小")),
            ("设置静音", lambda: self.execute_mute_step(True, "设置静音")),
            ("取消静音", lambda: self.execute_mute_step(False, "取消静音")),
        ]

        self.get_logger().info(
            f"TTS accepted. The first control step will run in {int(STEP_INTERVAL_SEC)} seconds."
        )

        try:
            for step_name, step_func in steps:
                time.sleep(STEP_INTERVAL_SEC)
                if not step_func():
                    self.get_logger().error(f"Volume control step failed: {step_name} (continuing to next step)")
        except KeyboardInterrupt:
            pass

        # Restore original volume if available (normal finish or Ctrl+C)
        if self.original_volume_ is not None:
            self.get_logger().info(f"Restoring original volume: {self.original_volume_}")
            if not self.execute_volume_step(self.original_volume_, "恢复原始音量"):
                self.get_logger().warning("Failed to restore original volume.")
            self.get_logger().info(
                f"Volume control demo finished. Volume restored to {self.original_volume_}, mute=false. "
                f"(Check logs for any failed steps)"
            )
        else:
            self.get_logger().info(
                "Volume control demo finished. Volume restoration skipped "
                "(initial volume unknown). (Check logs for any failed steps)"
            )
        return True


def main(args=None):
    rclpy.init(args=args)
    # init 之后立即覆盖 ROS2 的 SIGINT 处理器
    # 阻止 Ctrl+C 触发 rclpy.shutdown()，保持上下文有效以恢复音量
    def _sigint_handler(sig, frame):
        raise KeyboardInterrupt
    signal.signal(signal.SIGINT, _sigint_handler)
    node = None
    try:
        node = VolumeControlClient()
        if not node.wait_for_services():
            node.get_logger().error("Failed to initialize volume control demo.")
            return 1
        ok = node.run_demo()
        return 0 if ok else 1
    except KeyboardInterrupt:
        return 0
    except Exception as error:  # noqa: BLE001
        rclpy.logging.get_logger("volume_control_main").error(
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
