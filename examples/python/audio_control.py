#!/usr/bin/env python3

import time

import rclpy
import rclpy.logging
from rclpy.node import Node

from aimdk_msgs.msg import CommonRequest, TtsPriorityLevel
from aimdk_msgs.srv import GetMute, GetVolume, PlayTts, SetMute, SetVolume

TTS_TEXT = (
    "大家好，我是启元机器人Q1。现在为你演示音频控制示例。"
    "接下来，我会先持续播报一段介绍内容，在播报过程中，系统会依次执行设置音量、查询音量、设置静音和查询静音等操作。"
    "你将听到默认音量、音量调小、音量调大、开启静音以及取消静音这几个阶段的变化。"
    "如果整段流程能够顺利完成，就说明启元的TTS播放、音量控制和静音控制链路都已经正常工作。"
    "感谢你体验启元机器人Q1的音频控制能力。"
)

STEP_INTERVAL_SEC = 5.0
SERVICE_WAIT_SEC = 2.0
SERVICE_CALL_TIMEOUT_SEC = 5.0


class AudioControlClient(Node):
    def __init__(self):
        super().__init__("audio_control_client")
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

        self.get_logger().info("Audio control demo node created.")

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
        future = client.call_async(request)
        rclpy.spin_until_future_complete(
            self, future, timeout_sec=SERVICE_CALL_TIMEOUT_SEC
        )
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

        return response.tts_resp.is_success

    def execute_volume_step(self, target_volume: int, action_message: str) -> bool:
        self.get_logger().info(
            f"Running volume step. target={target_volume} action={action_message}"
        )

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
            f"code={set_response.reponse.header.code} "
            f"status={set_response.reponse.status.value} "
            f"audio_volume={set_response.audio_volume}"
        )

        get_request = GetVolume.Request()
        get_request.request = self._build_common_request()

        get_response = self._call_service(
            self.get_volume_client, get_request, "GetVolume"
        )
        if get_response is None:
            return False

        self.get_logger().info(
            "GetVolume response: "
            f"code={get_response.reponse.header.code} "
            f"status={get_response.reponse.status.value} "
            f"audio_volume={get_response.audio_volume}"
        )

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
            f"code={set_response.reponse.header.code} "
            f"status={set_response.reponse.status.value} "
            f"is_mute={int(set_response.is_mute)} "
            "(final result is verified by GetMute)"
        )

        get_request = GetMute.Request()
        get_request.request = self._build_common_request()

        get_response = self._call_service(self.get_mute_client, get_request, "GetMute")
        if get_response is None:
            return False

        self.get_logger().info(
            "GetMute response: "
            f"code={get_response.reponse.header.code} "
            f"status={get_response.reponse.status.value} "
            f"is_mute={int(get_response.is_mute)}"
        )

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

        if not self.play_tts():
            self.get_logger().error("PlayTts failed.")
            return False

        steps = [
            lambda: self.execute_volume_step(30, "设为默认音量"),
            lambda: self.execute_volume_step(40, "音量调小"),
            lambda: self.execute_volume_step(20, "音量调大"),
            lambda: self.execute_mute_step(True, "设置静音"),
            lambda: self.execute_mute_step(False, "取消静音"),
        ]

        self.get_logger().info(
            f"TTS accepted. The first control step will run in {int(STEP_INTERVAL_SEC)} seconds."
        )

        for step in steps:
            time.sleep(STEP_INTERVAL_SEC)
            if not step():
                self.get_logger().error("Volume control step failed.")
                return False

        self.get_logger().info(
            "Volume control demo finished. Final state: volume=20, mute=false."
        )
        return True


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = AudioControlClient()
        if not node.wait_for_services():
            node.get_logger().error("Failed to initialize audio control demo.")
            return 1
        ok = node.run_demo()
        return 0 if ok else 1
    except Exception as error:  # noqa: BLE001
        rclpy.logging.get_logger("audio_control_main").error(
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
