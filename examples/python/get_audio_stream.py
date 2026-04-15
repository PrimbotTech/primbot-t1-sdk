#!/usr/bin/env python3

"""
Example subscriber for /aima/hal/audio/capture

The following ROS parameters can be set via startup arguments:
--ros-args -p <name>:=<value>

Supported parameters:
  - output_file: PCM output path. Leave empty to disable file dump.
  - capture_seconds: stop automatically after the first packet arrives and
    this many seconds have elapsed. Set <= 0 to run until Ctrl+C.
  - log_every_n_messages: print progress every N messages. Set <= 0 to
    disable periodic progress logs.
  - enable_playback: if True, re-publishes the captured audio to the playback
    topic to echo the audio out to the speakers. Default is False.

Examples:
  # 1. Capture for 5 seconds to a specific file:
  python3 examples/python/get_audio_stream.py --ros-args \
    -p output_file:=/tmp/audio_capture.pcm -p capture_seconds:=5

  # 2. Capture continuously until Ctrl+C is pressed:
  python3 examples/python/get_audio_stream.py --ros-args \
    -p capture_seconds:=-1

  # 3. Echo audio to speakers (loopback) and capture for 10 seconds:
  python3 examples/python/get_audio_stream.py --ros-args \
    -p enable_playback:=true -p capture_seconds:=10
"""

from pathlib import Path
import time
import os
import threading

import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy

from aimdk_msgs.msg import AudioCapture, AudioPlayback


class AudioStreamSubscriber(Node):
    def __init__(self) -> None:
        super().__init__("get_audio_stream")

        self.output_file = self.declare_parameter(
            "output_file", "/tmp/audio_capture.pcm"
        ).value
        self.capture_seconds = self.declare_parameter("capture_seconds", 5).value
        self.log_every_n_messages = self.declare_parameter(
            "log_every_n_messages", 100
        ).value
        self.enable_playback = self.declare_parameter(
            "enable_playback", False
        ).value

        self.file_enabled = bool(self.output_file)
        self.output_stream = None
        self.first_packet_received = False
        self.size_mismatch_logged = False
        self.summary_logged = False
        self.message_count = 0
        self.total_bytes = 0
        self.first_packet_monotonic = None
        self.last_sample_rate = 0
        self.last_total_channels = 0
        self.last_sample_format = ""
        self.last_coding_format = ""

        if self.file_enabled:
            self._prepare_output_file()

        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=20,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.subscription = self.create_subscription(
            AudioCapture,
            "/aima/hal/audio/capture",
            self.on_audio_capture,
            qos,
        )

        if self.enable_playback:
            self.playback_publisher = self.create_publisher(
                AudioPlayback,
                "/aima/hal/audio/playback",
                qos,
            )
            self.get_logger().info("Audio playback loopback is ENABLED.")
        else:
            self.playback_publisher = None

        self.timer = self.create_timer(0.2, self.check_auto_stop)

        if self.file_enabled:
            self.get_logger().info(f"PCM output file: {self.output_file}")
        else:
            self.get_logger().info(
                "PCM output disabled because output_file is empty."
            )

        if self.capture_seconds > 0:
            self.get_logger().info(
                f"The node will stop {self.capture_seconds} second(s) after the first packet."
            )
        else:
            self.get_logger().info("Auto stop disabled. Press Ctrl+C to exit.")

    def _prepare_output_file(self) -> None:
        output_path = Path(self.output_file)
        output_path.parent.mkdir(parents=True, exist_ok=True)
        self.output_stream = output_path.open("wb")

    def _close_output_file(self) -> None:
        if self.output_stream is not None and not self.output_stream.closed:
            self.output_stream.close()

    def _log_first_packet(self, msg: AudioCapture, payload_bytes: int) -> None:
        self.get_logger().info(
            f"First packet received: stamp={msg.stamps.sec}.{msg.stamps.nanosec:09d} "
            f"mic_channels={msg.mic_channels} ref_channels={msg.ref_channels} "
            f"total_channels={msg.info.channels} sample_rate={msg.info.sample_rate} "
            f"sample_format={msg.info.sample_format} coding_format={msg.info.coding_format} "
            f"mic_source={msg.mic_source} payload_bytes={payload_bytes}"
        )

    def on_audio_capture(self, msg: AudioCapture) -> None:
        payload = bytes(msg.data.data)
        payload_bytes = len(payload)
        self.message_count += 1
        self.total_bytes += payload_bytes

        if not self.first_packet_received:
            self.first_packet_received = True
            self.first_packet_monotonic = time.monotonic()
            self.last_sample_rate = msg.info.sample_rate
            self.last_total_channels = msg.info.channels
            self.last_sample_format = msg.info.sample_format
            self.last_coding_format = msg.info.coding_format
            self._log_first_packet(msg, payload_bytes)

        if (
            not self.size_mismatch_logged
            and msg.info.size != 0
            and msg.info.size != payload_bytes
        ):
            self.size_mismatch_logged = True
            self.get_logger().warn(
                f"AudioInfo.size={msg.info.size} but actual payload is {payload_bytes} bytes. "
                "The example will use len(msg.data.data)."
            )

        if self.file_enabled and self.output_stream is not None and payload_bytes > 0:
            try:
                self.output_stream.write(payload)
                self.output_stream.flush()
            except OSError as error:
                self.get_logger().error(
                    f"Failed while writing to {self.output_file}: {error}"
                )
                self.file_enabled = False
                self._close_output_file()

        if self.playback_publisher is not None:
            playback_msg = AudioPlayback()
            playback_msg.stamps = msg.stamps
            playback_msg.info = msg.info
            playback_msg.data = msg.data
            playback_msg.pkg_name = "get_audio_stream_loopback"
            playback_msg.token_id = "loopback_session"
            self.playback_publisher.publish(playback_msg)

        if (
            self.log_every_n_messages > 0
            and self.message_count % self.log_every_n_messages == 0
        ):
            self.get_logger().info(
                f"Received {self.message_count} packets, total_bytes={self.total_bytes}, latest_payload={payload_bytes}"
            )

    def check_auto_stop(self) -> None:
        if (
            not self.first_packet_received
            or self.capture_seconds <= 0
            or self.first_packet_monotonic is None
        ):
            return

        if time.monotonic() - self.first_packet_monotonic < self.capture_seconds:
            return

        self._close_output_file()
        self.log_summary()
        self.get_logger().info(
            f"Reached capture_seconds={self.capture_seconds}, shutting down."
        )
        rclpy.shutdown()

    def log_summary(self) -> None:
        if self.summary_logged:
            return
        self.summary_logged = True

        if not self.first_packet_received or self.first_packet_monotonic is None:
            self.get_logger().warn("No audio packet received before shutdown.")
            return

        elapsed_ms = int((time.monotonic() - self.first_packet_monotonic) * 1000)
        self.get_logger().info(
            f"Audio stream summary: packets={self.message_count} total_bytes={self.total_bytes} elapsed={elapsed_ms} ms "
            f"sample_rate={self.last_sample_rate} total_channels={self.last_total_channels} "
            f"sample_format={self.last_sample_format} coding_format={self.last_coding_format}"
        )

        if self.file_enabled or self.output_file:
            self.get_logger().info(f"Captured PCM file: {self.output_file}")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = AudioStreamSubscriber()
    try:
        rclpy.spin(node)
    finally:
        # Start a watchdog thread to force exit if cleanup hangs for more than 3s
        def watchdog_exit():
            time.sleep(3)
            # Use os._exit to bypass any remaining cleanup and force terminate
            os._exit(0)
        
        watchdog_thread = threading.Thread(target=watchdog_exit, daemon=True)
        watchdog_thread.start()

        node._close_output_file()
        node.log_summary()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
