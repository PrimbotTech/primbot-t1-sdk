#!/usr/bin/env python3

"""Audio Stream Capture Example Script

Description:
  This script demonstrates how to capture audio streams from the robot using the AudioCapture
  topic and playback using the AudioPlayback service. Supports automated recording with
  configurable duration and optional conversion/playback.

Prerequisites:
  - Audio capture topic must be publishing
  - Audio playback service must be available

Usage:
  python3 get_audio_stream.py --ros-args -p output_file:=<path> -p capture_seconds:=<seconds>

Parameters:
  - output_file: Path to save the original 24kHz PCM file (Default: /tmp/audio_capture.pcm).
  - capture_seconds: Duration of automated recording in seconds (Default: 5s).
  - log_every_n_messages: Print progress log every N received packets.

Notes:
  - After the recording duration expires, you MUST type 'y' in the terminal to trigger conversion and playback.

Example:
  python3 get_audio_stream.py --ros-args -p capture_seconds:=10
"""

from pathlib import Path
import time
import os
import threading
import struct

import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy

from aimdk_msgs.msg import AudioCapture, AudioPlayback


class AudioStreamSubscriber(Node):
    def __init__(self) -> None:
        super().__init__("get_audio_stream")

        self.output_file = self.declare_parameter("output_file", "/tmp/audio_capture.pcm").value
        self.capture_seconds = self.declare_parameter("capture_seconds", 5).value
        self.log_every_n_messages = self.declare_parameter("log_every_n_messages", 100).value

        self.output_stream = None
        self.first_packet_received = False
        self.capture_done = False
        self.summary_logged = False
        self.message_count = 0
        self.total_bytes = 0
        self.first_packet_monotonic = None
        self.audio_info = None

        output_path = Path(self.output_file)
        output_path.parent.mkdir(parents=True, exist_ok=True)
        self.output_stream = output_path.open("wb")

        qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=20, reliability=ReliabilityPolicy.RELIABLE)
        self.subscription = self.create_subscription(AudioCapture, "/aima/hal/audio/capture", self.on_audio_capture, qos)
        self.playback_pub = self.create_publisher(AudioPlayback, "/aima/hal/audio/playback", qos)
        self.timer = self.create_timer(0.2, self.check_auto_stop)

        self.get_logger().info(f"PCM output file: {self.output_file}")

    def on_audio_capture(self, msg: AudioCapture) -> None:
        if self.capture_done: return
        payload = bytes(msg.data.data)
        self.message_count += 1
        self.total_bytes += len(payload)

        if not self.first_packet_received:
            self.first_packet_received = True
            self.first_packet_monotonic = time.monotonic()
            self.audio_info = msg.info # 保存原始元数据
            self.get_logger().info(f"Recording started (Channels: {self.audio_info.channels}, Rate: {self.audio_info.sample_rate}Hz)...")

        if self.output_stream: self.output_stream.write(payload)

    def check_auto_stop(self) -> None:
        if not self.first_packet_received or self.capture_seconds <= 0 or self.capture_done: return
        if time.monotonic() - self.first_packet_monotonic >= self.capture_seconds:
            self.capture_done = True
            self.get_logger().info("Recording finished.")
            if self.output_stream: self.output_stream.close()

    def process_and_play(self) -> None:
        """直接以24kHz播放采集到的音频"""
        self.get_logger().info("Direct playback at 24kHz (no resampling needed)...")

        try:
            self.stream_play(self.output_file)
        except Exception as e:
            self.get_logger().error(f"Processing failed: {e}")

    def stream_play(self, filename: str) -> None:
        """流式发送 24k 采样率数据"""
        while self.playback_pub.get_subscription_count() == 0:
            time.sleep(0.1)
            if not rclpy.ok(): return

        # Generate a unique token for the entire playback session
        session_token = f"play_{int(time.time())}"

        with open(filename, "rb") as f:
            chunk_size = 14400
            while rclpy.ok():
                data = f.read(chunk_size)
                if not data: break

                msg = AudioPlayback()
                msg.info.channels = 1
                msg.info.sample_rate = 24000
                msg.info.size = len(data)
                msg.info.sample_format = "S16_LE"
                msg.info.coding_format = "pcm"
                msg.stamps = self.get_clock().now().to_msg()
                msg.data.data = list(data)
                msg.pkg_name = "audio_stream_pro"
                msg.token_id = session_token
                self.playback_pub.publish(msg)
                time.sleep(0.05)

def main(args=None) -> None:
    rclpy.init(args=args)
    node = AudioStreamSubscriber()
    try:
        while rclpy.ok() and not node.capture_done:
            rclpy.spin_once(node, timeout_sec=0.1)
        
        if rclpy.ok():
            print("\n" + "="*50)
            choice = input("Capture done. Play back via robot? (y/n): ").strip().lower()
            if choice == 'y':
                node.process_and_play()
    except KeyboardInterrupt: pass
    finally:
        def watchdog_exit():
            time.sleep(2)
            os._exit(0)
        threading.Thread(target=watchdog_exit, daemon=True).start()
        node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()

if __name__ == "__main__":
    main()
