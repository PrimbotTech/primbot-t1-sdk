#!/usr/bin/env python3

"""RTSP Video Stream Reader Example
  
RTSP URL Format:
  rtsp://{ip}:2554/live_{camera_id}

Default MP4 Output Path:
  /tmp/video_capture.mp4

Supported Arguments:
  --camera_id, --camera: Camera identifier (interactive prompt if not provided)
  --robot_ip, --ip: Robot IP address (interactive prompt if not provided)
  --output_file, --output: MP4 output file path (default: /tmp/video_capture.mp4)
  --capture_seconds, --duration: Stop automatically after this many seconds
    (default: 5.0). Set <= 0 to run until Ctrl+C.

Interactive Mode:
  If --camera or --ip is not provided, the script will prompt for input.
  Press Enter to use the default values shown in brackets.

Examples:
  # Interactive mode (will prompt for camera and IP)
  python3 examples/python/get_video_stream.py
"""

from __future__ import annotations

import argparse
from pathlib import Path
import sys
import time
from typing import Any

RTSP_URL = "rtsp://{ip}:2554/live_{camera_id}"
DEFAULT_OUTPUT_FILE = "/tmp/video_capture.mp4"
DEFAULT_CAPTURE_SECONDS = 5.0
DEFAULT_LOG_EVERY_N_FRAMES = 100
DEFAULT_FALLBACK_FPS = 25.0
DEFAULT_OPEN_TIMEOUT_MS = 5000
DEFAULT_READ_TIMEOUT_MS = 5000
MP4_CODEC = "mp4v"


def load_opencv() -> Any:
    try:
        import cv2
    except ImportError as error:
        raise RuntimeError(
            "This example requires `opencv-python`. Install it in the runtime "
            "environment before running the script."
        ) from error

    return cv2


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Read decoded frames from the fixed RTSP stream and save MP4.",
        epilog=f"Default MP4 output: {DEFAULT_OUTPUT_FILE}",
    )
    parser.add_argument(
        "--output_file",
        "--output",
        dest="output_file",
        default=DEFAULT_OUTPUT_FILE,
        help="Output MP4 file path.",
    )
    parser.add_argument(
        "--capture_seconds",
        "--duration",
        dest="capture_seconds",
        type=float,
        default=DEFAULT_CAPTURE_SECONDS,
        help="Stop automatically after this many seconds. Use <= 0 to run forever.",
    )
    parser.add_argument(
        "--camera_id",
        "--camera",
        dest="camera_id",
        default=None,  
        help="Camera identifier (e.g., head_monocular_centra, head_stereo_left).",
    )
    parser.add_argument(
        "--robot_ip",
        "--ip",
        dest="robot_ip",
        default=None,  
        help="Robot IP address (e.g., 10.1.1.100 or 192.168.1.100).",
    )
    
    args = parser.parse_args()
    
    if args.robot_ip is None:
        print()
        print("robot_ip is required. Please provide it via command line argument.")
        args.robot_ip = input("Please enter robot IP address : ").strip()
        if not args.robot_ip:
            sys.exit(1) 
    
    if args.camera_id is None:
        print()
        print("\n" + "="*60)
        print("Available Camera IDs:")
        print("="*60)
        print("  head_monocular_centra  - 头部单目中央相机")
        print("  head_stereo_left       - 头部双目左相机")
        print("  head_stereo_right      - 头部双目右相机")
        print("  head_fisheye_1         - 头部鱼眼相机 1")
        print("  head_fisheye_2         - 头部鱼眼相机 2")
        print("  head_TOF3D             - 头部 TOF 3D 相机")
        print("="*60)
        print("For camera_id corresponding to different robot configurations, please refer to the interface documentation.")
        print("")
        args.camera_id = input("Please enter camera ID : ").strip()
        if not args.camera_id:
            sys.exit(1)
    
    return args


class RtspVideoStreamReader:
    def __init__(self, args: argparse.Namespace) -> None:
        self.cv2 = load_opencv()
        self.output_file = args.output_file
        self.capture_seconds = args.capture_seconds
        self.camera_id = args.camera_id    
        self.robot_ip = args.robot_ip
        self.url = RTSP_URL.format(
            ip=self.robot_ip,
            camera_id=self.camera_id
        )
        self.log_every_n_frames = DEFAULT_LOG_EVERY_N_FRAMES

        self.capture = None
        self.output_path = self._prepare_output_file(self.output_file)
        self.output_writer = None
        self.first_frame_received = False
        self.summary_logged = False
        self.frame_count = 0
        self.total_frame_bytes = 0
        self.first_frame_monotonic = None
        self.frame_width = 0
        self.frame_height = 0
        self.stream_fps = 0.0
        self.output_fps = 0.0
        self.backend_name = ""

    def _prepare_output_file(self, output_file: str) -> Path:
        output_path = Path(output_file).expanduser()
        if output_path.suffix.lower() != ".mp4":
            raise ValueError("output_file must use the .mp4 extension.")

        output_path.parent.mkdir(parents=True, exist_ok=True)
        return output_path

    def _try_open_capture(
        self,
        api_preference: int,
        backend_label: str,
        quiet_opencv_logs: bool = False,
    ):
        capture = self.cv2.VideoCapture()
        previous_log_level = None

        try:
            if quiet_opencv_logs and hasattr(self.cv2, "getLogLevel") and hasattr(
                self.cv2, "setLogLevel"
            ):
                previous_log_level = self.cv2.getLogLevel()
                self.cv2.setLogLevel(0)

            try:
                opened = capture.open(
                    self.url,
                    api_preference,
                    [
                        self.cv2.CAP_PROP_OPEN_TIMEOUT_MSEC,
                        DEFAULT_OPEN_TIMEOUT_MS,
                        self.cv2.CAP_PROP_READ_TIMEOUT_MSEC,
                        DEFAULT_READ_TIMEOUT_MS,
                    ],
                )
            except TypeError:
                opened = capture.open(self.url, api_preference)
        except self.cv2.error:
            capture.release()
            return None
        finally:
            if previous_log_level is not None:
                self.cv2.setLogLevel(previous_log_level)

        if not opened or not capture.isOpened():
            capture.release()
            return None

        actual_backend = backend_label
        try:
            actual_backend = capture.getBackendName()
        except self.cv2.error:
            pass

        print(
            "Opened RTSP stream: backend=%s open_timeout_ms=%d read_timeout_ms=%d"
            % (
                actual_backend,
                DEFAULT_OPEN_TIMEOUT_MS,
                DEFAULT_READ_TIMEOUT_MS,
            ),
            flush=True,
        )
        self.backend_name = actual_backend
        return capture

    def _open_capture(self) -> None:
        self.capture = self._try_open_capture(self.cv2.CAP_FFMPEG, "FFMPEG")
        if self.capture is not None:
            return

        print(
            "FFMPEG backend open failed, falling back to the default backend.",
            flush=True,
        )
        self.capture = self._try_open_capture(
            self.cv2.CAP_ANY,
            "DEFAULT",
            quiet_opencv_logs=True,
        )
        if self.capture is not None:
            return

        raise RuntimeError(f"Failed to open RTSP stream: {self.url}")

    def _is_valid_fps(self, value: float) -> bool:
        return value > 0.0 and value < 1000.0

    def _resolve_output_fps(self) -> float:
        stream_fps = self.capture.get(self.cv2.CAP_PROP_FPS)
        if self._is_valid_fps(stream_fps):
            return stream_fps

        return DEFAULT_FALLBACK_FPS

    def _create_output_writer(self) -> None:
        self.output_writer = self.cv2.VideoWriter(
            str(self.output_path),
            self.cv2.VideoWriter_fourcc(*MP4_CODEC),
            self.output_fps,
            (self.frame_width, self.frame_height),
        )
        if not self.output_writer.isOpened():
            self.output_writer.release()
            self.output_writer = None
            raise RuntimeError(f"Failed to create output file: {self.output_path}")

    def _write_frame(self, frame) -> None:
        if self.output_writer is None:
            return

        try:
            self.output_writer.write(frame)
        except self.cv2.error as error:
            raise RuntimeError(
                f"Failed while writing to {self.output_path}: {error}"
            ) from error

        if not self.output_writer.isOpened():
            raise RuntimeError("Video writer closed unexpectedly during capture.")

    def _log_first_frame(self, frame) -> None:
        payload_bytes = frame.nbytes
        print(
            "First frame received: resolution=%dx%d channels=%d payload_bytes=%d "
            "stream_fps=%.3f output_fps=%.3f backend=%s"
            % (
                self.frame_width,
                self.frame_height,
                1 if frame.ndim == 2 else frame.shape[2],
                payload_bytes,
                self.stream_fps,
                self.output_fps,
                self.backend_name,
            ),
            flush=True,
        )

    def _handle_frame(self, frame) -> None:
        frame_bytes = frame.nbytes
        self.frame_count += 1
        self.total_frame_bytes += frame_bytes

        if not self.first_frame_received:
            self.first_frame_received = True
            self.first_frame_monotonic = time.monotonic()
            self.frame_height, self.frame_width = frame.shape[:2]

            stream_fps = self.capture.get(self.cv2.CAP_PROP_FPS)
            if self._is_valid_fps(stream_fps):
                self.stream_fps = stream_fps
            self.output_fps = self._resolve_output_fps()
            self._create_output_writer()

            self._log_first_frame(frame)
            print(f"MP4 output file: {self.output_path}", flush=True)
            if self.capture_seconds > 0:
                print(
                    "The reader will stop %.3f second(s) after the first frame."
                    % self.capture_seconds,
                    flush=True,
                )
            else:
                print("Auto stop disabled. Press Ctrl+C to exit.", flush=True)

        # `frame` is the decoded image read from the RTSP stream.
        # Feed it directly into your vision algorithm here.
        # Example:
        # result = your_algorithm(frame)
        self._write_frame(frame)

        if (
            self.log_every_n_frames > 0
            and self.frame_count % self.log_every_n_frames == 0
            and self.first_frame_monotonic is not None
        ):
            elapsed_ms = int((time.monotonic() - self.first_frame_monotonic) * 1000)
            print(
                "Received %d frames, total_frame_bytes=%d, elapsed=%d ms"
                % (
                    self.frame_count,
                    self.total_frame_bytes,
                    elapsed_ms,
                ),
                flush=True,
            )

    def _should_stop(self) -> bool:
        if (
            not self.first_frame_received
            or self.capture_seconds <= 0
            or self.first_frame_monotonic is None
        ):
            return False

        return time.monotonic() - self.first_frame_monotonic >= self.capture_seconds

    def _release_resources(self) -> None:
        if self.capture is not None:
            self.capture.release()
            self.capture = None

        if self.output_writer is not None:
            self.output_writer.release()
            self.output_writer = None

    def log_summary(self) -> None:
        if self.summary_logged:
            return
        self.summary_logged = True

        if not self.first_frame_received or self.first_frame_monotonic is None:
            print("No video frame received before shutdown.", flush=True)
            return

        elapsed_ms = int((time.monotonic() - self.first_frame_monotonic) * 1000)
        print(
            "Video stream summary: frames=%d total_frame_bytes=%d elapsed=%d ms "
            "resolution=%dx%d stream_fps=%.3f output_fps=%.3f backend=%s"
            % (
                self.frame_count,
                self.total_frame_bytes,
                elapsed_ms,
                self.frame_width,
                self.frame_height,
                self.stream_fps,
                self.output_fps,
                self.backend_name,
            ),
            flush=True,
        )
        print(f"Captured MP4 file: {self.output_path}", flush=True)

    def run(self) -> int:
        self._open_capture()
        try:
            while True:
                ok, frame = self.capture.read()
                if not ok or frame is None or frame.size == 0:
                    raise RuntimeError("Failed to read frame from the RTSP stream.")

                self._handle_frame(frame)
                if self._should_stop():
                    print(
                        "Reached capture_seconds=%.3f, shutting down."
                        % self.capture_seconds,
                        flush=True,
                    )
                    break
        except KeyboardInterrupt:
            print("Capture interrupted by user.", flush=True)
        finally:
            self._release_resources()
            self.log_summary()

        return 0


def main() -> int:
    try:
        args = parse_args()
        reader = RtspVideoStreamReader(args)
        return reader.run()
    except Exception as error:  # noqa: BLE001
        print(f"Error: {error}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
