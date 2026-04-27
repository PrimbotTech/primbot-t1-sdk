#!/usr/bin/env python3

"""Example client for CaptureJpegImage service.

The following ROS parameters can be set via startup arguments:
--ros-args -p <name>:=<value>

Supported parameters:
  - service_name: CaptureJpegImage service name
  - camera_id: Camera identifier
        Available camera_id options:
        - head_monocular_centra
        - head_stereo_left
        - head_stereo_right
        For details, please refer to the interface documentation.
  - timeout_ms: Wait time for a fresh JPEG frame, in milliseconds. Use 0 to
    follow the server default.
  - output_file: Local JPEG output path. Leave empty to auto-generate one.

Interactive Mode:
  If camera_id is not provided via --ros-args -p camera_id:=<value>,
  the script will prompt for input. Press Enter to use the default value.

Examples:

  # Interactive mode (will prompt for camera_id)
  python3 /workspace/sdk/examples/python/get_jpg.py 
"""

from pathlib import Path
import sys

import rclpy
import rclpy.logging
from rclpy.node import Node

from aimdk_msgs.msg import CommonRequest, CommonState
from aimdk_msgs.srv import CaptureJpegImage



DEFAULT_SERVICE_NAME = "/aima/hal/camera/CaptureJpegImage"
DEFAULT_OUTPUT_FILE = "/tmp/camera_capture.jpg"
DEFAULT_REQUEST_TIMEOUT_MS = 5000
SERVICE_WAIT_SECONDS = 2.0
MIN_CALL_TIMEOUT_MS = 6000


def prepare_output_path(output_file: str) -> Path:
    output_path = Path(output_file or DEFAULT_OUTPUT_FILE).expanduser()
    if output_path.suffix.lower() not in {".jpg", ".jpeg"}:
        raise ValueError("output_file must use the .jpg or .jpeg extension.")

    output_path.parent.mkdir(parents=True, exist_ok=True)
    return output_path


def get_camera_id_from_user() -> str:
    """Interactively get camera_id from user input."""
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
    camera_id = input("\nPlease enter camera ID: ").strip()
    
    if not camera_id:
        print("\nError: camera_id cannot be empty.")
        print("Exiting...\n")
        sys.exit(1)
    
    print(f"\nUsing camera_id: {camera_id}")
    print("="*60 + "\n", flush=True)
    return camera_id


class CaptureJpegClient(Node):
    def __init__(self, camera_id: str) -> None:
        super().__init__("get_jpg")

        self.service_name = self.declare_parameter(
            "service_name", DEFAULT_SERVICE_NAME
        ).value
        self.timeout_ms = self.declare_parameter(
            "timeout_ms", DEFAULT_REQUEST_TIMEOUT_MS
        ).value
        self.output_file = self.declare_parameter("output_file", "").value 
        self.camera_id = camera_id  

        if not self.service_name:
            raise ValueError("service_name must not be empty.")

        if self.timeout_ms < 0:
            raise ValueError("timeout_ms must be greater than or equal to 0.")

        self.output_path = prepare_output_path(self.output_file)
        self.client = self.create_client(CaptureJpegImage, self.service_name)

        # 打印客户端创建成功日志
        self.get_logger().info(
            "CaptureJpegImage client created. "
            f"service={self.service_name} "
            f"camera_id={self.camera_id} "
            f"timeout_ms={self.timeout_ms} "
            f"output_file={self.output_path}"
        )

    def wait_for_service(self) -> bool:
        while not self.client.wait_for_service(timeout_sec=SERVICE_WAIT_SECONDS):
            if not rclpy.ok():
                return False
            self.get_logger().info(f"Waiting for service: {self.service_name}")

        self.get_logger().info(f"Service available: {self.service_name}")
        return True

    def capture_once(self) -> bool:
        if not self.wait_for_service():
            return False

        request = CaptureJpegImage.Request()
        request.request = CommonRequest()
        request.request.header.stamp = self.get_clock().now().to_msg()
        request.camera_id = self.camera_id
        request.timeout_ms = self.timeout_ms

        self.get_logger().info(
            f"Sending CaptureJpegImage request: timeout_ms={request.timeout_ms}"
        )

        future = self.client.call_async(request)
        call_timeout_sec = max(MIN_CALL_TIMEOUT_MS, self.timeout_ms + 1000) / 1000.0
        rclpy.spin_until_future_complete(self, future, timeout_sec=call_timeout_sec)

        if not future.done():
            self.get_logger().error(
                f"CaptureJpegImage timed out after {int(call_timeout_sec * 1000)} ms."
            )
            return False

        response = future.result()
        if response is None:
            self.get_logger().error("CaptureJpegImage returned an empty response.")
            return False

        return self.save_response(response)

    def save_response(self, response: CaptureJpegImage.Response) -> bool:
        code = response.response.header.code
        status = response.response.status.value
        if code != 0 and status != CommonState.SUCCESS:
            self.get_logger().error(
                "CaptureJpegImage failed. "
                f"code={code} status={status} msg={response.response.message}"
            )
            return False

        jpeg = response.jpeg
        image = jpeg.image
        jpeg_bytes = bytes(image.data)
        if not jpeg_bytes:
            self.get_logger().error("CaptureJpegImage returned empty JPEG data.")
            return False

        try:
            self.output_path.write_bytes(jpeg_bytes)
        except OSError:
            self.get_logger().error(f"Failed to write JPEG file: {self.output_path}")
            return False

        self.get_logger().info(
            "JPEG saved: "
            f"file={self.output_path} "
            f"bytes={len(jpeg_bytes)} "
            f"format={image.format} "
            f"camera_id={jpeg.camera_id} "
            f"device={jpeg.device} "
            f"width={jpeg.width} "
            f"height={jpeg.height} "
            f"framerate={jpeg.framerate} "
            f"frame_id={image.header.frame_id}"
        )
        return True


def main(args=None) -> int:
    rclpy.init(args=args)
    node = None

    try:
        camera_id = get_camera_id_from_user()
        
        node = CaptureJpegClient(camera_id)
        return 0 if node.capture_once() else 1
    except Exception as error:  # noqa: BLE001
        rclpy.logging.get_logger("get_jpg").error(
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
