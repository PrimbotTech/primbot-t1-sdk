#!/usr/bin/env python3

"""Camera JPEG Capture Example Script

Description:
  This script demonstrates how to capture a single JPEG frame from the robot's camera
  using the CaptureJpegImage service. Supports T1 camera IDs with interactive selection
  when camera_id is not provided.

Prerequisites:
  - CaptureJpegImage service must be available
  - Camera hardware must be connected

Usage:
  python3 get_jpg.py --ros-args -p camera_id:=<ID> -p output_file:=<path> -p timeout_ms:=<ms>

Supported parameters:
  - service_name: CaptureJpegImage service name
  - camera_id: Camera identifier
        Available camera_id options:
        - head_stereo_left
        - head_stereo_right
        - head_TOF3D
        - hip_TOF3D_front
        - hip_TOF3D_back
        - TOF3D_left
        - TOF3D_right
        - fisheye_left
        - fisheye_right
        For details, please refer to the interface documentation.
  - timeout_ms: Wait time for a fresh JPEG frame, in milliseconds. Use 0 to
    follow the server default.
  - output_file: Local JPEG output path. Leave empty to auto-generate one.

Interactive Mode:
  If camera_id is not provided via --ros-args -p camera_id:=<value>,
  the script will prompt for input. Press Enter to use the default value.

Examples:
  # Interactive mode (will prompt for camera_id)
  python3 get_jpg.py

  # Non-interactive mode
  python3 get_jpg.py --ros-args -p camera_id:=head_stereo_left
"""

from pathlib import Path
import sys
import time

import rclpy
import rclpy.logging
from rclpy.node import Node

from aimdk_msgs.msg import CommonRequest, CommonState
from aimdk_msgs.srv import CaptureJpegImage
from aimdk_msgs.srv import SetServo



DEFAULT_SERVICE_NAME = "/aima/hal/camera/CaptureJpegImage"
DEFAULT_OUTPUT_FILE = "/tmp/camera_capture.jpg"
DEFAULT_REQUEST_TIMEOUT_MS = 5000
SERVICE_WAIT_SECONDS = 2.0
MIN_CALL_TIMEOUT_MS = 6000
SET_SERVO_SERVICE_NAME = "/aimdk_5Fmsgs/srv/SetServo"


def prepare_output_path(output_file: str) -> Path:
    output_path = Path(output_file or DEFAULT_OUTPUT_FILE).expanduser()
    if output_path.suffix.lower() not in {".jpg", ".jpeg"}:
        raise ValueError("output_file must use the .jpg or .jpeg extension.")

    output_path.parent.mkdir(parents=True, exist_ok=True)
    return output_path


def get_camera_id_from_user() -> str:
    """Interactively get camera_id from user input."""
    print("\n" + "="*60)
    print("The list of T1 Camera IDs is as follows:")
    print("="*60)
    print("  head_stereo_left       - 头部双目左机")
    print("  head_stereo_right      - 头部双目右机")
    print("  head_TOF3D             - 头部TOF")
    print("  hip_TOF3D_front        - 胯部前TOF")
    print("  hip_TOF3D_back         - 胯部后TOF")
    print("  TOF3D_left             - 左侧TOF")
    print("  TOF3D_right            - 右侧TOF")
    print("  fisheye_left           - 左侧鱼眼")
    print("  fisheye_right          - 右侧鱼眼")
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


def read_int(prompt: str, default: int) -> int:
    """Read integer from user input with default value."""
    text = input(prompt).strip()
    if not text:
        return default
    return int(text)


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
        self.servo_client = self.create_client(SetServo, SET_SERVO_SERVICE_NAME)

        # 打印客户端创建成功日志
        self.get_logger().info(
            "CaptureJpegImage client created. "
            f"service={self.service_name} "
            f"camera_id={self.camera_id} "
            f"timeout_ms={self.timeout_ms} "
            f"output_file={self.output_path}"
        )
        self.get_logger().info(
            f"SetServo client created. service={SET_SERVO_SERVICE_NAME}"
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

        for i in range(3):
            future = self.client.call_async(request)
            call_timeout_sec = max(MIN_CALL_TIMEOUT_MS, self.timeout_ms + 1000) / 1000.0
            rclpy.spin_until_future_complete(self, future, timeout_sec=call_timeout_sec)

            if future.done():
                break

            self.get_logger().info(f'trying ... [{i}]')
            time.sleep(0.2)
            
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
                f"CaptureJpegImage failed. "
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
            f"exposure={jpeg.exposure} "
            f"gain={jpeg.gain} "
            f"frame_id={image.header.frame_id}"
        )
        return True

    def set_servo(self, position: int, speed: int) -> bool:
        """Set servo position and speed.
        
        Args:
            position: Servo position in degrees (0-90)
                     0 = quadruped mode, 90 = biped mode
            speed: Movement speed (100-1000 steps/s)
                  1 step = 0.225 degrees
        """
        # Validate parameters
        if position < 0 or position > 90:
            self.get_logger().error(f"Invalid position: {position}. Must be 0-90 degrees.")
            return False
        
        if speed < 100 or speed > 1000:
            self.get_logger().error(f"Invalid speed: {speed}. Must be 100-1000.")
            return False
        
        # Wait for service
        while not self.servo_client.wait_for_service(timeout_sec=SERVICE_WAIT_SECONDS):
            if not rclpy.ok():
                return False
            self.get_logger().info(f"Waiting for service: {SET_SERVO_SERVICE_NAME}")
        
        # Create request
        request = SetServo.Request()
        request.request = CommonRequest()
        request.request.header.stamp = self.get_clock().now().to_msg()
        request.position = position
        request.speed = speed
        
        self.get_logger().info(
            f"Sending SetServo request: position={position}°, speed={speed}"
        )
        
        # Call service
        future = self.servo_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        
        if not future.done():
            self.get_logger().error("SetServo timed out.")
            return False
        
        response = future.result()
        if response is None:
            self.get_logger().error("SetServo returned an empty response.")
            return False
        
        # Check response
        code = response.header.header.code
        if code != 0:
            self.get_logger().error(
                f"SetServo failed. code={code}"
            )
            return False
        
        self.get_logger().info(
            f"SetServo success. position={position}°, speed={speed}"
        )
        return True


def main(args=None) -> int:
    rclpy.init(args=args)
    node = None

    try:
        # Print menu
        print("\n" + "="*60)
        print("  Camera & Servo Control Menu")
        print("="*60)
        print("\nSelect control mode:")
        print("  1. Capture JPEG Image")
        print("  2. Set Servo Position")
        print("="*60)
        
        choice = read_int("\nEnter choice (1-2, default 1): ", 1)
        
        if choice == 1:
            # Camera capture mode
            camera_id = get_camera_id_from_user()
            node = CaptureJpegClient(camera_id)
            return 0 if node.capture_once() else 1
            
        elif choice == 2:
            # Servo control mode
            node = CaptureJpegClient("dummy")  # Still need node for servo client
            
            position = read_int("\nEnter servo position (0-90 degrees): ", 0, 90)
            speed = read_int("Enter servo speed (100-1000): ", 100, 1000)
            
            return 0 if node.set_servo(position, speed) else 1
            
        else:
            print("\n✗ Invalid choice")
            return 1
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
