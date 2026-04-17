#!/usr/bin/env python3
# 指定使用 Python3 解释器执行此脚本

"""
CaptureJpegImage 服务的示例客户端

以下 ROS 参数可以通过启动参数设置：
--ros-args -p <name>:=<value>

支持的参数：
  - service_name: CaptureJpegImage 服务名称
  - timeout_ms: 等待新鲜 JPEG 帧的时间（毫秒）。使用 0 跟随服务器默认值。
  - output_file: 本地 JPEG 输出路径。留空则自动生成。

示例：
  python3 /workspace/sdk/examples/python/get_jpg.py

  python3 /workspace/sdk/examples/python/get_jpg.py --ros-args \
    -p output_file:=/tmp/camera_left.jpg

  python3 /workspace/sdk/examples/python/get_jpg.py --ros-args \
    -p service_name:=/aima/hal/camera/bgr_5Fcamera_5Fr/capture_5Fjpeg \
    -p timeout_ms:=0
"""

from pathlib import Path  # 导入路径处理模块

import rclpy  # 导入 ROS2 Python 客户端库
import rclpy.logging  # 导入 ROS2 日志模块
from rclpy.node import Node  # 导入 ROS2 节点基类

from aimdk_msgs.msg import CommonRequest, CommonState  # 导入通用请求和状态消息类型
from aimdk_msgs.srv import CaptureJpegImage  # 导入 JPEG 图像捕获服务类型


# 默认服务名称（左摄像头）
DEFAULT_SERVICE_NAME = "/aima/hal/camera/bgr_5Fcamera_5Fl/capture_5Fjpeg"
# 默认输出文件路径
DEFAULT_OUTPUT_FILE = "/tmp/camera_capture.jpg"
# 默认请求超时时间（毫秒）
DEFAULT_REQUEST_TIMEOUT_MS = 5000
# 服务等待时间（秒）
SERVICE_WAIT_SECONDS = 2.0
# 最小调用超时时间（毫秒）
MIN_CALL_TIMEOUT_MS = 6000


def prepare_output_path(output_file: str) -> Path:  # 准备输出路径的函数
    output_path = Path(output_file or DEFAULT_OUTPUT_FILE).expanduser()  # 使用默认值或用户指定路径，并展开用户目录
    if output_path.suffix.lower() not in {".jpg", ".jpeg"}:  # 检查文件扩展名
        raise ValueError("output_file must use the .jpg or .jpeg extension.")  # 如果不是 jpg/jpeg 则抛出错误

    output_path.parent.mkdir(parents=True, exist_ok=True)  # 创建父目录（如果不存在）
    return output_path  # 返回路径对象


class CaptureJpegClient(Node):  # 定义 JPEG 捕获客户端类，继承自 Node
    def __init__(self) -> None:  # 初始化方法
        super().__init__("get_jpg")  # 调用父类初始化，设置节点名称

        # 声明并获取 service_name 参数
        self.service_name = self.declare_parameter(
            "service_name", DEFAULT_SERVICE_NAME
        ).value
        # 声明并获取 timeout_ms 参数
        self.timeout_ms = self.declare_parameter(
            "timeout_ms", DEFAULT_REQUEST_TIMEOUT_MS
        ).value
        # 声明并获取 output_file 参数
        self.output_file = self.declare_parameter("output_file", "").value
        # 声明并获取 camera_id 参数（可选）
        self.camera_id = self.declare_parameter("camera_id", "").value

        if not self.service_name:  # 检查服务名称是否为空
            raise ValueError("service_name must not be empty.")  # 抛出错误

        if self.timeout_ms < 0:  # 检查超时时间是否为负数
            raise ValueError("timeout_ms must be greater than or equal to 0.")  # 抛出错误

        self.output_path = prepare_output_path(self.output_file)  # 准备输出路径
        self.client = self.create_client(CaptureJpegImage, self.service_name)  # 创建服务客户端

        # 打印客户端创建成功日志
        self.get_logger().info(
            "CaptureJpegImage client created. "
            f"service={self.service_name} "
            f"timeout_ms={self.timeout_ms} "
            f"output_file={self.output_path}"
        )

    def wait_for_service(self) -> bool:  # 等待服务可用的方法
        while not self.client.wait_for_service(timeout_sec=SERVICE_WAIT_SECONDS):  # 每 2 秒检查一次
            if not rclpy.ok():  # 如果 ROS2 已关闭
                return False  # 返回失败
            self.get_logger().info(f"Waiting for service: {self.service_name}")  # 打印等待日志

        self.get_logger().info(f"Service available: {self.service_name}")  # 打印服务可用日志
        return True  # 返回成功

    def capture_once(self) -> bool:  # 捕获一次图像的方法
        if not self.wait_for_service():  # 等待服务可用
            return False  # 如果服务不可用，返回失败

        request = CaptureJpegImage.Request()  # 创建服务请求对象
        request.request = CommonRequest()  # 创建通用请求对象
        request.request.header.stamp = self.get_clock().now().to_msg()  # 设置请求时间戳
        request.camera_id = self.camera_id  # 设置相机 ID
        request.timeout_ms = self.timeout_ms  # 设置超时时间

        # 打印请求日志
        self.get_logger().info(
            f"Sending CaptureJpegImage request: timeout_ms={request.timeout_ms}"
        )

        future = self.client.call_async(request)  # 异步调用服务
        call_timeout_sec = max(MIN_CALL_TIMEOUT_MS, self.timeout_ms + 1000) / 1000.0  # 计算调用超时（秒）
        rclpy.spin_until_future_complete(self, future, timeout_sec=call_timeout_sec)  # 等待响应

        if not future.done():  # 如果超时未完成
            self.get_logger().error(  # 打印超时错误
                f"CaptureJpegImage timed out after {int(call_timeout_sec * 1000)} ms."
            )
            return False  # 返回失败

        response = future.result()  # 获取响应结果
        if response is None:  # 如果响应为空
            self.get_logger().error("CaptureJpegImage returned an empty response.")  # 打印错误
            return False  # 返回失败

        return self.save_response(response)  # 保存响应到文件

    def save_response(self, response: CaptureJpegImage.Response) -> bool:  # 保存响应的方法
        code = response.response.header.code  # 获取响应代码
        status = response.response.status.value  # 获取状态值
        if code != 0 and status != CommonState.SUCCESS:  # 如果失败
            self.get_logger().error(  # 打印错误日志
                "CaptureJpegImage failed. "
                f"code={code} status={status} msg={response.response.message}"
            )
            return False  # 返回失败

        jpeg = response.jpeg  # 获取 JPEG 数据
        image = jpeg.image  # 获取图像信息
        jpeg_bytes = bytes(image.data)  # 转换为字节数组
        if not jpeg_bytes:  # 如果数据为空
            self.get_logger().error("CaptureJpegImage returned empty JPEG data.")  # 打印错误
            return False  # 返回失败

        try:  # 尝试写入文件
            self.output_path.write_bytes(jpeg_bytes)  # 写入 JPEG 数据
        except OSError:  # 捕获操作系统错误
            self.get_logger().error(f"Failed to write JPEG file: {self.output_path}")  # 打印错误
            return False  # 返回失败

        # 打印保存成功日志
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
        return True  # 返回成功


def main(args=None) -> int:  # 主函数入口
    rclpy.init(args=args)  # 初始化 ROS2
    node = None  # 初始化节点变量

    try:  # 尝试执行主逻辑
        node = CaptureJpegClient()  # 创建 JPEG 捕获客户端
        return 0 if node.capture_once() else 1  # 捕获成功返回 0，失败返回 1
    except Exception as error:  # noqa: BLE001  # 捕获所有异常
        rclpy.logging.get_logger("get_jpg").error(  # 打印异常日志
            f"Program exited with exception: {error}"
        )
        return 1  # 异常退出返回 1
    finally:  # 无论是否异常都执行
        if node is not None:  # 如果节点存在
            node.destroy_node()  # 销毁节点
        if rclpy.ok():  # 如果 ROS2 仍然正常
            rclpy.shutdown()  # 关闭 ROS2


if __name__ == "__main__":  # 如果直接运行此脚本
    raise SystemExit(main())  # 调用主函数并退出程序
