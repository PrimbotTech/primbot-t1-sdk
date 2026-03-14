#!/usr/bin/env python3
"""
启元机器人 SDK 示例节点

本示例展示了如何调用 SDK 提供的核心服务接口，包括播放表情、执行运动控制动作以及播放 TTS 语音。

使用示例:
    # 确保已 source 环境
    ros2 run py_examples demo --ros-args -p param_name:=value
    # 或者直接运行脚本
    python3 demo.py
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from aimdk_msgs.srv import PlayEmotion, SetMcAction, PlayTts
from aimdk_msgs.msg import PlayTtsRequest, TtsPriorityLevel, McActionCommand, McAction, RequestHeader

class DemoNode(Node):
    def __init__(self):
        super().__init__('demo_node')
        self.get_logger().info('Demo Node Initialized.')

    def call_service(self, srv_type, srv_name, request, timeout=5.0):
        """通用同步 Service 调用"""
        cli = self.create_client(srv_type, srv_name)
        
        while not cli.wait_for_service(timeout_sec=1.0):
            if not rclpy.ok():
                self.get_logger().error(f'Interrupted while waiting for the service {srv_name}')
                return None
            self.get_logger().info(f'Waiting for service {srv_name} to become available...')

        future = cli.call_async(request)
        
        # 结合超时机制旋转，防卡死
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)
        
        if future.done():
            return future.result()
        else:
            self.get_logger().error(f'Service call {srv_name} timed out.')
            return None

    def run_demo(self):
        self.get_logger().info('Starting 启元机器人 aimdk_msgs Python Demo...')

        # 1. 播放表情 (Interaction层 srv)
        self.get_logger().info('[1/3] 播放表情(happy)...')
        emotion_req = PlayEmotion.Request()
        # 使用常量 (EMOTION_EYE_HAPPY = 90)
        emotion_req.emotion_id = 90 
        emotion_req.mode = PlayEmotion.Request.EMOTION_MODE_ONCE
        emotion_req.priority = 10
        resp = self.call_service(PlayEmotion, '/interaction/play_emotion', emotion_req)
        if resp:
            self.get_logger().info(f'命令发送成功: {resp.success}')

        # 2. 做一个动作 (MC层 srv)
        # 我们以发送 "握手"(QUADRUPED_LOCOMOTION_HANDSHAKE = 107) 为例
        self.get_logger().info('[2/3] 执行动作(握手)...')
        action_req = SetMcAction.Request()
        action_req.header = RequestHeader()
        action_req.command = McActionCommand()
        action_req.command.action = McAction()
        action_req.command.action.value = McAction.QUADRUPED_LOCOMOTION_HANDSHAKE
        action_req.command.action_desc = "handshake demo from python"
        resp = self.call_service(SetMcAction, '/mc/set_action', action_req)
        if resp:
            self.get_logger().info('动作命令发送完成')

        # 3. 播放 TTS 语音 (Interaction层 srv)
        self.get_logger().info('[3/3] 播放TTS语音...')
        tts_req = PlayTts.Request()
        tts_req.tts_req = PlayTtsRequest()
        tts_req.tts_req.text = "你好，我是启元机器人。SDK测试成功！"
        tts_req.tts_req.priority_level = TtsPriorityLevel()
        tts_req.tts_req.priority_level.value = TtsPriorityLevel.INTERACTION_L6
        tts_req.tts_req.domain = "sdk_demo_py"
        tts_req.tts_req.is_interrupted = True
        resp = self.call_service(PlayTts, '/interaction/play_tts', tts_req)
        if resp:
            self.get_logger().info('TTS请求发送完成')

        self.get_logger().info('=== Demo 完成 ===')

def main(args=None):
    rclpy.init(args=args)
    node = None

    try:
        node = DemoNode()
        node.run_demo()
    except KeyboardInterrupt:
        pass
    except Exception as e:
        import rclpy.logging
        rclpy.logging.get_logger('main').error(f'Unexpected Error: {e}')
    finally:
        if node:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
