# SDK 示例代码逐行解读文档

本文档对 `primebot_sdk` 示例代码（`demo.cpp` 和 `demo.py`）进行逐行解读，并详细说明其中的核心技术点，帮助开发者理解 ROS 2 节点编写规范及 SDK 接口调用流程。

---

## 1. C++ 示例解读 (`demo.cpp`)

### 源代码逐行注释
```cpp
#include <signal.h>   // 引入信号处理头文件，用于处理 Ctrl+C
#include <chrono>     // 时间库，用于定义超时（如 3s, 5s）
#include <iostream>   // 标准输入输出
#include <memory>     // 智能指针支持

#include "aimdk_msgs/srv/play_emotion.hpp" // 引用表情播放服务
#include "aimdk_msgs/srv/play_tts.hpp"     // 引用语音合成服务
#include "aimdk_msgs/srv/set_mc_action.hpp" // 引用运动控制服务
#include "rclcpp/rclcpp.hpp"               // ROS 2 C++ 核心库

using namespace std::chrono_literals;      // 允许使用 1s, 500ms 等时间后缀

// 全局指针，用于在外部信号处理函数中操作 Node
std::shared_ptr<rclcpp::Node> g_node = nullptr;

// 信号处理函数：捕获 Ctrl+C（SIGINT）等，确保进程安全退出
void signal_handler(int signal)
{
  if (g_node) {
    RCLCPP_INFO(g_node->get_logger(), "Received signal %d, shutting down...", signal);
    g_node.reset(); // 销毁节点实例
  }
  rclcpp::shutdown(); // 关闭 ROS 2 通讯
  exit(signal);       // 退出程序
}

// 定义演示节点类，继承自 rclcpp::Node
class DemoNode : public rclcpp::Node
{
 public:
  DemoNode() : Node("cpp_demo_node") // 构造函数，初始化节点名为 cpp_demo_node
  {
    RCLCPP_INFO(this->get_logger(), "Demo Node Initialized.");
  }

  // 通用同步调用 Service 的模板函数
  template <typename ServiceT>
  typename ServiceT::Response::SharedPtr call_service(
    const std::string& service_name,
    typename ServiceT::Request::SharedPtr request)
  {
    // 动态创建客户端
    auto client = this->create_client<ServiceT>(service_name);

    // 等待服务上线（最多 3 秒），防止调用未就绪的服务
    if (!client->wait_for_service(3s)) {
      if (!rclcpp::ok()) { // 检查 ROS 环境是否仍然存活
        RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service %s.", service_name.c_str());
        return nullptr;
      }
      RCLCPP_ERROR(this->get_logger(), "Service %s not available after waiting.", service_name.c_str());
      return nullptr;
    }

    // 异步发送请求并获取 Future
    auto result_future = client->async_send_request(request);

    // 等待异步结果（设置 5 秒超时），防止网络波动导致无限阻塞
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result_future, 5s) == rclcpp::FutureReturnCode::SUCCESS) {
      return result_future.get(); // 返回响应指针
    } else {
      RCLCPP_ERROR(this->get_logger(), "Failed to call service %s or timeout occurred.", service_name.c_str());
      return nullptr;
    }
  }

  // 业务演示逻辑主入口
  void run_demo()
  {
    RCLCPP_INFO(this->get_logger(), "Starting 启元机器人 aimdk_msgs C++ Demo...");

    // --- 步骤 1: 播放表情 ---
    RCLCPP_INFO(this->get_logger(), "[1/3] 播放表情(happy)...");
    auto emotion_req            = std::make_shared<aimdk_msgs::srv::PlayEmotion::Request>();
    emotion_req->emotion_id     = aimdk_msgs::srv::PlayEmotion::Request::EMOTION_EYE_HAPPY; // 使用内建 ID (90)
    emotion_req->mode           = aimdk_msgs::srv::PlayEmotion::Request::EMOTION_MODE_ONCE; // 仅播放一次
    emotion_req->priority       = 10; // 优先级系数
    auto emotion_resp           = this->call_service<aimdk_msgs::srv::PlayEmotion>("/interaction/play_emotion", emotion_req);
    if (emotion_resp) {
      RCLCPP_INFO(this->get_logger(), "命令发送成功: %s", (emotion_resp->success ? "true" : "false"));
    }

    // --- 步骤 2: 执行运动控制动作 ---
    RCLCPP_INFO(this->get_logger(), "[2/3] 执行动作(握手)...");
    auto action_req                  = std::make_shared<aimdk_msgs::srv::SetMcAction::Request>();
    action_req->command.action.value = aimdk_msgs::msg::McAction::QUADRUPED_LOCOMOTION_HANDSHAKE; // 握手动作 (107)
    action_req->command.action_desc  = "handshake demo from cpp";
    auto action_resp                 = this->call_service<aimdk_msgs::srv::SetMcAction>("/mc/set_action", action_req);
    if (action_resp) {
      RCLCPP_INFO(this->get_logger(), "动作命令发送完成");
    }

    // --- 步骤 3: 播放语音合成 (TTS) ---
    RCLCPP_INFO(this->get_logger(), "[3/3] 播放TTS语音...");
    auto tts_req                          = std::make_shared<aimdk_msgs::srv::PlayTts::Request>();
    tts_req->tts_req.text                 = "你好，我是启元机器人。C++ SDK 测试成功！";
    tts_req->tts_req.priority_level.value = aimdk_msgs::msg::TtsPriorityLevel::INTERACTION_L6;
    tts_req->tts_req.domain               = "sdk_demo_cpp";
    tts_req->tts_req.is_interrupted       = true; // 允许抢占当前音频
    auto tts_resp                         = this->call_service<aimdk_msgs::srv::PlayTts>("/interaction/play_tts", tts_req);
    if (tts_resp) {
      RCLCPP_INFO(this->get_logger(), "TTS请求发送完成");
    }

    RCLCPP_INFO(this->get_logger(), "=== Demo 完成 ===");
  }
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv); // 初始化 ROS 2
  signal(SIGINT, signal_handler); // 注册系统信号
  signal(SIGTERM, signal_handler);

  try {
    g_node         = std::make_shared<DemoNode>(); // 实例化节点
    auto demo_node = std::static_pointer_cast<DemoNode>(g_node);

    demo_node->run_demo(); // 执行流程
  } catch (const std::exception& e) {
    RCLCPP_ERROR(rclcpp::get_logger("main"), "Exception: %s", e.what());
  }

  if (g_node) g_node.reset(); // 显式释放资源
  rclcpp::shutdown();
  return 0;
}
```

### 核心技术点解析

#### 1. 关于 `std::chrono_literals`
我们在代码开头使用了 `using namespace std::chrono_literals;`。
- **作用**：启用 C++“字面量”语法糖，允许直接在数字后面写时间单位（如 `3s` 代表 3 秒，`500ms` 代表 500 毫秒）。
- **优势**：使得代码中的超时设置（如 `wait_for_service(3s)`）极其易读，且符合 ROS 2 推荐的编码方案。

#### 2. “万能调用器”模板函数 `call_service`
模板定义：`template <typename ServiceT> ... call_service(...)`
- **为什么要用模板**：SDK 中有各种功能的服务类型。模板允许一个函数适配所有类型的服务，避免了重复编写“连接-发送-等待-处理”这种冗长的代码。
- **功能逻辑**：
  - **同步化**：将 ROS 2 原生异步调用封装为同步等待，确保机器人动作“按顺序”执行（如：播完语音再做动作）。
  - **安全性**：内置了 3 秒的服务上线检测和 5 秒的强制超时保护，防止程序因为模块掉线或网络卡顿而永久卡死。

---

## 2. Python 示例解读 (`demo.py`)

### 源代码逐行注释
```python
#!/usr/bin/env python3
"""
Python 版 SDK 示例。
主要流程：初始化环境 -> 类封装 -> 服务调用逻辑 -> 完善的异常捕获。
"""

import rclpy
from rclpy.node import Node
# 导入所需的 QoS 配置和接口定义
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from aimdk_msgs.srv import PlayEmotion, SetMcAction, PlayTts
from aimdk_msgs.msg import PlayTtsRequest, TtsPriorityLevel, McActionCommand, McAction, RequestHeader

class DemoNode(Node):
    def __init__(self):
        super().__init__('demo_node') # 调用父类初始化节点名
        self.get_logger().info('Demo Node Initialized.')

    def call_service(self, srv_type, srv_name, request, timeout=5.0):
        """通用同步 Service 调用辅助函数"""
        cli = self.create_client(srv_type, srv_name) # 创建客户端
        
        # 轮询等待服务上线，每秒检查一次
        while not cli.wait_for_service(timeout_sec=1.0):
            if not rclpy.ok(): # 预防等待过程中主环境被关闭
                self.get_logger().error(f'Interrupted while waiting for the service {srv_name}')
                return None
            self.get_logger().info(f'Waiting for service {srv_name} to become available...')

        # 异步调用并获取 Future 对象
        future = cli.call_async(request)
        
        # 使用 spin_until_future_complete 将异步转同步，并附加超时机制
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)
        
        if future.done(): # 正常返回
            return future.result()
        else: # 超时场景
            self.get_logger().error(f'Service call {srv_name} timed out.')
            return None

    def run_demo(self):
        """核心演示流程"""
        self.get_logger().info('Starting 启元机器人 Python Demo...')

        # --- 步骤 1: 播放表情 ---
        self.get_logger().info('[1/3] 播放表情(happy)...')
        emotion_req = PlayEmotion.Request()
        emotion_req.emotion_id = 90 # ID 常量 90 对应 Happy
        emotion_req.mode = PlayEmotion.Request.EMOTION_MODE_ONCE
        emotion_req.priority = 10
        resp = self.call_service(PlayEmotion, '/interaction/play_emotion', emotion_req)
        if resp:
            self.get_logger().info(f'命令发送成功: {resp.success}')

        # --- 步骤 2: 运动控制 ---
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

        # --- 步骤 3: 播放 TTS ---
        self.get_logger().info('[3/3] 播放TTS语音...')
        tts_req = PlayTts.Request()
        tts_req.tts_req = PlayTtsRequest()
        tts_req.tts_req.text = "你好，我是启元机器人。SDK测试成功！"
        tts_req.tts_req.priority_level = TtsPriorityLevel()
        tts_req.tts_req.priority_level.value = TtsPriorityLevel.INTERACTION_L6
        tts_req.tts_req.domain = "sdk_demo_py"
        tts_req.tts_req.is_interrupted = True # 开启中断/抢占模式
        resp = self.call_service(PlayTts, '/interaction/play_tts', tts_req)
        if resp:
            self.get_logger().info('TTS请求发送完成')

        self.get_logger().info('=== Demo 完成 ===')

def main(args=None):
    rclpy.init(args=args) # 初始化
    node = None

    try:
        node = DemoNode()
        node.run_demo() # 执行业务流程
    except KeyboardInterrupt: # 捕获 Ctrl+C
        pass
    except Exception as e: # 捕获其他非预期异常，并记录日志
        import rclpy.logging
        rclpy.logging.get_logger('main').error(f'Unexpected Error: {e}')
    finally: # 无论是否异常，最终都要确保环境干净退出
        if node:
            node.destroy_node() # 销毁节点对象
        if rclpy.ok():
            rclpy.shutdown() # 关闭 ROS 2 连接

if __name__ == '__main__':
    main()
```
