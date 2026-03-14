# SDK 代码示例编写规范指南

本文档旨在统一基于本 SDK（ROS 2 架构）的代码示例（Examples）编写风格。清晰、易读、健壮的代码示例不仅能帮助开发者快速了解内部接口，更能作为二次开发的重要参考基线。本规范分为 **通用原则**、**C++ 规范** 以及 **Python 规范** 三大部分，并针对目前 SDK 现状给出了优化改进建议。

---

## 1. 通用原则 (General Principles)

1. **节点化设计**：每个独立功能的示例应被设计为一个完整的 ROS 2 节点（Node）程序，能够作为一个独立进程被唤醒与结束。
2. **职责单一且集中**：不要为了区分“发消息”和“收消息”而强行拆分节点。如果是一套完整的展示逻辑（如：感知+判断+控制），应封装在**同一个类的同一个节点**中。
3. **健壮退出机制**：不管是 C++ 还是 Python，都需要能够处理键盘中断（Ctrl+C）和其他停机信号，从而保证能安全断开通讯（如调用 `rclcpp::shutdown()`）和正确回收系统资源。
4. **友好的控制台输出**：
   - 使用 ROS 2 官方 Logger（如 `RCLCPP_INFO` / `self.get_logger().info()`），而不是简单的 `std::cout` / `print`。
   - 针对高频主题（例如 10Hz/50Hz 的 Lidar 或 IMU），引入**输出频率节流控制**（如 1 秒钟打印一次统计信息和 FPS），避免刷屏。
   - 通过 Emoji 表情（如 `🟢`, `✅`, `❌`, `⏳`, `📨`）增加关键日志的可读性。
5. **防御性编程**：考虑到分布式系统的网络延迟及节点启动先后顺序，对于服务调用（Service Client），务必添加**等待服务就绪（wait_for_service）**以及**失败重试（Retry）**机制。

---

## 2. 示例程序目录结构 (Examples Directory Structure)

SDK 中的示例程序严格遵循 ROS 2 的标准包（Package）结构组织，分别提供 C++ 和 Python 两种语言的实现资源。

### 2.1 C++ 示例包 (examples / ament_cmake)
C++ 示例位于 `src/examples/` 目录下，源码按功能模块在 `src` 文件夹下垂直划分，具体代码框架如下：
```text
examples/
├── CMakeLists.txt     # 编译配置文件
├── package.xml        # 包级依赖声明文件
├── launch/            # [可选] 启动配置及参数化启动脚本目录
└── src/               # 源码存放核心目录，按底层功能模块进一步划分：
    ├── hal/           # 硬件抽象层相关示例（传感器数据获取与解析等）
    │   ├── echo_camera_head_rear.cpp
    │   ├── echo_camera_rgbd.cpp
    │   ├── echo_camera_stereo.cpp
    │   ├── echo_imu_data.cpp
    │   ├── echo_lidar_data.cpp
    │   ├── hand_control.cpp
    │   ├── motocontrol.cpp
    │   ├── omnihand_control.cpp
    │   └── take_photo.cpp
    ├── interaction/   # 交互系统相关示例（多模态互动输出与接收）
    │   ├── mic_receiver.cpp
    │   ├── play_emoji.cpp
    │   ├── play_lights.cpp
    │   ├── play_media.cpp
    │   ├── play_tts.cpp
    │   └── play_video.cpp
    └── mc/            # 运动控制相关示例（运动状态与位姿控制等）
        ├── get_current_input_source.cpp
        ├── get_mc_action.cpp
        ├── keyboard.cpp
        ├── mc_locomotion_velocity.cpp
        ├── preset_motion_client.cpp
        ├── set_mc_action.cpp
        └── set_mc_input_source.cpp
```

### 2.2 Python 示例包 (py_examples / ament_python)
Python 示例位于 `src/py_examples/` 目录下，采用标准的 Python 安装包模块化组织：
```text
py_examples/
├── package.xml        # 包级依赖声明文件
├── setup.py           # Python 安装规范化执行脚本
├── setup.cfg          # ROS2 ament_python 必备的基础配置
├── py_examples/       # 业务逻辑代码框架目录，存放所有的独立 Python 节点脚本
│   ├── echo_camera_head_rear.py
│   ├── echo_camera_rgbd.py
│   ├── echo_camera_stereo.py
│   ├── echo_imu_data.py
│   ├── echo_lidar_data.py
│   ├── get_current_input_source.py
│   ├── get_mc_action.py
│   ├── hand_control.py
│   ├── keyboard.py
│   ├── mc_locomotion_velocity.py
│   ├── mic_receiver.py
│   ├── motocontrol.py
│   ├── omnihand_control.py
│   ├── play_emoji.py
│   ├── play_lights.py
│   ├── play_media.py
│   ├── play_tts.py
│   ├── play_video.py
│   ├── preset_motion_client.py
│   ├── set_mc_action.py
│   ├── set_mc_input_source.py
│   └── take_photo.py
├── resource/          # 供 ament 资源索引平台使用的包名标识文件目录
└── test/              # 测试及校验脚本存放目录
```

---

## 3. C++ 代码示例规范

### 3.1 文件结构与封装
- **面向对象封装**：推荐将功能模块封装为继承自 `rclcpp::Node` 的类（如 `class LidarChestEcho : public rclcpp::Node`），在类的构造函数中进行参数声明及订阅者/发布者/客户端的初始化。
- **极简单功能除外**：若该示例仅作为一个最简的请求指令工具（如通过命令行传参调用一次 TTS），允许在 `main` 函数中直接实例化 `rclcpp::Node::make_shared("node_name")` 而不定义类，以保持代码极简。

### 3.2 健壮性与生命周期
- **Signal 处理**：在 `main` 函数中通过 `signal(SIGINT, signal_handler)` 监听中断，安全重置全局或局部 Node 指针。
- **服务调用的防死锁**：若在回调函数（如订阅者收到数据的 Callback）中调用 Service，必须使用异步方式并结合 Future 的回调处理；如果在主线程/独立的事件循环中，可采用 `async_send_request` 配合 `rclcpp::spin_until_future_complete` ，并设置超时时间（如 250ms），以防一直死等。

### 3.3 QoS 质量配置
- 针对传感器高频数据（Video, IMU, Lidar），配置 `rclcpp::SensorDataQoS()`，其底层为 `BEST_EFFORT` 投递策略。
- 针对重要控制项和服务，使用默认的可靠连接。

### 3.4 C++ 节点执行代码流程框架

以下是 C++ 示例节点标准的生命周期与执行流：

![C++ 节点执行代码流程框架](cpp_node_flow.png)

### 3.5 C++ 示例代码模板
```cpp
#include <rclcpp/rclcpp.hpp>
#include <memory>
#include <signal.h>

// 1. 全局指针用于信号拦截
std::shared_ptr<rclcpp::Node> g_node = nullptr;

void signal_handler(int signal) {
  if (g_node) {
    RCLCPP_INFO(g_node->get_logger(), "Received signal %d, shutting down...", signal);
    g_node.reset();
  }
  rclcpp::shutdown();
  exit(signal);
}

// 2. 核心逻辑类定义
class MyExampleNode : public rclcpp::Node {
public:
  MyExampleNode() : Node("my_example_node") {
    // 参数声明与获取
    topic_name_ = this->declare_parameter<std::string>("topic_name", "/default/topic");
    
    // 初始化订阅者/发布者/客户端...
    RCLCPP_INFO(this->get_logger(), "✅ Node initialized matching topic: %s", topic_name_.c_str());
  }

  // 可通过外部接口触发任务
  void do_work() {
      // 执行业务逻辑...
  }

private:
  std::string topic_name_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  signal(SIGINT, signal_handler);
  signal(SIGTERM, signal_handler);

  try {
    g_node = std::make_shared<MyExampleNode>();
    auto example = std::dynamic_pointer_cast<MyExampleNode>(g_node);
    
    // 如果是服务/单发指令，可在此调用 example->do_work() 后退出
    // 如果是持续订阅/运行，调用 spin
    rclcpp::spin(g_node);
  } catch (const std::exception &e) {
    RCLCPP_ERROR(rclcpp::get_logger("main"), "Exception: %s", e.what());
  }

  g_node.reset();
  rclcpp::shutdown();
  return 0;
}
```

---

## 4. Python 代码示例规范

### 4.1 文件头部要求
- **Shebang 及文档字符串**：首行必须包含 `#!/usr/bin/env python3`。紧跟该行必须是一段详细的 `""" Docstring """`，说明当前脚本订阅/发布了哪些 Topic，需要的参数（`--ros-args -p ...`），以及一段可以直接复制粘贴运行的示例命令。

### 4.2 节点构建与运行生命周期
- **面向对象派生**：需定一个继承自 `Node` 的类（`class MyExampleNode(Node):`），并调用 `super().__init__('node_name')`。
- **资源清理（Destroy）**：Python 异常抛出情况可能难以捕获完整，要求在 `main` 函数的最外围包裹 `try...except KeyboardInterrupt...finally` 块，确保在 `finally` 中执行 `node.destroy_node()` 并且检测 `if rclpy.ok(): rclpy.shutdown()`。

### 4.3 QoS 配置与通信
- **QoS 规则**：涉及雷达、IMU、相机的必须手动声明 `QoSProfile`，设置 `reliability=QoSReliabilityPolicy.BEST_EFFORT`。
- **客户端防卡死机制**：使用 `client.call_async(req)`，并结合 `rclpy.spin_until_future_complete(self, future, timeout_sec=0.25)`，并在外部增加 `for i in range(retry_times):` 循环来防范远端节点初始化或网络阻塞导致的单次掉线问题。

### 4.4 Python 节点执行代码流程框架

以下是 Python 示例节点标准的生命周期与捕获异常的执行流：

![Python 节点执行代码流程框架](py_node_flow.png)

### 4.5 Python 示例代码模板
```python
#!/usr/bin/env python3
"""
示例节点简述说明

使用示例:
  ros2 run py_examples example_node --ros-args -p param_name:=value
"""

import rclpy
import rclpy.logging
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

class MyExampleNode(Node):
    def __init__(self):
        super().__init__('my_example_node')
        
        self.declare_parameter('param_name', 'default_value')
        self.param_value = self.get_parameter('param_name').value
        
        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=5
        )
        # 初始化 Sub/Pub/Client
        self.get_logger().info('✅ Node Initialized.')

def main(args=None):
    rclpy.init(args=args)
    node = None

    try:
        node = MyExampleNode()
        # 持续运行监听
        rclpy.spin(node)
        # 或者如果是只运行一次的服务
        # node.send_request()
    except KeyboardInterrupt:
        pass
    except Exception as e:
        rclpy.logging.get_logger('main').error(f'Error: {e}')
    finally:
        if node:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
```

---