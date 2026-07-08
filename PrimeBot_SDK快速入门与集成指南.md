# PrimeBot SDK 快速入门与集成指南
本指南旨在带领开发者完成启元机器人（PrimeBot）的开发环境配置、连接验证，以及在不同业务场景下的系统集成开发（支持 Python 与 C++）。

## 目录
- [1. 系统架构总览](#1-系统架构总览)
- [2. 快速开始](#2-快速开始)
    - [2.1 环境依赖](#21-环境依赖)
        - [2.1.1 网络环境](#211-网络环境)
        - [2.1.2 系统环境](#212-系统环境)
        - [2.1.3 通讯环境](#213-通讯环境)
        - [2.1.4 用户可操作目录](#214-用户可操作目录)
        - [2.1.5 三方库与编译依赖](#215-三方库与编译依赖)
    - [2.2 安装与编译](#22-安装与编译)
        - [2.2.1 编译操作](#221-编译操作)
        - [2.2.2 命令行交互验证](#222-命令行交互验证)
    - [2.3 运行示例](#23-运行示例)
- [3. SDK 开发集成指南（Python & C++）](#3-sdk-开发集成指南python--c)
    - [3.1 Python 开发集成](#31-python-开发集成)
        - [3.1.1 方式 A：在 SDK 内部开发](#311-方式-a在-sdk-内部开发)
        - [3.1.2 方式 B：作为第三方依赖集成](#312-方式-b作为第三方依赖集成)
    - [3.2 C++ 开发集成](#32-c-开发集成)
        - [3.2.1 方式 A：在 SDK 内部新增节点](#321-方式-a在-sdk-内部新增节点)
        - [3.2.2 方式 B：作为独立第三方库集成](#322-方式-b作为独立第三方库集成)
- [4. 开发者模式说明](#4-开发者模式说明)
- [5. 常见问题](#5-常见问题)
    - [5.1 节点发现异常排查](#51-节点发现异常排查)
    - [5.2 数据收发异常排查](#52-数据收发异常排查)
    - [5.3 编译异常排查](#53-编译异常排查)
    - [5.4 机器人域配置](#54-机器人域配置)
    - [5.5 colcon 安装异常排查](#55-colcon-安装异常排查)
    - [5.6 登录与设置相关](#56-登录与设置相关)

---

## 1. 系统架构总览
```
+-----------------------+         +-----------------------+         +-----------------------+
|    开发 PC / 外部设备   |         |         运控板          |         |         大脑板         |
|    (Ubuntu 22.04)     |         |    IP: 10.1.1.100     |         |     IP: 10.1.1.101     |
|                       |  局域网  | [HAL/MC/交互 基础服务]  |  局域网  | [核心算法/供应商程序]    |
|     SDK 用户程序        | ◀═════▶|     SDK 用户程序        | ◀═════▶ |     SDK 用户程序       |
|     ROS2 节点          | FastDDS |     ROS2 节点          | FastDDS |     ROS2 节点         |
+-----------------------+         +-----------------------+         +-----------------------+
           ╚══════════════════════════ 局域网 FastDDS ══════════════════════════╝
```

- **开发 PC / 外部设备**：外部开发者的主要开发与运行环境。通常部署 Ubuntu 22.04 与 ROS2 Humble，通过局域网接入机器人。
- **运控板**：机器人的运动控制大脑。负责底层硬件抽象（HAL）、运控算法（MC）及各类基础交互服务（表情、语音等）。
- **大脑板**：机器人的高阶感知与导航大脑。负责运行核心 AI 算法、感知、规划及部分复杂的供应商程序。
- **局域网 DDS**：所有硬件节点均通过局域网物理连接，并使用 DDS 协议作为底层的 ROS2 通信总线，实现跨设备的高性能数据交换。

---

## 2. 快速开始
> **说明**：以下流程以**开发 PC** 为例。在 **运控板** / **大脑板** 上直接开发时，流程相同。
### 2.1 环境依赖
请确保开发环境满足以下基础要求，包括：**物理网络拓扑**（开发 PC 与机器人网线直连或处于同一局域网子网）的连通、**操作系统与中间件**（Ubuntu 22.04.x + ROS2 Humble）的正确安装，以及**跨设备通讯协议**（基于 FastDDS 的 ROS2 通信）的顺畅，以保证您的程序能够正常发现并控制机器人节点。

#### 2.1.1 网络环境
**有线直连（推荐默认开发方式）：**

将开发 PC 与机器人通过网线直连，并将两端网络接口配置到同一 IP 子网内，确保在同一网段。机器人出厂默认 IP 为大脑板 `10.1.1.101`、运控板 `10.1.1.100`，如与实际不符请以实际 IP 为准。

**配置开发 PC 静态 IP：**

为了能够直接与机器人通信，建议将开发 PC 的有线网卡 IP 配置为 `10.1.1.99`。在开发 PC 上可进行如下操作配置IP：
1. 打开 **Settings**（设置） -> **Network**（网络）。
2. 找到对应的有线网卡（Wired），点击齿轮图标 ⚙️ 进入设置。
3. 切换到 **IPv4** 标签页。
4. 将 IPv4 Method 改为 **Manual**（手动）。
5. 在 Addresses 区域填入：
   - Address（地址）: `10.1.1.99`
   - Netmask（子网掩码）: `255.255.255.0`
   - Gateway（网关）: 留空
6. 点击 **Apply** 保存配置，并重新开关一次网络开关以生效。
配置完成后，可以在终端测试是否能与机器人通信：
```bash
ping 10.1.1.101   # 测试连接大脑板
ping 10.1.1.100  # 测试连接运控板
```
- **正常连接输出**（按下 `Ctrl+C` 停止测试）：
  ```text
  64 bytes from 10.1.1.100: icmp_seq=1 ttl=64 time=0.428 ms
  64 bytes from 10.1.1.100: icmp_seq=2 ttl=64 time=0.370 ms
  ```
- **异常连接输出**：
  ```text
  From 10.1.1.99 icmp_seq=1 Destination Host Unreachable
  ```
  *(若出现异常，请检查网线是否插紧、配置是否生效，或尝试重启电脑网络)*

**无线 AP 接入（仅限临时调试，不推荐作为默认开发方式）：**

除有线直连外，系统亦支持通过机器人自带的无线热点（AP）进行临时接入。步骤如下：
1. 开启开发 PC 或外部设备的 WLAN 功能。
2. 搜索并连接 SSID 为 `T1-【sn后10位】` 的无线网络。
   * **密码获取**：AP 密码在机器人首次启动时自动随机生成。您需要先登录机器人，并在配置文件 `/robot/data/var/hostapd/hostapd.conf` 中查看 `wpa_passphrase` 的具体取值。

> **注意**：
> - **网络延迟与丢包风险**：由于无线网络易受空间电磁环境干扰，在传输高频通信数据（如底层运动控制、传感器裸流等）时极易产生较高的延迟或丢包。**为了保证控制精度与系统稳定性，不推荐将其作为默认开发方式，正式开发请务必使用有线连接**。
> - **避免指令并发冲突**：为防止控制指令冲突导致机器人行为异常，应避免同时通过有线和无线两种连接方式向机器人下发控制指令。

---

#### 2.1.2 系统环境
推荐在 Ubuntu 22.04 + ROS2 Humble 环境下进行开发，暂不支持在 Mac、Windows 系统下进行开发。除开发 PC 外，机器人自带的运控板与大脑板均支持直接进行二次开发。

**Ubuntu 22.04**

通过如下方式确认操作系统版本：
```bash
lsb_release -d
```
- **正常输出**：
  `Description: Ubuntu 22.04.x LTS`

若版本不匹配，请参考官方 [下载页面](https://releases.ubuntu.com/22.04/) 安装 Ubuntu 22.04。

**ROS2 Humble**

通过如下方式确认ROS2版本：
```bash
echo $ROS_DISTRO
```
- **正常输出**：
  `humble`

若版本不匹配或未安装，请参考官方 [安装指南](https://docs.ros.org/en/humble/Installation.html) 或参考[鱼香ROS安装指南](https://fishros.github.io/install/)完成安装。

建议将 ROS2 环境加载写入 `~/.bashrc` 以永久生效：
```bash
if ! grep -q "source /opt/ros/humble/setup.bash" ~/.bashrc; then
  echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
fi
source ~/.bashrc
```
> 若未写入 `~/.bashrc`，则每次打开新终端需手动执行 `source /opt/ros/humble/setup.bash`。

**构建工具 (colcon)**

ROS2 使用 `colcon` 作为统一的构建工具。请确保已安装 `python3-colcon-common-extensions`：
```bash
sudo apt update && sudo apt install python3-colcon-common-extensions
```
验证安装：
```bash
# 直接查看核心包信息确认版本
pip3 show colcon-core
```
- **预期结果**：能够显示出版本号信息（如 `0.20.x`）即表示 `colcon` 已成功安装且已被配置到系统环境变量中。
- **异常排查**：若安装失败或无法识别命令，请参考 [5.5 colcon 安装异常排查](#55-colcon-安装异常排查)。

---

#### 2.1.3 通讯环境
在“网络环境（已连通网线）”与“系统环境（已安装 ROS2）”均就绪后，开发 PC 不会默认接入机器人的 ROS2 分布式通信网络（DDS），需要手动切换至“开发者模式”并重启机器人才可正常进行 ROS2 通讯。

**1. 切换开发者模式**

机器人默认工作在“标准生产模式”，该模式下外部设备无法直接发现机器人节点。在进行二次开发前，您必须先手动切换至“开发者模式”：

1.  **SSH 登录机器人**：执行 `ssh run@<IP>` 登录机器人控制板（<IP>为机器人控制板的IP地址，默认为10.1.1.100或您自定义的 IP）
2.  **进入开发者模式**：在机器人终端执行相应切换命令 `aima mode edit`，等待片刻，进入交互式菜单，使用上下箭头选择 `develop - 开发模式`，按回车键确认。进入下一级菜单，使用上下箭头选择 `basic - 基础开发`，按回车键确认。
3.  **生效**：若界面显示 `INFO      模式选择成功，请重启机器人后生效`，则表示切换成功，对机器人重新上下电使能配置。
4.  **退出开发者模式**：执行 `aima mode edit`，等待片刻，进入交互式菜单，使用上下箭头选择 `standard - 标准模式`，按回车键确认，界面显示 `INFO      模式选择成功，请重启机器人后生效`后，对机器人重新上下电使能配置。

> **注意：**
> 1. 若选择完成后界面无响应或显示`ERROR      模式选择失败`，请进行再次尝试或重启机器人后再次尝试。若仍未解决，请联系售后技术支持。
> 2. 其他可选择的开发模式及模式说明，请参见：[4. 开发者模式说明](#4-开发者模式说明)。

**2. 检查节点列表**

在进入开发者模式后，您可以**在不编译 SDK 的情况下**，通过原生的 ROS 2 命令验证是否能正常发现机器人的通讯通道：

```bash
ros2 node list
```
- **正常**：列出机器人端的 ROS2 节点名称（如 `/hal_camera`、`/hal_audio`、`/mc_ros2_node`等）
- **异常**：输出为空或长时间卡住

**3. 检查 Topic 列表**

```bash
ros2 topic list
```
- **正常**：列出若干 `/aima/hal/..`、`/aima/mc/..` 等开头的 topic
- **异常**：输出为空或仅有 `/rosout`

**4. 检查 Service 列表**

```bash
ros2 service list
```
- **正常**：列出 `/aimdk_5Fmsgs/srv/...`、`/hal_audio/...` 等服务
- **异常**：输出为空

> **注意**：若上述任何命令输出**异常**，请参考 [常见问题：节点发现异常排查](#51-节点发现异常排查)。
>
> **提示**：虽然此时您可以查看到所有的通道，但因为没有编译 SDK，您将无法使用 `ros2 topic echo` 实际查看需要自定义消息类型（如 `aimdk_msgs`）的数据内容。

---

#### 2.1.4 用户可操作目录

为方便开发者在机器人上进行二次开发、资源部署及数据管理，系统定义了标准的存储方案。开发者应遵循以下目录规范进行资源存放：

| 数据类型 | 说明 | 存储方案与建议路径 |
| :--- | :--- | :--- |
| **预制资源文件** | 系统级音视频、模型权重等静态文件 | `/robot/data/system/*` (如：`/robot/data/system/audio/xxx.mp3`) |
| **预制配置文件** | 系统公共标准配置、模块自身特定的项目/SKU配置文件 | `/robot/protected/*` (公共配置)<br>`/robot/software/*` (模块私有配置) |
| **运行时用户数据** | 下载文件、临时缓存、不随版本升级变更的持久化数据 | `/robot/data/var/*` (如：`/robot/data/var/agent/face_id`) |
| **日志与数据包** | 各模块运行日志、系统日志、ROS2 Bag 数据包 | `/robot/persist/log/*` (日志)<br>`/robot/persist/bag/*` (数据包) |
| **系统级应用安装** | 第三方软件包、开发者自有服务及可执行文件 | `/opt` (推荐用于部署长期运行的第三方组件) |

**使用建议：**
- **资源限制**：运控板资源极度有限，主要保障实时运动控制，**不建议**在此部署应用或存储大数据。
- **OTA 备份提示**：系统版本（OTA）升级，上述部分目录（尤其是非持久化分区）可能存在数据丢失风险。建议开发者定期备份核心数据，并在 OTA 升级后重新部署相关资源。
- **分级策略**：`/home/run`目录：作为运行时工作目录，适合存放程序运行产生的日志、缓存、配置输出等临时数据。建议用于开发调试阶段的数据持久化；`/opt`目录：具有写权限，可在此目录下创建子目录、安装应用程序。适合部署需要长期运行的第三方组件或服务。
- **持久化原则**：对于需要跨版本保留的用户特征数据（如人脸特征、用户偏好），请务必存放在 `/robot/data/var/` 目录下。
- **日志管理**：建议开发者将自定义模块的日志输出至 `/robot/persist/log/` 的子目录下，以便系统统一收集和排查。
- **隔离性**：建议在上述目录下创建以项目名或公司名命名的子目录，以避免与系统原生资源产生命名冲突。

> **注意**：请勿在系统其他目录（如 `/usr`、`/etc` 等）进行随意写入操作，以免影响系统稳定性或导致服务异常。

---

#### 2.1.5 三方库与编译依赖

为满足不同语言开发者的需求，环境依赖分为以下三个部分。由于 SDK 包含自定义消息编译及运动算法绑定，**建议完整安装**。

**1. 核心依赖说明**

| 依赖分类 | 库/工具名称 | 使用场景 | 安装类型 |
| :--- | :--- | :--- | :--- |
| **公共 (必选)** | **ROSIDL** | 生成并编译自定义 `aimdk_msgs` (支持 C++/Python) | APT |
| | **Colcon** | ROS 2 包的统一构建入口 (`colcon build`) | APT |
| | **Python3-Dev** | 提供 C 扩展编译所需的 Python 开发头文件 | APT |
| **C++ 专用** | **OpenCV** | 支撑 `/aima/hal/video/stream` 流接收及图像处理开发 | **SDK 内置 (预编译 4.13.0，位于 `examples/opencv/`，无需自行安装)** |
| | **FFmpeg 运行库** | 内置 OpenCV 的 RTSP/视频解码后端依赖 `libavcodec` 等系统库，需在运行环境中提供 | APT |
| | **YAML-CPP** | 用于解析机器人本地或自定义的 YAML 配置文件 (可选) | APT |
| **Python 专用** | **NumPy** | 支撑 Python 图像处理及音频流的高效矩阵运算 | Pip |
| | **OpenCV-Python** | 支撑 Python 视频流读取脚本 (`get_video_stream.py`) | Pip |
| | **Ruckig (Python 模块)** | 支撑 Python 轨迹规划脚本 (`joint_control.py`) | **SDK 内置 (预编译 `.so`，位于 `examples/python/`，无需自行编译)** |

**2. 安装脚本**

请根据您的开发需求，按顺序执行以下安装步骤：

**A. 基础公共依赖 (必选)**
无论使用何种语言，若要编译 SDK 消息协议及使用 `colcon` 构建工具，必须执行：
```bash
sudo apt update && sudo apt install -y \
    python3-colcon-common-extensions \
    python3-dev \
    ros-humble-rosidl-default-generators \
    ros-humble-rosidl-default-runtime
```

**B. C++ 专用开发环境 (推荐 C++ 开发者安装)**
SDK 已内置预编译的 OpenCV 4.13.0（位于 `examples/opencv/`），**无需再安装 `libopencv-dev`**。但内置 OpenCV 的视频后端依赖系统的 FFmpeg 运行库（`libavcodec` 等），因此仍需安装 `ffmpeg`：
```bash
sudo apt install -y \
    ffmpeg \
    libyaml-cpp-dev
```
> **说明**：内置 OpenCV 版本（4.13.0）在 RTSP 流接收上比 Ubuntu 系统源自带的 4.5.x 更稳定，故 SDK 直接随包分发，`examples/cpp/CMakeLists.txt` 已通过 `set(OpenCV_DIR ...)` 指向该内置副本。

**C. Python 专用开发环境 (针对 Python 示例运行)**
如果您需要运行 Python 示例脚本（如视频、音频、轨迹规划处理）：
```bash
pip3 install numpy opencv-python
```
> **说明**：Ruckig 的 Python 模块（`ruckig_for_primebot.cpython-*.so`）已随 SDK 预编译并置于 `examples/python/`，无需再安装 `nanobind`、`scikit-build-core` 自行编译。

---

### 2.2 安装与编译
#### 2.2.1 编译操作
在确认系统和通讯环境就绪后，我们需要将 SDK 源代码编译为 ROS2 运行环境可识别的包和自定义消息格式。以下是完整的解压、编译与环境加载流程：

**1. 解压 SDK**

建议将下载好的 `primebot_sdk.tar.gz` 压缩包放入您指定的开发目录中，然后在该目录下执行解压：
```bash
tar -xzf primebot_sdk.tar.gz
cd primebot_sdk
```

**2. 编译 SDK**

```bash
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
```

> **提示**：执行 `colcon build` 后控制台无严重报错，且末尾输出 `Summary: X packages finished`。
> 若编译出现失败（如提示 `failed` 或报缺少系统依赖），请参考 [5.3 编译异常排查](#53-编译异常排查)。

**3. 加载编译产物**

> *(每次打开新终端运行 SDK 相关节点前都需执行此步。**注意：请务必将下方命令的路径替换为您实际解压 SDK 的绝对路径！**)*
```bash
source /path/to/your/primebot_sdk/install/setup.bash
```

**4. 验证数据接收**

在完成 SDK 编译并加载环境变量后，可以测试是否能成功解析并接收到具体的业务数据：
```bash
# 检查电池管理系统 (BMS) 数据
timeout 5 ros2 topic echo /aima/hal/bms/state --once
```
- **正常**：输出对应的数据报文（如电压、电流、电量百分比等）。
- **异常**：5 秒无输出（超时退出）或报错 → 参考 [5.2 数据收发异常排查](#52-数据收发异常排查)

---

#### 2.2.2 命令行交互验证
以下为通过原生 ROS2 命令行直接调用机器人接口的示例，可用于快速验证 Topic 和 Service 是否正常工作。

**Topic 验证（触摸事件订阅验证）**

我们可以通过“终端 A 模拟发布”和“终端 B 订阅数据”的方式，验证触摸事件接口是否正常工作。

*终端 A（模拟发布数据）：*
```bash
# 1. 加载环境变量
source /opt/ros/humble/setup.bash
source /path/to/your/primebot_sdk/install/setup.bash
# 2. 模拟发布一个“单次点击”触摸事件 (event_type=1)，每秒发布一次（-r 1）
ros2 topic pub /aima/hal/touch/state aimdk_msgs/msg/TouchState \
  '{header: {}, event_type: 1}' -r 1
```

*终端 B（订阅数据）：*
```bash
# 1. 开一个新终端，需先重新加载环境变量
source /opt/ros/humble/setup.bash
source /path/to/your/primebot_sdk/install/setup.bash
# 2. 持续接收并打印接收到的触摸事件
ros2 topic echo /aima/hal/touch/state
```

*结果验证：*
- **正常**：终端 B 将会**每隔 1 秒持续不断地**刷新出以下数据段，按 `Ctrl+C` 可停止接收：
  ```text
  header:
    stamp:
      sec: 0
      nanosec: 0
    frame_id: ''
  event_type: 1
  ---
  ```
- **异常**：如果终端 B 持续卡住无任何输出，或提示 `Cannot determine type for...` 错误，请参考 [5.2 数据收发异常排查](#52-数据收发异常排查)。

**Service 验证（调用 TTS 语音播报）**

我们可以直接在终端调用机器人的 TTS（Text-to-Speech）语音播报服务，验证服务响应是否成功以及机器人能否正常发声。

*终端（调用 TTS 服务）：*
```bash
# 1. 开一个新终端，需先重新加载环境变量
source /opt/ros/humble/setup.bash
source /path/to/your/primebot_sdk/install/setup.bash
# 调用 TTS 服务，让机器人播报一句话
ros2 service call /aimdk_5Fmsgs/srv/PlayTts aimdk_msgs/srv/PlayTts \
  '{header: {}, tts_req: {text: "你好，我是启元机器人", priority_level: {value: 6}, domain: "sdk_test", is_interrupted: true}}'
```

*结果验证：*
- **正常**：终端打印如下信息（`success=True` 表示 TTS 请求已被接受，机器人将开口播报）：
  ```text
  response:
    aimdk_msgs.srv.PlayTts_Response(header=..., tts_resp=...)
  ```
- **异常**：如果命令卡在 `waiting for service to become available...` 阶段，或者直接提示 `Service not available`，请参考 [5.2 数据收发异常排查](#52-数据收发异常排查)。

---

### 2.3 运行示例
每次打开新终端，运行示例前需先加载环境：
```bash
source /opt/ros/humble/setup.bash
source /path/to/your/primebot_sdk/install/setup.bash
```
运行示例：
```bash
# Python 示例
python3 examples/python/demo.py
# C++ 示例
ros2 run aimdk_examples_cpp demo
```

> **注意**：若修改了 C++ 示例源码，请重新执行 [2.2.1 编译操作](#221-编译操作) 中的 **第 2 步：编译 SDK**，后根据本章节操作重新运行示例。

---

## 3. SDK 开发集成指南（Python & C++）
本章节将指导您基于 SDK 进行二次开发，包括两种典型场景：**直接在 SDK 内部新建模块**（推荐新手或小型工程），或**将 SDK 接入您已有的独立项目**（推荐复杂或已有项目）。根据您使用的编程语言不同，分为 **Python** 和 **C++** 两种开发流程。
> **前提条件**：在开始本章节之前，请确保已完成以下步骤：
> 1. 完成环境设置 [2.1.2 系统环境](#212-系统环境) 
> 2. 完成SDK编译 [2.2.1 编译操作](#221-编译操作) 
> **💡 建议**：在正式开发前，先通过 [2.3 运行示例](#23-运行示例) 验证通讯与示例正常运行，可帮助您在开发阶段更快定位问题根因。

---

### 3.1 Python 开发集成
Python 的集成最为简单，因为它是解释型语言，不需要配置复杂的编译环境，天然支持 ROS2 动态加载机制。

#### 3.1.1 方式 A：在 SDK 内部开发
如果您刚开始尝试，或者工程量不大，可以直接在 SDK 的 `examples/python` 目录下新建脚本：
1. **新建文件**：在 `primebot_sdk/examples/python` 目录下创建您的 Python 脚本，例如 `my_robot_app.py`。
2. **编写代码**：参考同目录下的 `demo.py`，导入所需的包以发送指令：
    ```python
    import rclpy
    from aimdk_msgs.msg import ...
    ```
3. **运行程序**：在该目录下直接运行您的脚本：
    ```bash
    python3 my_robot_app.py
    ```

---

#### 3.1.2 方式 B：作为第三方依赖集成
如果您的主工程有独立的包管理或环境（如 `venv` 或 `conda`），可以直接通过环境叠加（Overlaying）的方式引入 SDK，保持主工程结构纯粹。

工程目录结构示例：
```text
my_ai_backend/
├── deps/                      # 【依赖区】统一管理非纯 Python 的复杂依赖
│   ├── primebot_sdk/          # 放入 SDK 源码
│   └── other_sdk/             # 放入其他外部 SDK（如有）
└── app/
    ├── main.py                # 【业务区】您的原生 Python 业务入口
    └── api/...
```

**集成操作步骤**：
1. **统一存放并集中预编译**：
    将源码全部放入 `deps/` 目录，并执行一次合并编译：
    ```bash
    cd ./deps
    colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
    ```
2. **编写业务代码**：
    由于不受目录限制，您可以像引用普通三方包一样自由导入 SDK 消息体开始编写业务：
    ```python
    # main.py
    import rclpy
    from aimdk_msgs.srv import PlayTts
    # ... 编写高阶业务
    ```
3. **环境注入与启动**：
    创建启动包装脚本 `run.sh`，**并将以下内容保存到该脚本中**：
    ```bash
    #!/bin/bash
    # 挂载底座与依赖总库
    source /opt/ros/humble/setup.bash
    source ./deps/install/setup.bash
    
    # 激活自带的 Python 运行环境（如使用 venv/conda）
    source venv/bin/activate 
    
    # 最后，由本脚本一并拉起您的 Python 业务程序
    python3 app/main.py
    ```
    以后每次启动程序，只需运行该包装脚本即可：
    ```bash
    ./run.sh
    ```
    *(注：如果是新建的脚本，首次执行前需要使用 `chmod +x run.sh` 命令为其添加执行权限。)*

---

### 3.2 C++ 开发集成
C++ 集成涉及 CMake 找包与链接的过程。本 SDK 基于 ROS 2 的 `ament` / `colcon` 构建体系。工作空间根目录下包含两个可编译的 ament 包：aimdk_msgs（消息/服务定义）与 examples/cpp（C++ 示例节点包）。此外，`examples/opencv/`（预编译 OpenCV 4.13.0）与 `examples/ruckig_for_primebot/`（预编译关节轨迹规划库，仅含 `include/` 与 `lib/`）均为**随包分发的预编译依赖**，不参与 `colcon` 编译，由 `examples/cpp/CMakeLists.txt` 通过 `find_package` 直接引用。
#### 3.2.1 方式 A：在 SDK 内部新增节点
如果您希望利用现成的编译配置进行快速开发：
1. **新建源文件**：将您的 `.cpp` 源文件（例如 `my_robot_node.cpp`）放入 `primebot_sdk/examples/cpp/src/` 目录下。
2. **修改编译配置**：打开 `primebot_sdk/examples/cpp/CMakeLists.txt`，在文件顶部已有的 `set(EXAMPLE_TARGETS ...)` 列表中，参考 `get_bms_state` 等现有示例的写法，追加您的节点名称（即源文件去掉 `.cpp` 后缀的名字）：
    ```cmake
    set(EXAMPLE_TARGETS
      get_audio_stream
      get_bms_state
      # ... 其他已有示例 ...
      upper_body_control
      my_robot_node          # ← 在此处追加您的节点名称
    )
    ```
    > **说明**：`CMakeLists.txt` 内部使用 `foreach` 循环遍历 `EXAMPLE_TARGETS`，会自动在 `src/` 目录下查找与目标同名的 `.cpp` 文件（如 `src/my_robot_node.cpp`）并通过 `add_executable` 完成编译，同时统一绑定 `rclcpp` 与 `aimdk_msgs` 依赖，无需为每个节点单独编写 `add_executable`。若找不到同名源文件，编译会直接报 `Missing example source` 错误。
    >
    > 如果您的节点需要额外依赖，可在 `foreach` 循环体内参考现有示例追加条件块：OpenCV 参考 `get_video_stream`，关节轨迹规划库参考 `joint_control`：
    > ```cmake
    > # OpenCV（参考 get_video_stream）
    > if(EXAMPLE_TARGET STREQUAL "my_robot_node")
    >   target_include_directories(${EXAMPLE_TARGET} PRIVATE ${OpenCV_INCLUDE_DIRS})
    >   target_link_libraries(${EXAMPLE_TARGET} ${OpenCV_LIBS})
    > endif()
    > # 轨迹规划库 ruckig_for_primebot（参考 joint_control）
    > if(EXAMPLE_TARGET STREQUAL "my_robot_node")
    >   target_link_libraries(${EXAMPLE_TARGET} ruckig_for_primebot::ruckig_for_primebot)
    > endif()
    > ```
    > 若使用 OpenCV 或 ruckig，请确保文件顶部已有对应的 `find_package(OpenCV ...)` / `find_package(ruckig_for_primebot REQUIRED)`（SDK 默认已包含）。这两个库均由 SDK 以预编译形式内置（`examples/opencv/`、`examples/ruckig_for_primebot/`），`CMakeLists.txt` 顶部已通过 `set(OpenCV_DIR ...)` 与 `set(ruckig_for_primebot_DIR ...)` 指向内置副本，无需额外安装；编译产物会自动将对应 `.so` 拷贝到二进制同目录并写入 `$ORIGIN` RPATH，运行时无需设置 `LD_LIBRARY_PATH`。

3. **编译工程**：在工作空间根目录下执行编译：
    ```bash
    cd /path/to/your/primebot_sdk
    colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
    ```
4. **加载与运行**：加载环境并运行节点：
    ```bash
    source /opt/ros/humble/setup.bash
    source /path/to/your/primebot_sdk/install/setup.bash
    ros2 run aimdk_examples_cpp my_robot_node
    ```

#### 3.2.2 方式 B：作为独立第三方库集成
如果您希望保持主工程的独立性，建议将 SDK 的引入逻辑**封装在专用的 `cmake/` 目录脚本**中，利用 CMake 原生的 `FetchContent` 模块实现“一次配置、统一编译”。

> **注意**：SDK 工作空间中，`aimdk_msgs` 是需从源码编译的 ament 包（用 `FetchContent` 纳入）；而 `ruckig_for_primebot` 已改为**预编译库**（`examples/ruckig_for_primebot/` 下仅含 `include/` 与 `lib/`，不含源码），因此改用 `find_package` 直接引用其内置的 CMake 配置，无需再编译。OpenCV 同理（内置 `examples/opencv/`）。

工程目录结构示例：
```text
my_project/
├── cmake/
│   ├── GetPrimebotSDK.cmake   # SDK 的外部抓取与编译封装脚本
│   └── primebot_sdk/          # 可以将 SDK 源码放在这里，或通过网络拉取
├── src/
│   └── my_robot_node.cpp      # 您的业务与控制代码
└── CMakeLists.txt             # 您的工程主 CMakeLists
└── package.xml
```

**集成操作步骤**：
1. **编写封装脚本**：在 `cmake/` 目录下创建 `GetPrimebotSDK.cmake`。通过这种方式能高度屏蔽底层依赖引入的复杂性：
    ```cmake
    include(FetchContent)
    message(STATUS "Fetching primebot_sdk ...")

    # ── 1. 引入消息定义包 aimdk_msgs（需从源码编译，用 FetchContent）──────────
    FetchContent_Declare(
      aimdk_msgs
      SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/cmake/primebot_sdk/aimdk_msgs
    )
    FetchContent_MakeAvailable(aimdk_msgs)

    # ── 2. 引入预编译的轨迹规划库 ruckig_for_primebot（按需）─────────────────
    # 该库以预编译形式随 SDK 分发，直接用 find_package 指向其内置 CMake 配置，
    # 无需再从源码编译。如果您的业务不需要关节轨迹规划，可省略此段。
    set(ruckig_for_primebot_DIR
        ${CMAKE_CURRENT_SOURCE_DIR}/cmake/primebot_sdk/examples/ruckig_for_primebot/lib/cmake/ruckig_for_primebot)
    find_package(ruckig_for_primebot REQUIRED)

    # ── 3. 引入预编译的 OpenCV 4.13.0（按需，用于视频流处理）────────────────
    # set(OpenCV_DIR
    #     ${CMAKE_CURRENT_SOURCE_DIR}/cmake/primebot_sdk/examples/opencv/lib/cmake/opencv4)
    # find_package(OpenCV REQUIRED COMPONENTS core videoio)
    ```

2. **在主配置中调用脚本并链接**：在主项目的顶级 `CMakeLists.txt` 中引入上面写好的模块：
    ```cmake
    cmake_minimum_required(VERSION 3.16)
    project(my_robot_project)

    # 寻找必需的 ROS 2 底层通信库
    find_package(ament_cmake REQUIRED)
    find_package(rclcpp REQUIRED)

    # 1. 引入并执行第三方依赖的获取脚本
    include(cmake/GetPrimebotSDK.cmake)

    # 【关键】确保程序在运行时能自动找到 build 目录下的共享库（解决 .so 找不到的问题）
    set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)
    set(CMAKE_BUILD_WITH_INSTALL_RPATH FALSE)

    # 2. 声明您的业务节点文件
    add_executable(my_robot_node src/my_robot_node.cpp)

    # 3. 链接目标依赖项
    target_link_libraries(my_robot_node
        PRIVATE
        rclcpp::rclcpp
        # 注意：使用 FetchContent 集成时，需显式链接具体的类型支持库以确保运行路径正确
        aimdk_msgs__rosidl_typesupport_cpp
        aimdk_msgs__rosidl_typesupport_fastrtps_cpp
        # 若需关节轨迹规划，追加预编译 ruckig 库（配合上面的 find_package）：
        # ruckig_for_primebot::ruckig_for_primebot
        # 若需视频流处理，追加预编译 OpenCV：
        # ${OpenCV_LIBS}
    )

    ament_package()
    ```

3. **编写业务代码**：
    消息头文件按 ROS 2 标准规则生成（子目录结构会被展平到 `msg/` 或 `srv/` 命名空间下），可像引用普通三方包一样导入：
    ```cpp
    // src/my_robot_node.cpp
    #include "rclcpp/rclcpp.hpp"
    #include "aimdk_msgs/msg/mc_action.hpp"
    #include "aimdk_msgs/srv/set_mc_action.hpp"

    // 使用示例
    auto action_msg = aimdk_msgs::msg::McAction();

    auto set_action_client = node->create_client<aimdk_msgs::srv::SetMcAction>(
        "/aimdk_5Fmsgs/srv/SetMcAction");
    ```

4. **一次性整体编译工程**：配置完成后，直接在主工程目录下发起编译即可，系统会自动解析并打包编译 SDK 与您的代码：

    ```bash
    mkdir build && cd build
    cmake ..
    make
    ```

5. **加载并运行**：

    ```bash
    source /opt/ros/humble/setup.bash
    source install/setup.bash
    ./build/my_robot_node
    ```

---

## 4. 开发者模式说明

机器人系统采用了多维度、分层级的权限管理架构，以平衡系统的安全性与二次开发的灵活性。通过在机器人终端执行 `aima mode edit` 指令，开发者可以根据实际需求对系统的开放程度进行精细化编排及选择。

模式架构分为以下五个层级：

| 层级 | 维度名称 | 模式/选项 | 功能说明与适用场景 | 支持状态 |
| :--- | :--- | :--- | :--- | :--- |
| **第一层** | **运行模式** | `Standard` (标准模式) | **生产与交付环境**。系统处于全闭环状态，仅运行官方认证业务组件。强调极致稳定性，不接受外部控制指令。 | 已支持 |
| | | `Develop` (开发模式) | **开发与调试环境**。系统开启外部接入通道，允许开发者注入自定义控制逻辑与算法模型。 | 已支持 |
| **第二层** | **开发方式** | `API` | 基于标准 RESTful / gRPC 协议栈。提供跨语言、硬件解耦的交互能力，适用于 Web、移动端或低代码平台等轻量级应用。 | 暂未支持 |
| | | `ROS2` | 基于分布式 DDS 通信总线。支持高频、强时效的节点间通信，适合高性能算法移植。**(需重启系统以正式生效)**。 | 当前默认支持，无需显示设置 |
| **第三层** | **开发层级** | `Basic` (基础开发) | **无损能力增强**。原厂业务与安全策略保持完整，外部指令并发注入。适用于非侵入式功能扩展与逻辑开发。 | 已支持 |
| | | `Advanced` (高级开发) | **深度逻辑接管**。允许关闭特定官方模块，获取系统核心资源（如传感器裸流、底层运控）的独占权，支持深度算法替代。 | 已支持 |
| **第四层** | **领域设置** | 领域名称 | 开发者可选择性地接管机器人特定的功能领域：运动控制、语音交互、作业规划或传感器数据。 | 已支持，领域范畴支持拓展 |
| **第五层** | **能力配置** | 原子功能开关集 | 最终的功能开关编排。系统将根据勾选自动执行底层配置的动态编排。 | 已支持，功能支持拓展 |

> **版本说明与现状须知 (Current Version Status)**：
> 1. **开发方式**：当前版本暂未开放 **API 方式**，系统默认启用并全局支持 **ROS2** 通讯。
> 2. **层级配置**：当前版本将“领域设置”与“能力配置”暂予合并，目前仅生效 **「运动控制 - 低层运控开发」** 选项，后续将逐步上线支持交互、作业及传感器领域的原子功能。
> 3. **各领域原子能力对照表**：功能说明参见 [领域-原子功能对照表](#领域-原子功能对照表)。

#### 领域-原子功能对照表
| 领域分类 | 原子功能 | 选项指导 | 功能核心说明 |
| :--- | :--- | :--- | :--- |
| **运动控制** | 低层运控开发 | `domains.mc.low_level_dev - 低层运控开发` | 1. 授权开发者对机器人全身关节进行直接指令下发。<br>2. 机器人本体高层运动控制功能需要手动关闭。 |

---

## 5. 常见问题
### 5.1 节点发现异常排查

**Q: 运行 `ros2 node list` 无输出、或仅有 `/rosout`？**

如果在 2.1.3 章节验证通讯环境时发现异常、无法与机器人发现彼此，通常代表底层的 DDS 节点发现（Discovery）机制受阻。请按以下步骤依次排查：
1. **硬件与连通性检查**：确认机器人已开机，开发 PC 与机器人处于同一网段（例如 IP `10.1.1.99`）。使用 `ping 10.1.1.101` 和 `ping 10.1.1.100` 测试能正常收到回复，确认 ICMP 链路畅通。
2. **`ROS_DOMAIN_ID` 不一致**：确保您的 PC 端没有设置其他杂乱的 `ROS_DOMAIN_ID` 环境变量（机器人默认通常是 `0`），导致与机器人的隔离在不同的域内。
3. **多网卡冲突**：如果 PC 同时连接了多个网络（例如插着网线的同时连着 WiFi），DDS 初始化时可能绑定到了错误的网卡（如无线网卡）。此时 DDS 发现报文无法到达机器人局域网。**强烈建议在网线直连时，临时禁用其他无关网卡（如断开 WiFi 或关闭手机热点）**。

> **参考**：关于更深度的跨网段或多复杂的 DDS 发现配置，可参阅官方指南 [ROS 2 Installation Troubleshooting（含多播与多网卡冲突章节）](https://docs.ros.org/en/humble/How-To-Guides/Installation-Troubleshooting.html#enable-multicast)。

---

### 5.2 数据收发异常排查

**Q: 节点能看到，但 `ros2 topic echo` 超时无输出，或者 SDK 提示 `Service not available`？**

这种现象说明 DDS 发现（Discovery）成功建连，但在**数据包实际传输（UDP Traffic）**或**消息反序列化**阶段失败了。请排查以下几点：
1. **防火墙拦截 UDP 流量**：节点发现用的是特定的组播端口，而具体的数据收发使用的是随机大端口（往往几万起步）。部分系统的默认防火墙会拦截这些大端口的 UDP 数据传输报文。**请尝试关闭防火墙**：
   ```bash
   sudo ufw disable
   ```
2. **环境变量未加载（报错 Cannot determine type）**：在对特定自定义消息进行操作时，如果当前终端没有先执行 `source /path/to/your/primebot_sdk/install/setup.bash`（需替换为您实际路径），电脑环境里就不存在该消息协议，无法做二进制的反序列化导致报错。
3. **消息按事件触发（无源数据）**：部分 Topic（如触摸事件），只有在发生物理接触时才会发送数据产生流量。如果您此时监听该 Topic，可能只需实际触发一次（如摸一下机器人头部）即可触发数据产生。

---

### 5.3 编译异常排查

**Q: 执行 `colcon build` 编译报错（如提示编译失败或缺少依赖）？**

如果 SDK 在编译阶段提示 `Failed` 或某些包引发严重错误，通常是因为系统环境不满足编译要求：
1. **使用了中文路径（重要）**：**强烈建议不要将 SDK 目录放置在包含中文字符的路径下。** 在 ROS2 的 CMake 构建工具链中，中文路径极易导致路径解析失败，编译器无法正确识别头文件包含路径。请始终确保项目路径为全英文且无空格。
2. **缺少 ROS2 构建工具或构建依赖**：请确保您已经完成了前面的 2.1 依赖安装步骤，特别是已安装了 `ros-humble-desktop` 及 `python3-colcon-common-extensions`。
3. **终端未初始化 ROS2 基础环境**：在输入 `colcon build` 前，当前终端必须已经能够识别 ROS2 命令。可通过运行 `source /opt/ros/humble/setup.bash` 来加载系统级的基础环境。
4. **C++ 编译器版本过低**：SDK 所用到的现代 C++ 特性需要 `g++` 支持。由于推荐系统为 Ubuntu 22.04，系统自带的默认编译器即满足要求，通常不会因此报错。

> **参考**：更多关于底层构建工具配置的细节，可参阅官方指南 [Colcon documentation](https://design.ros2.org/articles/build_tool.html)。

---

### 5.4 机器人域配置

**Q: 同一局域网有多台机器人，如何区分？**

- 为每台机器人配置不同的 `ROS_DOMAIN_ID`。

> **参考**：有关环境隔离机制的详细说明，请参阅官方指南 [The ROS_DOMAIN_ID](https://docs.ros.org/en/humble/Concepts/Intermediate/About-Domain-ID.html)。

---

### 5.5 colcon 安装异常排查

**Q: 执行 `sudo apt install` 失败或提示“无法定位软件包”？**

这种现象通常发生在软件源未更新或操作系统版本不匹配时：
1. **更新软件源**：确保执行了 `sudo apt update`。如果依然无法找到，请确认是否已将 ROS 2 的官方软件源添加到系统的 APT 列表中。
2. **官方安装指导**：参考 [colcon 官方安装方法总结](https://colcon.readthedocs.io/en/released/user/installation.html)。

**Q: 安装完成后输入 `colcon` 提示 `command not found`？**

这通常是环境变量没有生效的问题：
1. **Shell 刷新**：尝试关闭当前终端并重新打开，或者执行 `hash -r` 强制刷新 shell 的命令缓存。
2. **Pip 用户路径**：如果是通过 `pip3 install --user` 安装的，请检查 `~/.local/bin` 是否已加入 `PATH` 环境变量：
   ```bash
   # 测试路径是否存在
   ls ~/.local/bin/colcon
   # 临时加入环境变量（建议写入 ~/.bashrc）
   export PATH=$PATH:$HOME/.local/bin
   ```
3. **彻底重新安装**：建议使用系统包管理器（APT）安装以获得最佳兼容性：
   ```bash
   sudo apt remove python3-colcon-common-extensions
   sudo apt install python3-colcon-common-extensions
   ```

---

### 5.6 登录与设置相关

**Q: 如何通过 SSH 登录机器人底层板卡？**

如需通过 SSH 登录机器人底层板卡，相关的登录凭证（密码/密钥）获取，请您直接**联系我们的售后服务团队**。

**Q: 如果系统环境损坏，如何恢复出厂设置？**

若板卡在开发过程中发生系统级文件误删、网络服务彻底瘫痪、或由于错误装载第三方库导致原厂服务无法拉起的情况：
**为了防止您丢失核心授权或进一步导致硬件闭锁，我们不建议客户自行使用三方工具强刷固件。**
如果您的开发面临需要“恢复出厂设置”场景，请您**停止一切危险操作，并及时联系我们的售后服务团队**。

---

> **温馨提示（售后支持）**
> 如果您在集成 PrimeBot SDK 或部署调试的过程中，遇到了以上 FAQ 未涵盖的报错或难以自行排查的底层问题，请**及时联系我们的售后服务团队**。
