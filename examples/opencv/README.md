# SDK 内置 OpenCV 4.13.0

本目录以源代码形式交付 OpenCV 4.13.0，供 `get_video_stream` C++ RTSP 示例使用。
不提供预编译 `.so`：x86_64 和 ARM 用户均由 `colcon` 在自己的环境中构建，避免
CPU 架构、glibc、C++ ABI 与 FFmpeg 版本不兼容。

## 构建

安装构建与媒体后端依赖（Ubuntu/Debian）：

```bash
sudo apt update
sudo apt install -y cmake build-essential pkg-config \
  libavcodec-dev libavformat-dev libavutil-dev libswscale-dev \
  libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev
```

在 SDK 工作区根目录执行：

```bash
colcon build --packages-up-to aimdk_examples_cpp --symlink-install
```

`aimdk_opencv` 会先编译 `examples/opencv/opencv-4.13.0` 中的源码，并只构建
`core`、`imgproc`、`imgcodecs` 与 `videoio` 模块；随后 `aimdk_examples_cpp` 自动
链接该版本。构建产物位于工作区的 `build/`、`install/` 和 `log/`，不提交到 SDK。

## 运行

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run aimdk_examples_cpp get_video_stream --help
```

可执行文件带有相对运行时库路径：隔离安装时加载同工作区
`aimdk_opencv/lib`，合并安装时加载 `install/lib`，不会被系统同名 OpenCV 库替代。

OpenCV 上游许可证见 [opencv-4.13.0/LICENSE](opencv-4.13.0/LICENSE)。
