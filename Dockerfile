# ============================================================================
# PrimeBot SDK (T1) Dockerfile
# 构建 T1 机器人二次开发 SDK 的完整运行环境
# 用户可直接在容器内运行 C++ / Python 示例程序
# ============================================================================
#
# 构建镜像:
#   docker build -t primebot-sdk-t1 .
#
# 启动容器 (交互式开发):
#   docker run -it primebot-sdk-t1
#
# 启动容器 (与真机通讯，需同一局域网):
#   docker run -it --net=host primebot-sdk-t1
#
# 容器内运行示例:
#   ros2 run aimdk_examples_cpp set_mc_action --ros-args -p type:=action
#   python3 /ros2_ws/src/primebot_sdk/examples/python/set_mc_action.py --type action

# ============================================================================

# (交叉编译 ARM64 镜像)

# # 1. 启用 QEMU 模拟支持
# docker run --privileged --rm tonistiigi/binfmt --install all

# # 2. 创建 buildx 构建器
# docker buildx create --name arm64-builder --use
# docker buildx inspect arm64-builder --bootstrap

# # 3. 构建 ARM64 镜像并直接导出
# cd /home/agiuser/下载/primebot_sdk_origin/sdk_t1
# docker build --platform linux/arm64 -t primebot-sdk-t1 .

# # 4. 导出传输
# docker save primebot-sdk-t1 | gzip > primebot-sdk-t1-arm64.tar.gz
# scp primebot-sdk-t1-arm64.tar.gz run@<开发板IP>:~

# # 5. 开发板加载运行
# gunzip -c primebot-sdk-t1-arm64.tar.gz | sudo docker load
# sudo docker run -it --net=host primebot-sdk-t1

# ============================================================================

FROM ros:humble-ros-base

ENV DEBIAN_FRONTEND=noninteractive

# ── 第1阶段：安装系统依赖 ──
RUN apt-get update && apt-get install -y \
    python3-pip \
    libopencv-dev \
    libyaml-cpp-dev \
    libeigen3-dev \
    git \
    cmake \
    build-essential \
    && pip3 install nanobind \
    && rm -rf /var/lib/apt/lists/*

# ── 第2阶段：拷贝 SDK 源码 ──
# 注意：ruckig_for_primebot 已包含在 SDK 中，无需额外下载
WORKDIR /ros2_ws/src

COPY aimdk_msgs/        ./primebot_sdk/aimdk_msgs/
COPY examples/          ./primebot_sdk/examples/
COPY README.md          ./primebot_sdk/README.md
COPY 接口说明.md         ./primebot_sdk/接口说明.md
COPY PrimeBot_SDK快速入门与集成指南.md  ./primebot_sdk/PrimeBot_SDK快速入门与集成指南.md
COPY 友好用户使用声明.md   ./primebot_sdk/友好用户使用声明.md

# ── 第3阶段：编译 SDK ──
WORKDIR /ros2_ws

SHELL ["/bin/bash", "-c"]

RUN source /opt/ros/humble/setup.bash \
    && colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release \
    && rm -rf build/ log/

# ── 第4阶段：配置运行环境 ──
# 创建 entrypoint 脚本，自动 source 环境
RUN echo -e '#!/bin/bash\n\
set -e\n\
source /opt/ros/humble/setup.bash\n\
source /ros2_ws/install/setup.bash\n\
exec "$@"' > /ros2_ws/entrypoint.sh \
    && chmod +x /ros2_ws/entrypoint.sh

WORKDIR /ros2_ws

ENTRYPOINT ["/ros2_ws/entrypoint.sh"]
CMD ["/bin/bash"]