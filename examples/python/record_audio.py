#!/usr/bin/env python3

"""录音示例

支持本机（共享内存）和远程（TCP）两种录音模式。

本机模式：在机器人上直接运行，通过 PulseAudio unix socket（共享内存）传输音频，效率更高。
远程模式：在 PC 端运行，通过 PulseAudio TCP 协议从机器人麦克风阵列录音。

录音格式：8声道 48kHz S16LE（机器人麦克风原始格式）
声道说明：
  - ch1-2：回采
  - ch3-8：麦克

输出文件：
  - raw_8ch.wav：完整 8 声道原始录音
  - mic_mono.wav：ch5-6 混合的单声道录音（10 倍放大）

前提条件：
  - 远程模式：示例脚本会通过 SSH 自动在机器人端加载 TCP 录音模块（端口 4713）
  - 本机模式：无需额外配置，直接使用 PulseAudio 本地 socket
  - 本机已安装 parec：sudo apt install pulseaudio-utils

用法：
  python3 examples/python/record_audio.py <机器人IP> [录音秒数]

  机器人IP 为 127.0.0.1 或 localhost 时自动使用本机模式（共享内存），
  否则使用远程模式（TCP）。

示例：
  # 本机录音（共享内存）
  python3 examples/python/record_audio.py 127.0.0.1 5

  # 远程录音（TCP，网线连接时 IP 为 10.1.1.100）
  python3 examples/python/record_audio.py 10.1.1.100 5
"""

import subprocess
import threading
import wave
import sys
import os
import time

try:
    import numpy as np
    USE_NUMPY = True
except ImportError:
    import struct
    USE_NUMPY = False
    print("警告: 未安装 numpy，分析会较慢")

RATE = 48000
CHANNELS = 8
DEVICE = "alsa_input.platform-aw89403_sound.pro-input-0"

# 本机 PulseAudio unix socket 路径（共享内存传输）
LOCAL_PULSE_SOCKETS = [
    "/run/user/1000/pulse/native",
]


def is_local_robot(robot_ip):
    """判断是否在机器人本机运行。

    检测方式：
      1. IP 为 127.0.0.1 或 localhost
      2. 存在机器人特征目录 /robot/software/

    Returns:
        True 表示本机模式（使用共享内存），False 表示远程模式（使用 TCP）。
    """
    if robot_ip in ("127.0.0.1", "localhost", "::1"):
        return True
    if os.path.isdir("/robot/software"):
        return True
    return False


def get_local_pulse_server():
    """获取本机 PulseAudio unix socket 路径。

    Returns:
        unix socket 路径字符串，或 None（使用 PulseAudio 默认路径）。
    """
    for socket_path in LOCAL_PULSE_SOCKETS:
        if os.path.exists(socket_path):
            return "unix:%s" % socket_path
    return None


def setup_robot_audio_tcp(robot_ip):
    """Auto-load PulseAudio TCP recording module on robot via SSH.

    先检查模块是否已加载，已加载则跳过。
    SSH 输出直接显示在终端，以便用户看到密码提示并完成认证。
    """
    # 一条 SSH 命令：先检查，未加载才加载，避免重复输入密码
    cmd = (
        'ssh -o StrictHostKeyChecking=no -o ConnectTimeout=5 run@%s \''
        'if pactl list modules short 2>/dev/null | grep -q module-native-protocol-tcp; then '
        'echo MODULE_ALREADY_LOADED; '
        'else '
        'pactl load-module module-native-protocol-tcp auth-anonymous=1 listen=0.0.0.0 && '
        'echo MODULE_LOADED; '
        'fi\''
        % robot_ip
    )
    try:
        result = subprocess.run(cmd, shell=True, stdout=subprocess.PIPE, text=True)
        output = result.stdout.strip()
        if "MODULE_ALREADY_LOADED" in output:
            print("录音模块已在机器人端加载（跳过加载）")
        elif "MODULE_LOADED" in output:
            print("已在机器人端加载录音模块 (端口 4713)")
        else:
            stderr_msg = "(stderr 已输出到终端，未捕获)"
            print("警告: 录音模块加载可能未成功: %s" % stderr_msg)
    except KeyboardInterrupt:
        print("\n已取消")
        sys.exit(0)
    except Exception as e:
        print("SSH 连接失败: %s" % e)
        sys.exit(1)


def record_pcm(robot_ip, duration=5, local_mode=False):
    """Record multi-channel audio from robot via PulseAudio.

    Args:
        robot_ip: Robot IP address.
        duration: Recording duration in seconds.
        local_mode: True for local shared memory, False for remote TCP.

    Returns:
        Raw PCM bytes (8ch, 48kHz, S16LE).
    """
    cmd = [
        'parec',
        '--channels', str(CHANNELS),
        '--rate', str(RATE),
        '--format', 's16le',
        '--device', DEVICE,
    ]

    if local_mode:
        pulse_server = get_local_pulse_server()
        if pulse_server:
            env = dict(os.environ, PULSE_SERVER=pulse_server)
            print("本机模式（共享内存: %s），录音 %d 秒..." % (pulse_server, duration))
        else:
            env = dict(os.environ)
            print("本机模式（默认 socket），录音 %d 秒..." % duration)
    else:
        env = dict(os.environ, PULSE_SERVER="tcp:%s:4713" % robot_ip)
        print("远程模式（TCP: %s:4713），录音 %d 秒..." % (robot_ip, duration))
    proc = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, env=env)

    bytes_per_sec = RATE * CHANNELS * 2
    total_bytes = bytes_per_sec * duration
    audio = bytearray()
    start_time = time.time()
    stop_display = threading.Event()

    def progress_thread():
        """独立线程，每 0.1 秒刷新一次进度，显示平滑的真实录音时间。"""
        while not stop_display.is_set():
            elapsed = time.time() - start_time
            display_sec = min(elapsed, duration)
            received_sec = len(audio) / bytes_per_sec
            print("\r录音中 %.1f / %d 秒  (已接收 %.1f 秒)" % (display_sec, duration, received_sec), end="", flush=True)
            stop_display.wait(0.1)

    t = threading.Thread(target=progress_thread, daemon=True)
    t.start()

    # 主线程专注读数据，以接收字节数达到目标为结束条件
    while len(audio) < total_bytes and proc.poll() is None:
        chunk = proc.stdout.read1(65536)
        if chunk:
            audio.extend(chunk)
        else:
            time.sleep(0.01)

    # 停止进度显示线程
    stop_display.set()
    t.join()

    proc.terminate()
    proc.wait()

    # Drain remaining data after terminate
    while True:
        chunk = proc.stdout.read1(65536)
        if not chunk:
            break
        audio.extend(chunk)

    # 打印 parec stderr（如有错误信息）
    stderr_data = proc.stderr.read()
    if stderr_data:
        print("\nparec: %s" % stderr_data.decode(errors='replace').strip())

    print()
    return bytes(audio)


if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("用法: %s <机器人IP> [录音秒数]" % sys.argv[0])
        print()
        print("  机器人IP 为 127.0.0.1 或 localhost 时自动使用本机模式（共享内存）")
        print("  否则使用远程模式（TCP）")
        sys.exit(1)

    robot_ip = sys.argv[1]
    duration = int(sys.argv[2]) if len(sys.argv) > 2 else 5

    # 检测本机/远程模式
    local_mode = is_local_robot(robot_ip)

    if local_mode:
        print("检测到本机环境，使用共享内存传输（效率更高）")
    else:
        print("检测到远程环境，使用 TCP 传输")
        # 远程模式需要通过 SSH 加载 TCP 录音模块
        setup_robot_audio_tcp(robot_ip)

    data = record_pcm(robot_ip, duration=duration, local_mode=local_mode)

    if not data:
        print("错误: 未录到任何音频数据，请检查 PulseAudio 连接")
        sys.exit(1)

    # Save 8-channel WAV
    with wave.open("raw_8ch.wav", "wb") as w:
        w.setnchannels(CHANNELS)
        w.setsampwidth(2)
        w.setframerate(RATE)
        w.writeframes(data)

    print("录音 %d 字节 (%.1f 秒)" % (len(data), len(data) / (RATE * CHANNELS * 2)))
    print("分析中...")

    if USE_NUMPY:
        samples = np.frombuffer(data, dtype=np.int16).reshape(-1, CHANNELS)
        avg = np.abs(samples.astype(np.int32)).mean(axis=0)
        peak = np.abs(samples).max(axis=0)

        # Extract channels 5-6 (index 4,5), mix and amplify 10x
        mono = samples[:, [4, 5]].mean(axis=1).astype(np.float32) * 10
        mono = np.clip(mono, -32768, 32767).astype(np.int16)
    else:
        stats = [0] * CHANNELS
        peak = [0] * CHANNELS
        mono = bytearray()
        n = 0
        frame_size = CHANNELS * 2  # bytes per frame
        for i in range(0, len(data) - frame_size + 1, frame_size):
            s = struct.unpack("<%dh" % CHANNELS, data[i:i + frame_size])
            for c in range(CHANNELS):
                stats[c] += abs(s[c])
                if abs(s[c]) > peak[c]:
                    peak[c] = abs(s[c])
            mixed = (s[4] + s[5]) // 2 * 10  # Mix ch5-6, amplify 10x
            mixed = max(-32768, min(32767, mixed))
            mono.extend(struct.pack("<h", mixed))
            n += 1
            if n % RATE == 0:
                print("\r处理 %.1f 秒..." % (n / RATE), end="", flush=True)
        avg = [stats[c] / n if n else 0 for c in range(CHANNELS)]
        mono = bytes(mono)
        print()

    # Save mono WAV (amplified mic channels)
    with wave.open("mic_mono.wav", "wb") as w:
        w.setnchannels(1)
        w.setsampwidth(2)
        w.setframerate(RATE)
        if USE_NUMPY:
            w.writeframes(mono.tobytes())
        else:
            w.writeframes(mono)

    print("\n各通道音量:")
    for c in range(CHANNELS):
        a = avg[c]
        tag = "有声" if a > 50 else "静音"
        print("  通道 %d: 平均 %6.1f  峰值 %5d  %s" % (c + 1, a, peak[c], tag))

    print("\n保存: raw_8ch.wav, mic_mono.wav")
    print("播放: paplay mic_mono.wav")
