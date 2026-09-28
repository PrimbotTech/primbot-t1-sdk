#!/usr/bin/env python3

"""录音示例（自动检测内置麦/外置麦）

支持本机（共享内存）和远程（TCP）两种录音模式。
自动检测当前 PulseAudio 默认录音源，适配不同麦克风：

内置麦（麦克风阵列 aw89403）：
  - 8声道 48kHz S16LE
  - ch1-2：回采, ch3-8：麦克
  - 输出：raw_8ch.wav + mic_mono.wav（ch5-6 混合, 10倍放大）

外置麦（USB 麦克风）：
  - 2声道 48kHz S24LE（24-bit）
  - ch1-2：左右声道
  - 输出：raw_2ch.wav + mic_mono.wav（ch1-2 混合, 24→16bit）

前提条件：
  - 远程模式：示例脚本会通过 SSH 自动在机器人端加载 TCP 录音模块（端口 4713）
  - 本机模式：无需额外配置，直接使用 PulseAudio 本地 socket
  - 本机已安装 parec：sudo apt install pulseaudio-utils
  - 使用 SSH 密钥时：将售后提供的密钥目录（<机器人SN>_soc0/）放到 SDK 根目录下
    （与 examples/ 同级），并修改权限：
    sudo chown -R $USER:$USER ./<机器人SN>_soc0/
    chmod 600 ./<机器人SN>_soc0/id_ed25519
    然后在 SDK 根目录下执行示例命令，使用 -i ./<机器人SN>_soc0/id_ed25519 指定密钥。

用法：
  python3 examples/python/record_audio.py [-i SSH_KEY] <机器人IP> [录音秒数]

  机器人IP 为 127.0.0.1 或 localhost 时自动使用本机模式（共享内存），
  否则使用远程模式（TCP）。

  可选参数：
    -i SSH_KEY    SSH 私钥文件路径（机器人需要密钥登录）

示例：
  # 本机录音（共享内存）
  python3 examples/python/record_audio.py 127.0.0.1 5

  # 远程录音（TCP）
  python3 examples/python/record_audio.py <机器人IP> 5

  # 远程录音（使用 SSH 密钥，网线连接时 IP 为 10.1.1.100）
  python3 examples/python/record_audio.py -i ./<机器人SN>_soc0/id_ed25519 10.1.1.100 5
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
BUILTIN_DEVICE = "alsa_input.platform-aw89403_sound.pro-input-0"

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


def setup_and_detect_source(robot_ip, ssh_key=None):
    """通过一次 SSH 同时完成 TCP 模块加载检查 + 默认录音源检测。

    合并两个 SSH 调用以避免重复输入密码。

    Args:
        robot_ip: 机器人 IP 地址。
        ssh_key: SSH 私钥文件路径（机器人需要密钥登录）。

    Returns:
        str: 默认录音源名称（空字符串表示检测失败）。
    """
    ssh_opts = "-o StrictHostKeyChecking=no -o ConnectTimeout=5"
    if ssh_key:
        ssh_opts += " -i %s" % ssh_key
    cmd = (
        'ssh %s run@%s \''
        'export PULSE_SERVER=unix:/run/user/1000/pulse/native; '
        'if pactl list modules short 2>/dev/null | grep -q module-native-protocol-tcp; then '
        'echo __TCP_MODULE__:ALREADY_LOADED; '
        'else '
        'pactl load-module module-native-protocol-tcp auth-anonymous=1 listen=0.0.0.0 >/dev/null 2>&1 && '
        'echo __TCP_MODULE__:LOADED; '
        'fi; '
        'echo __DEFAULT_SOURCE__; '
        'pactl get-default-source 2>/dev/null\''
        % (ssh_opts, robot_ip)
    )
    try:
        result = subprocess.run(cmd, shell=True, stdout=subprocess.PIPE, text=True)
        output = result.stdout
    except KeyboardInterrupt:
        print("\n已取消")
        sys.exit(0)
    except Exception as e:
        print("SSH 连接失败: %s" % e)
        sys.exit(1)

    # 解析 TCP 模块加载状态
    if "__TCP_MODULE__:ALREADY_LOADED" in output:
        print("录音模块已在机器人端加载（跳过加载）")
    elif "__TCP_MODULE__:LOADED" in output:
        print("已在机器人端加载录音模块 (端口 4713)")
    else:
        print("警告: 录音模块加载可能未成功（stderr 已输出到终端）")

    # 解析默认录音源名称（__DEFAULT_SOURCE__ 标记之后的第一行非空内容）
    source = ""
    marker = "__DEFAULT_SOURCE__"
    pos = output.find(marker)
    if pos != -1:
        rest = output[pos + len(marker):].strip()
        if rest:
            source = rest.split("\n")[0].strip()
    return source


def parse_mic_profile(source):
    """根据录音源名称解析麦克风配置字典。"""
    if not source:
        print("无法检测默认录音源，使用内置麦默认配置")
        return {
            'device': BUILTIN_DEVICE,
            'channels': 8,
            'sample_width': 2,
            'format': 's16le',
            'bits_per_sample': 16,
            'is_builtin': True,
        }

    if 'aw89403' in source:
        profile = {
            'device': source,
            'channels': 8,
            'sample_width': 2,
            'format': 's16le',
            'bits_per_sample': 16,
            'is_builtin': True,
        }
        print("检测到内置麦（aw89403）: %s" % source)
    else:
        profile = {
            'device': source,
            'channels': 2,
            'sample_width': 3,
            'format': 's24le',
            'bits_per_sample': 24,
            'is_builtin': False,
        }
        print("检测到外置麦（USB）: %s" % source)

    print("  格式: %s, %dch, %dHz, %d-bit" % (
        profile['format'], profile['channels'], RATE, profile['bits_per_sample']))
    return profile


# ---------------------------------------------------------------------------
# Microphone profile detection
# ---------------------------------------------------------------------------

def get_default_source(local_mode, robot_ip, ssh_key=None):
    """获取 PulseAudio 默认录音源名称。

    Args:
        local_mode: True 表示本机模式，False 表示远程模式。
        robot_ip: 机器人 IP 地址。
        ssh_key: SSH 私钥文件路径（机器人需要密钥登录）。
    """
    if local_mode:
        cmd = "pactl get-default-source"
    else:
        ssh_opts = "-o StrictHostKeyChecking=no -o ConnectTimeout=5"
        if ssh_key:
            ssh_opts += " -i %s" % ssh_key
        cmd = (
            "ssh %s run@%s '"
            "PULSE_SERVER=unix:/run/user/1000/pulse/native "
            "pactl get-default-source 2>/dev/null'"
            % (ssh_opts, robot_ip)
        )
    try:
        result = subprocess.run(cmd, shell=True, capture_output=True, text=True)
        return result.stdout.strip()
    except Exception:
        return ""


def detect_mic_profile(local_mode, robot_ip, ssh_key=None):
    """检测当前默认录音源，返回麦克风配置字典。

    Args:
        local_mode: True 表示本机模式，False 表示远程模式。
        robot_ip: 机器人 IP 地址。
        ssh_key: SSH 私钥文件路径（机器人需要密钥登录）。

    Returns:
        dict: {device, channels, sample_width, format, bits_per_sample, is_builtin}
    """
    source = get_default_source(local_mode, robot_ip, ssh_key)

    if not source:
        print("无法检测默认录音源，使用内置麦默认配置")
        return {
            'device': BUILTIN_DEVICE,
            'channels': 8,
            'sample_width': 2,
            'format': 's16le',
            'bits_per_sample': 16,
            'is_builtin': True,
        }

    if 'aw89403' in source:
        profile = {
            'device': source,
            'channels': 8,
            'sample_width': 2,
            'format': 's16le',
            'bits_per_sample': 16,
            'is_builtin': True,
        }
        print("检测到内置麦（aw89403）: %s" % source)
    else:
        profile = {
            'device': source,
            'channels': 2,
            'sample_width': 3,
            'format': 's24le',
            'bits_per_sample': 24,
            'is_builtin': False,
        }
        print("检测到外置麦（USB）: %s" % source)

    print("  格式: %s, %dch, %dHz, %d-bit" % (
        profile['format'], profile['channels'], RATE, profile['bits_per_sample']))
    return profile


# ---------------------------------------------------------------------------
# S24LE helpers
# ---------------------------------------------------------------------------

def decode_s24le_numpy(data):
    """将 S24LE 原始字节解码为 int32 numpy 数组。"""
    raw = np.frombuffer(data, dtype=np.uint8)
    n = len(raw) // 3
    raw = raw[:n * 3].reshape(n, 3)
    val = (raw[:, 0].astype(np.int32) |
           (raw[:, 1].astype(np.int32) << 8) |
           (raw[:, 2].astype(np.int32) << 16))
    val[val >= 0x800000] -= 0x1000000  # sign extend
    return val


def s24_to_s16_numpy(s24):
    """将 S24LE int32 numpy 数组转换为 int16（右移 8 位 + 钳位）。"""
    return np.clip(s24 >> 8, -32768, 32767).astype(np.int16)


def decode_s24le_bytes(data):
    """将 S24LE 原始字节解码为 int32 列表（无 numpy 路径）。"""
    samples = []
    for i in range(0, len(data) - 2, 3):
        val = data[i] | (data[i + 1] << 8) | (data[i + 2] << 16)
        if val >= 0x800000:
            val -= 0x1000000
        samples.append(val)
    return samples


def s24_to_s16_scalar(v):
    """将 S24LE int32 标量转换为 int16。"""
    v = v >> 8
    if v > 32767:
        v = 32767
    if v < -32768:
        v = -32768
    return v


def record_pcm(robot_ip, profile, duration=5, local_mode=False):
    """Record multi-channel audio from robot via PulseAudio.

    Args:
        robot_ip: Robot IP address.
        profile: Mic profile dict from detect_mic_profile().
        duration: Recording duration in seconds.
        local_mode: True for local shared memory, False for remote TCP.

    Returns:
        Raw PCM bytes.
    """
    channels = profile['channels']
    sample_width = profile['sample_width']
    fmt = profile['format']
    device = profile['device']

    cmd = [
        'parec',
        '--channels', str(channels),
        '--rate', str(RATE),
        '--format', fmt,
        '--device', device,
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

    bytes_per_sec = RATE * channels * sample_width
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
        print("用法: %s [-i SSH_KEY] <机器人IP> [录音秒数]" % sys.argv[0])
        print()
        print("  机器人IP 为 127.0.0.1 或 localhost 时自动使用本机模式（共享内存）")
        print("  否则使用远程模式（TCP）")
        print()
        print("  可选参数：")
        print("    -i SSH_KEY    SSH 私钥文件路径（机器人需要密钥登录）")
        sys.exit(1)

    # 解析可选参数 -i SSH_KEY
    ssh_key = None
    args = sys.argv[1:]
    if args[0] == "-i":
        if len(args) < 3:
            print("错误: -i 参数需要指定 SSH 私钥文件路径")
            sys.exit(1)
        ssh_key = args[1]
        args = args[2:]

    robot_ip = args[0]
    duration = int(args[1]) if len(args) > 1 else 5

    # 检测本机/远程模式
    local_mode = is_local_robot(robot_ip)

    if local_mode:
        print("检测到本机环境，使用共享内存传输（效率更高）")
    else:
        print("检测到远程环境，使用 TCP 传输")
        if ssh_key:
            print("使用 SSH 密钥: %s" % ssh_key)
        # 远程模式：通过一次 SSH 同时完成 TCP 模块加载 + 录音源检测（避免重复输入密码）
        source = setup_and_detect_source(robot_ip, ssh_key)

    # 检测麦克风类型
    if local_mode:
        profile = detect_mic_profile(local_mode, robot_ip, ssh_key)
    else:
        profile = parse_mic_profile(source)
    channels = profile['channels']
    sample_width = profile['sample_width']
    bits_per_sample = profile['bits_per_sample']
    is_builtin = profile['is_builtin']

    print("麦克风配置: %s, %dch, %dHz, %d-bit" % (
        profile['format'], channels, RATE, bits_per_sample))

    data = record_pcm(robot_ip, profile, duration=duration, local_mode=local_mode)

    if not data:
        print("错误: 未录到任何音频数据，请检查 PulseAudio 连接")
        sys.exit(1)

    # Save raw WAV (8ch or 2ch depending on mic type)
    raw_filename = "raw_8ch.wav" if is_builtin else "raw_2ch.wav"
    with wave.open(raw_filename, "wb") as w:
        w.setnchannels(channels)
        w.setsampwidth(sample_width)
        w.setframerate(RATE)
        w.writeframes(data)

    bytes_per_frame = channels * sample_width
    print("录音 %d 字节 (%.1f 秒)" % (len(data), len(data) / (RATE * bytes_per_frame)))
    print("分析中...")

    if USE_NUMPY:
        if is_builtin:
            # Built-in mic: 8ch S16LE
            samples = np.frombuffer(data, dtype=np.int16).reshape(-1, channels)
            avg = np.abs(samples.astype(np.int32)).mean(axis=0)
            peak = np.abs(samples).max(axis=0)

            # Extract channels 5-6 (index 4,5), mix and amplify 10x
            mono = samples[:, [4, 5]].mean(axis=1).astype(np.float32) * 10
            mono = np.clip(mono, -32768, 32767).astype(np.int16)
        else:
            # External USB mic: 2ch S24LE
            s24_samples = decode_s24le_numpy(data).reshape(-1, channels)
            # Convert to 16-bit for analysis
            s16_samples = s24_to_s16_numpy(s24_samples)
            avg = np.abs(s16_samples.astype(np.int32)).mean(axis=0)
            peak = np.abs(s16_samples).max(axis=0)

            # Mix ch1-2, convert 24→16 bit (use integer arithmetic to keep int32 dtype)
            mixed_s24 = (s24_samples[:, 0] + s24_samples[:, 1]) // 2
            mono = s24_to_s16_numpy(mixed_s24)
    else:
        stats = [0] * channels
        peak = [0] * channels
        mono = bytearray()
        n = 0
        frame_size = bytes_per_frame
        for i in range(0, len(data) - frame_size + 1, frame_size):
            frame_data = data[i:i + frame_size]
            if is_builtin:
                # Built-in mic: 8ch S16LE
                s = struct.unpack("<%dh" % channels, frame_data)
                for c in range(channels):
                    stats[c] += abs(s[c])
                    if abs(s[c]) > peak[c]:
                        peak[c] = abs(s[c])
                mixed = (s[4] + s[5]) // 2 * 10  # Mix ch5-6, amplify 10x
                mixed = max(-32768, min(32767, mixed))
                mono.extend(struct.pack("<h", mixed))
            else:
                # External USB mic: 2ch S24LE
                s24 = decode_s24le_bytes(frame_data)
                for c in range(channels):
                    s16 = max(-32768, min(32767, s24[c] >> 8))
                    stats[c] += abs(s16)
                    if abs(s16) > peak[c]:
                        peak[c] = abs(s16)
                mixed_s24 = (s24[0] + s24[1]) // 2
                mixed_s16 = max(-32768, min(32767, mixed_s24 >> 8))
                mono.extend(struct.pack("<h", mixed_s16))
            n += 1
            if n % RATE == 0:
                print("\r处理 %.1f 秒..." % (n / RATE), end="", flush=True)
        avg = [stats[c] / n if n else 0 for c in range(channels)]
        mono = bytes(mono)
        print()

    # Save mono WAV (always 16-bit)
    with wave.open("mic_mono.wav", "wb") as w:
        w.setnchannels(1)
        w.setsampwidth(2)
        w.setframerate(RATE)
        if USE_NUMPY:
            w.writeframes(mono.tobytes())
        else:
            w.writeframes(mono)

    print("\n各通道音量:")
    for c in range(channels):
        tag = "有声" if avg[c] > 50 else "静音"
        print("  通道 %d: 平均 %6.1f  峰值 %5d  %s" % (c + 1, avg[c], peak[c], tag))

    print("\n保存: %s, mic_mono.wav" % raw_filename)
    print("播放: paplay mic_mono.wav")
