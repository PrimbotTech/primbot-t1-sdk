#!/usr/bin/env python3

"""录音示例

通过 PulseAudio TCP 协议从机器人麦克风阵列录音。

录音格式：8声道 48kHz S16LE（机器人麦克风原始格式）
声道说明：
  - ch1-2：扬声器回放
  - ch3-6：麦克风信号（ch5-6 为主要拾音通道）
  - ch7-8：全零

输出文件：
  - raw_8ch.wav：完整 8 声道原始录音
  - mic_mono.wav：ch5-6 混合的单声道录音（10 倍放大）

前提条件：
  - 示例脚本会通过 SSH 自动在机器人端加载 TCP 录音模块（端口 4713）
  - 本机已安装 parec：sudo apt install pulseaudio-utils

用法：
  python3 record_audio.py <机器人IP> [录音秒数]

示例：
  python3 record_audio.py <机器人IP> 5
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


def setup_robot_audio_tcp(robot_ip):
    """Auto-load PulseAudio TCP recording module on robot via SSH."""
    cmd = (
        'ssh -o StrictHostKeyChecking=no -o ConnectTimeout=5 run@%s '
        '"pactl load-module module-native-protocol-tcp '
        'auth-anonymous=1 listen=0.0.0.0"'
        % robot_ip
    )
    try:
        subprocess.run(cmd, shell=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, timeout=10)
        print("已在机器人端加载录音模块 (端口 4713)")
    except Exception:
        pass  # Module may already be loaded


def record_pcm(robot_ip, duration=5):
    """Record multi-channel audio from robot via PulseAudio TCP.

    Args:
        robot_ip: Robot IP address.
        duration: Recording duration in seconds.

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

    env = dict(os.environ, PULSE_SERVER="tcp:%s:4713" % robot_ip)

    print("连接 %s，录音 %d 秒..." % (robot_ip, duration))
    proc = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, env=env)

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

    print()
    return bytes(audio)


if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("用法: %s <机器人IP> [录音秒数]" % sys.argv[0])
        sys.exit(1)

    robot_ip = sys.argv[1]
    duration = int(sys.argv[2]) if len(sys.argv) > 2 else 5

    # Auto-load TCP recording module on robot via SSH
    setup_robot_audio_tcp(robot_ip)

    data = record_pcm(robot_ip, duration=duration)

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
