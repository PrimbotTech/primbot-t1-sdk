#!/usr/bin/env python3

"""Play Audio Example

通过 PulseAudio TCP 协议（simple-protocol-tcp）将音频文件发送到机器人扬声器播放。

支持格式：
  - WAV 文件：自动检测 RIFF 头，转换为 48kHz mono S16LE
  - PCM 文件：48kHz mono S16LE，直接传输
  - 多声道 WAV 自动智能混音：识别活跃通道，避免静音通道稀释信号
  - 非 48kHz 采样率自动重采样

前提条件：
  示例脚本会通过 SSH 自动在机器人端加载 TCP 播放模块（端口 6001）。

用法：
  python3 play_audio.py <机器人IP> <音频文件>

示例：
  python3 play_audio.py 172.31.8.168 mic_mono.wav
"""

import socket
import subprocess
import sys
import signal
import struct
import time
import wave
from io import BytesIO

# ===== Configuration (must match robot-side pactl parameters) =====
RATE = 48000       # Robot speaker native sample rate
CHANNELS = 1       # Mono
SAMPLE_WIDTH = 2   # 16-bit = 2 bytes
PORT = 6001


def setup_robot_audio_tcp(robot_ip):
    """Auto-load PulseAudio TCP playback module on robot via SSH."""
    cmd = (
        'ssh -o StrictHostKeyChecking=no -o ConnectTimeout=5 run@%s '
        '"pactl load-module module-simple-protocol-tcp '
        'sink=@DEFAULT_SINK@ playback=true port=%d '
        'format=s16le rate=%d channels=%d listen=0.0.0.0"'
        % (robot_ip, PORT, RATE, CHANNELS)
    )
    try:
        subprocess.run(cmd, shell=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, timeout=10)
        print("已在机器人端加载播放模块 (端口 %d)" % PORT)
    except KeyboardInterrupt:
        print("\n已取消")
        sys.exit(0)
    except Exception:
        pass  # Module may already be loaded


def is_wav_data(data):
    """Check if data starts with RIFF header (WAV format)."""
    return len(data) >= 4 and data[:4] == b"RIFF"


def load_wav_as_pcm(data):
    """Load WAV data from bytes buffer and convert to 48kHz mono S16LE PCM bytes.

    Handles:
      - Any sample rate (resamples to 48kHz)
      - Multi-channel (mixes down to mono)
      - 16-bit PCM format

    Returns:
        PCM bytes ready for streaming.
    """
    with wave.open(BytesIO(data), "rb") as wf:
        src_rate = wf.getframerate()
        src_channels = wf.getnchannels()
        src_sampwidth = wf.getsampwidth()
        n_frames = wf.getnframes()

        if src_sampwidth != 2:
            print("警告: 仅支持 16-bit PCM WAV，当前 %d-bit" % (src_sampwidth * 8))
            sys.exit(1)

        print("WAV 格式: %dHz, %dch, 16bit, %d帧 (%.1f秒)" % (
            src_rate, src_channels, n_frames, n_frames / src_rate))

        raw = wf.readframes(n_frames)

    # Parse samples
    n_samples = len(raw) // 2
    samples = struct.unpack("<%dh" % n_samples, raw)

    # Mix down to mono if multi-channel
    if src_channels > 1:
        # Find active channels: per-channel peak, only mix channels with signal
        ch_peaks = [0] * src_channels
        for i in range(0, n_samples, src_channels):
            for c in range(min(src_channels, n_samples - i)):
                a = abs(samples[i + c])
                if a > ch_peaks[c]:
                    ch_peaks[c] = a

        # A channel is "active" if its peak exceeds a noise floor
        active = [c for c in range(src_channels) if ch_peaks[c] > 200]
        if not active:
            active = list(range(src_channels))  # all quiet, use all

        print("混音: %d 声道 -> 单声道 (活跃通道: %s)" % (
            src_channels, ", ".join("ch%d" % (c + 1) for c in active)))
        print("各声道峰值: %s" % " ".join("ch%d=%d" % (c + 1, ch_peaks[c]) for c in range(src_channels)))

        mono_samples = []
        for i in range(0, n_samples, src_channels):
            s = 0
            for c in active:
                if i + c < n_samples:
                    s += samples[i + c]
            avg = s // len(active)
            mono_samples.append(int(max(-32768, min(32767, avg))))
        samples = mono_samples
        n_samples = len(samples)

    # Resample if not 48kHz
    if src_rate != RATE:
        print("重采样: %dHz -> %dHz" % (src_rate, RATE))
        ratio = RATE / src_rate
        new_len = int(n_samples * ratio)
        resampled = []
        for i in range(new_len):
            src_idx = i / ratio
            idx_low = int(src_idx)
            idx_high = min(idx_low + 1, n_samples - 1)
            frac = src_idx - idx_low
            # Linear interpolation
            val = samples[idx_low] * (1 - frac) + samples[idx_high] * frac
            resampled.append(int(max(-32768, min(32767, val))))
        samples = resampled

    return struct.pack("<%dh" % len(samples), *samples)


def load_file(path):
    """Load file as raw bytes from local path."""
    try:
        with open(path, "rb") as f:
            return f.read()
    except IOError:
        print("错误: 无法打开本地文件: %s" % path)
        sys.exit(1)


def main():
    if len(sys.argv) < 3:
        print("用法: %s <机器人IP> <audio_file>" % sys.argv[0])
        print("支持格式: .pcm (48kHz mono S16LE), .wav (自动转换)")
        sys.exit(1)

    robot_ip = sys.argv[1]
    audio_file = sys.argv[2]

    stop = False

    def on_signal(*_):
        nonlocal stop
        stop = True
    signal.signal(signal.SIGINT, on_signal)

    # Auto-load TCP playback module on robot via SSH
    setup_robot_audio_tcp(robot_ip)

    # Load raw file data (local)
    raw = load_file(audio_file)

    # Auto-detect format and convert to PCM
    if is_wav_data(raw):
        print("检测到 WAV 文件，正在转换...")
        pcm_data = load_wav_as_pcm(raw)
        print("转换完成: %d 字节 (%.1f 秒)" % (len(pcm_data), len(pcm_data) / (RATE * CHANNELS * SAMPLE_WIDTH)))
    else:
        print("按 PCM 格式使用 (假设 48kHz mono S16LE)")
        pcm_data = raw

    # Connect to robot
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.settimeout(5)  # 5 second connection timeout
    try:
        sock.connect((robot_ip, PORT))
    except (ConnectionRefusedError, OSError) as e:
        if isinstance(e, ConnectionRefusedError):
            print("连接被拒绝，请检查机器人 IP 是否正确以及 TCP 播放模块是否已加载。")
        else:
            print("连接失败: %s" % e)
            print("请检查机器人 IP 是否正确，以及网络是否可达")
        sys.exit(1)
    sock.settimeout(None)

    print("已连接 %s:%d，播放 %s ..." % (robot_ip, PORT, audio_file))

    # Streaming parameters
    bytes_per_sec = RATE * CHANNELS * SAMPLE_WIDTH  # 96000 bytes/sec
    chunk_duration = 0.1  # 100ms per chunk
    chunk_bytes = int(bytes_per_sec * chunk_duration)  # 9600 bytes

    total = 0
    offset = 0
    next_send_time = time.monotonic()

    while not stop and offset < len(pcm_data):
        chunk = pcm_data[offset:offset + chunk_bytes]
        offset += len(chunk)

        try:
            sock.sendall(chunk)
        except (BrokenPipeError, ConnectionResetError):
            print("\n连接断开")
            break

        total += len(chunk)
        print("\r已播放 %.1f / %.1f 秒" % (
            total / bytes_per_sec, len(pcm_data) / bytes_per_sec), end="", flush=True)

        # Precise timing: compensate for network and processing delay
        next_send_time += chunk_duration
        sleep_time = next_send_time - time.monotonic()
        if sleep_time > 0:
            time.sleep(sleep_time)

    print("\n播放结束")
    sock.close()


if __name__ == "__main__":
    main()
