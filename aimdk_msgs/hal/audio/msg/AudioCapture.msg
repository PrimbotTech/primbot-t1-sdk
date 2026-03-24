# 必要项，时间戳
builtin_interfaces/Time stamps

# 必要项，麦克风通道数
uint8 mic_channels

# 必要项，回采信号通道数
uint8 ref_channels

# 必要项，音频格式
AudioInfo info

# 必要项，音频数据
AudioData data

# 可选项，标识发送方来源
string pkg_name

#可选项，0-内置麦 1-外置麦
uint32 mic_source
