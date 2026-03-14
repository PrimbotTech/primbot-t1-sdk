# 必要项，通道数
uint8 channels

# 必要项，采样率[Hz]
uint32 sample_rate

# 可选项，写入size[byte]
uint32 size

# 可选项，Audio format (e.g. S16LE， S32_LE) 默认S16_LE
string sample_format

# 可选项，Audio coding format (e.g. pcm, wave, opus)
string coding_format