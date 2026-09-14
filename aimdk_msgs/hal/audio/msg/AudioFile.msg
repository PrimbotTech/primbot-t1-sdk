# 必要项，标识调用方来源
string pkg_name

# 必要项，文件名
string file_name

# 必要项，不设置时使用系统默认路径
string file_path

# pcm格式必要项，wav格式可选项，音频格式
AudioInfo info

# 必要项，优先级(1~10, 默认6)
uint32 priority

# 可选项，(1~100) priority + priority_weight% is final priority
uint32 priority_weight

# 可选项，PulseAudio media.role，不传默认 Music
# 可选值：Alert(系统告警,优先级100) | Notification(系统提示音,优先级80) | Communication(对话语音,优先级60) | Music(音乐/媒体,优先级40,默认)
# 非 Music 角色播放时会自动降低(duck)音乐音量
string media_role