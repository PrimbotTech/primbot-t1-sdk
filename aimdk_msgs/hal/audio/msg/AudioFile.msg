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