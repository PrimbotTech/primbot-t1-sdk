# Header 包含时间辍, trace_id、domain
MessageHeader header

# 表情文件的路径
string e_path

# 当前一组表情的路径列表
string[] e_path_list

# 表情id，可以为空
uint8 e_id

# 播放模式，1-仅播放一次，2-循环播放
uint8 mode

# 默认表情优先级为 0, 数值越大优先级越高
int32 priority

# 当前状态枚举：idle | start（仅广播一次）| running | finished（仅广播一次）| stopped（仅广播一次）
uint8 STATUS_IDLE=0
uint8 STATUS_START=1
uint8 STATUS_RUNNING=2
uint8 STATUS_FINISHED=3
uint8 STATUS_STOPPED=4

uint8 status

# 剩余时长，单位：秒；如果重复，会将剩余重复时间计算在内
float64 time_to_end_ms
