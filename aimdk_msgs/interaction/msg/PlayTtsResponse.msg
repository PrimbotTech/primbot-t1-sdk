# PlayMediaFileRequest 同样复用此 rsp，file_name 放在 text 中
string text                                

TtsPriorityLevel priority_level

# 加权参数(0-10)，最小单位为 1
uint32 priority_weight   

# 标识调用方来源
string domain

string trace_id

# 是否成功播放
bool is_success

# 错误信息
string error_message

# 当前文本预计耗时（单位：毫秒）
uint32 estimated_duration