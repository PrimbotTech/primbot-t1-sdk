# topic: /aima/agent/asr_result
MessageHeader header
# 识别文本
string text
# 事件ID，用于关联对话上下文
string event_id
# 是否为最终结果（false 表示中间/部分结果）
bool is_final
# 置信度 [0.0, 1.0]
float32 confidence
# 语言标签，如 "zh"、"en"
string language
