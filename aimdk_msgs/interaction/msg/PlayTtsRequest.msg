# 必要项
string text

# 必要项，默认使用 INTERACTION_L6
TtsPriorityLevel priority_level 

# 可选，非产品要求勿用，优先级加权参数(0-99)，最小单位为 1
uint32 priority_weight    

# 必要项，标识调用方来源
string domain

# 可选，用于标识播报的请求id
string trace_id

# 必要项，是否打断同等优先级播报（ true：打断；false：不打断，进入队列播报）
bool is_interrupted

# 发音人ID（可选，与SetTTSParametersRequest中的voice_id一致）
# uint32 voice_id    

# 情感风格（可选，如"happy"、"sad"等）
# string emotion