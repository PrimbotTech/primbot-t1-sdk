# 定义 Action 触发源
int32 value

# 未知/未指定
int32 UNKNOWN = 0

# APP
int32 APP = 1

# 云端
int32 CLOUD = 2

# 遥控器
int32 RC = 3

# SDK/RPC 直接调用
int32 SDK_RPC = 4

# agent
int32 AGENT = 5

# task engine 自动任务
int32 TASK_ENGINE = 6

# MC 内部自动切换（如 Moving → Stable 稳态复位）
int32 MC_INTERNAL = 7

# Insta360
int32 INSTA360 = 8

# 其他
int32 OTHER       = 255
