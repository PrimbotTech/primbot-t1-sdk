int32 value

# 未知
int32 UNKNOWN = 0
# 成功
int32 SUCCESS = 1
# 失败
int32 FAILURE = 2
# 中止
int32 ABORTED = 3
# 超时
int32 TIMEOUT = 4
# 无效
int32 INVALID = 5
# 手动
int32 IN_MANUAL = 6

# 未就绪
int32 NOT_READY = 100
# 等待中
int32 PENDING = 200
# 已创建
int32 CREATED = 300
# 运行中
int32 RUNNING = 400


uint32 reason

uint32 REASON_OPENBOX          = 1   # 开箱状态中
uint32 REASON_STARTUP_CHECK    = 2   # 开机自检中
uint32 REASON_SHUTDOWN         = 3   # 关机状态中
uint32 REASON_FROM             = 4   # 当前形态不支持
uint32 REASON_LOW_BATTERY      = 5   # 低电量限制
uint32 REASON_CHARGE           = 6   # 正在充电中
uint32 REASON_NO_WHITELIST     = 7   # 动作不在白名单
uint32 REASON_HDS              = 8   # HDS故障
uint32 REASON_ACTION           = 9   # 当前模式不支持
uint32 REASON_OBSTACLE_FORWARD = 10  # 前方有障碍物
uint32 REASON_OBSTACLE_BACK    = 11  # 后方有障碍物
uint32 REASON_OBSTACLE_LEFT    = 12  # 左方有障碍物
uint32 REASON_OBSTACLE_RIGHT   = 13  # 右方有障碍物
uint32 REASON_OBSTACLE_UP      = 14  # 上方有障碍物
uint32 REASON_TASK_RUNNING     = 15  # 其他任务正在运行
uint32 REASON_TARGET_STATE     = 16  # 机器人已经是目标状态

uint8 REASON_NOT_FOUND            = 17
uint8 REASON_NO_SKILL_LIST        = 18
uint8 REASON_FORM_MIS_MATCH       = 19
uint8 REASON_ACTION_NOT_MATCH     = 20
uint8 REASON_ACTION_BLACKLISTED   = 21
uint8 REASON_NO_TRANSFER_PATH     = 22
uint8 REASON_NO_ELIGIBLE_SKILL    = 23