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

uint32 REASON_SUCCESS          = 0   # 成功
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

uint32 REASON_NOT_FOUND            = 17   # 未找到相关动作
uint32 REASON_NO_SKILL_LIST        = 18   # 无可用技能列表
uint32 REASON_FORM_MIS_MATCH       = 19   # 无法执行任务，机器人需要处于双足站立状态
uint32 REASON_ACTION_NOT_MATCH     = 20   # 当前动作不匹配，无法执行动画
uint32 REASON_ACTION_BLACKLISTED   = 21   # 前动作在黑名单中，无法执行动画
uint32 REASON_NO_TRANSFER_PATH     = 22   # 当前姿态下无法进行动作转移
uint32 REASON_NO_ELIGIBLE_SKILL    = 23   # 无符合条件的技能

uint32 REASON_INDOOR_WHOLE_BODY    = 24   # 室内模式下暂不支持该动作
uint32 REASON_ENGINE_NOT_READY     = 25   # 动作配置未就绪，无法执行动画
uint32 REASON_NOT_SUPPORTED        = 26   # 当前机型不支持该功能
uint32 REASON_STAND_UP_IN_PROGRESS = 27   # 机器人正在起立，暂不能收起默认动作
uint32 REASON_NOT_INTERRUPTIBLE    = 28   # 当前动画不可打断
uint32 REASON_INTERRUPT_INVALID    = 29   # 当前动画 ID 无效，无法打断
uint32 REASON_TRANSFORM_CONDITION  = 30   # 当前状态不满足变形/动作执行条件