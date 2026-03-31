# 定义枚举字段
int32 value

# ============================================================================
# 急停
# ============================================================================

# 被动模式
int32 PASSIVE_DEFAULT = 1

# 阻尼模式
int32 DAMPING_DEFAULT = 2

# ============================================================================
# 形态切换
# ============================================================================

# 四足切换两足形态-常规
int32 QUADRUPED_TO_BIPED = 10

# 四足切换两足形态-带旋转
int32 QUADRUPED_TO_BIPED_ROTATE = 11

# 四足切换双足形态-前空翻
int32 QUADRUPED_TO_BIPED_FRONTFLIP = 12

# 两足切换四足形态-常规
int32 BIPED_TO_QUADRUPED = 20

# 两足切换四足形态-旋转
int32 BIPED_TO_QUADRUPED_ROTATE = 21

# 两足切换四足形态-前空翻
int32 BIPED_TO_QUADRUPED_FRONTFLIP = 22

# ============================================================================
# 四足相关动作 (100-199)
# ============================================================================

# 倒地四足站起
int32 QUADRUPED_RECOVERY = 100

# 四足站立模式
int32 QUADRUPED_STAND_DEFAULT = 101

# 四足行走/奔跑模式
int32 QUADRUPED_LOCOMOTION_DEFAULT = 102

# 四足奔跑模式
int32 QUADRUPED_LOCOMOTION_RUN = 112

# 四足越障行走模式
int32 QUADRUPED_LOCOMOTION_TERRAIN = 103

# 四足仿生模式
int32 QUADRUPED_LOCOMOTION_BIONIC = 104

# 四足前空翻
int32 QUADRUPED_LOCOMOTION_FRONTFLIP = 105

# 四足后空翻
int32 QUADRUPED_LOCOMOTION_BACKFLIP = 106

# 四足握手
int32 QUADRUPED_LOCOMOTION_HANDSHAKE = 107

# 四足坐下
int32 QUADRUPED_LOCOMOTION_SIT = 108

# 四足跳舞
int32 QUADRUPED_LOCOMOTION_DANCE = 109

# 四足趴下
int32 QUADRUPED_GET_DOWN_DEFAULT = 110

# 四足坐下
int32 QUADRUPED_SIT_DOWN_DEFAULT = 111

# 四足骄傲
int32 QUADRUPED_LOCOMOTION_PRIDE = 113

# 四足愉快
int32 QUADRUPED_LOCOMOTION_PLEASURE = 114

# 四足跳跃
int32 QUADRUPED_LOCOMOTION_JUMP = 116

# ============================================================================
# 两足相关动作 (200-299)
# ============================================================================

# 倒地两足站起
int32 BIPED_RECOVERY = 200

# 两足站立模式
int32 BIPED_STAND_DEFAULT = 201

# 两足行走/奔跑模式
int32 BIPED_LOCOMOTION_DEFAULT = 202

# 两足越障行走模式
int32 BIPED_LOCOMOTION_TERRAIN = 203

# 双足原地旋转
int32 BIPED_LOCOMOTION_ROTATE_LOCAL = 204

# 双足下腰模式
int32 BIPED_LOCOMOTION_BACKBEND = 205

# 双足全身控制模式
int32 BIPED_LOCOMOTION_WBC = 300

# ============================================================================
# Q 系列Action
# ============================================================================

# 折叠收储
int32 STORE_DEFAULT = 1001

# 走跑
int32 BIPED_WALK_RUN = 1002

# 模仿动作
int32 MIMIC = 1003

# 模仿动作-灵创
int32 MIMIC_LINKCRAFT = 1004

int32 BIPED_WHOLE_BODY_CTRL = 1005
