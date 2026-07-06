# 定义枚举字段
int32 value

# ============================================================================
# 急停
# ============================================================================

# 被动模式
int32 PASSIVE_DEFAULT = 1

# 阻尼模式
int32 DAMPING_DEFAULT = 2

# 四足躺下
int32 QUAD_LIE_DOWN = 3

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

# 四足准备站立
int32 QUADRUPED_READY_STAND_DEFAULT = 115

# 四足跳跃
int32 QUADRUPED_LOCOMOTION_JUMP = 116

# 四足伸展
int32 QUADRUPED_LOCOMOTION_STRETCH = 117

# 四足俯卧撑
int32 QUADRUPED_PUSH_UP = 118

# 四足模仿动作
int32 QUADRUPED_MIMIC = 119

# 四足蜷缩
int32 QUADRUPED_CROUCH_DEFAULT = 120

# 四足踏步模式
int32 QUADRUPED_LOCOMOTION_STEP = 121

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

# 双足太空步
int32 BIPED_LOCOMOTION_MOONWALK = 206

# 双足奔跑模式
int32 BIPED_LOCOMOTION_RUN = 207

# 双足模仿动作
int32 BIPED_MIMIC = 208

# 双足羽生结弦
int32 BIPED_YUZURU = 209

# 双足劈叉
int32 BIPED_SPLIT = 210

# 双足全身控制模式
int32 BIPED_LOCOMOTION_WBC = 300

# 双足动画控制模式
int32 BIPED_LOCOMOTION_ANIMATION = 301

# 双足控制上肢录制模式
int32 BIPED_RECORD_UPPER = 302

# 双足转圈
int32 BIPED_MIMIC_ROTATE_DANCE = 400 

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

# 全身动作控制
int32 BIPED_WHOLE_BODY_CTRL = 1005

# 模仿动作-开源
int32 MIMIC_OPENSOURCE = 1006

# 站起
int32 STAND_UP = 1007

# 躺姿
int32 LYING_DEFAULT = 1008

# 躺下-俯卧
int32 LIE_FACE_DOWN = 1009

# 躺下-仰躺
int32 LIE_FACE_UP = 1010

# 轻柔步态(室内)
int32 BIPED_QUIET_WALK = 1011

# 越障模式(室外)
int32 BIPED_BLIND_TERRAIN = 1012

# 坐下
int32 BIPED_SIT_DOWN = 1013

# 二开上肢控制
int32 BIPED_CUSTOM_UPPER = 1014
