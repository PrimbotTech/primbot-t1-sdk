# --------------------------------------------------------------------------------------------------------------
# 机器人触摸传感器状态
# --------------------------------------------------------------------------------------------------------------

# 触摸传感器状态消息
MessageHeader header

# 触摸事件类型
uint8 event_type

# 触摸事件类型常量定义
uint8 TOUCH_EVENT_NONE         = 0  # 无触摸事件
uint8 TOUCH_EVENT_SINGLE_CLICK = 1  # 单击
uint8 TOUCH_EVENT_DOUBLE_CLICK = 2  # 双击
uint8 TOUCH_EVENT_TRIPLE_CLICK = 3  # 三击
uint8 TOUCH_EVENT_QUADRUPLE_CLICK = 4  # 四击
uint8 TOUCH_EVENT_QUINTUPLE_CLICK = 5  # 五击
uint8 TOUCH_EVENT_SEXTUPLE_CLICK = 6  # 六击
uint8 TOUCH_EVENT_MULTI_CLICK  = 7  # 多次点击（超过六次）
uint8 TOUCH_EVENT_SLIDE_LEFT   = 8  # 向左滑动
uint8 TOUCH_EVENT_SLIDE_RIGHT  = 9  # 向右滑动



