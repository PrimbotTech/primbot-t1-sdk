# --------------------------------------------------------------------------------------------------------------
# 机器人触摸传感器状态
# --------------------------------------------------------------------------------------------------------------

# 触摸传感器状态消息
MessageHeader header

# 触摸事件类型
uint8 event_type

# 触摸事件类型常量定义
uint8 TOUCH_EVENT_NONE         = 0
uint8 TOUCH_EVENT_CLICK_SINGLE = 1
uint8 TOUCH_EVENT_CLICK_DOUBLE = 2
uint8 TOUCH_EVENT_CLICK_TRIPLE = 3
uint8 TOUCH_EVENT_CLICK_QUAD   = 4
uint8 TOUCH_EVENT_CLICK_QUINT  = 5
uint8 TOUCH_EVENT_CLICK_SEXT   = 6
uint8 TOUCH_EVENT_CLICK_MULTI  = 7
uint8 TOUCH_EVENT_SLIDE_LEFT   = 8
uint8 TOUCH_EVENT_SLIDE_RIGHT  = 9



