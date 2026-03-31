MessageHeader header        # 消息头信息

# 当前显示模式
uint8 display_mode
           
# 红色分量，自定义模式下生效
uint8 r

# 绿色分量，自定义模式下生效
uint8 g

# 蓝色分量，自定义模式下生效
uint8 b

# 周期，自定义模式下生效。单位ms，0表示常亮，非0值表示点亮或者熄灭的保持时间
uint16 period