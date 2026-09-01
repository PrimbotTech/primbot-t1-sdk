MessageHeader header        # 数据头

# 版本信息
string manufacturer         # 厂家信息
string serial_number        # 序列号
string protocol_version     # 协议版本
string hardware_version     # 硬件版本
string software_version     # 软件版本

# 电源状态
uint16 power_request
# 1-触发长按—关机请求
# 2-请求短按+长按关机请求

# 状态位和状态码
uint32 status
# bit 0:  充电标志
# bit 1:  充电过流标志
# bit 2:  放电标志
# bit 3:  放电过流标志
# bit 4:  电池短路标志
# bit 5:  电芯电压过压标志
# bit 6:  电芯电压欠压标志
# bit 7:  电池总电压过压标志
# bit 8:  电池总电压欠压标志
# bit 9:  预留：电芯检测到开路标志
# bit 10: 温感传感器检测到开路标志
# bit 11: 电芯温度超过放电温度上限标志
# bit 12: 电芯温度超过充电温度上限标志
# bit 13: 电芯温度低于放电温度上限标志
# bit 14: 电芯温度低于充电温度上限标志
# bit 15: 电芯最大压差超过上限标志
# bit 16: MOSFET禁止充电标志（表示真正故障禁用）
# bit 17: MOSFET禁止放电标志（表示真正故障禁用）
# bit 18: MOSFET此温度超过工作温度上限标志
# bit 19: 电池包均衡线电阻超过上限标志
# bit 20: 电池充满状态
# bit 21: 电池包固件版本，0是老固件，1新固件
# bit 22: 电池座key是否短接，0短接，1断开
# bit 23: 电池包异常，0正常，1异常
# bit 24: 预留：二级过温警告，0正常，1异常
# bit 25: 有线充电器插入Key是否短接，1短接，0断开
# bit 26: 无线充电器介入状态，0空闲，1介入
# bit 27: 正常关机触发状态
# bit 28: 长按关机触发状态
# bit 29~31: 保留

uint16 balance_line_resistance      # 电池包均衡线电阻（单位：mΩ）
uint32 voltage                      # 电池包电压（单位：mV）
int32 current                       # 电池包电流（单位：mA，充电为正，放电为负）
int32 power                         # 电池包输出功率（单位：mW）
float32 temperature                  # 电池包当前温度（单位：℃）
uint32 remaining_capacity           # 当前电池剩余容量（单位：mAh）
uint8 remaining_capacity_percentage # 当前电池剩余容量百分比（单位：％）
uint16 cycle_count                  # 当前电池包完整循环次数（单位：次）
uint32 cycle_total_capacity         # 总充放电容量累计（单位：mAh）
