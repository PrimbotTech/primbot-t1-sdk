#!/usr/bin/env python3
# 指定Python解释器路径

# 导入ROS2 Python客户端库
import rclpy
# 导入ROS2节点基类
from rclpy.node import Node
# 导入传感器数据QoS配置
from rclpy.qos import qos_profile_sensor_data

# 导入PMU状态消息类型
from aimdk_msgs.msg import PmuState


# PMU状态监听节点类
class PmuStateEcho(Node):
    # 构造函数：初始化节点
    def __init__(self):
        # 调用父类构造函数，设置节点名称为"get_pmu_state"
        super().__init__("get_pmu_state")

        # 创建订阅器，订阅PMU状态话题
        self.subscription = self.create_subscription(
            PmuState,                              # 消息类型：PMU状态
            "/aima/hal/pmu/state",                 # 话题名称
            self.callback,                         # 回调函数
            qos_profile_sensor_data,               # QoS配置：传感器数据类型
        )

        # 打印日志：订阅成功提示
        self.get_logger().info("Subscribing pmu topic: /aima/hal/pmu/state")

    # 静态方法：格式化数值为3位小数
    @staticmethod
    def _fmt3(value) -> str:
        # 将值转换为浮点数并保留3位小数
        return f"{float(value):.3f}"

    # 回调函数：处理接收到的PMU状态消息
    def callback(self, msg: PmuState):
        # 将电源供应和电池字段分组输出
        lines = [
            "PmuState received",                                                          # 标题：接收到PMU状态
            f"  pmu_protocol_version: {msg.pmu_protocol_version}",                        # PMU协议版本
            f"  pmu_hardware_version: {msg.pmu_hardware_version}",                        # PMU硬件版本
            f"  pmu_software_version: {msg.pmu_software_version}",                        # PMU软件版本
            f"  pmu_bool_status: {msg.pmu_bool_status}",                                  # PMU布尔状态标志
            f"  soc_5v_voltage:       {self._fmt3(msg.soc_5v_voltage)} V",                # SoC 5V电压
            f"  sys_5v_voltage:       {self._fmt3(msg.sys_5v_voltage)} V",                # 系统5V电压
            f"  sys_12v_voltage:      {self._fmt3(msg.sys_12v_voltage)} V",               # 系统12V电压
            f"  orin_12v_voltage:     {self._fmt3(msg.orin_12v_voltage)} V",              # Orin 12V电压
            f"  ext_12v_voltage:      {self._fmt3(msg.ext_12v_voltage)} V",               # 外部12V电压
            f"  ext_24v_voltage:      {self._fmt3(msg.ext_24v_voltage)} V",               # 外部24V电压
            f"  arm_48v_voltage:      {self._fmt3(msg.arm_48v_voltage)} V",               # 机械臂48V电压
            f"  leg_48v_voltage:      {self._fmt3(msg.leg_48v_voltage)} V",               # 腿部48V电压
            f"  soc_5v_current:       {self._fmt3(msg.soc_5v_current)} A",                # SoC 5V电流
            f"  sys_5v_current:       {self._fmt3(msg.sys_5v_current)} A",                # 系统5V电流
            f"  sys_12v_current:      {self._fmt3(msg.sys_12v_current)} A",               # 系统12V电流
            f"  orin_12v_current:     {self._fmt3(msg.orin_12v_current)} A",              # Orin 12V电流
            f"  ext_12v_current:      {self._fmt3(msg.ext_12v_current)} A",               # 外部12V电流
            f"  ext_24v_current:      {self._fmt3(msg.ext_24v_current)} A",               # 外部24V电流
            f"  arm_48v_current:      {self._fmt3(msg.arm_48v_current)} A",               # 机械臂48V电流
            f"  leg_48v_current:      {self._fmt3(msg.leg_48v_current)} A",               # 腿部48V电流
            f"  bms_manufacturer:                 {msg.bms_manufacturer}",                # BMS制造商
            f"  bms_serial_number:                {msg.bms_serial_number}",               # BMS序列号
            f"  bms_protocol_version:             {msg.bms_protocol_version}",            # BMS协议版本
            f"  bms_hardware_version:             {msg.bms_hardware_version}",            # BMS硬件版本
            f"  bms_software_version:             {msg.bms_software_version}",            # BMS软件版本
            f"  bms_status:                       {msg.bms_status}",                      # BMS状态
            f"  bms_balance_line_resistance:      {msg.bms_balance_line_resistance} mOhm",# BMS平衡线电阻
            f"  bms_voltage:                      {self._fmt3(msg.bms_voltage)} V",       # BMS电压
            f"  bms_current:                      {self._fmt3(msg.bms_current)} A",       # BMS电流
            f"  bms_power:                        {self._fmt3(msg.bms_power)} W",         # BMS功率
            f"  bms_temperature:                  {msg.bms_temperature} degC",            # BMS温度
            f"  bms_remaining_capacity:           {msg.bms_remaining_capacity} mAh",      # BMS剩余容量
            f"  bms_remaining_capacity_percentage:{int(msg.bms_remaining_capacity_percentage)} %", # BMS剩余容量百分比
            f"  bms_cycle_count:                  {msg.bms_cycle_count}",                 # BMS循环次数
            f"  bms_cycle_total_capacity:         {msg.bms_cycle_total_capacity} Ah",     # BMS循环总容量
        ]

        # 使用换行符连接所有行并打印日志
        self.get_logger().info("\n".join(lines))


# 主函数：程序入口
def main(args=None):
    # 初始化ROS2通信系统
    rclpy.init(args=args)
    # 创建PMU状态监听节点
    node = PmuStateEcho()
    try:
        # 进入事件循环，持续处理回调
        rclpy.spin(node)
        return 0
    finally:
        # 清理节点资源
        node.destroy_node()
        # 关闭ROS2通信系统
        rclpy.shutdown()


# 程序入口：当直接运行此脚本时执行
if __name__ == "__main__":
    # 调用主函数并设置系统退出码
    raise SystemExit(main())
