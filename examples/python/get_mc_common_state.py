#!/usr/bin/env python3

"""
MC Common State Monitor Example Script

Description:
  This script demonstrates how to subscribe to the robot's MC common state topic.
  It displays real-time motion control status including input source, action info,
  form state, FSM state, posture, motion status, and more.

Prerequisites:
  - Robot MC service must be running

Usage:
  python3 get_mc_common_state.py

Example:
  python3 get_mc_common_state.py

Parameters:
  - None
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from aimdk_msgs.msg import McCommonState


# ---------- 枚举映射 ----------

FORM_STATE_MAP = {0: "UNKNOWN", 1: "QUADRUPEDAL", 2: "BIPEDAL"}

FSM_STATE_MAP = {
    0: "UNKNOWN",
    1: "STARTING",
    2: "STABLE",
    3: "MOVING",
    4: "SAFE",
    5: "SPECIAL",
    6: "TEST",
}

PLAYER_STATE_MAP = {
    0: "IDLE",
    1: "PRE_PLAYING",
    2: "PLAYING",
    3: "INTERRUPTING",
    4: "ERROR",
}

MOTION_TYPE_MAP = {
    0: "NONE",
    1: "ANIMATION",
    2: "MIMIC",
    3: "FOUNDATION",
    4: "MIMIC_OPENSOURCE",
}

INTR_TYPE_MAP = {0: "INTR_NO", 1: "INTR_FREE", 2: "INTR_BREAKPOINT"}


class McCommonStateEcho(Node):
    def __init__(self):
        super().__init__("get_mc_common_state")

        self.subscription = self.create_subscription(
            McCommonState,
            "/aima/mc/base/state",
            self.callback,
            qos_profile_sensor_data,
        )

        self.get_logger().info(
            "Subscribing MC common state topic: /aima/mc/base/state"
        )

    @staticmethod
    def _to_seconds(stamp) -> float:
        return float(stamp.sec) + float(stamp.nanosec) / 1_000_000_000.0

    def callback(self, msg: McCommonState):
        lines = [
            "======== McCommonState ========",
            f"  stamp:           {self._to_seconds(msg.header.stamp):.6f} s",
            # --- 输入源 ---
            f"  [InputSource]",
            f"    name:          {msg.input_source.name}",
            f"    priority:      {msg.input_source.priority}",
            f"    timeout:       {msg.input_source.timeout} ms",
            # --- Action ---
            f"  [ActionInfo]",
            f"    action:        {msg.action_info.current_action.value}",
            f"    desc:          {msg.action_info.action_desc}",
            f"    status:        {msg.action_info.status.value}",
            f"    switch_ready:  {msg.action_info.switch_ready}",
            # --- 形态 ---
            f"  [FormState]",
            f"    form:          {FORM_STATE_MAP.get(msg.form_state.current_form, msg.form_state.current_form)}",
            # --- FSM ---
            f"  [FsmState]",
            f"    state:         {FSM_STATE_MAP.get(msg.fsm_state.current_state, msg.fsm_state.current_state)}",
            # --- 姿态 ---
            f"  [Posture]",
            f"    roll:          {msg.posture.roll:.4f} rad",
            f"    pitch:         {msg.posture.pitch:.4f} rad",
            f"    yaw:           {msg.posture.yaw:.4f} rad",
            # --- 运动状态 ---
            f"  is_moving:       {msg.is_moving}",
            f"  is_fallen:       {msg.is_fallen}",
            # --- 动作运行 ---
            f"  [MotionStatus]",
            f"    player:        {PLAYER_STATE_MAP.get(msg.motion_status.player_state.value, msg.motion_status.player_state.value)}",
            f"    motion:        {msg.motion_status.motion}",
            f"    type:          {MOTION_TYPE_MAP.get(msg.motion_status.type.value, msg.motion_status.type.value)}",
            f"    intr_type:     {INTR_TYPE_MAP.get(msg.motion_status.intr_info.intr_type.value, msg.motion_status.intr_info.intr_type.value)}",
            f"    steps:         {msg.motion_status.current_steps}",
            # --- 提起检测 ---
            f"  [LiftState]",
            f"    is_lifted:     {msg.lift_state.is_lifted}",
            f"    probability:   {msg.lift_state.probability:.4f}",
            # --- 任务 ---
            f"  [TaskInfo]",
            f"    task_id:       {msg.task_info.task_id}",
        ]

        self.get_logger().info("\n".join(lines))


def main(args=None):
    rclpy.init(args=args)
    node = McCommonStateEcho()
    try:
        rclpy.spin(node)
        return 0
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
