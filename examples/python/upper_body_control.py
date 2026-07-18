#!/usr/bin/env python3

"""
T1 Custom Upper Control Example

Description:
  Demonstrates how to publish upper-body joint commands to
  /aima/mc/custom/joint/command with aimdk_msgs/msg/McCustomJointCommand.

Prerequisites:
  - MC must stay running. Do not disable the robot motion control module.
  - Robot must be in a safe environment for motion testing.
  - State machine will auto-transition to BIPED_CUSTOM_UPPER before publishing commands.

Usage:
  python3 examples/python/upper_body_control.py
"""

import math
import signal
import time
from typing import Dict

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

from aimdk_msgs.msg import (
    JointCommand,
    McCustomJointCommand,
)

from common.mc_action_switcher import McActionSwitchOptions, McActionSwitcher


PUBLISH_RATE_HZ = 100.0
DEFAULT_STIFFNESS = 20.0
DEFAULT_DAMPING = 3.0

JOINT_NAMES = [
    "FL_HIP_ROLL_Joint",
    "FL_HIP_PITCH_Joint",
    "FL_KNEE_Joint",
    "FR_HIP_ROLL_Joint",
    "FR_HIP_PITCH_Joint",
    "FR_KNEE_Joint",
]

DEFAULT_STAND_POSITIONS = [
    -0.0031864201078528787,
    1.5550110086252016,
    0.5175245036476512,
    0.003206134017743055,
    1.554158788692746,
    0.5169196042010172,
]

g_stop = False


def signal_handler(_signum, _frame) -> None:
    global g_stop
    g_stop = True


def smooth_step(value: float) -> float:
    value = max(0.0, min(1.0, value))
    return 0.5 - 0.5 * math.cos(math.pi * value)


class CustomUpperControlNode(Node):
    def __init__(self) -> None:
        super().__init__("custom_upper_control")

        self.qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )

        self.command_pub = self.create_publisher(
            McCustomJointCommand, "/aima/mc/custom/joint/command", self.qos
        )

        self.current_demo_pose = self.default_stand_positions()
        self.sequence = 0

        self.get_logger().info(
            "custom_upper_control started. Make sure MC is running. The robot "
            "will auto-transition to BIPED_CUSTOM_UPPER before publishing commands."
        )

    def switch_to_custom_upper(self) -> bool:
        switcher = McActionSwitcher(self)
        options = McActionSwitchOptions(
            source="upper_body_control",
            total_timeout=30.0,
            should_cancel=lambda: g_stop,
        )
        result = switcher.switch_to("BIPED_CUSTOM_UPPER", options)
        if not result.success:
            current_action = result.current_action or "(unknown)"
            self.get_logger().error(
                "Failed to switch from "
                f"{current_action} to BIPED_CUSTOM_UPPER: {result.message}"
            )
        return result.success

    def default_stand_positions(self) -> Dict[str, float]:
        return dict(zip(JOINT_NAMES, DEFAULT_STAND_POSITIONS))

    def interpolate_pose(
        self, start_pose: Dict[str, float], target_pose: Dict[str, float], alpha: float
    ) -> Dict[str, float]:
        ratio = smooth_step(alpha)
        return {
            name: start_pose[name] + (target_pose[name] - start_pose[name]) * ratio
            for name in JOINT_NAMES
        }

    def wave_pose(self, base_pose: Dict[str, float], t: float) -> Dict[str, float]:
        pose = dict(base_pose)
        pose["FR_HIP_ROLL_Joint"] = -0.18
        pose["FR_HIP_PITCH_Joint"] = 1.9
        pose["FR_KNEE_Joint"] = 0.80 + 0.18 * math.sin(2.0 * math.pi * 1.5 * t)
        return pose

    def publish_pose(self, pose: Dict[str, float]) -> None:
        msg = McCustomJointCommand()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.sequence = self.sequence
        self.sequence += 1

        for name in JOINT_NAMES:
            joint = JointCommand()
            joint.name = name
            joint.position = pose[name]
            joint.velocity = 0.0
            joint.effort = 0.0
            joint.stiffness = DEFAULT_STIFFNESS
            joint.damping = DEFAULT_DAMPING
            msg.joints.append(joint)

        self.command_pub.publish(msg)

    def sleep_to_next_tick(self, next_tick: float) -> float:
        now = time.perf_counter()
        if next_tick > now:
            time.sleep(next_tick - now)
        return next_tick + 1.0 / PUBLISH_RATE_HZ

    def publish_interpolation(
        self, start_pose: Dict[str, float], target_pose: Dict[str, float], duration_s: float
    ) -> bool:
        total_steps = max(1, int(duration_s * PUBLISH_RATE_HZ))
        next_tick = time.perf_counter()
        for step in range(total_steps + 1):
            if not rclpy.ok() or g_stop:
                return False
            alpha = step / total_steps
            self.current_demo_pose = self.interpolate_pose(start_pose, target_pose, alpha)
            self.publish_pose(self.current_demo_pose)
            next_tick = self.sleep_to_next_tick(next_tick)
        return True

    def publish_wave_motion(self, base_pose: Dict[str, float], duration_s: float) -> bool:
        total_steps = max(1, int(duration_s * PUBLISH_RATE_HZ))
        next_tick = time.perf_counter()
        for step in range(total_steps + 1):
            if not rclpy.ok() or g_stop:
                return False
            t = step / total_steps
            self.current_demo_pose = self.wave_pose(base_pose, t)
            self.publish_pose(self.current_demo_pose)
            next_tick = self.sleep_to_next_tick(next_tick)
        return True

    def publish_hold(self, pose: Dict[str, float], duration_s: float) -> bool:
        total_steps = max(1, int(duration_s * PUBLISH_RATE_HZ))
        next_tick = time.perf_counter()
        for _ in range(total_steps):
            if not rclpy.ok():
                return False
            self.publish_pose(pose)
            next_tick = self.sleep_to_next_tick(next_tick)
        return True

    def run_demo(self) -> bool:
        stand_pose = self.default_stand_positions()

        self.get_logger().info("Moving to default upper posture.")
        if not self.publish_interpolation(self.current_demo_pose, stand_pose, 1.0):
            return False

        wave_start = self.wave_pose(stand_pose, 0.0)
        self.get_logger().info("Extending right upper limb smoothly.")
        if not self.publish_interpolation(stand_pose, wave_start, 2.0):
            return False

        self.get_logger().info("Running a small upper-body demo motion.")
        if not self.publish_wave_motion(stand_pose, 4.0):
            return False

        self.get_logger().info("Returning to default upper posture.")
        if not self.publish_interpolation(self.current_demo_pose, stand_pose, 2.0):
            return False

        self.get_logger().info("Holding default upper posture before exit.")
        return self.publish_hold(stand_pose, 1.0)

    def publish_default_for_shutdown(self) -> None:
        self.publish_hold(self.default_stand_positions(), 1.0)


def main() -> int:
    rclpy.init()
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

    node = CustomUpperControlNode()
    ret = 0

    try:
        if not node.switch_to_custom_upper():
            ret = 1
        else:
            time.sleep(1)
            if not g_stop and not node.run_demo():
                ret = 0 if g_stop else 1

            if rclpy.ok():
                node.publish_default_for_shutdown()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    return ret


if __name__ == "__main__":
    raise SystemExit(main())
