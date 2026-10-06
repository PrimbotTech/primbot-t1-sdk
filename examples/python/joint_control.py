# WARNING: Before running this script, you MUST disable the robot's built-in motion
# control module and place the robot flat on the ground or suspend it.
# Otherwise, unpredictable behavior may occur.
#!/usr/bin/env python3

"""Joint Position Control Example Script

Description:
  This script demonstrates how to control robot joint positions using the Ruckig trajectory
  planner. Publishes trajectory commands to /aima/hal/joint/command at 500Hz (default) and
  monitors joint state via /aima/hal/joint/state. Supports configurable joint names, target
  positions, stiffness, and damping.

Prerequisites:
  - Robot must be placed flat on the ground or suspended
  - Robot motion control module must be disabled
  - SDK must be built: colcon build
  - Environment must be sourced: source install/setup.bash
  - Ruckig library must be available (ruckig_for_primebot)

Usage:
  # List available joints (no parameters needed):
  python3 joint_control.py

  # Control specific joints:
  python3 joint_control.py --ros-args -p joint_names:=<names> -p target_positions:=<positions>

T1 Available Joints:
  Left  Arm : FL_HIP_ROLL_Joint  [-0.47, 0.47], FL_HIP_PITCH_Joint [-1.22, 4.36], FL_KNEE_Joint [-2.62, 2.62]
  Right Arm : FR_HIP_ROLL_Joint  [-0.47, 0.47], FR_HIP_PITCH_Joint [-1.22, 4.36], FR_KNEE_Joint [-2.62, 2.62]
  Left  Leg : RL_HIP_ROLL_Joint  [-1.57, 1.57], RL_HIP_PITCH_Joint [0.00, 3.14], RL_KNEE_Joint [-2.62, -0.17], RL_FOOT_Joint [-10.0, 10.0]
  Right Leg : RR_HIP_ROLL_Joint  [-1.57, 1.57], RR_HIP_PITCH_Joint [0.00, 3.14], RR_KNEE_Joint [-2.62, -0.17], RR_FOOT_Joint [-10.0, 10.0]
  (Position unit: rad)

Parameters:
  - joint_names: String array of joint names to control (required for control; omit to list joints)
  - target_positions: Double array of target positions (same length as joint_names)
  - default_stiffness: Default stiffness value for all joints (default: 20.0)
  - default_damping: Default damping value for all joints (default: 2.0)
  - max_velocity: Ruckig max velocity limit in rad/s (default: 3.0)
  - max_acceleration: Ruckig max acceleration limit in rad/s^2 (default: 10.0)
  - max_jerk: Ruckig max jerk limit in rad/s^3 (default: 25.0)
  - control_period_s: Control loop period in seconds (default: 0.002 = 500Hz, recommended 500-1000Hz)
  - state_timeout_s: Timeout waiting for joint state in seconds (default: 10.0)
  - publish_full_command: Publish all joints (true) or target joints only (false) (default: true)

Notes:
  - joint_names and target_positions must have the same length.
  - Running without joint_names will list available joints from /aima/hal/joint/state and exit.
  - The control frequency must be 500Hz-1000Hz; below 100Hz triggers hardware watchdog.
  - stiffness and damping are optional. If omitted, default values are used.
  - publish_full_command=true is recommended to prevent unintended motion of non-target joints.
  - On exit, the script sends stiffness=0, damping=5.0 (damping protection mode).

Example:
  python3 joint_control.py --ros-args -p joint_names:="['FL_HIP_PITCH_Joint']" -p target_positions:="[2.0]"
"""

import signal
import time
from typing import List, Optional

import rclpy
import rclpy.logging
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

from aimdk_msgs.msg import JointCommand, JointCommandArray, JointStateArray
from ruckig_for_primebot import InputParameter, OutputParameter, Result, Ruckig


g_node: Optional["JointControlNode"] = None


def signal_handler(signum, _frame) -> None:
    if g_node is not None:
        g_node.get_logger().info(
            f"Received signal {signum}, shutting down joint_control..."
        )
    if rclpy.ok():
        rclpy.shutdown()
    raise SystemExit(signum)


class JointControlNode(Node):
    def __init__(self) -> None:
        super().__init__("joint_control")

        self.declare_parameter("joint_names", Parameter.Type.STRING_ARRAY)
        self.declare_parameter("target_positions", Parameter.Type.DOUBLE_ARRAY)
        self.declare_parameter("stiffness", Parameter.Type.DOUBLE_ARRAY)
        self.declare_parameter("damping", Parameter.Type.DOUBLE_ARRAY)
        self.default_stiffness = self.declare_parameter(
            "default_stiffness", 20.0
        ).value
        self.default_damping = self.declare_parameter("default_damping", 2.0).value
        self.max_velocity = self.declare_parameter("max_velocity", 3.0).value
        self.max_acceleration = self.declare_parameter(
            "max_acceleration", 10.0
        ).value
        self.max_jerk = self.declare_parameter("max_jerk", 25.0).value
        self.control_period_s = self.declare_parameter("control_period_s", 0.002).value
        self.state_timeout_s = self.declare_parameter("state_timeout_s", 10.0).value
        self.publish_full_command = self.declare_parameter(
            "publish_full_command", True
        ).value

        self.joint_names = list(
            self.get_parameter_or(
                "joint_names",
                Parameter("joint_names", Parameter.Type.STRING_ARRAY, []),
            ).value
        )
        self.target_positions = list(
            self.get_parameter_or(
                "target_positions",
                Parameter("target_positions", Parameter.Type.DOUBLE_ARRAY, []),
            ).value
        )
        self.stiffness = list(
            self.get_parameter_or(
                "stiffness",
                Parameter("stiffness", Parameter.Type.DOUBLE_ARRAY, []),
            ).value
        )
        self.damping = list(
            self.get_parameter_or(
                "damping",
                Parameter("damping", Parameter.Type.DOUBLE_ARRAY, []),
            ).value
        )

        self.validate_parameters()

        self.dofs = len(self.joint_names)
        self.fill_optional_gains()

        self.ruckig = Ruckig(self.dofs, self.control_period_s)
        self.input = InputParameter(self.dofs)
        self.output = OutputParameter(self.dofs)

        self.input.max_velocity = [self.max_velocity] * self.dofs
        self.input.max_acceleration = [self.max_acceleration] * self.dofs
        self.input.max_jerk = [self.max_jerk] * self.dofs
        self.input.current_acceleration = [0.0] * self.dofs
        self.input.target_velocity = [0.0] * self.dofs
        self.input.target_acceleration = [0.0] * self.dofs

        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )

        self.command_pub = self.create_publisher(
            JointCommandArray, "/aima/hal/joint/command", qos
        )
        self.state_sub = self.create_subscription(
            JointStateArray,
            "/aima/hal/joint/state",
            self.on_joint_state,
            qos,
        )
        # Timer to print joint state every 10 seconds
        self.print_timer = self.create_timer(10.0, self.print_joint_state)

        self.latest_state: Optional[JointStateArray] = None
        self.target_full_indices: List[int] = []

        self.trajectory_started = False
        self.finished = False
        self.failed = False
        self.sequence = 0

        self.cycle_count = 0
        self.overrun_count = 0
        self.total_cycle_cost_s = 0.0
        self.max_cycle_cost_s = 0.0
        self.total_overrun_s = 0.0
        self.max_overrun_s = 0.0

        self.wait_started_at = time.perf_counter()
        self.control_started_at = 0.0

        self.state_timeout_timer = self.create_timer(0.1, self.check_state_timeout)
        self.control_timer = None

        self.get_logger().info(
            "Waiting for /aima/hal/joint/state, "
            f"joints={self.format_strings(self.joint_names)}, "
            f"targets={self.format_doubles(self.target_positions)}, "
            f"control_period_s={self.control_period_s:.3f}, "
            f"state_timeout_s={self.state_timeout_s:.3f}, "
            f"publish_full_command={'true' if self.publish_full_command else 'false'}"
        )

    def validate_parameters(self) -> None:
        if not self.joint_names:
            raise ValueError("Parameter 'joint_names' must not be empty.")
        if not self.target_positions:
            raise ValueError("Parameter 'target_positions' must not be empty.")
        if len(self.joint_names) != len(self.target_positions):
            raise ValueError(
                "Parameter 'joint_names' and 'target_positions' must have the same length."
            )
        if self.stiffness and len(self.stiffness) != len(self.joint_names):
            raise ValueError(
                "Parameter 'stiffness' must be empty or match joint_names length."
            )
        if self.damping and len(self.damping) != len(self.joint_names):
            raise ValueError(
                "Parameter 'damping' must be empty or match joint_names length."
            )
        if self.control_period_s <= 0.0:
            raise ValueError("Parameter 'control_period_s' must be greater than zero.")
        if self.state_timeout_s <= 0.0:
            raise ValueError("Parameter 'state_timeout_s' must be greater than zero.")
        if (
            self.max_velocity <= 0.0
            or self.max_acceleration <= 0.0
            or self.max_jerk <= 0.0
        ):
            raise ValueError(
                "Motion limits 'max_velocity', 'max_acceleration', and 'max_jerk' "
                "must be greater than zero."
            )

    def fill_optional_gains(self) -> None:
        if not self.stiffness:
            self.stiffness = [self.default_stiffness] * self.dofs
        if not self.damping:
            self.damping = [self.default_damping] * self.dofs

    def on_joint_state(self, msg: JointStateArray) -> None:
        self.latest_state = msg
        if self.trajectory_started or self.finished:
            return
        self.start_trajectory()

    def print_joint_state(self) -> None:
        """Print the latest joint state every 10 seconds after it is received."""
        if self.latest_state is None:
            self.get_logger().warning("No joint state received yet – waiting for /aima/hal/joint/state.")
            return
        lines = []
        for joint in self.latest_state.joints:
            lines.append(f"{joint.name}: pos={joint.position:.3f}, vel={joint.velocity:.3f}")
        self.get_logger().info("Current joint state (every 10 s): " + ", ".join(lines))

    def start_trajectory(self) -> None:
        if self.latest_state is None:
            return

        state_index_by_name = {
            joint.name: idx for idx, joint in enumerate(self.latest_state.joints)
        }
        missing_state_joints: List[str] = []
        current_positions = [0.0] * self.dofs
        current_velocities = [0.0] * self.dofs
        self.target_full_indices = [0] * self.dofs

        for i, joint_name in enumerate(self.joint_names):
            state_index = state_index_by_name.get(joint_name)
            if state_index is None:
                missing_state_joints.append(joint_name)
                continue

            joint_state = self.latest_state.joints[state_index]
            self.target_full_indices[i] = state_index
            current_positions[i] = float(joint_state.position)
            current_velocities[i] = float(joint_state.velocity)

        if missing_state_joints:
            self.get_logger().error(
                "Missing joints in /aima/hal/joint/state: "
                f"{self.format_strings(missing_state_joints)}"
            )
            self.shutdown_with_error()
            return

        self.input.current_position = current_positions
        self.input.current_velocity = current_velocities
        self.input.current_acceleration = [0.0] * self.dofs
        self.input.target_position = self.target_positions
        self.input.target_velocity = [0.0] * self.dofs
        self.input.target_acceleration = [0.0] * self.dofs

        self.control_started_at = time.perf_counter()
        self.trajectory_started = True

        if self.state_timeout_timer is not None:
            self.state_timeout_timer.cancel()

        self.control_timer = self.create_timer(
            self.control_period_s, self.on_control_timer
        )

        self.get_logger().info(
            "Trajectory initialized from "
            f"current={self.format_doubles(current_positions)} "
            f"to target={self.format_doubles(self.target_positions)}, "
            f"stiffness={self.format_doubles(self.stiffness)}, "
            f"damping={self.format_doubles(self.damping)}"
        )

    def on_control_timer(self) -> None:
        if self.finished:
            return

        cycle_start = time.perf_counter()
        result = self.ruckig.update(self.input, self.output)

        if result not in (Result.Working, Result.Finished):
            self.get_logger().error(
                f"Ruckig trajectory planning failed with result={result}"
            )
            self.shutdown_with_error()
            return

        self.publish_command(list(self.output.new_position))
        self.output.pass_to_input(self.input)

        cycle_cost_s = time.perf_counter() - cycle_start
        self.cycle_count += 1
        self.total_cycle_cost_s += cycle_cost_s
        self.max_cycle_cost_s = max(self.max_cycle_cost_s, cycle_cost_s)

        if cycle_cost_s > self.control_period_s:
            overrun_s = cycle_cost_s - self.control_period_s
            self.overrun_count += 1
            self.total_overrun_s += overrun_s
            self.max_overrun_s = max(self.max_overrun_s, overrun_s)

        if result == Result.Finished:
            self.publish_command(self.target_positions)
            self.log_stats_and_shutdown()

    def publish_command(self, positions: List[float]) -> None:
        cmd = JointCommandArray()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.header.sequence = self.sequence
        self.sequence += 1

        if self.latest_state is not None:
            cmd.header.frame_id = self.latest_state.header.frame_id
            cmd.header.meas_stamp = self.latest_state.header.meas_stamp

        if (
            self.publish_full_command
            and self.latest_state is not None
            and len(self.target_full_indices) == self.dofs
        ):
            for state_joint in self.latest_state.joints:
                joint = JointCommand()
                joint.name = state_joint.name
                joint.position = state_joint.position
                joint.velocity = 0.0
                joint.effort = 0.0
                joint.stiffness = self.default_stiffness
                joint.damping = self.default_damping
                cmd.joints.append(joint)

            for i, full_index in enumerate(self.target_full_indices):
                if full_index >= len(cmd.joints):
                    continue

                joint = cmd.joints[full_index]
                joint.position = float(positions[i])
                joint.stiffness = float(self.stiffness[i])
                joint.damping = float(self.damping[i])
        else:
            for i, joint_name in enumerate(self.joint_names):
                joint = JointCommand()
                joint.name = joint_name
                joint.position = float(positions[i])
                joint.velocity = 0.0
                joint.effort = 0.0
                joint.stiffness = float(self.stiffness[i])
                joint.damping = float(self.damping[i])
                cmd.joints.append(joint)

        self.command_pub.publish(cmd)

    def check_state_timeout(self) -> None:
        if self.trajectory_started or self.finished:
            return

        wait_s = time.perf_counter() - self.wait_started_at
        if wait_s > self.state_timeout_s:
            self.get_logger().error(
                "Timed out after "
                f"{wait_s:.3f} s waiting for /aima/hal/joint/state"
            )
            self.shutdown_with_error()

    def shutdown_with_error(self) -> None:
        if self.finished:
            return

        self.finished = True
        self.failed = True
        if self.control_timer is not None:
            self.control_timer.cancel()
        if self.state_timeout_timer is not None:
            self.state_timeout_timer.cancel()

    def log_stats_and_shutdown(self) -> None:
        if self.finished:
            return

        self.finished = True
        if self.control_timer is not None:
            self.control_timer.cancel()
        if self.state_timeout_timer is not None:
            self.state_timeout_timer.cancel()

        runtime_s = time.perf_counter() - self.control_started_at
        actual_hz = self.cycle_count / runtime_s if runtime_s > 0.0 else 0.0
        avg_cycle_ms = (
            self.total_cycle_cost_s / self.cycle_count * 1000.0
            if self.cycle_count > 0
            else 0.0
        )
        avg_overrun_ms = (
            self.total_overrun_s / self.overrun_count * 1000.0
            if self.overrun_count > 0
            else 0.0
        )
        trajectory_duration_s = float(self.output.trajectory.duration)

        self.get_logger().info(
            "Trajectory completed. "
            f"planned_duration={trajectory_duration_s:.6f} s, "
            f"runtime={runtime_s:.6f} s, "
            f"cycles={self.cycle_count}, "
            f"actual_hz={actual_hz:.2f}, "
            f"avg_cycle_ms={avg_cycle_ms:.3f}, "
            f"max_cycle_ms={self.max_cycle_cost_s * 1000.0:.3f}, "
            f"overruns={self.overrun_count}, "
            f"avg_overrun_ms={avg_overrun_ms:.3f}, "
            f"max_overrun_ms={self.max_overrun_s * 1000.0:.3f}"
        )

    @staticmethod
    def format_strings(values: List[str]) -> str:
        return "[" + ", ".join(values) + "]"

    @staticmethod
    def format_doubles(values: List[float]) -> str:
        return "[" + ", ".join(f"{value:.6f}" for value in values) + "]"


def main(args=None) -> int:
    global g_node

    rclpy.init(args=args)
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

    node = None
    try:
        node = JointControlNode()
        g_node = node
        while rclpy.ok() and not node.finished:
            rclpy.spin_once(node, timeout_sec=0.1)
        g_node = None
        return 1 if node.failed else 0
    except Exception as error:  # noqa: BLE001
        rclpy.logging.get_logger("joint_control").error(
            f"Failed to start joint_control: {error}"
        )
        if rclpy.ok():
            rclpy.shutdown()
        return 1
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
