#!/usr/bin/env python3

"""
Action Switch Example Script

Description:
  This script demonstrates how to call the SetMcAction service to switch robot action
  states. Uses BFS on ACTION_GRAPH to automatically navigate the state machine
  transition path from the current action to the target action.

Prerequisites:
  - Robot motion control service must be running
  - SetMcAction and GetMcAction services must be available

Usage:
  python3 set_mc_action.py --ros-args -p type:=action [-p action_desc:=<ACTION>]

Example:
  # Interactive mode: input target action via terminal
  python3 set_mc_action.py --ros-args -p type:=action

  # Non-interactive mode: specify action via parameter
  python3 set_mc_action.py --ros-args -p type:=action -p \
  action_desc:=QUADRUPED_LOCOMOTION_JUMP

Parameters:
  - type: "action" or "motion", required (motion currently disabled)
  - action_desc: string, optional; if set, skips interactive input and executes directly
  - motion: string, required when type=motion
  - interrupt: bool, optional when type=motion, default=true

Notes:
  - Single Switch: Switches to target action, holds for 2s, then auto-returns to QUADRUPED_LOCOMOTION_DEFAULT.
  - Auto Path: BFS finds shortest transition path on ACTION_GRAPH, skipping redundant steps.
  - Recovery: Handles DAMPING_DEFAULT by recovering to PASSIVE_DEFAULT first.
  - Retry: Each path step retries up to 5 times if the robot is still moving.
  - Ctrl+C Safety: During execution, Ctrl+C navigates back to QUADRUPED_LOCOMOTION_DEFAULT before exit.
  - WARNING: Do NOT use QUADRUPED_LOCOMOTION_JUMP for testing — the robot will jump and may cause injury or damage.
"""

import signal
import time
from collections import deque

import rclpy
import rclpy.logging
from rclpy.node import Node

from aimdk_msgs.msg import CommonRequest, CommonState, McAction, McActionCommand, McActionStatus, RequestHeader
from aimdk_msgs.srv import GetMcAction, SetMcAction, SetMcMotion

SERVICE_CALL_TIMEOUT_SEC = 2.0
MAX_RETRY_COUNT = 3

# CommonState reason 字段对应的中文描述
REASON_DESCRIPTIONS = {
    0: '无错误',
    1: '开箱状态中',
    2: '开机自检中',
    3: '关机状态中',
    4: '当前形态不支持',
    5: '低电量限制',
    6: '正在充电中',
    7: '动作不在白名单',
    8: 'HDS故障',
    9: '当前模式不支持',
    10: '前方有障碍物',
    11: '后方有障碍物',
    12: '左方有障碍物',
    13: '右方有障碍物',
    14: '上方有障碍物',
    15: '其他任务正在运行',
    16: '机器人已经是目标状态'
}

def get_reason_description(reason: int) -> str:
    """获取失败原因的中文描述"""
    return REASON_DESCRIPTIONS.get(reason, f'未知原因({reason})')

# T1狗形状态机有向图（基于 qd1_t1d5/action_ruler.yaml 及状态机流程图）
# 定义状态间的合法转换边，navigate_to_action 使用 BFS 自动寻路
# 所有四足目标状态统一经过 QUADRUPED_LOCOMOTION_DEFAULT 后跳转
# 所有双足目标状态统一经过 BIPED_LOCOMOTION_WBC 后跳转
ACTION_GRAPH = {
    'PASSIVE_DEFAULT': [
        'QUADRUPED_STAND_DEFAULT',
        'QUADRUPED_GET_DOWN_DEFAULT',
        'QUADRUPED_SIT_DOWN_DEFAULT',
        'QUADRUPED_RECOVERY',
        'DAMPING_DEFAULT',
    ],
    'QUADRUPED_STAND_DEFAULT': [
        'QUADRUPED_LOCOMOTION_DEFAULT',
        'QUADRUPED_GET_DOWN_DEFAULT',
        'QUADRUPED_SIT_DOWN_DEFAULT',
        'QUADRUPED_LOCOMOTION_TERRAIN',
        'QUADRUPED_LOCOMOTION_RUN',
    ],
    'QUADRUPED_LOCOMOTION_DEFAULT': [
        'QUADRUPED_LOCOMOTION_TERRAIN',
        'QUADRUPED_LOCOMOTION_RUN',
        'QUADRUPED_LOCOMOTION_BIONIC',
        'QUADRUPED_LOCOMOTION_PRIDE',
        'QUADRUPED_LOCOMOTION_PLEASURE',
        'QUADRUPED_LOCOMOTION_JUMP',
        'QUADRUPED_LOCOMOTION_HANDSHAKE',
        'QUADRUPED_LOCOMOTION_STRETCH',
        'QUADRUPED_LOCOMOTION_DANCE',
        'QUADRUPED_LOCOMOTION_FRONTFLIP',
        'QUADRUPED_LOCOMOTION_BACKFLIP',
        'QUADRUPED_TO_BIPED',
        'QUADRUPED_TO_BIPED_ROTATE',
        'QUADRUPED_TO_BIPED_FRONTFLIP',
        'QUADRUPED_STAND_DEFAULT',
    ],
    'BIPED_LOCOMOTION_WBC': [
        'BIPED_LOCOMOTION_DEFAULT',
        'BIPED_LOCOMOTION_TERRAIN',
        'BIPED_LOCOMOTION_RUN',
        'BIPED_TO_QUADRUPED',
        'BIPED_TO_QUADRUPED_ROTATE',
        'BIPED_TO_QUADRUPED_FRONTFLIP',
        'BIPED_LOCOMOTION_ROTATE_LOCAL',
        'BIPED_LOCOMOTION_BACKBEND',
        'BIPED_LOCOMOTION_MOONWALK',
    ],
    'BIPED_LOCOMOTION_DEFAULT': [
        'BIPED_LOCOMOTION_WBC',
        'BIPED_LOCOMOTION_TERRAIN',
        'BIPED_LOCOMOTION_RUN',
        'BIPED_LOCOMOTION_ROTATE_LOCAL',
        'BIPED_LOCOMOTION_BACKBEND',
        'BIPED_LOCOMOTION_MOONWALK',
    ],
    # ── 四足趴下/坐下 → 回到 四足位控站立 ──
    'QUADRUPED_GET_DOWN_DEFAULT': ['QUADRUPED_STAND_DEFAULT'],
    'QUADRUPED_SIT_DOWN_DEFAULT': ['QUADRUPED_STAND_DEFAULT'],
    # ── 四足技能/基础运动 → 回到 LOCOMOTION ──
    'QUADRUPED_LOCOMOTION_TERRAIN': ['QUADRUPED_LOCOMOTION_DEFAULT'],
    'QUADRUPED_LOCOMOTION_RUN': ['QUADRUPED_LOCOMOTION_DEFAULT'],
    # ── 双足基础运动 → 回到 WBC ──
    'BIPED_LOCOMOTION_TERRAIN': ['BIPED_LOCOMOTION_WBC'],
    'BIPED_LOCOMOTION_RUN': ['BIPED_LOCOMOTION_WBC'],
    # ── 双足技能运动 → 回到 WBC ──
    'BIPED_LOCOMOTION_ROTATE_LOCAL': ['BIPED_LOCOMOTION_WBC'],
    'BIPED_LOCOMOTION_BACKBEND': ['BIPED_LOCOMOTION_WBC'],
    'BIPED_LOCOMOTION_MOONWALK': ['BIPED_LOCOMOTION_WBC'],
    # ── 自动切换边（系统自动完成，navigate_to_action 会确认状态而非重设） ──
    'QUADRUPED_RECOVERY': ['QUADRUPED_LOCOMOTION_DEFAULT'],
    'QUADRUPED_TO_BIPED': ['BIPED_LOCOMOTION_WBC'],
    'QUADRUPED_TO_BIPED_ROTATE': ['BIPED_LOCOMOTION_WBC'],
    'QUADRUPED_TO_BIPED_FRONTFLIP': ['BIPED_LOCOMOTION_WBC'],
    'BIPED_TO_QUADRUPED': ['QUADRUPED_LOCOMOTION_DEFAULT'],
    'BIPED_TO_QUADRUPED_ROTATE': ['QUADRUPED_LOCOMOTION_DEFAULT'],
    'BIPED_TO_QUADRUPED_FRONTFLIP': ['QUADRUPED_LOCOMOTION_DEFAULT'],
    'QUADRUPED_LOCOMOTION_BIONIC': ['QUADRUPED_LOCOMOTION_DEFAULT'],
    'QUADRUPED_LOCOMOTION_PRIDE': ['QUADRUPED_LOCOMOTION_DEFAULT'],
    'QUADRUPED_LOCOMOTION_PLEASURE': ['QUADRUPED_LOCOMOTION_DEFAULT'],
    'QUADRUPED_LOCOMOTION_JUMP': ['QUADRUPED_LOCOMOTION_DEFAULT'],
    'QUADRUPED_LOCOMOTION_HANDSHAKE': ['QUADRUPED_LOCOMOTION_DEFAULT'],
    'QUADRUPED_LOCOMOTION_STRETCH': ['QUADRUPED_LOCOMOTION_DEFAULT'],
    'QUADRUPED_LOCOMOTION_DANCE': ['QUADRUPED_LOCOMOTION_DEFAULT'],
    'QUADRUPED_LOCOMOTION_FRONTFLIP': ['QUADRUPED_LOCOMOTION_DEFAULT'],
    'QUADRUPED_LOCOMOTION_BACKFLIP': ['QUADRUPED_LOCOMOTION_DEFAULT'],
}

# 需要先恢复到 PASSIVE_DEFAULT 的状态（这些状态无法直接跳转到其他路径）
RECOVERY_TO_PASSIVE = {
    'DAMPING_DEFAULT': 'PASSIVE_DEFAULT',
}


class SetMcActionClient(Node):
    def __init__(self):
        super().__init__('set_mc_action_client')
        self.type = self.declare_parameter('type', '').value
        self.action_desc = self.declare_parameter('action_desc', '').value
        self.motion = self.declare_parameter('motion', '').value
        self.interrupt = self.declare_parameter('interrupt', True).value

        self.set_action_client = self.create_client(
            SetMcAction, '/aimdk_5Fmsgs/srv/SetMcAction'
        )
        self.set_motion_client = self.create_client(
            SetMcMotion, '/aimdk_5Fmsgs/srv/SetMcMotion'
        )
        self.get_client = self.create_client(
            GetMcAction, '/aimdk_5Fmsgs/srv/GetMcAction'
        )
        self.get_logger().info(
            'SetMcAction client node created with '
            f'type={self.type} action_desc={self.action_desc} '
            f'motion={self.motion} interrupt={self.interrupt}'
        )

    def call_service_with_retry(self, client, request, service_name: str, timeout_sec=None, max_retries=None):
        """带重试机制的服务调用"""
        timeout_sec = timeout_sec or SERVICE_CALL_TIMEOUT_SEC
        max_retries = max_retries or MAX_RETRY_COUNT

        for i in range(max_retries):
            future = client.call_async(request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=timeout_sec)
            if future.done():
                return future
            self.get_logger().info(f'{service_name} attempt {i+1}/{max_retries} timed out, retrying...')
            time.sleep(0.2)

        self.get_logger().error(f'{service_name} failed after {max_retries} attempts')
        return None

    def execute(self) -> bool:
        if not self.validate_parameters():
            return False
        self.wait_for_services()

        if self.type == 'action':
            try:
                _, current_desc, current_status = self.get_action_status()
                self.get_logger().info(f'Current Action is: {current_desc}')

                try:
                    target_action = (
                        self.action_desc or input(
                            f"Current Action is: {current_desc}, please input the expected Action "
                            "according to the motion control state machine transition logic in the "
                            "interface documentation. The Action you need to switch: "
                        ).strip()
                    )
                except EOFError:
                    return True
                if not target_action:
                    return True

                # Execute SetMcAction (single switch)
                ok = self.navigate_to_action(target_action, current_desc, current_status) and \
                     self.wait_for_action(target_action, timeout_sec=5.0)
                if ok:
                    self.get_logger().info(f'Target action {target_action} reached, holding for 2s...')
                    time.sleep(2.0)
                print("Switch succeeded." if ok else
                      "Switch failed, please confirm if the expected Action complies with the state machine transition logic")

                # Switch completed, navigate back to QUADRUPED_LOCOMOTION_DEFAULT
                _, current_desc, current_status = self.get_action_status()
                self.get_logger().info(f'Switch done, navigating back to QUADRUPED_LOCOMOTION_DEFAULT, from {current_desc}...')
                if self.navigate_to_action('QUADRUPED_LOCOMOTION_DEFAULT', current_desc, current_status):
                    self.wait_for_action('QUADRUPED_LOCOMOTION_DEFAULT', timeout_sec=5.0)
                return True
            except KeyboardInterrupt:
                # Ctrl+C: navigate back to QUADRUPED_LOCOMOTION_DEFAULT before exit
                _, current_desc, current_status = self.get_action_status()
                self.get_logger().info(f'Ctrl+C received, navigating back to QUADRUPED_LOCOMOTION_DEFAULT from {current_desc}...')
                if self.navigate_to_action('QUADRUPED_LOCOMOTION_DEFAULT', current_desc, current_status):
                    self.wait_for_action('QUADRUPED_LOCOMOTION_DEFAULT', timeout_sec=5.0)
                return True
        else:
            # Motion mode is currently disabled
            self.get_logger().warning("Motion mode is currently disabled.")
            return False

            # Optimized logic for 'motion' type: Ensure robot is in QUADRUPED_LOCOMOTION_DEFAULT
            _, current_desc, current_status = self.get_action_status()

            if current_desc == 'QUADRUPED_LOCOMOTION_DEFAULT' and current_status == McActionStatus.RUNNING:
                self.get_logger().info('Robot already in QUADRUPED_LOCOMOTION_DEFAULT. Proceeding to motion...')
            else:
                self.get_logger().info(f'Current state is {current_desc}. Starting state machine transition sequence...')
                sequence = ['PASSIVE_DEFAULT', 'BIPED_STAND_DEFAULT', 'QUADRUPED_LOCOMOTION_DEFAULT']
                start_index = {'PASSIVE_DEFAULT': 1, 'BIPED_STAND_DEFAULT': 2}.get(current_desc, 0)
                for i in range(start_index, len(sequence)):
                    if not self.set_action(sequence[i]) or not self.wait_for_action(sequence[i]):
                        return False

            # Execute final target motion
            if not self.set_motion(self.motion, self.interrupt):
                return False
            return self.wait_for_motion()

    def _find_path_bfs(self, start: str, target: str) -> list:
        if start == target:
            return [start]

        visited = {start}
        queue = deque([(start, [start])])

        while queue:
            current, path = queue.popleft()
            for neighbor in ACTION_GRAPH.get(current, []):
                if neighbor == target:
                    return path + [neighbor]
                if neighbor not in visited:
                    visited.add(neighbor)
                    queue.append((neighbor, path + [neighbor]))

        return []

    def navigate_to_action(self, target_action: str, current_desc: str, current_status: int) -> bool:
        if current_desc == target_action and current_status == McActionStatus.RUNNING:
            self.get_logger().info(f'Already at target action: {target_action}')
            return True

        # 处理需要先恢复到 PASSIVE_DEFAULT 的状态
        actual_current = current_desc
        if current_desc in RECOVERY_TO_PASSIVE:
            recovery_target = RECOVERY_TO_PASSIVE[current_desc]
            self.get_logger().info(f'Recovery: switching from {current_desc} to {recovery_target}...')
            if not self.set_action(recovery_target) or not self.wait_for_action(recovery_target):
                return False
            actual_current = recovery_target

        # 特殊路径：如果从 PASSIVE_DEFAULT 开始，强制使用固定路径
        if actual_current == 'PASSIVE_DEFAULT' and target_action == 'BIPED_LOCOMOTION_WBC':
            self.get_logger().info('Using fixed path from PASSIVE_DEFAULT to BIPED_LOCOMOTION_WBC')
            path = [
                'PASSIVE_DEFAULT',
                'QUADRUPED_STAND_DEFAULT',
                'QUADRUPED_LOCOMOTION_DEFAULT',
                'QUADRUPED_TO_BIPED',
                'BIPED_LOCOMOTION_WBC'
            ]
        else:
            path = self._find_path_bfs(actual_current, target_action)

        if not path:
            # 图中找不到路径：尝试直接转换（由服务端校验合法性）
            self.get_logger().info(f'No path found for {actual_current} -> {target_action}, attempting direct transition...')
            if not self.set_action(target_action):
                return False
            return self.wait_for_action(target_action, timeout_sec=5.0)

        for i in range(1, len(path)):
            target = path[i]
            self.get_logger().info(f'Path step {i}/{len(path) - 1}: Switching to {target}...')
            # 重试机制：机器人可能还在运动中，等待稳定后再试
            for attempt in range(1, 6):
                if self.set_action(target):
                    if self.wait_for_action(target):
                        break
                    return False
                if attempt < 5:
                    self.get_logger().info(f'Step {i} attempt {attempt}/5 failed, waiting for robot to stabilize...')
                    time.sleep(2.0)
                else:
                    return False
            # 等待物理动作稳定后再执行下一步
            time.sleep(1.0)

        return True

    def validate_parameters(self) -> bool:
        if not self.type:
            self.get_logger().error(
                "Parameter 'type' must be set. Use 'action' or 'motion'."
            )
            return False

        if self.type not in ('action', 'motion'):
            self.get_logger().error(
                f"Invalid parameter 'type': {self.type}. "
                "Use 'action' or 'motion'."
            )
            return False

        if self.type == 'motion' and not self.motion:
            self.get_logger().error(
                "Parameter 'motion' must be set when type=motion."
            )
            return False

        return True

    def wait_for_service(self, client, service_name: str):
        while not client.wait_for_service(timeout_sec=2.0):
            if not rclpy.ok():
                return
            self.get_logger().info(f'Service unavailable, waiting: {service_name}')
        self.get_logger().info(f'Service available: {service_name}')

    def wait_for_services(self):
        """Check only the necessary services based on the operation type."""
        self.wait_for_service(self.get_client, '/aimdk_5Fmsgs/srv/GetMcAction')
        if self.type == 'action':
            self.wait_for_service(self.set_action_client, '/aimdk_5Fmsgs/srv/SetMcAction')
        elif self.type == 'motion':
            self.wait_for_service(self.set_motion_client, '/aimdk_5Fmsgs/srv/SetMcMotion')

    def set_action(self, action_desc: str) -> bool:
        try:
            request = SetMcAction.Request()
            request.header = RequestHeader()
            request.header.stamp = self.get_clock().now().to_msg()
            request.source = "node"  # 触发源标识
            request.command = McActionCommand()
            request.command.action = McAction()
            request.command.action.value = 0
            request.command.action_desc = action_desc

            self.get_logger().info(f'Sending request: action_desc={action_desc}')

            future = self.call_service_with_retry(self.set_action_client, request, "SetMcAction")

            # 超时后检查是否实际已到达目标状态
            if future is None or future.result() is None:
                _, current_desc, current_status = self.get_action_status()
                if current_desc == action_desc:
                    self.get_logger().warning(
                        f'SetMcAction timed out, but target action is already active: {current_desc} status={current_status}')
                    return True
                return False

            response = future.result()
            if response.response.status.value == CommonState.SUCCESS:
                self.get_logger().info('SetMcAction request accepted by service.')
                return True
            
            # 获取失败原因
            reason = getattr(response.response.status, 'reason', 0)
            if reason > 0:
                reason_desc = get_reason_description(reason)
                self.get_logger().warning(
                    f'SetMcAction rejected: reason={reason} - {reason_desc}'
                )

            self.get_logger().error(f'Failed to set robot mode: {response.response.message}')
            return False
        except Exception as e:
            self.get_logger().error(f'Exception occurred: {e}')
            return False

    def set_motion(self, motion_name: str, interrupt: bool) -> bool:
        try:
            for attempt in range(1, 6):
                request = SetMcMotion.Request()
                request.header = RequestHeader()
                request.header.stamp = self.get_clock().now().to_msg()
                request.motion = motion_name
                request.type = SetMcMotion.Request.MIMIC_QY
                request.interrupt = interrupt

                self.get_logger().info(f'Sending SetMcMotion request ({attempt}/5): motion={motion_name} interrupt={interrupt}')

                future = self.set_motion_client.call_async(request)
                rclpy.spin_until_future_complete(self, future, timeout_sec=1.0)

                if not future.done() or future.result() is None:
                    self.get_logger().warning(f'SetMcMotion request attempt {attempt}/5 failed or timed out.')
                    continue

                response = future.result()
                code = response.response.header.code
                state = response.response.state.value
                task_id = response.response.task_id

                if code == 0 and state in (CommonState.SUCCESS, CommonState.RUNNING):
                    self.get_logger().info(f'SetMcMotion request accepted: code={code} state={state} task_id={task_id}')
                    return True

                self.get_logger().warning(f'SetMcMotion request attempt {attempt}/5 not accepted: code={code} state={state} task_id={task_id}')

            self.get_logger().error('Failed to set motion after 5 attempts.')
            return False
        except Exception as e:
            self.get_logger().error(f'Exception occurred: {e}')
            return False

    def wait_for_action(self, expected_action_desc: str, timeout_sec: float = 10.0,
                        poll_interval_sec: float = 0.2) -> bool:
        deadline = time.monotonic() + timeout_sec
        self.get_logger().info(f'Waiting for target action_desc={expected_action_desc} to reach RUNNING state...')

        while rclpy.ok() and time.monotonic() < deadline:
            _, action_desc, status = self.get_action_status()
            if action_desc is not None and action_desc == expected_action_desc and status == McActionStatus.RUNNING:
                self.get_logger().info(f'Target action reached and is running: action_desc={expected_action_desc}')
                return True
            time.sleep(poll_interval_sec)

        self.get_logger().error(f'Timed out waiting for target action_desc={expected_action_desc} to reach RUNNING state.')
        return False

    def wait_for_motion(self, timeout_sec: float = 10.0, poll_interval_sec: float = 0.2) -> bool:
        deadline = time.monotonic() + timeout_sec
        self.get_logger().info('Waiting for current motion action to reach RUNNING state...')

        while rclpy.ok() and time.monotonic() < deadline:
            action_id, action_desc, status = self.get_action_status()
            if status == McActionStatus.RUNNING:
                self.get_logger().info(f'Current motion action is running: action_id={action_id} action_desc={action_desc}')
                return True
            time.sleep(poll_interval_sec)

        self.get_logger().error('Timed out waiting for current motion action to reach RUNNING state.')
        return False

    def get_action_status(self):
        try:
            request = GetMcAction.Request()
            request.request = CommonRequest()
            request.request.header.stamp = self.get_clock().now().to_msg()

            future = self.call_service_with_retry(self.get_client, request, "GetMcAction")
            if future is None or future.result() is None:
                self.get_logger().warning('Get current action request failed or timed out.')
                return None, None, None

            response = future.result()
            return response.info.current_action.value, response.info.action_desc, response.info.status.value
        except Exception as e:
            self.get_logger().error(f'Exception occurred: {e}')
            return None, None, None


def main(args=None):
    rclpy.init(args=args)
    # init 之后立即覆盖 ROS2 的 SIGINT 处理器
    # 阻止 Ctrl+C 触发 rclpy.shutdown()，保持上下文有效以恢复状态
    signal.signal(signal.SIGINT, lambda sig, frame: (_ for _ in ()).throw(KeyboardInterrupt))

    node = None
    try:
        node = SetMcActionClient()
        ok = node.execute()
        node.destroy_node()
        node = None
        if rclpy.ok():
            rclpy.shutdown()
        return 0 if ok else 1
    except KeyboardInterrupt:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        return 0
    except Exception as e:
        rclpy.logging.get_logger('main').error(f'Program exited with exception: {e}')
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        return 1


if __name__ == '__main__':
    raise SystemExit(main())
