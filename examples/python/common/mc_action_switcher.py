#!/usr/bin/env python3
"""Synchronous T1 motion-control action switcher.

The switcher mirrors the action graph in the T1 action ruler.  It plans a
valid route from the current MC action, requests each required transition, and
waits for the MC runner to report that the expected action is active.
"""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass
from enum import Enum
import time
from typing import Callable, Deque, Dict, List, Optional, Set

import rclpy
from rclpy.node import Node

from aimdk_msgs.msg import (
    CommonRequest,
    CommonState,
    McActionCommand,
    McActionStatus,
    RequestHeader,
)
from aimdk_msgs.srv import GetMcAction, SetMcAction


@dataclass
class McActionSwitchOptions:
    """Timing and cancellation settings for :meth:`McActionSwitcher.switch_to`."""

    source: str = "sdk_node"
    service_wait_timeout: float = 5.0
    service_call_timeout: float = 3.0
    poll_interval: float = 0.5
    minimum_set_action_interval: float = 0.5
    total_timeout: float = 20.0
    should_cancel: Optional[Callable[[], bool]] = None


class McActionSwitchError(Enum):
    NONE = "none"
    INVALID_ARGUMENT = "invalid_argument"
    SERVICE_UNAVAILABLE = "service_unavailable"
    GET_ACTION_FAILED = "get_action_failed"
    TARGET_NOT_ACTIVE = "target_not_active"
    NO_TRANSITION_PATH = "no_transition_path"
    SET_ACTION_FAILED = "set_action_failed"
    TIMEOUT = "timeout"
    SHUTDOWN = "shutdown"


@dataclass
class McActionSwitchResult:
    success: bool = False
    error: McActionSwitchError = McActionSwitchError.NONE
    current_action: str = ""
    target_action: str = ""
    message: str = ""


@dataclass(frozen=True)
class _ActionState:
    action_desc: str
    status: int


@dataclass(frozen=True)
class _PlannedTransition:
    command: str
    expected_action: str


# These directed edges mirror qd1_t2d0/action_ruler.yaml.  They intentionally
# include only destinations which SetMcAction may request directly.
_ACTION_RULES: Dict[str, List[str]] = {
    "PASSIVE_DEFAULT": [
        "QUADRUPED_STAND_DEFAULT",
        "QUADRUPED_SIT_DOWN_DEFAULT",
        "QUADRUPED_GET_DOWN_DEFAULT",
        "QUADRUPED_RECOVERY",
    ],
    "DAMPING_DEFAULT": ["PASSIVE_DEFAULT"],
    "QUAD_LIE_DOWN": ["PASSIVE_DEFAULT"],
    "QUADRUPED_STAND_DEFAULT": [
        "QUADRUPED_LOCOMOTION_DEFAULT",
        "QUADRUPED_LOCOMOTION_TERRAIN",
        "QUADRUPED_LOCOMOTION_RUN",
        "QUADRUPED_GET_DOWN_DEFAULT",
        "QUADRUPED_SIT_DOWN_DEFAULT",
        "QUADRUPED_LOCOMOTION_BIONIC",
        "QUADRUPED_LOCOMOTION_PRIDE",
        "QUADRUPED_LOCOMOTION_PLEASURE",
        "QUADRUPED_LOCOMOTION_JUMP",
        "QUADRUPED_LOCOMOTION_HANDSHAKE",
        "QUADRUPED_LOCOMOTION_STRETCH",
        "QUADRUPED_LOCOMOTION_DANCE",
        "QUADRUPED_PUSH_UP",
    ],
    "QUADRUPED_GET_DOWN_DEFAULT": [
        "QUADRUPED_STAND_DEFAULT",
        "QUADRUPED_SIT_DOWN_DEFAULT",
    ],
    "QUADRUPED_SIT_DOWN_DEFAULT": [
        "QUADRUPED_GET_DOWN_DEFAULT",
        "QUADRUPED_STAND_DEFAULT",
    ],
    "QUADRUPED_LOCOMOTION_DEFAULT": [
        "QUADRUPED_LOCOMOTION_TERRAIN",
        "QUADRUPED_LOCOMOTION_BIONIC",
        "QUADRUPED_LOCOMOTION_RUN",
        "QUADRUPED_TO_BIPED",
        "QUADRUPED_TO_BIPED_ROTATE",
        "QUADRUPED_TO_BIPED_FRONTFLIP",
        "QUADRUPED_LOCOMOTION_FRONTFLIP",
        "QUADRUPED_LOCOMOTION_BACKFLIP",
        "QUADRUPED_LOCOMOTION_STEP",
        "QUADRUPED_LOCOMOTION_PRIDE",
        "QUADRUPED_LOCOMOTION_HANDSHAKE",
        "QUADRUPED_LOCOMOTION_STRETCH",
        "QUADRUPED_LOCOMOTION_DANCE",
        "QUADRUPED_PUSH_UP",
        "QUADRUPED_STAND_DEFAULT",
        "QUADRUPED_GET_DOWN_DEFAULT",
        "QUADRUPED_SIT_DOWN_DEFAULT",
    ],
    "QUADRUPED_LOCOMOTION_STEP": ["QUADRUPED_LOCOMOTION_DEFAULT"],
    "QUADRUPED_LOCOMOTION_RUN": [
        "QUADRUPED_LOCOMOTION_DEFAULT",
        "QUADRUPED_LOCOMOTION_TERRAIN",
        "QUADRUPED_LOCOMOTION_BIONIC",
        "QUADRUPED_TO_BIPED",
        "QUADRUPED_TO_BIPED_ROTATE",
        "QUADRUPED_TO_BIPED_FRONTFLIP",
        "QUADRUPED_LOCOMOTION_FRONTFLIP",
        "QUADRUPED_LOCOMOTION_BACKFLIP",
        "QUADRUPED_LOCOMOTION_STEP",
        "QUADRUPED_LOCOMOTION_PRIDE",
        "QUADRUPED_LOCOMOTION_HANDSHAKE",
        "QUADRUPED_LOCOMOTION_STRETCH",
        "QUADRUPED_LOCOMOTION_DANCE",
        "QUADRUPED_PUSH_UP",
        "QUADRUPED_STAND_DEFAULT",
        "QUADRUPED_GET_DOWN_DEFAULT",
        "QUADRUPED_SIT_DOWN_DEFAULT",
    ],
    "QUADRUPED_LOCOMOTION_TERRAIN": [
        "QUADRUPED_LOCOMOTION_DEFAULT",
        "QUADRUPED_LOCOMOTION_BIONIC",
        "QUADRUPED_LOCOMOTION_RUN",
        "QUADRUPED_TO_BIPED",
        "QUADRUPED_TO_BIPED_ROTATE",
        "QUADRUPED_TO_BIPED_FRONTFLIP",
        "QUADRUPED_LOCOMOTION_FRONTFLIP",
        "QUADRUPED_LOCOMOTION_BACKFLIP",
        "QUADRUPED_LOCOMOTION_STEP",
        "QUADRUPED_LOCOMOTION_PRIDE",
        "QUADRUPED_LOCOMOTION_HANDSHAKE",
        "QUADRUPED_LOCOMOTION_STRETCH",
        "QUADRUPED_LOCOMOTION_DANCE",
        "QUADRUPED_PUSH_UP",
        "QUADRUPED_STAND_DEFAULT",
        "QUADRUPED_GET_DOWN_DEFAULT",
        "QUADRUPED_SIT_DOWN_DEFAULT",
    ],
    "QUADRUPED_LOCOMOTION_BIONIC": [
        "QUADRUPED_LOCOMOTION_DEFAULT",
        "QUADRUPED_LOCOMOTION_TERRAIN",
        "QUADRUPED_LOCOMOTION_RUN",
    ],
    "QUADRUPED_LOCOMOTION_PRIDE": [
        "QUADRUPED_LOCOMOTION_DEFAULT",
        "QUADRUPED_LOCOMOTION_PLEASURE",
    ],
    "QUADRUPED_PUSH_UP": [
        "QUADRUPED_LOCOMOTION_DEFAULT",
        "QUADRUPED_STAND_DEFAULT",
    ],
    "QUADRUPED_LOCOMOTION_PLEASURE": [
        "QUADRUPED_LOCOMOTION_DEFAULT",
        "QUADRUPED_LOCOMOTION_PRIDE",
    ],
    "QUADRUPED_LOCOMOTION_JUMP": [
        "QUADRUPED_LOCOMOTION_DEFAULT",
        "QUADRUPED_STAND_DEFAULT",
    ],
    "QUADRUPED_LOCOMOTION_HANDSHAKE": [
        "QUADRUPED_LOCOMOTION_DEFAULT",
        "QUADRUPED_STAND_DEFAULT",
    ],
    "QUADRUPED_RECOVERY": [
        "QUADRUPED_STAND_DEFAULT",
        "QUADRUPED_LOCOMOTION_DEFAULT",
        "QUADRUPED_LOCOMOTION_TERRAIN",
        "QUADRUPED_LOCOMOTION_RUN",
    ],
    "QUADRUPED_LOCOMOTION_STRETCH": [
        "QUADRUPED_LOCOMOTION_DEFAULT",
        "QUADRUPED_STAND_DEFAULT",
    ],
    "QUADRUPED_LOCOMOTION_DANCE": [
        "QUADRUPED_LOCOMOTION_DEFAULT",
        "QUADRUPED_STAND_DEFAULT",
    ],
    "BIPED_STAND_DEFAULT": ["BIPED_LOCOMOTION_WBC"],
    "BIPED_LOCOMOTION_WBC": [
        "BIPED_TO_QUADRUPED",
        "BIPED_TO_QUADRUPED_ROTATE",
        "BIPED_TO_QUADRUPED_FRONTFLIP",
        "BIPED_LOCOMOTION_TERRAIN",
        "BIPED_LOCOMOTION_RUN",
        "BIPED_LOCOMOTION_DEFAULT",
        "BIPED_LOCOMOTION_ROTATE_LOCAL",
        "BIPED_LOCOMOTION_BACKBEND",
        "BIPED_YUZURU",
        "BIPED_SPLIT",
        "BIPED_LOCOMOTION_MOONWALK",
        "BIPED_LOCOMOTION_DANCE",
        "BIPED_LOCOMOTION_ANIMATION",
        "BIPED_RECORD_UPPER",
        "BIPED_CUSTOM_UPPER",
    ],
    "BIPED_RECORD_UPPER": ["BIPED_LOCOMOTION_WBC"],
    "BIPED_LOCOMOTION_ANIMATION": [
        "BIPED_TO_QUADRUPED",
        "BIPED_TO_QUADRUPED_ROTATE",
        "BIPED_TO_QUADRUPED_FRONTFLIP",
        "BIPED_LOCOMOTION_TERRAIN",
        "BIPED_LOCOMOTION_RUN",
        "BIPED_LOCOMOTION_DEFAULT",
        "BIPED_LOCOMOTION_ROTATE_LOCAL",
        "BIPED_LOCOMOTION_BACKBEND",
        "BIPED_YUZURU",
        "BIPED_SPLIT",
        "BIPED_LOCOMOTION_MOONWALK",
        "BIPED_LOCOMOTION_WBC",
    ],
    "BIPED_LOCOMOTION_RUN": [
        "BIPED_LOCOMOTION_WBC",
        "BIPED_TO_QUADRUPED_ROTATE",
    ],
    "BIPED_LOCOMOTION_TERRAIN": ["BIPED_LOCOMOTION_WBC"],
    "BIPED_YUZURU": ["BIPED_LOCOMOTION_WBC"],
    "BIPED_SPLIT": ["BIPED_LOCOMOTION_WBC"],
    "BIPED_LOCOMOTION_DEFAULT": [
        "BIPED_LOCOMOTION_TERRAIN",
        "BIPED_LOCOMOTION_WBC",
        "BIPED_LOCOMOTION_RUN",
    ],
    "QUADRUPED_TO_BIPED": [
        "BIPED_LOCOMOTION_WBC",
        "BIPED_LOCOMOTION_TERRAIN",
    ],
    "QUADRUPED_TO_BIPED_ROTATE": [
        "BIPED_LOCOMOTION_WBC",
        "BIPED_LOCOMOTION_TERRAIN",
    ],
    "BIPED_TO_QUADRUPED": [
        "QUADRUPED_STAND_DEFAULT",
        "QUADRUPED_LOCOMOTION_DEFAULT",
        "QUADRUPED_LOCOMOTION_TERRAIN",
    ],
    "BIPED_TO_QUADRUPED_FRONTFLIP": ["QUADRUPED_LOCOMOTION_DEFAULT"],
    "BIPED_TO_QUADRUPED_ROTATE": ["QUADRUPED_LOCOMOTION_DEFAULT"],
    "QUADRUPED_LOCOMOTION_FRONTFLIP": ["QUADRUPED_LOCOMOTION_DEFAULT"],
    "QUADRUPED_LOCOMOTION_BACKFLIP": ["QUADRUPED_LOCOMOTION_DEFAULT"],
    "QUADRUPED_TO_BIPED_FRONTFLIP": ["BIPED_LOCOMOTION_WBC"],
    "BIPED_LOCOMOTION_ROTATE_LOCAL": ["BIPED_LOCOMOTION_WBC"],
    "BIPED_LOCOMOTION_BACKBEND": ["BIPED_LOCOMOTION_WBC"],
    "BIPED_LOCOMOTION_MOONWALK": ["BIPED_LOCOMOTION_WBC"],
    "BIPED_LOCOMOTION_DANCE": ["BIPED_LOCOMOTION_WBC"],
    "BIPED_CUSTOM_UPPER": ["BIPED_LOCOMOTION_WBC"],
}

_SKIP_ACTIONS: Set[str] = {
    "PASSIVE_DEFAULT",
    "BIPED_LOCOMOTION_WBC",
    "BIPED_LOCOMOTION_ANIMATION",
    "QUADRUPED_LOCOMOTION_DEFAULT",
    "BIPED_STAND_DEFAULT",
    "QUADRUPED_RECOVERY",
    "QUADRUPED_TO_BIPED_FRONTFLIP",
    "BIPED_TO_QUADRUPED_FRONTFLIP",
    "QUADRUPED_LOCOMOTION_FRONTFLIP",
    "QUADRUPED_LOCOMOTION_BACKFLIP",
    "QUADRUPED_GET_DOWN_DEFAULT",
    "QUADRUPED_SIT_DOWN_DEFAULT",
    "BIPED_TO_QUADRUPED",
    "QUADRUPED_TO_BIPED",
    "QUADRUPED_LOCOMOTION_TERRAIN",
    "QUADRUPED_LOCOMOTION_RUN",
}

# Commands below start a runner which then reports the bridge action.  Never
# send that bridge action again before observing it through GetMcAction.
_AUTOMATIC_NEXT_ACTIONS: Dict[str, str] = {
    "BIPED_STAND_DEFAULT": "BIPED_LOCOMOTION_WBC",
    "QUADRUPED_TO_BIPED": "BIPED_LOCOMOTION_WBC",
    "QUADRUPED_TO_BIPED_ROTATE": "BIPED_LOCOMOTION_WBC",
    "QUADRUPED_TO_BIPED_FRONTFLIP": "BIPED_LOCOMOTION_WBC",
    "BIPED_TO_QUADRUPED": "QUADRUPED_LOCOMOTION_DEFAULT",
    "BIPED_TO_QUADRUPED_FRONTFLIP": "QUADRUPED_LOCOMOTION_DEFAULT",
    "BIPED_TO_QUADRUPED_ROTATE": "QUADRUPED_LOCOMOTION_DEFAULT",
    "QUADRUPED_LOCOMOTION_FRONTFLIP": "QUADRUPED_LOCOMOTION_DEFAULT",
    "QUADRUPED_LOCOMOTION_BACKFLIP": "QUADRUPED_LOCOMOTION_DEFAULT",
}

_GLOBAL_DESTINATIONS = (
    "PASSIVE_DEFAULT",
    "DAMPING_DEFAULT",
    "QUAD_LIE_DOWN",
)


class McActionSwitcher:
    """Synchronously switch a node's MC action to a reachable target action."""

    def __init__(
        self,
        node: Optional[Node],
        set_action_service: str = "/aimdk_5Fmsgs/srv/SetMcAction",
        get_action_service: str = "/aimdk_5Fmsgs/srv/GetMcAction",
    ) -> None:
        self._node = node
        self._set_action_service = set_action_service
        self._get_action_service = get_action_service
        self._set_action_client = None
        self._get_action_client = None
        if node is not None:
            self._set_action_client = node.create_client(SetMcAction, set_action_service)
            self._get_action_client = node.create_client(GetMcAction, get_action_service)

    def switch_to(
        self,
        target_action: str,
        options: Optional[McActionSwitchOptions] = None,
    ) -> McActionSwitchResult:
        """Switch to ``target_action`` and wait until it becomes active."""
        options = options or McActionSwitchOptions()
        result = McActionSwitchResult(target_action=target_action)

        if self._node is None or not target_action:
            return self._fail(
                result,
                McActionSwitchError.INVALID_ARGUMENT,
                "Node and target_action must be provided.",
            )
        if self._is_cancelled(options):
            return self._fail(
                result, McActionSwitchError.SHUTDOWN, "Action switch cancelled."
            )
        if not self._wait_for_services(options, result):
            return result

        current = self._get_current_action(options)
        if current is None:
            if self._is_cancelled(options):
                return self._fail(
                    result,
                    McActionSwitchError.SHUTDOWN,
                    "Action switch cancelled while reading the current action.",
                )
            return self._fail(
                result,
                McActionSwitchError.GET_ACTION_FAILED,
                "GetMcAction did not return the current action.",
            )

        result.current_action = current.action_desc
        if current.action_desc == target_action:
            if self._is_active(current):
                result.success = True
                result.message = "Robot is already in the requested action."
                return result
            return self._fail(
                result,
                McActionSwitchError.TARGET_NOT_ACTIVE,
                "MC reports the target action as IDLE; refusing to leave "
                f"{target_action} through a fallback transition.",
            )

        path = self._plan_path(current.action_desc, target_action)
        if not path:
            return self._fail(
                result,
                McActionSwitchError.NO_TRANSITION_PATH,
                f"No T1 action_ruler path from {current.action_desc} to {target_action}.",
            )

        deadline = time.monotonic() + options.total_timeout
        next_request_time = time.monotonic()
        for transition in path:
            if not rclpy.ok():
                return self._fail(
                    result,
                    McActionSwitchError.SHUTDOWN,
                    "ROS shutdown while switching actions.",
                )
            if time.monotonic() >= deadline:
                return self._fail(
                    result,
                    McActionSwitchError.TIMEOUT,
                    "Timed out while switching actions.",
                )

            self._sleep_until(next_request_time)
            if time.monotonic() >= deadline:
                return self._fail(
                    result,
                    McActionSwitchError.TIMEOUT,
                    "Timed out while waiting to request the next action.",
                )
            if not self._request_action(transition.command, options):
                if self._is_cancelled(options):
                    return self._fail(
                        result,
                        McActionSwitchError.SHUTDOWN,
                        f"Action switch cancelled while requesting {transition.command}.",
                    )
                return self._fail(
                    result,
                    McActionSwitchError.SET_ACTION_FAILED,
                    f"SetMcAction rejected {transition.command}.",
                )
            next_request_time = time.monotonic() + options.minimum_set_action_interval

            reached = self._wait_for_action(transition.expected_action, options, deadline)
            if reached is None:
                if not rclpy.ok():
                    return self._fail(
                        result,
                        McActionSwitchError.SHUTDOWN,
                        f"ROS shutdown while waiting for {transition.expected_action}.",
                    )
                if self._is_cancelled(options):
                    return self._fail(
                        result,
                        McActionSwitchError.SHUTDOWN,
                        f"Action switch cancelled while waiting for "
                        f"{transition.expected_action}.",
                    )
                return self._fail(
                    result,
                    McActionSwitchError.TIMEOUT,
                    f"Timed out waiting for {transition.expected_action} after requesting "
                    f"{transition.command}.",
                )
            current = reached
            result.current_action = current.action_desc

        if current.action_desc != target_action:
            return self._fail(
                result,
                McActionSwitchError.TIMEOUT,
                "MC did not reach the requested target action.",
            )
        result.success = True
        result.message = "Robot reached the requested action."
        return result

    def _plan_path(
        self, current_action: str, target_action: str
    ) -> List[_PlannedTransition]:
        """Return the shortest executable route, or an empty list if none exists."""
        if current_action in {"DAMPING_DEFAULT", "QUAD_LIE_DOWN"}:
            if target_action == "PASSIVE_DEFAULT":
                return [_PlannedTransition("PASSIVE_DEFAULT", "PASSIVE_DEFAULT")]
            path = self._plan_path("PASSIVE_DEFAULT", target_action)
            if path:
                return [_PlannedTransition("PASSIVE_DEFAULT", "PASSIVE_DEFAULT"), *path]
            return []

        pending: Deque[str] = deque([current_action])
        visited = {current_action}
        previous: Dict[str, tuple[str, _PlannedTransition]] = {}

        while pending:
            source = pending.popleft()
            candidates = list(_ACTION_RULES.get(source, []))
            candidates.extend(_GLOBAL_DESTINATIONS)

            for candidate in candidates:
                if candidate == target_action:
                    transition = _PlannedTransition(candidate, candidate)
                    arrival = candidate
                elif candidate in _AUTOMATIC_NEXT_ACTIONS:
                    arrival = _AUTOMATIC_NEXT_ACTIONS[candidate]
                    transition = _PlannedTransition(candidate, arrival)
                elif self._is_bridge_action(candidate) or candidate not in _SKIP_ACTIONS:
                    arrival = candidate
                    transition = _PlannedTransition(candidate, candidate)
                else:
                    continue

                if arrival in visited:
                    continue
                visited.add(arrival)
                previous[arrival] = (source, transition)
                if arrival == target_action:
                    path: List[_PlannedTransition] = []
                    cursor = target_action
                    while cursor != current_action:
                        predecessor = previous.get(cursor)
                        if predecessor is None:
                            return []
                        cursor, planned_transition = predecessor
                        path.append(planned_transition)
                    path.reverse()
                    return path
                pending.append(arrival)
        return []

    def _is_bridge_action(self, action: str) -> bool:
        return action in {
            "QUADRUPED_LOCOMOTION_DEFAULT",
            "BIPED_LOCOMOTION_WBC",
        }

    def _is_cancelled(self, options: McActionSwitchOptions) -> bool:
        return options.should_cancel is not None and options.should_cancel()

    def _is_active(self, action: _ActionState) -> bool:
        return action.status != McActionStatus.IDLE

    def _sleep_until(self, deadline: float) -> None:
        remaining = deadline - time.monotonic()
        if remaining > 0:
            time.sleep(remaining)

    def _fail(
        self,
        result: McActionSwitchResult,
        error: McActionSwitchError,
        message: str,
    ) -> McActionSwitchResult:
        result.success = False
        result.error = error
        result.message = message
        return result

    def _wait_for_services(
        self, options: McActionSwitchOptions, result: McActionSwitchResult
    ) -> bool:
        if self._set_action_client is None or self._get_action_client is None:
            self._fail(
                result,
                McActionSwitchError.INVALID_ARGUMENT,
                "Node and target_action must be provided.",
            )
            return False
        if self._is_cancelled(options):
            self._fail(
                result,
                McActionSwitchError.SHUTDOWN,
                "Action switch cancelled while waiting for services.",
            )
            return False
        if not self._set_action_client.wait_for_service(
            timeout_sec=options.service_wait_timeout
        ):
            self._fail(
                result,
                McActionSwitchError.SERVICE_UNAVAILABLE,
                f"SetMcAction service is unavailable: {self._set_action_service}",
            )
            return False
        if self._is_cancelled(options):
            self._fail(
                result,
                McActionSwitchError.SHUTDOWN,
                "Action switch cancelled while waiting for services.",
            )
            return False
        if not self._get_action_client.wait_for_service(
            timeout_sec=options.service_wait_timeout
        ):
            self._fail(
                result,
                McActionSwitchError.SERVICE_UNAVAILABLE,
                f"GetMcAction service is unavailable: {self._get_action_service}",
            )
            return False
        return True

    def _get_current_action(
        self, options: McActionSwitchOptions
    ) -> Optional[_ActionState]:
        for attempt in range(1, 4):
            if self._is_cancelled(options):
                return None
            request = GetMcAction.Request()
            request.request = CommonRequest()
            request.request.header.stamp = self._node.get_clock().now().to_msg()
            try:
                future = self._get_action_client.call_async(request)
                rclpy.spin_until_future_complete(
                    self._node, future, timeout_sec=options.service_wait_timeout
                )
                if future.done():
                    response = future.result()
                    if response is None or response.header.code != 0:
                        return None
                    return _ActionState(
                        response.info.action_desc,
                        response.info.status.value,
                    )
            except Exception as error:  # ROS futures surface transport failures here.
                self._node.get_logger().warning(
                    f"GetMcAction attempt {attempt}/3 failed: {error}"
                )
            else:
                self._node.get_logger().warning(
                    f"GetMcAction attempt {attempt}/3 timed out or was interrupted."
                )

            if attempt < 3:
                time.sleep(options.poll_interval)
        return None

    def _request_action(
        self, action_desc: str, options: McActionSwitchOptions
    ) -> bool:
        if self._is_cancelled(options):
            return False
        request = SetMcAction.Request()
        request.header = RequestHeader()
        request.header.stamp = self._node.get_clock().now().to_msg()
        request.source = options.source
        request.command = McActionCommand()
        request.command.action_desc = action_desc

        self._node.get_logger().info(f"Requesting MC action: {action_desc}")
        try:
            future = self._set_action_client.call_async(request)
            rclpy.spin_until_future_complete(
                self._node, future, timeout_sec=options.service_call_timeout
            )
            if not future.done():
                return False
            response = future.result()
            return (
                response is not None
                and response.response.status.value == CommonState.SUCCESS
            )
        except Exception as error:  # ROS futures surface transport failures here.
            self._node.get_logger().warning(
                f"SetMcAction request for {action_desc} failed: {error}"
            )
            return False

    def _wait_for_action(
        self,
        expected_action: str,
        options: McActionSwitchOptions,
        deadline: float,
    ) -> Optional[_ActionState]:
        while (
            rclpy.ok()
            and not self._is_cancelled(options)
            and time.monotonic() < deadline
        ):
            current = self._get_current_action(options)
            if (
                current is not None
                and current.action_desc == expected_action
                and self._is_active(current)
            ):
                self._node.get_logger().info(
                    f"MC reached action: {expected_action} (status={current.status})"
                )
                return current

            remaining = deadline - time.monotonic()
            if remaining <= 0:
                break
            time.sleep(min(options.poll_interval, remaining))
        return None
