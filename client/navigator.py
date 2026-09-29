"""The navigator interface the executor drives, plus a ROS-free simulated navigator.

The executor only ever calls these four methods, so it neither knows nor cares
whether the robot is Nav2 in Gazebo (client/nav2_navigator.py) or the straight-
line simulation below (tests, and `--fake-nav` for trying the MQTT flow without
a simulator).
"""
from __future__ import annotations

import math
import time
from enum import Enum
from typing import Callable, Protocol

from vda5050.messages import Position


class NavResult(Enum):
    SUCCEEDED = "SUCCEEDED"
    FAILED = "FAILED"        # Nav2 ABORTED, goal rejected, or no Nav2 server
    CANCELED = "CANCELED"    # we cancelled it (pause, in milestone 5)


class Navigator(Protocol):
    def go_to(self, x: float, y: float, theta: float | None) -> None:
        """Start driving to (x, y, theta) in the map frame. Returns immediately."""

    def poll(self) -> NavResult | None:
        """Result of the latest goal, or None while it is still running."""

    def cancel(self) -> None:
        """Stop the current goal. poll() then reports CANCELED."""

    def pose(self) -> Position | None:
        """Current pose in the map frame, or None if not localized yet."""

    def close(self) -> None:
        """Release resources."""


class SimulatedNavigator:
    """Drives in a straight line at constant speed. No ROS required."""

    def __init__(
        self,
        start: Position = Position(0.0, 0.0, 0.0),
        speed_mps: float = 0.25,
        clock: Callable[[], float] = time.monotonic,
        fail_at: set[tuple[float, float]] | None = None,
    ) -> None:
        """`fail_at`: goal (x, y) coordinates that will fail halfway, to test error handling."""
        self._pose = start
        self._speed = speed_mps
        self._clock = clock
        self._fail_at = fail_at or set()
        self._goal: Position | None = None
        self._start: Position = start
        self._t0 = 0.0
        self._result: NavResult | None = None

    def go_to(self, x: float, y: float, theta: float | None) -> None:
        self._start = self.pose()
        self._goal = Position(x, y, self._start.theta if theta is None else theta)
        self._t0 = self._clock()
        self._result = None

    def poll(self) -> NavResult | None:
        if self._goal is None:
            return self._result
        progress = self._progress()
        if (self._goal.x, self._goal.y) in self._fail_at and progress >= 0.5:
            self._stop(NavResult.FAILED)
        elif progress >= 1.0:
            self._pose, self._goal, self._result = self._goal, None, NavResult.SUCCEEDED
        return self._result

    def cancel(self) -> None:
        if self._goal is not None:
            self._stop(NavResult.CANCELED)

    def pose(self) -> Position:
        if self._goal is None:
            return self._pose
        f = min(self._progress(), 1.0)
        heading = math.atan2(self._goal.y - self._start.y, self._goal.x - self._start.x)
        return Position(
            self._start.x + f * (self._goal.x - self._start.x),
            self._start.y + f * (self._goal.y - self._start.y),
            heading,
        )

    def close(self) -> None:
        pass

    def _progress(self) -> float:
        distance = math.hypot(self._goal.x - self._start.x, self._goal.y - self._start.y)
        if distance < 1e-9:
            return 1.0
        return (self._clock() - self._t0) * self._speed / distance

    def _stop(self, result: NavResult) -> None:
        self._pose, self._goal, self._result = self.pose(), None, result
