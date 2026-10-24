"""Pure-Python motion logic (no ROS imports) so it can be unit-tested anywhere.

Design patterns used in this module
-----------------------------------
* State    : RotateToHeading -> DriveToPoint -> RotateToYaw -> Done.
             Each phase of goToPose is its own class; the controller only
             delegates to the current state.
* Strategy : ``SaturatedP`` is the control law injected into the states, so the
             law can be swapped (e.g. PID) without touching the state logic.
"""
from __future__ import annotations

import math
from abc import ABC, abstractmethod
from dataclasses import dataclass
from typing import Tuple


def wrap_angle(angle: float) -> float:
    """Normalise an angle to [-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


@dataclass(frozen=True)
class Pose2D:
    x: float
    y: float
    theta: float


@dataclass(frozen=True)
class Limits:
    max_linear: float
    max_angular: float


@dataclass(frozen=True)
class Tolerances:
    position: float = 0.05
    yaw: float = 0.03
    heading: float = 0.05


@dataclass(frozen=True)
class Command:
    linear: float = 0.0
    angular: float = 0.0


# ---------------------------------------------------------------- Strategy
class SaturatedP:
    """Proportional control law with symmetric saturation."""

    def __init__(self, gain: float):
        self.gain = gain

    def __call__(self, error: float, limit: float) -> float:
        return clamp(self.gain * error, -limit, limit)


# ------------------------------------------------------------------- State
class State(ABC):
    name = 'state'
    finished = False

    @abstractmethod
    def step(self, pose: Pose2D, goal: Pose2D, limits: Limits,
             tol: Tolerances) -> Tuple[Command, 'State']:
        """Return (command, next_state). next_state is self to stay."""


def _distance(pose: Pose2D, goal: Pose2D) -> float:
    return math.hypot(goal.x - pose.x, goal.y - pose.y)


def _heading_error(pose: Pose2D, goal: Pose2D) -> float:
    return wrap_angle(math.atan2(goal.y - pose.y, goal.x - pose.x) - pose.theta)


class RotateToHeading(State):
    """Turn on the spot until the turtle faces the target point."""
    name = 'rotate_to_heading'

    def __init__(self, angular_law: SaturatedP = None):
        self.angular_law = angular_law or SaturatedP(4.0)

    def step(self, pose, goal, limits, tol):
        if _distance(pose, goal) < tol.position:
            return Command(), RotateToYaw(self.angular_law)
        err = _heading_error(pose, goal)
        if abs(err) < tol.heading:
            return Command(), DriveToPoint(self.angular_law)
        return Command(angular=self.angular_law(err, limits.max_angular)), self


class DriveToPoint(State):
    """Drive forward, steering gently to stay on the line to the target."""
    name = 'drive_to_point'
    CLOSE = 0.2   # inside this radius the bearing is noisy -> stop steering

    def __init__(self, angular_law: SaturatedP = None, linear_law: SaturatedP = None):
        self.angular_law = angular_law or SaturatedP(4.0)
        self.linear_law = linear_law or SaturatedP(1.5)

    def step(self, pose, goal, limits, tol):
        dist = _distance(pose, goal)
        if dist < tol.position:
            return Command(), RotateToYaw(self.angular_law)
        err = _heading_error(pose, goal)
        if dist > self.CLOSE and abs(err) > math.pi / 4:
            return Command(), RotateToHeading(self.angular_law)   # badly off course
        v = self.linear_law(dist, limits.max_linear)
        w = self.angular_law(err, limits.max_angular) if dist > self.CLOSE else 0.0
        return Command(linear=v, angular=w), self


class RotateToYaw(State):
    """Final rotation to the requested yaw."""
    name = 'rotate_to_yaw'

    def __init__(self, angular_law: SaturatedP = None):
        self.angular_law = angular_law or SaturatedP(4.0)

    def step(self, pose, goal, limits, tol):
        err = wrap_angle(goal.theta - pose.theta)
        if abs(err) < tol.yaw:
            return Command(), Done()
        return Command(angular=self.angular_law(err, limits.max_angular)), self


class Done(State):
    name = 'done'
    finished = True

    def step(self, pose, goal, limits, tol):
        return Command(), self


class GoToPoseController:
    """Context of the State pattern. Limits are passed on EVERY call, so a
    parameter change takes effect on the very next control tick."""

    def __init__(self):
        self._state: State = RotateToHeading()

    @property
    def phase(self) -> str:
        return self._state.name

    @property
    def finished(self) -> bool:
        return self._state.finished

    def update(self, pose: Pose2D, goal: Pose2D, limits: Limits,
               tol: Tolerances) -> Command:
        for _ in range(4):                      # allow chained transitions
            cmd, nxt = self._state.step(pose, goal, limits, tol)
            if nxt is self._state:
                return cmd
            self._state = nxt
        return Command()


def pose_errors(pose: Pose2D, goal: Pose2D) -> Tuple[float, float]:
    return _distance(pose, goal), wrap_angle(goal.theta - pose.theta)
