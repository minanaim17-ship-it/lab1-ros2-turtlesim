"""Path generation for the square action.

Design pattern: Factory (``PathFactory``) with a registry, so a new shape
(triangle, ...) is added by registering a class - the action server is untouched.
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Dict, List, Type

from turtle_control import constants as C


@dataclass(frozen=True)
class Waypoint:
    x: float
    y: float
    yaw: float


class PathFactory:
    _registry: Dict[str, Type] = {}

    @classmethod
    def register(cls, kind: str):
        def decorator(path_cls):
            cls._registry[kind] = path_cls
            return path_cls
        return decorator

    @classmethod
    def create(cls, kind: str, **kwargs):
        if kind not in cls._registry:
            raise ValueError(f'unknown path kind: {kind}')
        return cls._registry[kind](**kwargs)


@PathFactory.register('square')
class SquarePath:
    """Counter-clockwise square whose bottom-left corner is the start point.
    The 4 waypoints are the 4 corners visited in order (last one = start)."""

    def __init__(self, x0: float, y0: float, side: float):
        if not math.isfinite(side) or side <= 0.0:
            raise ValueError('side length must be > 0')
        self.x0, self.y0, self.side = x0, y0, side
        lo, hi = C.WORLD_MIN + C.SAFE_MARGIN, C.WORLD_MAX - C.SAFE_MARGIN
        for wp in self.waypoints():
            if not (lo <= wp.x <= hi and lo <= wp.y <= hi):
                raise ValueError(
                    f'square of side {side:.2f} from ({x0:.2f}, {y0:.2f}) leaves '
                    f'the window [{lo:.1f}, {hi:.1f}]; max side is {hi - max(x0, y0):.2f}')

    def waypoints(self) -> List[Waypoint]:
        x0, y0, s = self.x0, self.y0, self.side
        return [
            Waypoint(x0 + s, y0, math.pi / 2),
            Waypoint(x0 + s, y0 + s, math.pi),
            Waypoint(x0, y0 + s, -math.pi / 2),
            Waypoint(x0, y0, 0.0),
        ]
