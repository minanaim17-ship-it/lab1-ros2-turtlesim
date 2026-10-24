"""Closed-loop tests: the controller drives a simulated turtle (same kinematics
as turtlesim: x += v*cos(th)*dt, th += w*dt)."""
import math

import pytest

from turtle_control import constants as C
from turtle_control.motion import (GoToPoseController, Limits, Pose2D,
                                   Tolerances, pose_errors, wrap_angle)
from turtle_control.waypoints import PathFactory

DT = 1.0 / C.CONTROL_RATE_HZ
START = Pose2D(C.START_X, C.START_Y, 0.0)


def simulate(start, goal, limits, tol=Tolerances(), max_t=60.0, limits_at=None):
    """Return (final_pose, elapsed). Asserts the speed limits are never exceeded."""
    ctrl, pose, t = GoToPoseController(), start, 0.0
    while t < max_t:
        lim = limits_at(t) if limits_at else limits
        cmd = ctrl.update(pose, goal, lim, tol)
        if ctrl.finished:
            return pose, t
        assert abs(cmd.linear) <= lim.max_linear + 1e-9
        assert abs(cmd.angular) <= lim.max_angular + 1e-9
        th = pose.theta
        pose = Pose2D(pose.x + cmd.linear * math.cos(th) * DT,
                      pose.y + cmd.linear * math.sin(th) * DT,
                      wrap_angle(th + cmd.angular * DT))
        t += DT
    raise AssertionError('goal not reached in time')


@pytest.mark.parametrize('gx,gy,gyaw', [
    (8.5, 5.5, 0.0),      # right
    (2.5, 5.5, 3.0),      # left (behind the turtle)
    (5.5, 9.5, 1.57),     # up
    (5.5, 1.5, -1.57),    # down
    (9.0, 9.0, -2.5),     # diagonal
    (1.0, 1.0, 3.14),     # far corner
    (5.54, 5.54, 2.0),    # only rotate
])
def test_reaches_pose_in_all_directions(gx, gy, gyaw):
    goal = Pose2D(gx, gy, wrap_angle(gyaw))
    final, _ = simulate(START, goal, Limits(2.0, 2.0))
    dist, yaw_err = pose_errors(final, goal)
    assert dist < 0.1 and abs(yaw_err) < 0.05      # assignment tolerances


def test_lower_speed_is_slower():
    goal = Pose2D(9.0, 5.5, 0.0)
    _, t_fast = simulate(START, goal, Limits(2.0, 2.0))
    _, t_slow = simulate(START, goal, Limits(0.5, 0.5))
    assert t_slow > 2.0 * t_fast


def test_live_speed_change_takes_effect():
    goal = Pose2D(10.0, 5.5, 0.0)
    _, t_const = simulate(START, goal, Limits(2.0, 2.0))
    _, t_changed = simulate(START, goal, None,
                            limits_at=lambda t: Limits(2.0, 2.0) if t < 1.0 else Limits(0.3, 0.3))
    assert t_changed > t_const + 2.0


def test_square_four_goals_equal_sides_and_right_angles():
    side = 3.0
    path = PathFactory.create('square', x0=C.START_X, y0=C.START_Y, side=side)
    wps = path.waypoints()
    assert len(wps) == 4
    pose, corners = START, [(START.x, START.y)]
    for wp in wps:
        goal = Pose2D(wp.x, wp.y, wp.yaw)
        pose, _ = simulate(pose, goal, Limits(2.0, 2.0))
        dist, yaw_err = pose_errors(pose, goal)
        assert dist < 0.1 and abs(yaw_err) < 0.05
        corners.append((pose.x, pose.y))
    sides = [math.dist(corners[i], corners[i + 1]) for i in range(4)]
    assert all(abs(s - side) < 0.15 for s in sides)
    assert math.dist(corners[0], corners[4]) < 0.15       # closed loop


def test_square_rejects_out_of_window_and_bad_values():
    with pytest.raises(ValueError):
        PathFactory.create('square', x0=C.START_X, y0=C.START_Y, side=6.0)
    with pytest.raises(ValueError):
        PathFactory.create('square', x0=C.START_X, y0=C.START_Y, side=-1.0)
    with pytest.raises(ValueError):
        PathFactory.create('triangle')
    PathFactory.create('square', x0=C.START_X, y0=C.START_Y, side=5.0)   # max OK
