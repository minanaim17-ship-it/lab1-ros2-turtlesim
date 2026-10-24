#!/usr/bin/env python3
"""draw_square action server.

Built ONLY from the other building blocks:
  1. calls the reset_turtle service (clean screen + known start pose)
  2. sends exactly 4 goals to the go_to_pose action server, one per corner,
     waiting for each to finish before sending the next.
"""
import math
import threading
import time

import rclpy
from action_msgs.msg import GoalStatus
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_srvs.srv import Trigger

from turtle_control import constants as C
from turtle_control.waypoints import PathFactory
from turtle_interfaces.action import DrawSquare, GoToPose
from turtle_interfaces.msg import TargetPose


class _Cancelled(Exception):
    pass


class _Failed(Exception):
    pass


class DrawSquareServer(Node):
    def __init__(self):
        super().__init__('draw_square_server')
        self._group = ReentrantCallbackGroup()
        self._busy = False
        self._busy_lock = threading.Lock()

        self.declare_parameter('default_side_length', 3.0)
        self.declare_parameter('start_x', C.START_X)
        self.declare_parameter('start_y', C.START_Y)

        self._reset_client = self.create_client(Trigger, 'reset_turtle',
                                                callback_group=self._group)
        self._goto_client = ActionClient(self, GoToPose, 'go_to_pose',
                                         callback_group=self._group)
        self._server = ActionServer(
            self, DrawSquare, 'draw_square',
            execute_callback=self._execute,
            goal_callback=self._on_goal,
            cancel_callback=lambda _h: CancelResponse.ACCEPT,
            callback_group=self._group)
        self.get_logger().info('draw_square server ready (action: draw_square)')

    # --------------------------------------------------------------- helpers
    def _side(self, requested: float) -> float:
        if requested > 0.0:
            return float(requested)
        return float(self.get_parameter('default_side_length').value)

    def _path(self, side: float):
        return PathFactory.create('square',
                                  x0=float(self.get_parameter('start_x').value),
                                  y0=float(self.get_parameter('start_y').value),
                                  side=side)

    def _wait(self, future, goal_handle, timeout):
        """Block (in the executor thread) until `future` is done."""
        done = threading.Event()
        future.add_done_callback(lambda _f: done.set())
        deadline = time.monotonic() + timeout
        while not done.wait(0.05):
            if goal_handle.is_cancel_requested:
                raise _Cancelled()
            if not rclpy.ok():
                raise _Failed('shutdown')
            if time.monotonic() > deadline:
                raise _Failed('timeout while waiting for a server')
        return future.result()

    # ---------------------------------------------------------------- action
    def _on_goal(self, goal_request):
        side = goal_request.side_length
        if not math.isfinite(side) or side < 0.0:
            self.get_logger().warn('Goal rejected: invalid side length')
            return GoalResponse.REJECT
        try:
            self._path(self._side(side))
        except ValueError as exc:
            self.get_logger().warn(f'Goal rejected: {exc}')
            return GoalResponse.REJECT
        with self._busy_lock:
            if self._busy:
                self.get_logger().warn('Goal rejected: a square is already running')
                return GoalResponse.REJECT
            self._busy = True
        return GoalResponse.ACCEPT

    def _execute(self, goal_handle):
        try:
            return self._run(goal_handle)
        finally:
            with self._busy_lock:
                self._busy = False

    def _run(self, goal_handle):
        result = DrawSquare.Result()
        result.sides_completed = 0
        feedback = DrawSquare.Feedback()
        side = self._side(goal_handle.request.side_length)
        waypoints = self._path(side).waypoints()
        inner = None

        def publish(current, status):
            feedback.current_side = current
            feedback.status = status
            goal_handle.publish_feedback(feedback)

        try:
            publish(0, 'resetting turtle')
            if not self._reset_client.wait_for_service(timeout_sec=5.0):
                raise _Failed('reset_turtle service not available')
            reset = self._wait(self._reset_client.call_async(Trigger.Request()),
                               goal_handle, 15.0)
            if not reset.success:
                raise _Failed(f'reset failed: {reset.message}')

            if not self._goto_client.wait_for_server(timeout_sec=5.0):
                raise _Failed('go_to_pose action server not available')

            for i, wp in enumerate(waypoints, start=1):
                publish(i, f'driving side {i}/4 to corner ({wp.x:.2f}, {wp.y:.2f})')
                goal = GoToPose.Goal()
                goal.target = TargetPose(x=float(wp.x), y=float(wp.y), yaw=float(wp.yaw))
                inner = self._wait(self._goto_client.send_goal_async(goal), goal_handle, 5.0)
                if not inner.accepted:
                    inner = None
                    raise _Failed(f'go_to_pose rejected side {i}')
                wrapped = self._wait(inner.get_result_async(), goal_handle,
                                     C.GOAL_TIMEOUT_SEC + 10.0)
                inner = None
                if wrapped.status != GoalStatus.STATUS_SUCCEEDED or not wrapped.result.success:
                    raise _Failed(f'side {i} failed: {wrapped.result.message}')
                result.sides_completed = i
                self.get_logger().info(f'Side {i}/4 done')

            goal_handle.succeed()
            result.success = True
            result.message = 'Square completed with 4 go_to_pose goals'
            return result

        except _Cancelled:
            if inner is not None:
                inner.cancel_goal_async()
            goal_handle.canceled()
            result.success = False
            result.message = 'Canceled'
            return result
        except _Failed as exc:
            self.get_logger().error(str(exc))
            goal_handle.abort()
            result.success = False
            result.message = str(exc)
            return result


def main(args=None):
    rclpy.init(args=args)
    node = DrawSquareServer()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
