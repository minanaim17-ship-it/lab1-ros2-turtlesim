#!/usr/bin/env python3
"""goToPose action server + speed-limit parameters + set_speed_limits service.

The speed limits are ROS parameters of THIS node (the node that moves the
turtle). They are read on every control tick, so `ros2 param set` and the
`set_speed_limits` service both take effect while the turtle is moving.
"""
import math
import threading
import time

import rclpy
from geometry_msgs.msg import Twist
from rcl_interfaces.msg import SetParametersResult
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from turtlesim.msg import Pose

from turtle_control import constants as C
from turtle_control.motion import (Command, GoToPoseController, Limits, Pose2D,
                                   Tolerances, pose_errors, wrap_angle)
from turtle_interfaces.action import GoToPose
from turtle_interfaces.srv import SetSpeedLimits

SPEED_PARAMS = ('max_linear_speed', 'max_angular_speed')


class GoToPoseServer(Node):
    def __init__(self):
        super().__init__('go_to_pose_server')
        self._group = ReentrantCallbackGroup()
        self._pose = None
        self._pose_lock = threading.Lock()
        self._busy = False
        self._busy_lock = threading.Lock()

        # validation callback first, so launch/CLI values are checked too
        self.add_on_set_parameters_callback(self._on_set_parameters)
        self.declare_parameter('max_linear_speed', C.DEFAULT_MAX_LINEAR)
        self.declare_parameter('max_angular_speed', C.DEFAULT_MAX_ANGULAR)
        self.declare_parameter('position_tolerance', Tolerances.position)
        self.declare_parameter('yaw_tolerance', Tolerances.yaw)

        self._cmd_pub = self.create_publisher(Twist, C.CMD_VEL_TOPIC, 10)
        self.create_subscription(Pose, C.POSE_TOPIC, self._on_pose, 10,
                                 callback_group=self._group)
        self.create_service(SetSpeedLimits, 'set_speed_limits', self._on_set_speed,
                            callback_group=self._group)
        self._server = ActionServer(
            self, GoToPose, 'go_to_pose',
            execute_callback=self._execute,
            goal_callback=self._on_goal,
            cancel_callback=lambda _h: CancelResponse.ACCEPT,
            callback_group=self._group)
        self.get_logger().info('goToPose server ready (action: go_to_pose)')

    # ------------------------------------------------------------ parameters
    def _on_set_parameters(self, params):
        for p in params:
            if p.name not in SPEED_PARAMS:
                continue
            if p.type_ not in (Parameter.Type.DOUBLE, Parameter.Type.INTEGER):
                return SetParametersResult(successful=False,
                                           reason=f'{p.name} must be a number')
            value = float(p.value)
            if not math.isfinite(value) or value <= 0.0:
                return SetParametersResult(
                    successful=False, reason=f'{p.name} must be > 0 (got {value})')
            if value > C.SPEED_UPPER_BOUND:
                return SetParametersResult(
                    successful=False,
                    reason=f'{p.name} must be <= {C.SPEED_UPPER_BOUND} (got {value})')
        return SetParametersResult(successful=True)

    def _on_set_speed(self, request, response):
        result = self.set_parameters_atomically([
            Parameter('max_linear_speed', Parameter.Type.DOUBLE, request.max_linear_speed),
            Parameter('max_angular_speed', Parameter.Type.DOUBLE, request.max_angular_speed),
        ])
        response.success = result.successful
        if result.successful:
            response.message = (f'Speed limits set: linear={request.max_linear_speed:.2f}, '
                                f'angular={request.max_angular_speed:.2f}')
            self.get_logger().info(response.message)
        else:
            response.message = f'Rejected, limits unchanged: {result.reason}'
            self.get_logger().warn(response.message)
        return response

    def _limits(self) -> Limits:
        return Limits(float(self.get_parameter('max_linear_speed').value),
                      float(self.get_parameter('max_angular_speed').value))

    def _tolerances(self) -> Tolerances:
        return Tolerances(position=float(self.get_parameter('position_tolerance').value),
                          yaw=float(self.get_parameter('yaw_tolerance').value))

    # ------------------------------------------------------------------ pose
    def _on_pose(self, msg: Pose):
        with self._pose_lock:
            self._pose = Pose2D(msg.x, msg.y, msg.theta)

    def _get_pose(self):
        with self._pose_lock:
            return self._pose

    def _wait_for_pose(self, timeout=2.0):
        t0 = time.monotonic()
        while self._get_pose() is None and time.monotonic() - t0 < timeout:
            time.sleep(0.05)
        return self._get_pose()

    def _publish(self, cmd: Command):
        msg = Twist()
        msg.linear.x = float(cmd.linear)
        msg.angular.z = float(cmd.angular)
        self._cmd_pub.publish(msg)

    def _stop(self):
        self._publish(Command())

    # ---------------------------------------------------------------- action
    def _on_goal(self, goal_request):
        t = goal_request.target
        if not all(math.isfinite(v) for v in (t.x, t.y, t.yaw)):
            self.get_logger().warn('Goal rejected: non-finite value')
            return GoalResponse.REJECT
        if not (C.WORLD_MIN <= t.x <= C.WORLD_MAX and C.WORLD_MIN <= t.y <= C.WORLD_MAX):
            self.get_logger().warn(
                f'Goal rejected: ({t.x:.2f}, {t.y:.2f}) is outside the window '
                f'[{C.WORLD_MIN}, {C.WORLD_MAX}]')
            return GoalResponse.REJECT
        with self._busy_lock:
            if self._busy:
                self.get_logger().warn('Goal rejected: another goal is running')
                return GoalResponse.REJECT
            self._busy = True
        self.get_logger().info(f'Goal accepted: x={t.x:.2f} y={t.y:.2f} yaw={t.yaw:.2f}')
        return GoalResponse.ACCEPT

    def _execute(self, goal_handle):
        try:
            return self._run(goal_handle)
        finally:
            self._stop()
            with self._busy_lock:
                self._busy = False

    def _run(self, goal_handle):
        target = goal_handle.request.target
        goal = Pose2D(float(target.x), float(target.y), wrap_angle(float(target.yaw)))
        result = GoToPose.Result()
        feedback = GoToPose.Feedback()
        controller = GoToPoseController()

        def finish(ok, message, pose):
            if pose is not None:
                dist, yaw_err = pose_errors(pose, goal)
                result.final_distance_error = float(dist)
                result.final_yaw_error = float(yaw_err)
            result.success = ok
            result.message = message
            return result

        pose = self._wait_for_pose()
        if pose is None:
            goal_handle.abort()
            return finish(False, 'No pose received - is turtlesim running?', None)

        period = 1.0 / C.CONTROL_RATE_HZ
        start = last_fb = time.monotonic()
        while rclpy.ok():
            if goal_handle.is_cancel_requested:
                self._stop()
                goal_handle.canceled()
                return finish(False, 'Canceled', self._get_pose())

            pose = self._get_pose()
            cmd = controller.update(pose, goal, self._limits(), self._tolerances())  # live params
            if controller.finished:
                self._stop()
                goal_handle.succeed()
                msg = 'Pose reached'
                self.get_logger().info(msg)
                return finish(True, msg, pose)

            self._publish(cmd)
            now = time.monotonic()
            if now - last_fb >= C.FEEDBACK_PERIOD_SEC:
                dist, yaw_err = pose_errors(pose, goal)
                feedback.distance_remaining = float(dist)
                feedback.yaw_remaining = float(yaw_err)
                feedback.phase = controller.phase
                goal_handle.publish_feedback(feedback)
                last_fb = now
            if now - start > C.GOAL_TIMEOUT_SEC:
                self._stop()
                goal_handle.abort()
                return finish(False, 'Timeout', pose)
            time.sleep(period)

        goal_handle.abort()
        return finish(False, 'Shutdown', self._get_pose())


def main(args=None):
    rclpy.init(args=args)
    node = GoToPoseServer()
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
