#!/usr/bin/env python3
"""reset_turtle service (std_srvs/Trigger).

Uses ONLY turtlesim's own services:
  set_pen(off) -> teleport_absolute(start pose) -> clear -> set_pen(on)

The pen is switched off first so no line is drawn during the teleport, and we
wait a moment before switching it on again because turtlesim applies a teleport
in its own 60 Hz update loop (switching the pen on too early would draw the line).
"""
import threading
import time

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_srvs.srv import Empty, Trigger
from turtlesim.srv import SetPen, TeleportAbsolute

from turtle_control import constants as C


class ResetService(Node):
    def __init__(self):
        super().__init__('reset_service')
        self._group = ReentrantCallbackGroup()
        self._lock = threading.Lock()          # serialise overlapping resets
        self._clear = self.create_client(Empty, C.CLEAR_SERVICE, callback_group=self._group)
        self._teleport = self.create_client(TeleportAbsolute, C.TELEPORT_SERVICE,
                                            callback_group=self._group)
        self._pen = self.create_client(SetPen, C.SET_PEN_SERVICE, callback_group=self._group)
        self.create_service(Trigger, 'reset_turtle', self._on_reset, callback_group=self._group)
        self.get_logger().info('reset_turtle service ready')

    def _call(self, client, request, timeout=3.0):
        if not client.wait_for_service(timeout_sec=timeout):
            raise RuntimeError(f'service {client.srv_name} not available')
        done = threading.Event()
        future = client.call_async(request)
        future.add_done_callback(lambda _f: done.set())
        if not done.wait(timeout):
            client.remove_pending_request(future)
            raise RuntimeError(f'service {client.srv_name} timed out')
        return future.result()

    def _set_pen(self, off: bool):
        req = SetPen.Request()
        req.r, req.g, req.b = C.PEN_R, C.PEN_G, C.PEN_B
        req.width = C.PEN_WIDTH
        req.off = 1 if off else 0
        self._call(self._pen, req)

    def _on_reset(self, _request, response):
        with self._lock:
            pen_is_off = False
            try:
                self._set_pen(off=True)
                pen_is_off = True
                tp = TeleportAbsolute.Request()
                tp.x, tp.y, tp.theta = C.START_X, C.START_Y, C.START_THETA
                self._call(self._teleport, tp)
                time.sleep(C.TELEPORT_SETTLE_SEC)
                self._call(self._clear, Empty.Request())
                response.success = True
                response.message = 'Screen cleared, turtle back at start pose'
            except Exception as exc:  # noqa: BLE001 - report any failure to the caller
                response.success = False
                response.message = f'Reset failed: {exc}'
                self.get_logger().error(response.message)
            finally:
                if pen_is_off:
                    try:
                        self._set_pen(off=False)
                    except Exception as exc:  # noqa: BLE001
                        self.get_logger().error(f'Could not restore pen: {exc}')
        return response


def main(args=None):
    rclpy.init(args=args)
    node = ResetService()
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
