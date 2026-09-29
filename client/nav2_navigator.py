"""Navigator backed by ROS 2 Nav2: NavigateToPose goals, pose from TF.

This is the only module that imports ROS. rclpy runs in a background thread;
its callbacks record the goal result, which the executor reads via poll().

Each goal gets a sequence number. Callbacks from an older goal (for example
one Nav2 preempted when we sent a new goal) are ignored.
"""
from __future__ import annotations

import logging
import math
import threading

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener

from client.navigator import NavResult
from vda5050.messages import Position

log = logging.getLogger(__name__)


class Nav2Navigator:
    def __init__(
        self,
        map_frame: str = "map",
        base_frame: str = "base_footprint",
        action_name: str = "navigate_to_pose",
        server_timeout_s: float = 5.0,
    ) -> None:
        self._map_frame, self._base_frame = map_frame, base_frame
        self._action_name = action_name
        self._server_timeout = server_timeout_s

        rclpy.init()
        # use_sim_time: Gazebo publishes its own clock; TF and Nav2 run on it.
        self._node = Node("vda5050_client", parameter_overrides=[Parameter("use_sim_time", value=True)])
        self._action = ActionClient(self._node, NavigateToPose, action_name)
        self._tf = Buffer()
        self._tf_listener = TransformListener(self._tf, self._node)
        self._executor = SingleThreadedExecutor()
        self._executor.add_node(self._node)
        self._spin_thread = threading.Thread(target=self._executor.spin, daemon=True)
        self._spin_thread.start()

        self._lock = threading.Lock()
        self._goal_seq = 0
        self._goal_handle = None
        self._cancel_requested = False
        self._result: NavResult | None = None

    def go_to(self, x: float, y: float, theta: float | None) -> None:
        with self._lock:
            self._goal_seq += 1
            seq = self._goal_seq
            self._goal_handle, self._cancel_requested, self._result = None, False, None

        if not self._action.wait_for_server(timeout_sec=self._server_timeout):
            log.error("Nav2 action server '%s' not available", self._action_name)
            self._set_result(seq, NavResult.FAILED)
            return

        if theta is None:
            current = self.pose()
            theta = current.theta if current else 0.0
        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = self._map_frame   # stamp left at 0 = "use latest transform"
        goal.pose.pose.position.x, goal.pose.pose.position.y = float(x), float(y)
        goal.pose.pose.orientation.z = math.sin(theta / 2.0)
        goal.pose.pose.orientation.w = math.cos(theta / 2.0)

        future = self._action.send_goal_async(goal)
        future.add_done_callback(lambda f: self._on_goal_response(seq, f))

    def poll(self) -> NavResult | None:
        with self._lock:
            return self._result

    def cancel(self) -> None:
        with self._lock:
            self._cancel_requested = True
            handle = self._goal_handle
        if handle is not None:
            handle.cancel_goal_async()
        # else: the goal response hasn't arrived yet; _on_goal_response cancels it

    def pose(self) -> Position | None:
        try:
            t = self._tf.lookup_transform(self._map_frame, self._base_frame, Time())
        except TransformException:
            return None   # not localized yet (no 2D Pose Estimate), or TF not up
        q = t.transform.rotation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        return Position(t.transform.translation.x, t.transform.translation.y, yaw)

    def close(self) -> None:
        # Order matters: stop spinning and wait for the spin thread to exit
        # *before* destroying anything it might still be using.
        self._executor.shutdown(timeout_sec=1.0)
        self._spin_thread.join(timeout=2.0)
        self._action.destroy()
        self._node.destroy_node()
        rclpy.try_shutdown()

    # ---- callbacks, run on the rclpy thread --------------------------------------

    def _on_goal_response(self, seq: int, future) -> None:
        handle = future.result()
        if not handle.accepted:
            log.error("Nav2 rejected the goal")
            self._set_result(seq, NavResult.FAILED)
            return
        with self._lock:
            if seq != self._goal_seq:
                return
            self._goal_handle = handle
            cancel_now = self._cancel_requested
        if cancel_now:
            handle.cancel_goal_async()
        handle.get_result_async().add_done_callback(lambda f: self._on_result(seq, f))

    def _on_result(self, seq: int, future) -> None:
        status = future.result().status
        result = {
            GoalStatus.STATUS_SUCCEEDED: NavResult.SUCCEEDED,
            GoalStatus.STATUS_CANCELED: NavResult.CANCELED,
        }.get(status, NavResult.FAILED)
        if result is NavResult.FAILED:
            log.warning("Nav2 goal ended with status %d", status)
        self._set_result(seq, result)

    def _set_result(self, seq: int, result: NavResult) -> None:
        with self._lock:
            if seq == self._goal_seq:
                self._result = result
