"""Collision-aware Cartesian MoveToPose bridge backed by MoveIt."""

from __future__ import annotations

import math
import time

import rclpy
from geometry_msgs.msg import Pose
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    BoundingVolume,
    Constraints,
    MoveItErrorCodes,
    OrientationConstraint,
    PositionConstraint,
)
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rebotarm_msgs.action import MoveToPose
from shape_msgs.msg import SolidPrimitive
from tf2_ros import Buffer, TransformListener


def validate_cartesian_goal(pose: Pose, duration: float) -> None:
    values = (
        pose.position.x,
        pose.position.y,
        pose.position.z,
        pose.orientation.x,
        pose.orientation.y,
        pose.orientation.z,
        pose.orientation.w,
        duration,
    )
    if any(not math.isfinite(value) for value in values):
        raise ValueError("Cartesian goal contains a non-finite value")
    if not 0.5 <= duration <= 30.0:
        raise ValueError("duration must be between 0.5 and 30 seconds")
    norm = math.sqrt(
        pose.orientation.x**2
        + pose.orientation.y**2
        + pose.orientation.z**2
        + pose.orientation.w**2
    )
    if abs(norm - 1.0) > 0.01:
        raise ValueError("target quaternion must be normalized")
    if not -0.8 <= pose.position.x <= 0.8:
        raise ValueError("target x is outside the guarded workspace")
    if not -0.8 <= pose.position.y <= 0.8:
        raise ValueError("target y is outside the guarded workspace")
    if not -0.3 <= pose.position.z <= 0.9:
        raise ValueError("target z is outside the guarded workspace")
    radius = math.sqrt(
        pose.position.x**2 + pose.position.y**2 + pose.position.z**2
    )
    if radius > 1.0:
        raise ValueError("target is outside the 1.0 m guarded radius")


def pose_constraints(pose: Pose) -> Constraints:
    region = BoundingVolume()
    sphere = SolidPrimitive(type=SolidPrimitive.SPHERE, dimensions=[0.005])
    region.primitives = [sphere]
    region.primitive_poses = [pose]
    position = PositionConstraint()
    position.header.frame_id = "base_link"
    position.link_name = "end_link"
    position.constraint_region = region
    position.weight = 1.0
    orientation = OrientationConstraint()
    orientation.header.frame_id = "base_link"
    orientation.link_name = "end_link"
    orientation.orientation = pose.orientation
    orientation.absolute_x_axis_tolerance = 0.02
    orientation.absolute_y_axis_tolerance = 0.02
    orientation.absolute_z_axis_tolerance = 0.02
    orientation.weight = 1.0
    return Constraints(
        name="rebotarm_cartesian_goal",
        position_constraints=[position],
        orientation_constraints=[orientation],
    )


class CartesianBridge(Node):
    def __init__(self) -> None:
        super().__init__("rebotarm_cartesian_bridge")
        self.declare_parameter("allow_cartesian_motion", False)
        self._allowed = bool(self.get_parameter("allow_cartesian_motion").value)
        self._group = ReentrantCallbackGroup()
        self._move_group = ActionClient(
            self, MoveGroup, "/move_action", callback_group=self._group
        )
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._server = ActionServer(
            self,
            MoveToPose,
            "/rebotarm/move_to_pose",
            goal_callback=self._goal_callback,
            cancel_callback=lambda _goal: CancelResponse.ACCEPT,
            execute_callback=self._execute,
            callback_group=self._group,
        )
        self.get_logger().info(
            f"Cartesian bridge started: allow_cartesian_motion={self._allowed}"
        )

    def _goal_callback(self, request):
        if not self._allowed:
            self.get_logger().warning("Cartesian goal rejected: motion is locked")
            return GoalResponse.REJECT
        try:
            validate_cartesian_goal(request.target_pose, float(request.duration))
        except Exception as exc:
            self.get_logger().warning(f"Cartesian goal rejected: {exc}")
            return GoalResponse.REJECT
        if not self._move_group.server_is_ready():
            self.get_logger().warning("Cartesian goal rejected: MoveIt is unavailable")
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def _execute(self, goal_handle):
        request = goal_handle.request
        result = MoveToPose.Result()
        goal = MoveGroup.Goal()
        goal.request.group_name = "arm"
        goal.request.num_planning_attempts = 5
        goal.request.allowed_planning_time = min(float(request.duration), 10.0)
        goal.request.max_velocity_scaling_factor = 0.2
        goal.request.max_acceleration_scaling_factor = 0.2
        goal.request.start_state.is_diff = True
        goal.request.goal_constraints = [pose_constraints(request.target_pose)]
        goal.planning_options.plan_only = False

        sent = self._move_group.send_goal_async(goal)
        if not self._wait_future(sent, 10.0):
            return self._abort(goal_handle, result, "timed out sending goal to MoveIt")
        move_handle = sent.result()
        if move_handle is None or not move_handle.accepted:
            return self._abort(goal_handle, result, "MoveIt rejected Cartesian goal")

        started = time.monotonic()
        move_result = move_handle.get_result_async()
        while not move_result.done():
            if goal_handle.is_cancel_requested:
                move_handle.cancel_goal_async()
                goal_handle.canceled()
                result.success = False
                result.message = "Cartesian motion canceled"
                result.final_pose = self._current_pose()
                return result
            elapsed = time.monotonic() - started
            feedback = MoveToPose.Feedback()
            feedback.current_pose = self._current_pose()
            feedback.time_elapsed = elapsed
            feedback.progress = min(elapsed / float(request.duration), 0.99)
            goal_handle.publish_feedback(feedback)
            if elapsed > float(request.duration) + 15.0:
                move_handle.cancel_goal_async()
                return self._abort(goal_handle, result, "Cartesian motion timed out")
            time.sleep(0.1)

        wrapped = move_result.result()
        if wrapped is None or wrapped.result.error_code.val != MoveItErrorCodes.SUCCESS:
            code = wrapped.result.error_code.val if wrapped is not None else "none"
            return self._abort(goal_handle, result, f"MoveIt error_code={code}")
        goal_handle.succeed()
        result.success = True
        result.message = "Cartesian pose reached through MoveIt"
        result.final_pose = self._current_pose()
        return result

    @staticmethod
    def _wait_future(future, timeout: float) -> bool:
        deadline = time.monotonic() + timeout
        while not future.done() and time.monotonic() < deadline:
            time.sleep(0.02)
        return future.done()

    def _current_pose(self) -> Pose:
        try:
            transform = self._tf_buffer.lookup_transform(
                "base_link", "end_link", rclpy.time.Time()
            ).transform
            pose = Pose()
            pose.position.x = transform.translation.x
            pose.position.y = transform.translation.y
            pose.position.z = transform.translation.z
            pose.orientation = transform.rotation
            return pose
        except Exception:
            return Pose()

    def _abort(self, goal_handle, result, message):
        goal_handle.abort()
        result.success = False
        result.message = message
        result.final_pose = self._current_pose()
        return result

    def close(self) -> None:
        self._server.destroy()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CartesianBridge()
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
