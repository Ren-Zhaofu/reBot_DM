"""Publish validated DM arm feedback as standard ROS 2 JointState messages."""

from __future__ import annotations

import math
import time

import rclpy
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rebotarm_msgs.msg import ArmStatus, JointMotorState
from rebotarm_msgs.srv import SetZero
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger

from .fake_robot import fake_robot_factory
from .hardware_config import load_hardware_config
from .hardware_manager import HardwareManager
from .joint_motion import resolve_tolerances, validate_trajectory
from .read_only_check import validate_channel_ready
from .state import ArmDiagnostics, JointStateSnapshot


class JointStateNode(Node):
    def __init__(self) -> None:
        super().__init__("rebotarm_joint_state")
        self.declare_parameter("channel", "")
        self.declare_parameter("publish_rate", 20.0)
        self.declare_parameter("use_real_hardware", False)
        self.declare_parameter("allow_enable", False)
        self.declare_parameter("allow_safe_home", False)
        self.declare_parameter("allow_calibration", False)
        self.declare_parameter("calibration_mode", False)
        self.declare_parameter("allow_joint_motion", False)
        self.declare_parameter("allow_gravity_compensation", False)

        channel = str(self.get_parameter("channel").value or "")
        publish_rate = float(self.get_parameter("publish_rate").value)
        use_real_hardware = bool(self.get_parameter("use_real_hardware").value)
        self._allow_enable = bool(self.get_parameter("allow_enable").value)
        self._allow_safe_home = bool(self.get_parameter("allow_safe_home").value)
        self._allow_calibration = bool(self.get_parameter("allow_calibration").value)
        self._calibration_mode = bool(self.get_parameter("calibration_mode").value)
        self._allow_joint_motion = bool(
            self.get_parameter("allow_joint_motion").value
        )
        self._allow_gravity_compensation = bool(
            self.get_parameter("allow_gravity_compensation").value
        )
        if not 1.0 <= publish_rate <= 100.0:
            raise ValueError("publish_rate must be between 1 and 100 Hz")

        config = load_hardware_config(model="dm", channel=channel)
        self._manager = HardwareManager(
            config,
            robot_factory=None if use_real_hardware else fake_robot_factory,
        )
        if use_real_hardware:
            validate_channel_ready(self._manager)
        self._manager.connect()

        self._publisher = self.create_publisher(JointState, "joint_states", 10)
        self._joint_publishers = {
            name: self.create_publisher(
                JointMotorState, f"/rebotarm/joints/{name}/state", 10
            )
            for name in config.arm_joints
        }
        status_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self._status_publisher = self.create_publisher(
            ArmStatus, "/rebotarm/arm_status", status_qos
        )
        self._disable_service = self.create_service(
            Trigger, "/rebotarm/disable", self._handle_disable
        )
        self._enable_service = self.create_service(
            Trigger, "/rebotarm/enable", self._handle_enable
        )
        self._safe_home_service = self.create_service(
            Trigger, "/rebotarm/safe_home", self._handle_safe_home
        )
        self._set_zero_service = self.create_service(
            SetZero, "/rebotarm/set_zero", self._handle_set_zero
        )
        self._gravity_start_service = self.create_service(
            Trigger,
            "/rebotarm/gravity_compensation/start",
            self._handle_gravity_start,
        )
        self._gravity_stop_service = self.create_service(
            Trigger,
            "/rebotarm/gravity_compensation/stop",
            self._handle_gravity_stop,
        )
        self._action_group = ReentrantCallbackGroup()
        self._trajectory_action = ActionServer(
            self,
            FollowJointTrajectory,
            "/rebotarm/follow_joint_trajectory",
            execute_callback=self._execute_trajectory,
            goal_callback=self._trajectory_goal,
            cancel_callback=lambda _goal: CancelResponse.ACCEPT,
            callback_group=self._action_group,
        )
        self._timer = self.create_timer(1.0 / publish_rate, self._publish_state)
        self._failed = False
        backend = "real-dm" if use_real_hardware else "fake"
        self.get_logger().info(
            f"joint state publisher started: backend={backend}, "
            f"rate={publish_rate:.1f} Hz, allow_enable={self._allow_enable}"
        )
        self._publish_status()

    def _publish_state(self) -> None:
        if self._failed:
            return
        try:
            snapshot = self._manager.read_state()
            stamp = self.get_clock().now().to_msg()
            self._publisher.publish(snapshot_to_message(snapshot, stamp))
            diagnostics = self._manager.diagnostics()
            for index, name in enumerate(snapshot.names):
                self._joint_publishers[name].publish(
                    snapshot_joint_to_message(
                        snapshot,
                        index,
                        diagnostics.per_joint_status_code[index],
                        stamp,
                    )
                )
            self._publish_status()
        except Exception as exc:
            self._failed = True
            self._timer.cancel()
            self.get_logger().error(f"joint state read stopped: {exc}")
            self._publish_status()

    def _publish_status(self) -> None:
        stamp = self.get_clock().now().to_msg()
        self._status_publisher.publish(
            diagnostics_to_message(self._manager.diagnostics(), stamp)
        )

    def _handle_disable(self, _request, response):
        return handle_disable(self._manager, response, self._publish_status)

    def _handle_enable(self, _request, response):
        return handle_enable(
            self._manager, response, self._allow_enable, self._publish_status
        )

    def _handle_safe_home(self, _request, response):
        if not self._allow_safe_home:
            response.success = False
            response.message = "safe_home is locked; launch with allow_safe_home:=true"
            return response
        try:
            self._manager.safe_home()
            response.success = True
            response.message = "safe_home reached calibrated joint zero"
        except Exception as exc:
            response.success = False
            response.message = str(exc)
        self._publish_status()
        return response

    def _handle_set_zero(self, request, response):
        return handle_set_zero(
            self._manager,
            request,
            response,
            allowed=self._allow_calibration,
            calibration_mode=self._calibration_mode,
            publish_status=self._publish_status,
        )

    def _handle_gravity_start(self, _request, response):
        if not self._allow_gravity_compensation:
            response.success = False
            response.message = (
                "gravity compensation is locked; launch with "
                "allow_gravity_compensation:=true"
            )
            return response
        try:
            self._manager.start_gravity_compensation()
            response.success = True
            response.message = "gravity compensation started"
        except Exception as exc:
            response.success = False
            response.message = str(exc)
        self._publish_status()
        return response

    def _handle_gravity_stop(self, _request, response):
        try:
            self._manager.stop_gravity_compensation()
            response.success = True
            response.message = "gravity compensation stopped; holding current position"
        except Exception as exc:
            response.success = False
            response.message = str(exc)
        self._publish_status()
        return response

    def _trajectory_goal(self, goal_request):
        if not self._allow_joint_motion:
            self.get_logger().warning("joint trajectory rejected: motion is locked")
            return GoalResponse.REJECT
        try:
            current = self._manager.read_state().position
            validate_trajectory(
                goal_request.trajectory,
                self._manager.config.arm_joints,
                current,
            )
            resolve_tolerances(goal_request, self._manager.config.arm_joints)
        except Exception as exc:
            self.get_logger().warning(f"joint trajectory rejected: {exc}")
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def _execute_trajectory(self, goal_handle):
        result = FollowJointTrajectory.Result()
        try:
            action_started = time.monotonic()
            current = self._manager.read_state().position
            validated = validate_trajectory(
                goal_handle.request.trajectory,
                self._manager.config.arm_joints,
                current,
            )
            tolerances = resolve_tolerances(
                goal_handle.request, self._manager.config.arm_joints
            )

            def publish_feedback(desired, actual):
                message = trajectory_feedback_message(
                    self._manager.config.arm_joints,
                    desired,
                    actual,
                    time.monotonic() - action_started,
                    self.get_clock().now().to_msg(),
                )
                goal_handle.publish_feedback(message)

            previous_time = 0.0
            for target, waypoint_time in zip(validated.targets, validated.times):
                completed = self._manager.move_to_joint_target(
                    target,
                    should_cancel=lambda: goal_handle.is_cancel_requested,
                    feedback=publish_feedback,
                    desired_duration=waypoint_time - previous_time,
                    path_tolerance=tolerances.path,
                    goal_tolerance=tolerances.goal,
                    settle_timeout=2.0 + tolerances.goal_time,
                )
                if not completed:
                    goal_handle.canceled()
                    result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
                    result.error_string = "trajectory canceled; holding current position"
                    return result
                previous_time = waypoint_time
            goal_handle.succeed()
            result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
            result.error_string = "trajectory completed"
        except Exception as exc:
            goal_handle.abort()
            result.error_code = FollowJointTrajectory.Result.PATH_TOLERANCE_VIOLATED
            result.error_string = str(exc)
        self._publish_status()
        return result

    def close(self) -> None:
        self._manager.close()


def snapshot_to_message(snapshot: JointStateSnapshot, stamp) -> JointState:
    message = JointState()
    message.header.stamp = stamp
    message.name = list(snapshot.names)
    message.position = list(snapshot.position)
    message.velocity = list(snapshot.velocity)
    message.effort = list(snapshot.effort)
    return message


def trajectory_feedback_message(
    joint_names,
    desired,
    actual,
    elapsed_seconds: float,
    stamp,
) -> FollowJointTrajectory.Feedback:
    """Create complete position feedback for FollowJointTrajectory clients."""
    names = tuple(joint_names)
    wanted = tuple(float(value) for value in desired)
    measured = tuple(float(value) for value in actual)
    if not names or len(names) != len(wanted) or len(names) != len(measured):
        raise ValueError("feedback names, desired and actual lengths must match")
    if elapsed_seconds < 0.0 or not math.isfinite(elapsed_seconds):
        raise ValueError("elapsed_seconds must be finite and non-negative")

    whole_seconds = int(elapsed_seconds)
    nanoseconds = int(round((elapsed_seconds - whole_seconds) * 1_000_000_000))
    if nanoseconds == 1_000_000_000:
        whole_seconds += 1
        nanoseconds = 0
    elapsed = Duration(sec=whole_seconds, nanosec=nanoseconds)

    message = FollowJointTrajectory.Feedback()
    message.header.stamp = stamp
    message.joint_names = list(names)
    message.desired.positions = list(wanted)
    message.actual.positions = list(measured)
    message.error.positions = [
        target - position for target, position in zip(wanted, measured)
    ]
    message.desired.time_from_start = elapsed
    message.actual.time_from_start = elapsed
    message.error.time_from_start = elapsed
    return message


def diagnostics_to_message(diagnostics: ArmDiagnostics, stamp) -> ArmStatus:
    message = ArmStatus()
    message.header.stamp = stamp
    message.mode = diagnostics.mode
    message.enabled = diagnostics.enabled
    message.control_loop_active = diagnostics.control_loop_active
    message.state_machine = diagnostics.state_machine
    message.joint_names = list(diagnostics.joint_names)
    message.per_joint_status_code = list(diagnostics.per_joint_status_code)
    message.error_codes = list(diagnostics.error_codes)
    return message


def snapshot_joint_to_message(
    snapshot: JointStateSnapshot,
    index: int,
    status_code: int,
    stamp,
) -> JointMotorState:
    if not 0 <= index < len(snapshot.names):
        raise IndexError(f"joint index out of range: {index}")
    if not 0 <= status_code <= 255:
        raise ValueError(f"status code must fit uint8: {status_code}")
    message = JointMotorState()
    message.header.stamp = stamp
    message.joint_name = snapshot.names[index]
    message.position = snapshot.position[index]
    message.velocity = snapshot.velocity[index]
    message.torque = snapshot.effort[index]
    message.status_code = status_code
    return message


def handle_disable(manager: HardwareManager, response, publish_status=lambda: None):
    try:
        manager.disable()
        response.success = True
        response.message = "DM controller disable command completed"
    except Exception as exc:
        response.success = False
        response.message = str(exc)
    publish_status()
    return response


def handle_enable(
    manager: HardwareManager,
    response,
    allowed: bool,
    publish_status=lambda: None,
):
    if not allowed:
        response.success = False
        response.message = "enable is locked; launch with allow_enable:=true"
        publish_status()
        return response
    try:
        manager.enable()
        response.success = True
        response.message = "DM controller enable command completed"
    except Exception as exc:
        response.success = False
        response.message = str(exc)
    publish_status()
    return response


def handle_set_zero(
    manager: HardwareManager,
    request,
    response,
    *,
    allowed: bool,
    calibration_mode: bool,
    publish_status=lambda: None,
):
    if not allowed or not calibration_mode:
        response.success = False
        response.message = (
            "set_zero is locked; both allow_calibration and calibration_mode "
            "must be true"
        )
        publish_status()
        return response
    try:
        manager.set_zero(request.joint_name)
        response.success = True
        response.message = f"{request.joint_name} zero calibration verified"
    except Exception as exc:
        response.success = False
        response.message = str(exc)
    publish_status()
    return response


def main(args=None) -> None:
    rclpy.init(args=args)
    node = JointStateNode()
    try:
        executor = MultiThreadedExecutor(num_threads=3)
        executor.add_node(node)
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        # ros2 launch may forward a second SIGINT while cleanup is already in
        # progress. Continue through every safety cleanup step in that case.
        for cleanup in (
            node._trajectory_action.destroy,
            node.close,
            node.destroy_node,
        ):
            try:
                cleanup()
            except KeyboardInterrupt:
                continue
        if rclpy.ok():
            try:
                rclpy.shutdown()
            except KeyboardInterrupt:
                pass
