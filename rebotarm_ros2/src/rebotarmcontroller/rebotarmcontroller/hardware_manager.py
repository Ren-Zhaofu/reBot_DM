"""Explicit, thread-safe lifecycle boundary around the hardware SDK."""

from __future__ import annotations

import argparse
import shutil
import tempfile
import threading
import time
from enum import Enum
from pathlib import Path
from typing import Any, Callable, Protocol

import yaml

from .hardware_config import HardwareConfig, load_hardware_config
from .paths import WorkspaceLayout
from .sdk_loader import load_sdk
from .state import ArmDiagnostics, JointStateSnapshot
from .trajectory import plan_minimum_jerk, plan_safe_home


class HardwareState(str, Enum):
    CREATED = "created"
    CONNECTING = "connecting"
    CONNECTED = "connected"
    DISCONNECTING = "disconnecting"
    DISCONNECTED = "disconnected"
    FAULT = "fault"


class Robot(Protocol):
    def connect(self) -> None: ...

    def disconnect(self) -> None: ...

    def get_state(self) -> tuple[Any, Any, Any]: ...

    def get_joint_status_codes(self, names: tuple[str, ...]) -> tuple[int, ...]: ...

    def disable_all(self) -> None: ...

    def enable_all(self) -> None: ...

    def enable_position_hold(
        self, positions: tuple[float, ...], max_velocity: float
    ) -> None: ...

    def send_position_target(
        self, positions: tuple[float, ...], max_velocity: float
    ) -> None: ...

    def set_zero(self, joint_name: str) -> None: ...

    def enter_gravity_compensation(
        self, positions, kp, kd, torque
    ) -> None: ...

    def send_gravity_command(self, positions, kp, kd, torque) -> None: ...


RobotFactory = Callable[[HardwareConfig], Robot]


class SdkRobotFactory:
    """Create SDK robots from merged configuration without connecting them."""

    def __init__(self) -> None:
        self._runtime_directories: list[Path] = []

    def __call__(self, config: HardwareConfig) -> Robot:
        sdk = load_sdk()
        runtime_dir = Path(tempfile.mkdtemp(prefix="rebotarm_config_"))
        runtime_yaml = runtime_dir / f"{config.model}.yaml"
        runtime_yaml.write_text(
            yaml.safe_dump(config.data, sort_keys=False), encoding="utf-8"
        )
        self._runtime_directories.append(runtime_dir)
        return SdkRobot(sdk.actuator.RebotArm(hw_yaml=str(runtime_yaml)))

    def cleanup(self) -> None:
        for directory in self._runtime_directories:
            shutil.rmtree(directory, ignore_errors=True)
        self._runtime_directories.clear()


class SdkRobot:
    """Narrow adapter around SDK APIs used by the read-only ROS layer."""

    def __init__(self, robot) -> None:
        self._robot = robot

    def connect(self) -> None:
        self._robot.connect()

    def disconnect(self) -> None:
        self._robot.disconnect()

    def get_state(self):
        return self._robot.get_state()

    def get_joint_status_codes(self, names: tuple[str, ...]) -> tuple[int, ...]:
        codes = []
        for name in names:
            motor = self._robot._motor_map.get(name)
            state = motor.get_state() if motor is not None else None
            codes.append(int(state.status_code) if state is not None else 255)
        return tuple(codes)

    def disable_all(self) -> None:
        # Upstream JointGroup.disable() catches CallError and only prints it.
        # Call each unique bus controller here so a failed disable reaches ROS.
        controllers = tuple(self._robot._ctrl_map.values())
        if not controllers:
            raise RuntimeError("SDK has no connected bus controller")
        for controller in controllers:
            controller.disable_all()

    def enable_all(self) -> None:
        # As with disable, bypass upstream exception swallowing.
        controllers = tuple(self._robot._ctrl_map.values())
        if not controllers:
            raise RuntimeError("SDK has no connected bus controller")
        for controller in controllers:
            controller.enable_all()

    def enable_position_hold(
        self, positions: tuple[float, ...], max_velocity: float
    ) -> None:
        import numpy as np

        arm = self._robot.groups.get("arm")
        if arm is None:
            raise RuntimeError("SDK has no arm group")
        target = np.asarray(positions, dtype=np.float64)
        velocity_limit = np.full(len(positions), max_velocity, dtype=np.float64)
        if not arm.mode_pos_vel(vlim=velocity_limit):
            raise RuntimeError("DM arm did not enter POS_VEL mode")
        # Prime the current-position target before enabling, then resend it
        # immediately after enable to avoid using any stale target.
        arm.send_pos_vel(target, vlim=velocity_limit)
        self.enable_all()
        arm.send_pos_vel(target, vlim=velocity_limit)

    def send_position_target(
        self, positions: tuple[float, ...], max_velocity: float
    ) -> None:
        import numpy as np

        arm = self._robot.groups.get("arm")
        if arm is None:
            raise RuntimeError("SDK has no arm group")
        target = np.asarray(positions, dtype=np.float64)
        velocity_limit = np.full(len(positions), max_velocity, dtype=np.float64)
        arm.send_pos_vel(target, vlim=velocity_limit)

    def set_zero(self, joint_name: str) -> None:
        motor = self._robot._motor_map.get(joint_name)
        if motor is None:
            raise KeyError(f"unknown SDK motor: {joint_name}")
        self.disable_all()
        controllers = tuple(self._robot._ctrl_map.values())
        deadline = time.monotonic() + 1.0
        while time.monotonic() < deadline:
            motor.request_feedback()
            for controller in controllers:
                controller.poll_feedback_once()
            state = motor.get_state()
            if state is not None and int(state.status_code) == 0:
                break
            time.sleep(0.02)
        else:
            raise TimeoutError(f"no healthy feedback before zeroing {joint_name}")
        motor.set_zero_position()

    def enter_gravity_compensation(self, positions, kp, kd, torque) -> None:
        import numpy as np

        arm = self._robot.groups.get("arm")
        if arm is None:
            raise RuntimeError("SDK has no arm group")
        kp_array = np.asarray(kp, dtype=np.float64)
        kd_array = np.asarray(kd, dtype=np.float64)
        if not arm.mode_mit(kp=kp_array, kd=kd_array):
            raise RuntimeError("DM arm did not enter MIT mode")
        self.disable_all()
        self.send_gravity_command(positions, kp_array, kd_array, torque)
        self.enable_all()
        self.send_gravity_command(positions, kp_array, kd_array, torque)

    def send_gravity_command(self, positions, kp, kd, torque) -> None:
        import numpy as np

        arm = self._robot.groups.get("arm")
        if arm is None:
            raise RuntimeError("SDK has no arm group")
        position = np.asarray(positions, dtype=np.float64)
        kp_array = np.asarray(kp, dtype=np.float64)
        kd_array = np.asarray(kd, dtype=np.float64)
        torque_array = np.asarray(torque, dtype=np.float64)
        velocity = np.zeros(len(position), dtype=np.float64)
        # JointGroup.send_mit() suppresses bus exceptions. Address motors
        # directly so a failed gravity command reaches the safety manager.
        for index, joint in enumerate(arm._jcfgs):
            arm._mm[joint.name].send_mit(
                float(position[index]),
                float(velocity[index]),
                float(kp_array[index]),
                float(kd_array[index]),
                float(torque_array[index]),
            )


class HardwareManager:
    """Own one SDK robot and enforce its connection lifecycle."""

    def __init__(
        self,
        config: HardwareConfig | None = None,
        *,
        robot_factory: RobotFactory | None = None,
    ) -> None:
        self.config = config or load_hardware_config()
        self._factory = robot_factory or SdkRobotFactory()
        self._robot: Robot | None = None
        self._state = HardwareState.CREATED
        self._lock = threading.RLock()
        self._errors: list[str] = []
        self._enabled = False
        self._hold_target: tuple[float, ...] | None = None
        self._operation = "IDLE"
        self._gravity_active = False
        self._gravity_stop = threading.Event()
        self._gravity_thread: threading.Thread | None = None
        self._gravity_model = None
        self._gravity_data = None
        self._gravity_compute = None

    @property
    def state(self) -> HardwareState:
        with self._lock:
            return self._state

    @property
    def connected(self) -> bool:
        return self.state is HardwareState.CONNECTED

    def connect(self) -> None:
        with self._lock:
            if self._state is HardwareState.CONNECTED:
                return
            if self._state in (HardwareState.CONNECTING, HardwareState.DISCONNECTING):
                raise RuntimeError(f"cannot connect while state={self._state.value}")

            self._state = HardwareState.CONNECTING
            robot: Robot | None = None
            try:
                robot = self._factory(self.config)
                robot.connect()
                self._wait_for_complete_feedback(robot)
            except Exception:
                if robot is not None:
                    try:
                        robot.disconnect()
                    except Exception:
                        pass
                self._robot = None
                self._state = HardwareState.FAULT
                self._enabled = False
                self._hold_target = None
                self._record_error("CONNECT_FAILED")
                raise

            self._robot = robot
            self._state = HardwareState.CONNECTED
            self._enabled = False
            self._hold_target = None

    def _wait_for_complete_feedback(
        self, robot: Robot, timeout: float = 1.0, poll_interval: float = 0.02
    ) -> None:
        """Wait until every arm motor has returned at least one valid frame."""
        deadline = time.monotonic() + timeout
        while True:
            robot.get_state()
            status_codes = robot.get_joint_status_codes(self.config.arm_joints)
            if len(status_codes) == len(self.config.arm_joints) and all(
                code != 255 for code in status_codes
            ):
                return
            if time.monotonic() >= deadline:
                raise TimeoutError("timed out waiting for complete arm feedback")
            time.sleep(poll_interval)

    def disconnect(self) -> None:
        with self._lock:
            if self._state in (HardwareState.CREATED, HardwareState.DISCONNECTED):
                self._state = HardwareState.DISCONNECTED
                return
            if self._state is HardwareState.DISCONNECTING:
                return

            robot = self._robot
            self._state = HardwareState.DISCONNECTING
            self._gravity_active = False
            self._gravity_stop.set()
            try:
                if robot is not None:
                    robot.disconnect()
            except Exception:
                self._state = HardwareState.FAULT
                self._record_error("DISCONNECT_FAILED")
                raise
            finally:
                self._robot = None
                self._enabled = False
                self._hold_target = None

            self._state = HardwareState.DISCONNECTED

    def read_state(self) -> JointStateSnapshot:
        with self._lock:
            if self._state is not HardwareState.CONNECTED or self._robot is None:
                raise RuntimeError(
                    f"cannot read hardware while state={self._state.value}"
                )
            try:
                position, velocity, effort = self._robot.get_state()
                snapshot = JointStateSnapshot.from_sdk(
                    names=self.config.arm_joints,
                    position=position,
                    velocity=velocity,
                    effort=effort,
                    monotonic_ns=time.monotonic_ns(),
                )
                self._enforce_hold_error(snapshot)
                return snapshot
            except Exception:
                self._record_error("STATE_READ_FAILED")
                raise

    def _enforce_hold_error(self, snapshot: JointStateSnapshot) -> None:
        if not self._enabled or self._hold_target is None:
            return
        limit = float(
            self.config.data.get("control", {}).get("hold_error_limit", 0.05)
        )
        error = max(
            abs(actual - target)
            for actual, target in zip(snapshot.position, self._hold_target)
        )
        if error <= limit:
            return
        try:
            assert self._robot is not None
            self._robot.disable_all()
        finally:
            self._enabled = False
            self._hold_target = None
            self._record_error("HOLD_ERROR_EXCEEDED")
        raise RuntimeError(
            f"position hold error {error:.6f} rad exceeds limit {limit:.6f} rad"
        )

    def diagnostics(self) -> ArmDiagnostics:
        with self._lock:
            if self._robot is not None and self._state is HardwareState.CONNECTED:
                try:
                    status_codes = self._robot.get_joint_status_codes(self.config.arm_joints)
                except Exception:
                    status_codes = (255,) * len(self.config.arm_joints)
                    self._record_error("STATUS_READ_FAILED")
            else:
                status_codes = (255,) * len(self.config.arm_joints)
            return ArmDiagnostics(
                mode=(
                    "gravity_compensation"
                    if self._gravity_active
                    else "position_hold" if self._enabled else "read_only"
                ),
                enabled=self._enabled,
                control_loop_active=self._gravity_active,
                state_machine=(
                    self._operation
                    if self._operation != "IDLE"
                    else self._state.value.upper()
                ),
                joint_names=self.config.arm_joints,
                per_joint_status_code=status_codes,
                error_codes=tuple(self._errors),
            )

    def disable(self) -> None:
        """Disable every configured motor while keeping feedback connected."""
        with self._lock:
            if self._state is not HardwareState.CONNECTED or self._robot is None:
                raise RuntimeError(
                    f"cannot disable hardware while state={self._state.value}"
                )
            try:
                self._gravity_active = False
                self._gravity_stop.set()
                self._robot.disable_all()
                self._enabled = False
                self._hold_target = None
            except Exception:
                self._record_error("DISABLE_FAILED")
                raise

    def start_gravity_compensation(self) -> None:
        """Enter low-gain MIT control with model gravity feed-forward."""
        with self._lock:
            if self._state is not HardwareState.CONNECTED or self._robot is None:
                raise RuntimeError(
                    f"cannot start gravity compensation while state={self._state.value}"
                )
            if self._operation != "IDLE":
                raise RuntimeError(f"arm is busy: {self._operation}")
            if self._gravity_active:
                return
            settings = self._gravity_settings()
            current = self.read_state().position
            torque = self._gravity_torque(current, settings)
            try:
                self._robot.enter_gravity_compensation(
                    current, settings["kp"], settings["kd"], torque
                )
            except Exception:
                self._enabled = False
                self._hold_target = None
                self._record_error("GRAVITY_COMPENSATION_START_FAILED")
                raise
            self._enabled = True
            self._hold_target = None
            self._gravity_active = True
            self._gravity_stop.clear()
            self._operation = "GRAVITY_COMPENSATION"
            self._gravity_thread = threading.Thread(
                target=self._gravity_loop,
                args=(settings,),
                name="rebotarm_gravity_compensation",
                daemon=True,
            )
            self._gravity_thread.start()

    def stop_gravity_compensation(self) -> None:
        """Leave gravity mode and safely hold the last measured position."""
        with self._lock:
            if not self._gravity_active:
                return
            self._gravity_active = False
            self._gravity_stop.set()
            if self._robot is None:
                raise RuntimeError("gravity compensation robot is unavailable")
            try:
                position, _, _ = self._robot.get_state()
                current = tuple(float(value) for value in position[: len(self.config.arm_joints)])
                max_velocity = float(
                    self.config.data.get("control", {}).get("hold_max_velocity", 0.2)
                )
                self._robot.enable_position_hold(current, max_velocity)
                self._hold_target = current
                self._enabled = True
            except Exception:
                try:
                    self._robot.disable_all()
                finally:
                    self._enabled = False
                    self._hold_target = None
                    self._record_error("GRAVITY_COMPENSATION_STOP_FAILED")
                raise
            finally:
                self._operation = "IDLE"

    def _gravity_loop(self, settings) -> None:
        interval = 1.0 / settings["rate"]
        try:
            while not self._gravity_stop.wait(interval):
                with self._lock:
                    if not self._gravity_active or self._robot is None:
                        return
                    position, velocity, _ = self._robot.get_state()
                    q = tuple(float(value) for value in position[: len(self.config.arm_joints)])
                    dq = tuple(float(value) for value in velocity[: len(self.config.arm_joints)])
                    if max(abs(value) for value in dq) > 2.0:
                        raise RuntimeError("joint velocity exceeded 2.0 rad/s in gravity mode")
                    torque = self._gravity_torque(q, settings)
                    self._robot.send_gravity_command(
                        q, settings["kp"], settings["kd"], torque
                    )
        except Exception:
            with self._lock:
                self._gravity_active = False
                self._operation = "IDLE"
                if self._robot is not None:
                    try:
                        self._robot.disable_all()
                    except Exception:
                        pass
                self._enabled = False
                self._hold_target = None
                self._record_error("GRAVITY_COMPENSATION_FAILED")

    def _gravity_settings(self):
        import numpy as np

        config = self.config.data.get("gravity_compensation", {})
        count = len(self.config.arm_joints)

        def vector(name, default):
            value = config.get(name, default)
            array = np.asarray(
                [value] * count if isinstance(value, (int, float)) else value,
                dtype=np.float64,
            ).reshape(-1)
            if len(array) != count or not np.all(np.isfinite(array)):
                raise ValueError(f"gravity_compensation.{name} must have {count} finite values")
            return tuple(float(item) for item in array)

        rate = float(config.get("rate", 100.0))
        if not 20.0 <= rate <= 100.0:
            raise ValueError("gravity compensation rate must be between 20 and 100 Hz")
        urdf = config.get("urdf")
        if not isinstance(urdf, str) or not urdf:
            raise ValueError("gravity_compensation.urdf is required")
        if self._gravity_model is None:
            layout = WorkspaceLayout.discover(self.config.source)
            sdk = load_sdk()
            self._gravity_model = sdk.kinematics.load_robot_model(
                str(layout.resolve(urdf))
            )
            self._gravity_data = self._gravity_model.createData()
            self._gravity_compute = sdk.dynamics.compute_generalized_gravity
        return {
            "rate": rate,
            "kp": vector("kp", 2.0),
            "kd": vector("kd", 1.5),
            "direction": vector("joint_direction", 1.0),
            "scale": vector("tau_scale", 1.0),
            "limit": vector("tau_limit", 1.0),
        }

    def _gravity_torque(self, positions, settings):
        import numpy as np

        direction = np.asarray(settings["direction"])
        q = np.asarray(positions, dtype=np.float64) * direction
        model_q = np.zeros(self._gravity_model.nq, dtype=np.float64)
        model_q[: len(q)] = q
        torque = np.asarray(
            self._gravity_compute(self._gravity_model, model_q, self._gravity_data)
        )[: len(q)]
        motor_torque = torque * direction * np.asarray(settings["scale"])
        limits = np.asarray(settings["limit"])
        if np.any(~np.isfinite(motor_torque)) or np.any(np.abs(motor_torque) > limits):
            raise RuntimeError(
                "gravity torque exceeds configured limits: "
                + str([round(float(value), 4) for value in motor_torque])
            )
        return tuple(float(value) for value in motor_torque)

    def enable(self, max_velocity: float | None = None) -> None:
        """Enable DM motors only after priming a current-position hold target."""
        with self._lock:
            if self._state is not HardwareState.CONNECTED or self._robot is None:
                raise RuntimeError(
                    f"cannot enable hardware while state={self._state.value}"
                )
            if self._gravity_active:
                raise RuntimeError("stop gravity compensation before enable")
            if self._enabled:
                return
            if max_velocity is None:
                max_velocity = float(
                    self.config.data.get("control", {}).get("hold_max_velocity", 0.2)
                )
            if not 0.01 <= max_velocity <= 1.0:
                raise ValueError("max_velocity must be between 0.01 and 1.0 rad/s")
            try:
                current = self.read_state().position
                self._robot.enable_position_hold(current, max_velocity)
                self._hold_target = current
                self._enabled = True
            except Exception:
                self._enabled = False
                self._hold_target = None
                self._record_error("ENABLE_FAILED")
                raise

    def safe_home(self) -> None:
        """Move all arm joints to their already calibrated zero position."""
        with self._lock:
            if self._state is not HardwareState.CONNECTED or self._robot is None:
                raise RuntimeError(
                    f"cannot safe-home hardware while state={self._state.value}"
                )
            if self._gravity_active or self._operation != "IDLE":
                raise RuntimeError(
                    f"stop active operation before safe_home: {self._operation}"
                )
            control = self.config.data.get("control", {})
            max_velocity = float(control.get("home_max_velocity", 0.2))
            sample_rate = float(control.get("home_sample_rate", 50.0))
            max_step = float(control.get("home_max_step", 0.01))
            settle_tolerance = float(control.get("home_tolerance", 0.01))
            settle_timeout = float(control.get("home_settle_timeout", 2.0))

            if not self._enabled:
                self.enable(max_velocity=max_velocity)
            start = self.read_state().position
            plan = plan_safe_home(
                start,
                max_velocity=max_velocity,
                sample_rate=sample_rate,
                max_step=max_step,
            )
            self._operation = "SAFE_HOMING"
            interval = 1.0 / sample_rate
            try:
                for point in plan.points[1:]:
                    self._hold_target = point
                    self._robot.send_position_target(point, max_velocity)
                    time.sleep(interval)
                    self.read_state()
                zero = (0.0,) * len(self.config.arm_joints)
                settle_deadline = time.monotonic() + settle_timeout
                error = float("inf")
                while time.monotonic() < settle_deadline:
                    self._hold_target = zero
                    self._robot.send_position_target(zero, max_velocity)
                    time.sleep(interval)
                    final = self.read_state().position
                    error = max(abs(value) for value in final)
                    if error <= settle_tolerance:
                        break
                if error > settle_tolerance:
                    raise RuntimeError(
                        f"safe_home final error {error:.6f} rad exceeds "
                        f"tolerance {settle_tolerance:.6f} rad"
                    )
                self._hold_target = zero
            except Exception:
                try:
                    self.disable()
                finally:
                    self._record_error("SAFE_HOME_FAILED")
                raise
            finally:
                self._operation = "IDLE"

    def move_to_joint_target(
        self,
        target: tuple[float, ...],
        *,
        should_cancel=lambda: False,
        feedback=lambda _desired, _actual: None,
        max_velocity: float = 0.2,
        sample_rate: float = 50.0,
        max_step: float = 0.01,
        goal_tolerance: float | tuple[float, ...] = 0.01,
        settle_timeout: float = 2.0,
        desired_duration: float = 0.0,
        path_tolerance: tuple[float, ...] | None = None,
    ) -> bool:
        """Execute one bounded joint target; return False after a safe cancel."""
        with self._lock:
            if self._state is not HardwareState.CONNECTED or self._robot is None:
                raise RuntimeError(f"cannot move hardware while state={self._state.value}")
            if self._operation != "IDLE":
                raise RuntimeError(f"arm is busy: {self._operation}")
            if not self._enabled:
                self.enable(max_velocity=max_velocity)
            start = self.read_state().position
            plan = plan_minimum_jerk(
                start, target, max_velocity=max_velocity,
                sample_rate=sample_rate, max_step=max_step,
                minimum_duration=desired_duration,
            )
            self._operation = "JOINT_TRAJECTORY"

        interval = 1.0 / sample_rate
        try:
            for point in plan.points[1:]:
                if should_cancel():
                    with self._lock:
                        current = self.read_state().position
                        self._hold_target = current
                        assert self._robot is not None
                        self._robot.send_position_target(current, max_velocity)
                    return False
                with self._lock:
                    if not self._enabled or self._robot is None:
                        raise RuntimeError("arm was disabled during trajectory")
                    self._hold_target = point
                    self._robot.send_position_target(point, max_velocity)
                time.sleep(interval)
                actual = self.read_state().position
                if path_tolerance is not None:
                    violations = [
                        (name, abs(measured - wanted), limit)
                        for name, measured, wanted, limit in zip(
                            self.config.arm_joints, actual, point, path_tolerance
                        )
                        if abs(measured - wanted) > limit
                    ]
                    if violations:
                        name, error, limit = violations[0]
                        raise RuntimeError(
                            f"{name} path error {error:.6f} exceeds "
                            f"tolerance {limit:.6f} rad"
                        )
                feedback(point, actual)
            settle_deadline = time.monotonic() + settle_timeout
            errors = (float("inf"),) * len(target)
            while time.monotonic() < settle_deadline:
                if should_cancel():
                    with self._lock:
                        current = self.read_state().position
                        self._hold_target = current
                        assert self._robot is not None
                        self._robot.send_position_target(current, max_velocity)
                    return False
                with self._lock:
                    if not self._enabled or self._robot is None:
                        raise RuntimeError("arm was disabled while settling")
                    self._hold_target = tuple(target)
                    self._robot.send_position_target(tuple(target), max_velocity)
                time.sleep(interval)
                final = self.read_state().position
                feedback(tuple(target), final)
                errors = tuple(
                    abs(actual - wanted) for actual, wanted in zip(final, target)
                )
                limits = (
                    (goal_tolerance,) * len(target)
                    if isinstance(goal_tolerance, (int, float))
                    else tuple(goal_tolerance)
                )
                if all(error <= limit for error, limit in zip(errors, limits)):
                    break
            if any(error > limit for error, limit in zip(errors, limits)):
                index = max(
                    range(len(errors)), key=lambda item: errors[item] - limits[item]
                )
                raise RuntimeError(
                    f"{self.config.arm_joints[index]} final error "
                    f"{errors[index]:.6f} exceeds tolerance {limits[index]:.6f} rad"
                )
            with self._lock:
                self._hold_target = tuple(target)
            return True
        except Exception:
            try:
                self.disable()
            finally:
                self._record_error("JOINT_TRAJECTORY_FAILED")
            raise
        finally:
            with self._lock:
                self._operation = "IDLE"

    def set_zero(self, joint_name: str) -> None:
        """Maintenance-only single-joint zero calibration primitive."""
        with self._lock:
            if self._state is not HardwareState.CONNECTED or self._robot is None:
                raise RuntimeError(
                    f"cannot set zero while state={self._state.value}"
                )
            if self._enabled:
                raise RuntimeError("disable the arm before set_zero")
            if not joint_name or joint_name not in self.config.arm_joints:
                raise ValueError(
                    "joint_name must be exactly one of: "
                    + ", ".join(self.config.arm_joints)
                )
            try:
                self._robot.set_zero(joint_name)
                time.sleep(0.05)
                snapshot = self.read_state()
                index = self.config.arm_joints.index(joint_name)
                if abs(snapshot.position[index]) > 0.01:
                    raise RuntimeError(
                        f"{joint_name} zero verification failed: "
                        f"position={snapshot.position[index]:.6f} rad"
                    )
            except Exception:
                self._record_error("SET_ZERO_FAILED")
                raise

    def _record_error(self, code: str) -> None:
        if code not in self._errors:
            self._errors.append(code)

    def close(self) -> None:
        self.disconnect()
        cleanup = getattr(self._factory, "cleanup", None)
        if callable(cleanup):
            cleanup()

    def __enter__(self) -> "HardwareManager":
        self.connect()
        return self

    def __exit__(self, exc_type, exc_value, traceback) -> None:
        self.close()


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Inspect HardwareManager without connecting hardware"
    )
    parser.add_argument("--model", default="")
    args = parser.parse_args()
    manager = HardwareManager(load_hardware_config(model=args.model))
    print(
        f"model={manager.config.model} joints={len(manager.config.arm_joints)} "
        f"state={manager.state.value} connection=not_opened"
    )
