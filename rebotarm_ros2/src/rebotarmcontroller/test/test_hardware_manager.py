from dataclasses import dataclass

import pytest

from rebotarmcontroller.hardware_config import HardwareConfig
from rebotarmcontroller.hardware_manager import HardwareManager, HardwareState


@dataclass
class FakeRobot:
    fail_connect: bool = False
    connect_calls: int = 0
    disconnect_calls: int = 0
    disable_calls: int = 0
    enable_calls: int = 0
    hold_target: tuple[float, ...] | None = None
    hold_max_velocity: float | None = None
    zeroed_joint: str | None = None
    sent_targets: list | None = None

    def connect(self) -> None:
        self.connect_calls += 1
        if self.fail_connect:
            raise OSError("test connection failure")

    def disconnect(self) -> None:
        self.disconnect_calls += 1

    def get_state(self):
        position = 0.0 if self.zeroed_joint else 1.0
        return ([position], [2.0], [3.0])

    def get_joint_status_codes(self, names):
        return (7,) * len(names)

    def disable_all(self):
        self.disable_calls += 1

    def enable_all(self):
        self.enable_calls += 1

    def enable_position_hold(self, positions, max_velocity):
        self.enable_all()
        self.hold_target = tuple(positions)
        self.hold_max_velocity = max_velocity

    def send_position_target(self, positions, max_velocity):
        if self.sent_targets is None:
            self.sent_targets = []
        self.sent_targets.append(tuple(positions))
        self.hold_target = tuple(positions)

    def enter_gravity_compensation(self, positions, kp, kd, torque):
        self.enable_all()
        self.hold_target = tuple(positions)
        self.gravity_torque = tuple(torque)

    def send_gravity_command(self, positions, kp, kd, torque):
        self.hold_target = tuple(positions)
        self.gravity_torque = tuple(torque)

    def set_zero(self, joint_name):
        self.zeroed_joint = joint_name


class FakeFactory:
    def __init__(self, robot: FakeRobot) -> None:
        self.robot = robot
        self.calls = 0

    def __call__(self, config: HardwareConfig) -> FakeRobot:
        self.calls += 1
        return self.robot


def config(tmp_path) -> HardwareConfig:
    data = {
        "channel": "fake",
        "rate": 100,
        "groups": {"arm": {"joints": ["joint1"]}},
        "joints": [{"name": "joint1"}],
    }
    return HardwareConfig("fake", tmp_path / "selector.yaml", tmp_path / "sdk.yaml", data)


def test_construction_does_not_create_or_connect_robot(tmp_path) -> None:
    robot = FakeRobot()
    factory = FakeFactory(robot)
    manager = HardwareManager(config(tmp_path), robot_factory=factory)
    assert manager.state is HardwareState.CREATED
    assert factory.calls == 0
    assert robot.connect_calls == 0


def test_connect_and_disconnect_are_idempotent(tmp_path) -> None:
    robot = FakeRobot()
    factory = FakeFactory(robot)
    manager = HardwareManager(config(tmp_path), robot_factory=factory)
    manager.connect()
    manager.connect()
    assert manager.state is HardwareState.CONNECTED
    assert factory.calls == 1
    assert robot.connect_calls == 1
    snapshot = manager.read_state()
    assert snapshot.names == ("joint1",)
    assert snapshot.position == (1.0,)
    assert snapshot.velocity == (2.0,)
    assert snapshot.effort == (3.0,)
    assert snapshot.monotonic_ns > 0

    manager.disconnect()
    manager.disconnect()
    assert manager.state is HardwareState.DISCONNECTED
    assert robot.disconnect_calls == 1


def test_failed_connection_enters_fault_and_cleans_up(tmp_path) -> None:
    robot = FakeRobot(fail_connect=True)
    manager = HardwareManager(config(tmp_path), robot_factory=FakeFactory(robot))
    with pytest.raises(OSError, match="test connection failure"):
        manager.connect()
    assert manager.state is HardwareState.FAULT
    assert robot.disconnect_calls == 1


def test_read_requires_connection(tmp_path) -> None:
    manager = HardwareManager(config(tmp_path), robot_factory=FakeFactory(FakeRobot()))
    with pytest.raises(RuntimeError, match="state=created"):
        manager.read_state()


def test_diagnostics_are_truthful_in_read_only_mode(tmp_path) -> None:
    manager = HardwareManager(config(tmp_path), robot_factory=FakeFactory(FakeRobot()))
    before = manager.diagnostics()
    assert before.state_machine == "CREATED"
    assert before.per_joint_status_code == (255,)
    assert before.enabled is False

    manager.connect()
    connected = manager.diagnostics()
    assert connected.mode == "read_only"
    assert connected.state_machine == "CONNECTED"
    assert connected.per_joint_status_code == (7,)
    assert connected.control_loop_active is False


def test_disable_requires_connection_and_delegates_once(tmp_path) -> None:
    robot = FakeRobot()
    manager = HardwareManager(config(tmp_path), robot_factory=FakeFactory(robot))
    with pytest.raises(RuntimeError, match="state=created"):
        manager.disable()
    manager.connect()
    manager.disable()
    assert robot.disable_calls == 1


def test_enable_requires_connection_is_idempotent_and_disable_clears(tmp_path) -> None:
    robot = FakeRobot()
    manager = HardwareManager(config(tmp_path), robot_factory=FakeFactory(robot))
    with pytest.raises(RuntimeError, match="state=created"):
        manager.enable()
    manager.connect()
    manager.enable()
    manager.enable()
    assert robot.enable_calls == 1
    assert robot.hold_target == (1.0,)
    assert robot.hold_max_velocity == 0.2
    assert manager.diagnostics().enabled is True
    assert manager.diagnostics().mode == "position_hold"
    manager.disable()
    assert manager.diagnostics().enabled is False


def test_connect_waits_for_complete_feedback(tmp_path) -> None:
    class DelayedRobot(FakeRobot):
        status_calls = 0

        def get_joint_status_codes(self, names):
            self.status_calls += 1
            if self.status_calls < 3:
                return (255,) * len(names)
            return (0,) * len(names)

    robot = DelayedRobot()
    manager = HardwareManager(config(tmp_path), robot_factory=FakeFactory(robot))
    manager.connect()
    assert manager.state is HardwareState.CONNECTED
    assert robot.status_calls == 3


def test_hold_error_auto_disables(tmp_path) -> None:
    class DriftingRobot(FakeRobot):
        state_reads = 0

        def get_state(self):
            self.state_reads += 1
            position = 1.0 if self.state_reads <= 2 else 1.2
            return ([position], [0.0], [0.0])

    robot = DriftingRobot()
    manager = HardwareManager(config(tmp_path), robot_factory=FakeFactory(robot))
    manager.connect()  # first state read
    manager.enable()  # second state read becomes the hold target
    with pytest.raises(RuntimeError, match="hold error"):
        manager.read_state()  # third state read exceeds the limit
    diagnostics = manager.diagnostics()
    assert diagnostics.enabled is False
    assert diagnostics.error_codes == ("HOLD_ERROR_EXCEEDED", "STATE_READ_FAILED")
    assert robot.disable_calls == 1


def test_set_zero_requires_disabled_single_known_joint(tmp_path) -> None:
    robot = FakeRobot()
    manager = HardwareManager(config(tmp_path), robot_factory=FakeFactory(robot))
    with pytest.raises(RuntimeError, match="state=created"):
        manager.set_zero("joint1")
    manager.connect()
    with pytest.raises(ValueError, match="exactly one"):
        manager.set_zero("")
    with pytest.raises(ValueError, match="exactly one"):
        manager.set_zero("all")
    manager.enable()
    with pytest.raises(RuntimeError, match="disable the arm"):
        manager.set_zero("joint1")
    manager.disable()
    manager.set_zero("joint1")
    assert robot.zeroed_joint == "joint1"


def test_joint_target_tracks_feedback_and_completes(tmp_path) -> None:
    class TrackingRobot(FakeRobot):
        position = 0.0

        def get_state(self):
            return ([self.position], [0.0], [0.0])

        def send_position_target(self, positions, max_velocity):
            super().send_position_target(positions, max_velocity)
            self.position = float(positions[0])

    robot = TrackingRobot()
    manager = HardwareManager(config(tmp_path), robot_factory=FakeFactory(robot))
    manager.connect()
    samples = []
    assert manager.move_to_joint_target(
        (0.02,), feedback=lambda desired, actual: samples.append((desired, actual))
    )
    assert samples
    assert samples[-1][1] == pytest.approx((0.02,))
    assert manager.diagnostics().enabled is True


def test_joint_target_cancel_holds_measured_position(tmp_path) -> None:
    class TrackingRobot(FakeRobot):
        position = 0.0

        def get_state(self):
            return ([self.position], [0.0], [0.0])

        def send_position_target(self, positions, max_velocity):
            super().send_position_target(positions, max_velocity)
            self.position = float(positions[0])

    robot = TrackingRobot()
    manager = HardwareManager(config(tmp_path), robot_factory=FakeFactory(robot))
    manager.connect()
    checks = 0

    def cancel_after_first_check():
        nonlocal checks
        checks += 1
        return checks > 1

    assert not manager.move_to_joint_target((0.02,), should_cancel=cancel_after_first_check)
    assert manager.diagnostics().enabled is True
    assert robot.hold_target == pytest.approx((robot.position,))


def test_path_tolerance_failure_disables_arm(tmp_path) -> None:
    class LaggingRobot(FakeRobot):
        def get_state(self):
            return ([0.0], [0.0], [0.0])

    robot = LaggingRobot()
    manager = HardwareManager(config(tmp_path), robot_factory=FakeFactory(robot))
    manager.connect()
    with pytest.raises(RuntimeError, match="path error"):
        manager.move_to_joint_target((0.02,), path_tolerance=(0.0001,))
    assert manager.diagnostics().enabled is False
    assert "JOINT_TRAJECTORY_FAILED" in manager.diagnostics().error_codes
