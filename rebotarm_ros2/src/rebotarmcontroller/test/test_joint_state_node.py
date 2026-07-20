from builtin_interfaces.msg import Duration, Time
import pytest
from std_srvs.srv import Trigger
from rebotarm_msgs.srv import SetZero

from rebotarmcontroller.joint_state_node import (
    diagnostics_to_message,
    handle_disable,
    handle_enable,
    handle_set_zero,
    snapshot_joint_to_message,
    snapshot_to_message,
    trajectory_feedback_message,
)
from rebotarmcontroller.state import ArmDiagnostics, JointStateSnapshot


def test_snapshot_to_standard_joint_state_message() -> None:
    snapshot = JointStateSnapshot.from_sdk(
        names=("joint1", "joint2"),
        position=(1.0, 2.0),
        velocity=(3.0, 4.0),
        effort=(5.0, 6.0),
        monotonic_ns=10,
    )
    stamp = Time(sec=12, nanosec=34)
    message = snapshot_to_message(snapshot, stamp)
    assert message.header.stamp == stamp
    assert message.name == ["joint1", "joint2"]
    assert list(message.position) == [1.0, 2.0]
    assert list(message.velocity) == [3.0, 4.0]
    assert list(message.effort) == [5.0, 6.0]


def test_trajectory_feedback_contains_desired_actual_error_and_time() -> None:
    stamp = Time(sec=12, nanosec=34)
    message = trajectory_feedback_message(
        ("joint1", "joint2"),
        (0.2, -0.3),
        (0.19, -0.28),
        1.25,
        stamp,
    )
    assert message.header.stamp == stamp
    assert message.joint_names == ["joint1", "joint2"]
    assert list(message.desired.positions) == [0.2, -0.3]
    assert list(message.actual.positions) == [0.19, -0.28]
    assert list(message.error.positions) == pytest.approx([0.01, -0.02])
    assert message.desired.time_from_start == Duration(sec=1, nanosec=250000000)
    assert message.actual.time_from_start == message.desired.time_from_start
    assert message.error.time_from_start == message.desired.time_from_start


def test_trajectory_feedback_rejects_bad_shape_or_time() -> None:
    with pytest.raises(ValueError, match="lengths"):
        trajectory_feedback_message(("joint1",), (0.0,), (0.0, 1.0), 0.0, Time())
    with pytest.raises(ValueError, match="non-negative"):
        trajectory_feedback_message(("joint1",), (0.0,), (0.0,), -1.0, Time())


def test_diagnostics_to_arm_status_message() -> None:
    diagnostics = ArmDiagnostics(
        mode="read_only",
        enabled=False,
        control_loop_active=False,
        state_machine="CONNECTED",
        joint_names=("joint1", "joint2"),
        per_joint_status_code=(0, 7),
        error_codes=("TEST_ERROR",),
    )
    stamp = Time(sec=56, nanosec=78)
    message = diagnostics_to_message(diagnostics, stamp)
    assert message.header.stamp == stamp
    assert message.mode == "read_only"
    assert message.enabled is False
    assert message.state_machine == "CONNECTED"
    assert message.joint_names == ["joint1", "joint2"]
    assert list(message.per_joint_status_code) == [0, 7]
    assert message.error_codes == ["TEST_ERROR"]


def test_snapshot_joint_to_motor_state_message() -> None:
    snapshot = JointStateSnapshot.from_sdk(
        names=("joint1", "joint2"),
        position=(1.0, 2.0),
        velocity=(3.0, 4.0),
        effort=(5.0, 6.0),
        monotonic_ns=10,
    )
    stamp = Time(sec=90, nanosec=12)
    message = snapshot_joint_to_message(snapshot, 1, 7, stamp)
    assert message.header.stamp == stamp
    assert message.joint_name == "joint2"
    assert message.position == 2.0
    assert message.velocity == 4.0
    assert message.torque == 6.0
    assert message.status_code == 7


def test_snapshot_joint_message_rejects_bad_index_or_status() -> None:
    snapshot = JointStateSnapshot.from_sdk(
        names=("joint1",),
        position=(0.0,),
        velocity=(0.0,),
        effort=(0.0,),
        monotonic_ns=10,
    )
    stamp = Time()
    import pytest

    with pytest.raises(IndexError, match="out of range"):
        snapshot_joint_to_message(snapshot, 1, 0, stamp)
    with pytest.raises(ValueError, match="uint8"):
        snapshot_joint_to_message(snapshot, 0, 256, stamp)


def test_disable_service_reports_success_and_publishes_status() -> None:
    class Manager:
        calls = 0

        def disable(self):
            self.calls += 1

    manager = Manager()
    published = []
    response = handle_disable(manager, Trigger.Response(), lambda: published.append(True))
    assert response.success is True
    assert response.message == "DM controller disable command completed"
    assert manager.calls == 1
    assert published == [True]


def test_disable_service_converts_exception_to_failure() -> None:
    class Manager:
        def disable(self):
            raise OSError("CAN disable failed")

    response = handle_disable(Manager(), Trigger.Response())
    assert response.success is False
    assert response.message == "CAN disable failed"


def test_enable_service_is_locked_by_default() -> None:
    class Manager:
        calls = 0

        def enable(self):
            self.calls += 1

    manager = Manager()
    response = handle_enable(manager, Trigger.Response(), allowed=False)
    assert response.success is False
    assert "allow_enable:=true" in response.message
    assert manager.calls == 0


def test_unlocked_enable_service_delegates_and_reports_success() -> None:
    class Manager:
        calls = 0

        def enable(self):
            self.calls += 1

    manager = Manager()
    published = []
    response = handle_enable(
        manager,
        Trigger.Response(),
        allowed=True,
        publish_status=lambda: published.append(True),
    )
    assert response.success is True
    assert response.message == "DM controller enable command completed"
    assert manager.calls == 1
    assert published == [True]


def test_set_zero_requires_both_calibration_locks() -> None:
    class Manager:
        calls = 0

        def set_zero(self, joint_name):
            self.calls += 1

    manager = Manager()
    request = SetZero.Request(joint_name="joint1")
    response = handle_set_zero(
        manager,
        request,
        SetZero.Response(),
        allowed=True,
        calibration_mode=False,
    )
    assert response.success is False
    assert "both allow_calibration and calibration_mode" in response.message
    assert manager.calls == 0


def test_unlocked_set_zero_delegates_one_joint() -> None:
    class Manager:
        joint_name = None

        def set_zero(self, joint_name):
            self.joint_name = joint_name

    manager = Manager()
    request = SetZero.Request(joint_name="joint3")
    response = handle_set_zero(
        manager,
        request,
        SetZero.Response(),
        allowed=True,
        calibration_mode=True,
    )
    assert response.success is True
    assert response.message == "joint3 zero calibration verified"
    assert manager.joint_name == "joint3"
