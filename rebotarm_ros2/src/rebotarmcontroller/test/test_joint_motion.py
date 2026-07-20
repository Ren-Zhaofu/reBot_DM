from builtin_interfaces.msg import Duration
import pytest
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.action import FollowJointTrajectory
from control_msgs.msg import JointTolerance

from rebotarmcontroller.joint_motion import resolve_tolerances, validate_trajectory


NAMES = tuple(f"joint{i}" for i in range(1, 7))


def trajectory(positions, *, names=NAMES, seconds=1):
    message = JointTrajectory(joint_names=list(names))
    message.points = [
        JointTrajectoryPoint(
            positions=list(positions), time_from_start=Duration(sec=seconds)
        )
    ]
    return message


def test_valid_trajectory_is_reordered_to_hardware_order() -> None:
    names = tuple(reversed(NAMES))
    values = (0.1, -0.1, -0.1, 0.1, 0.1, 0.1)
    result = validate_trajectory(
        trajectory(tuple(reversed(values)), names=names), NAMES, (0.0,) * 6
    )
    assert result.targets == (values,)
    assert result.times == (1.0,)


@pytest.mark.parametrize(
    "message, match",
    [
        (trajectory((0.0,) * 6, names=NAMES[:-1]), "exactly"),
        (trajectory((0.0,) * 5), "positions"),
        (trajectory((0.6, 0.0, 0.0, 0.0, 0.0, 0.0)), "delta"),
        (trajectory((0.0, 0.1, 0.0, 0.0, 0.0, 0.0)), "outside"),
    ],
)
def test_invalid_trajectory_is_rejected(message, match) -> None:
    with pytest.raises(ValueError, match=match):
        validate_trajectory(message, NAMES, (0.0,) * 6)


def test_time_must_be_positive_and_strictly_increasing() -> None:
    message = trajectory((0.0,) * 6, seconds=0)
    result = validate_trajectory(message, NAMES, (0.0,) * 6)
    assert result.times == (0.0,)

    message.points.append(
        JointTrajectoryPoint(
            positions=[0.0] * 6, time_from_start=Duration(sec=0)
        )
    )
    with pytest.raises(ValueError, match="strictly increasing"):
        validate_trajectory(message, NAMES, (0.0,) * 6)


def test_tolerances_default_and_reorder_by_joint_name() -> None:
    request = FollowJointTrajectory.Goal()
    request.path_tolerance = [JointTolerance(name="joint3", position=0.02)]
    request.goal_tolerance = [JointTolerance(name="joint1", position=0.005)]
    request.goal_time_tolerance = Duration(sec=3)
    result = resolve_tolerances(request, NAMES)
    assert result.path == (0.05, 0.05, 0.02, 0.05, 0.05, 0.05)
    assert result.goal == (0.005, 0.01, 0.01, 0.01, 0.01, 0.01)
    assert result.goal_time == 3.0


@pytest.mark.parametrize(
    "field, tolerance, match",
    [
        ("path_tolerance", JointTolerance(name="unknown", position=0.01), "unknown"),
        ("path_tolerance", JointTolerance(name="joint1", position=-1.0), "cannot disable"),
        (
            "goal_tolerance",
            JointTolerance(name="joint1", velocity=0.1),
            "position tolerance only",
        ),
    ],
)
def test_unsupported_tolerances_are_rejected(field, tolerance, match) -> None:
    request = FollowJointTrajectory.Goal()
    setattr(request, field, [tolerance])
    with pytest.raises(ValueError, match=match):
        resolve_tolerances(request, NAMES)
