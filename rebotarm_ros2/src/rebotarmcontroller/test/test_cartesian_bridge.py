from geometry_msgs.msg import Pose
import pytest

from rebotarmcontroller.cartesian_bridge import pose_constraints, validate_cartesian_goal


def pose(x=0.3, y=0.0, z=0.3):
    message = Pose()
    message.position.x = x
    message.position.y = y
    message.position.z = z
    message.orientation.w = 1.0
    return message


def test_valid_cartesian_goal_and_moveit_constraints() -> None:
    target = pose()
    validate_cartesian_goal(target, 3.0)
    constraints = pose_constraints(target)
    assert constraints.position_constraints[0].header.frame_id == "base_link"
    assert constraints.position_constraints[0].link_name == "end_link"
    assert constraints.orientation_constraints[0].link_name == "end_link"


@pytest.mark.parametrize(
    "target, duration, match",
    [
        (pose(x=1.1), 3.0, "workspace"),
        (pose(), 0.1, "duration"),
        (pose(), 31.0, "duration"),
    ],
)
def test_cartesian_goal_guards(target, duration, match) -> None:
    with pytest.raises(ValueError, match=match):
        validate_cartesian_goal(target, duration)


def test_cartesian_goal_rejects_bad_quaternion() -> None:
    target = pose()
    target.orientation.w = 0.0
    with pytest.raises(ValueError, match="normalized"):
        validate_cartesian_goal(target, 3.0)
