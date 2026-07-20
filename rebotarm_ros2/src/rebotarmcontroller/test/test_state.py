import math

import pytest

from rebotarmcontroller.state import JointStateSnapshot


def snapshot(**overrides) -> JointStateSnapshot:
    values = {
        "names": ["joint1", "joint2"],
        "position": [1, 2, 99],
        "velocity": [3, 4, 99],
        "effort": [5, 6, 99],
        "monotonic_ns": 123,
    }
    values.update(overrides)
    return JointStateSnapshot.from_sdk(**values)


def test_builds_immutable_arm_snapshot_and_excludes_gripper() -> None:
    state = snapshot()
    assert state.names == ("joint1", "joint2")
    assert state.position == (1.0, 2.0)
    assert state.velocity == (3.0, 4.0)
    assert state.effort == (5.0, 6.0)
    assert state.monotonic_ns == 123
    with pytest.raises(AttributeError):
        state.position = (0.0, 0.0)


@pytest.mark.parametrize("field", ["position", "velocity", "effort"])
def test_rejects_short_sdk_vectors(field: str) -> None:
    with pytest.raises(ValueError, match="expected at least 2"):
        snapshot(**{field: [1.0]})


@pytest.mark.parametrize("bad_value", [math.nan, math.inf, -math.inf])
def test_rejects_non_finite_values(bad_value: float) -> None:
    with pytest.raises(ValueError, match="non-finite"):
        snapshot(position=[0.0, bad_value])


def test_rejects_duplicate_joint_names() -> None:
    with pytest.raises(ValueError, match="must be unique"):
        snapshot(names=["joint1", "joint1"])
