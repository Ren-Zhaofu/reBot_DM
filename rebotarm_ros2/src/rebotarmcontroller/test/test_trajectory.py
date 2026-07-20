import math

import pytest

from rebotarmcontroller.trajectory import plan_minimum_jerk, plan_safe_home


def test_plan_has_exact_endpoints_and_synchronized_joints() -> None:
    plan = plan_minimum_jerk((0.2, -0.1), (0.0, 0.3))
    assert plan.points[0] == (0.2, -0.1)
    assert plan.points[-1] == (0.0, 0.3)
    assert plan.joint_count == 2
    assert all(len(point) == 2 for point in plan.points)


def test_plan_respects_velocity_and_step_limits() -> None:
    max_velocity = 0.2
    max_step = 0.01
    plan = plan_minimum_jerk(
        (0.4, -0.2),
        (0.0, 0.0),
        max_velocity=max_velocity,
        sample_rate=50.0,
        max_step=max_step,
    )
    dt = 1.0 / plan.sample_rate
    for previous, current in zip(plan.points, plan.points[1:]):
        changes = [abs(now - before) for before, now in zip(previous, current)]
        assert max(changes) <= max_step + 1e-12
        assert max(changes) / dt <= max_velocity + 1e-3


def test_safe_home_is_monotonic_toward_zero() -> None:
    plan = plan_safe_home((0.3, -0.2, 0.1))
    for joint_index in range(3):
        errors = [abs(point[joint_index]) for point in plan.points]
        assert all(now <= before + 1e-12 for before, now in zip(errors, errors[1:]))
    assert plan.points[-1] == (0.0, 0.0, 0.0)


def test_requested_minimum_duration_stretches_trajectory() -> None:
    plan = plan_minimum_jerk((0.0,), (0.02,), minimum_duration=1.0)
    assert plan.duration == 1.0
    assert len(plan.points) == 51


def test_zero_distance_returns_single_point() -> None:
    plan = plan_safe_home((0.0, 0.0))
    assert plan.points == ((0.0, 0.0),)
    assert plan.duration == 0.0


@pytest.mark.parametrize(
    "kwargs",
    [
        {"start": (0.0,), "target": ()},
        {"start": (math.nan,), "target": (0.0,)},
        {"start": (0.0,), "target": (0.0,), "max_velocity": 0.0},
        {"start": (0.0,), "target": (0.0,), "sample_rate": 1.0},
        {"start": (0.0,), "target": (0.0,), "max_step": 0.1},
    ],
)
def test_rejects_invalid_plans(kwargs) -> None:
    with pytest.raises(ValueError):
        plan_minimum_jerk(**kwargs)
