"""Bounded joint-space trajectories used by safe-home control."""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Iterable


@dataclass(frozen=True)
class JointTrajectoryPlan:
    points: tuple[tuple[float, ...], ...]
    duration: float
    sample_rate: float

    @property
    def joint_count(self) -> int:
        return len(self.points[0]) if self.points else 0


def plan_minimum_jerk(
    start: Iterable[float],
    target: Iterable[float],
    *,
    max_velocity: float = 0.2,
    sample_rate: float = 50.0,
    max_step: float = 0.01,
    minimum_duration: float = 0.0,
) -> JointTrajectoryPlan:
    """Plan a synchronized minimum-jerk trajectory with bounded samples."""
    q_start = _finite_vector("start", start)
    q_target = _finite_vector("target", target)
    if not q_start or len(q_start) != len(q_target):
        raise ValueError("start and target must have the same non-zero length")
    if not 0.01 <= max_velocity <= 1.0:
        raise ValueError("max_velocity must be between 0.01 and 1.0 rad/s")
    if not 10.0 <= sample_rate <= 200.0:
        raise ValueError("sample_rate must be between 10 and 200 Hz")
    if not 0.001 <= max_step <= 0.05:
        raise ValueError("max_step must be between 0.001 and 0.05 rad")
    if minimum_duration < 0.0 or not math.isfinite(minimum_duration):
        raise ValueError("minimum_duration must be finite and non-negative")

    delta = tuple(end - begin for begin, end in zip(q_start, q_target))
    distance = max(abs(value) for value in delta)
    if distance == 0.0:
        return JointTrajectoryPlan((q_target,), 0.0, sample_rate)

    # The maximum derivative of 10s^3 - 15s^4 + 6s^5 is 1.875.
    velocity_duration = 1.875 * distance / max_velocity
    # A conservative sample bound independent of derivative discretization.
    step_duration = distance / (max_step * sample_rate)
    duration = max(
        velocity_duration, step_duration, minimum_duration, 1.0 / sample_rate
    )
    intervals = max(1, math.ceil(duration * sample_rate))
    duration = intervals / sample_rate

    points = []
    for index in range(intervals + 1):
        phase = index / intervals
        blend = 10.0 * phase**3 - 15.0 * phase**4 + 6.0 * phase**5
        points.append(
            tuple(begin + change * blend for begin, change in zip(q_start, delta))
        )
    points[0] = q_start
    points[-1] = q_target

    plan = JointTrajectoryPlan(tuple(points), duration, sample_rate)
    _verify_step_bound(plan, max_step)
    return plan


def plan_safe_home(
    current: Iterable[float],
    *,
    max_velocity: float = 0.2,
    sample_rate: float = 50.0,
    max_step: float = 0.01,
) -> JointTrajectoryPlan:
    q_current = _finite_vector("current", current)
    return plan_minimum_jerk(
        q_current,
        (0.0,) * len(q_current),
        max_velocity=max_velocity,
        sample_rate=sample_rate,
        max_step=max_step,
    )


def _finite_vector(label: str, values: Iterable[float]) -> tuple[float, ...]:
    result = tuple(float(value) for value in values)
    if any(not math.isfinite(value) for value in result):
        raise ValueError(f"{label} contains a non-finite value")
    return result


def _verify_step_bound(plan: JointTrajectoryPlan, max_step: float) -> None:
    for previous, current in zip(plan.points, plan.points[1:]):
        step = max(abs(now - before) for before, now in zip(previous, current))
        if step > max_step + 1e-12:
            raise RuntimeError(
                f"planned step {step:.6f} rad exceeds limit {max_step:.6f} rad"
            )
