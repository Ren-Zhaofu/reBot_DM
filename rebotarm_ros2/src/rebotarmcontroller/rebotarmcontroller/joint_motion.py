"""Validation helpers for restricted FollowJointTrajectory commands."""

from __future__ import annotations

import math
from dataclasses import dataclass


JOINT_LIMITS = {
    "joint1": (-2.8, 2.8),
    "joint2": (-3.14, 0.0),
    "joint3": (-3.14, 0.0),
    "joint4": (-1.87, 1.57),
    "joint5": (-1.57, 1.57),
    "joint6": (-3.14, 3.14),
}


@dataclass(frozen=True)
class ValidatedTrajectory:
    targets: tuple[tuple[float, ...], ...]
    times: tuple[float, ...]


@dataclass(frozen=True)
class TrajectoryTolerances:
    path: tuple[float, ...]
    goal: tuple[float, ...]
    goal_time: float


def validate_trajectory(
    trajectory,
    expected_names: tuple[str, ...],
    current: tuple[float, ...],
    *,
    max_delta: float = 0.5,
) -> ValidatedTrajectory:
    """Validate and reorder a ROS JointTrajectory without changing hardware."""
    names = tuple(trajectory.joint_names)
    if len(names) != len(set(names)):
        raise ValueError("trajectory contains duplicate joint names")
    if set(names) != set(expected_names):
        raise ValueError("trajectory must contain exactly: " + ", ".join(expected_names))
    if not trajectory.points:
        raise ValueError("trajectory must contain at least one point")
    if not 0.01 <= max_delta <= 1.0:
        raise ValueError("max_delta must be between 0.01 and 1.0 rad")

    order = tuple(names.index(name) for name in expected_names)
    targets = []
    times = []
    previous_time = -1.0
    previous = tuple(current)
    for index, point in enumerate(trajectory.points):
        if len(point.positions) != len(names):
            raise ValueError(f"point {index} must contain {len(names)} positions")
        target = tuple(float(point.positions[source]) for source in order)
        if any(not math.isfinite(value) for value in target):
            raise ValueError(f"point {index} contains a non-finite position")
        seconds = float(point.time_from_start.sec) + point.time_from_start.nanosec * 1e-9
        if seconds < 0.0 or seconds <= previous_time:
            raise ValueError(
                "time_from_start must be non-negative and strictly increasing"
            )
        for name, value in zip(expected_names, target):
            lower, upper = JOINT_LIMITS[name]
            if not lower <= value <= upper:
                raise ValueError(
                    f"{name} target {value:.6f} is outside [{lower:.6f}, {upper:.6f}]"
                )
        delta = max(abs(value - before) for value, before in zip(target, previous))
        if delta > max_delta:
            raise ValueError(
                f"point {index} delta {delta:.6f} exceeds {max_delta:.6f} rad"
            )
        targets.append(target)
        times.append(seconds)
        previous = target
        previous_time = seconds
    return ValidatedTrajectory(tuple(targets), tuple(times))


def resolve_tolerances(
    request,
    expected_names: tuple[str, ...],
    *,
    default_path: float = 0.05,
    default_goal: float = 0.01,
) -> TrajectoryTolerances:
    """Resolve ROS JointTolerance arrays into hardware-order position limits."""
    path = _resolve_joint_tolerances(
        request.path_tolerance, expected_names, default_path, "path_tolerance"
    )
    goal = _resolve_joint_tolerances(
        request.goal_tolerance, expected_names, default_goal, "goal_tolerance"
    )
    duration = request.goal_time_tolerance
    goal_time = float(duration.sec) + duration.nanosec * 1e-9
    if goal_time < 0.0 or goal_time > 10.0:
        raise ValueError("goal_time_tolerance must be between 0 and 10 seconds")
    return TrajectoryTolerances(path, goal, goal_time)


def _resolve_joint_tolerances(items, names, default, label):
    if not 0.001 <= default <= 0.1:
        raise ValueError(f"default {label} must be between 0.001 and 0.1 rad")
    values = {name: default for name in names}
    seen = set()
    for item in items:
        if item.name not in values:
            raise ValueError(f"{label} contains unknown joint: {item.name}")
        if item.name in seen:
            raise ValueError(f"{label} contains duplicate joint: {item.name}")
        seen.add(item.name)
        if item.velocity != 0.0 or item.acceleration != 0.0:
            raise ValueError(f"{label} supports position tolerance only")
        if item.position < 0.0:
            raise ValueError(f"{label} cannot disable a hardware safety limit")
        if item.position > 0.0:
            if not 0.001 <= item.position <= 0.1:
                raise ValueError(f"{label} position must be between 0.001 and 0.1 rad")
            values[item.name] = float(item.position)
    return tuple(values[name] for name in names)
