"""Validated, immutable state values passed from hardware to ROS."""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Iterable


@dataclass(frozen=True)
class JointStateSnapshot:
    """One coherent sample of the arm joints.

    Gripper values are intentionally excluded for now. The SDK returns the arm
    group first, so only the number of values named by ``names`` is retained.
    """

    names: tuple[str, ...]
    position: tuple[float, ...]
    velocity: tuple[float, ...]
    effort: tuple[float, ...]
    monotonic_ns: int

    @classmethod
    def from_sdk(
        cls,
        *,
        names: Iterable[str],
        position: Iterable[float],
        velocity: Iterable[float],
        effort: Iterable[float],
        monotonic_ns: int,
    ) -> "JointStateSnapshot":
        joint_names = tuple(str(name) for name in names)
        if not joint_names:
            raise ValueError("joint state requires at least one joint name")
        if len(joint_names) != len(set(joint_names)):
            raise ValueError("joint state names must be unique")
        if monotonic_ns < 0:
            raise ValueError("monotonic_ns must not be negative")

        size = len(joint_names)
        return cls(
            names=joint_names,
            position=_validated_values("position", position, size),
            velocity=_validated_values("velocity", velocity, size),
            effort=_validated_values("effort", effort, size),
            monotonic_ns=int(monotonic_ns),
        )


def _validated_values(
    label: str, values: Iterable[float], required_size: int
) -> tuple[float, ...]:
    converted = tuple(float(value) for value in values)
    if len(converted) < required_size:
        raise ValueError(
            f"{label} has {len(converted)} values; expected at least {required_size}"
        )
    arm_values = converted[:required_size]
    invalid = [index for index, value in enumerate(arm_values) if not math.isfinite(value)]
    if invalid:
        raise ValueError(f"{label} contains non-finite values at indices {invalid}")
    return arm_values


@dataclass(frozen=True)
class ArmDiagnostics:
    """Read-only diagnostic view of the current hardware manager."""

    mode: str
    enabled: bool
    control_loop_active: bool
    state_machine: str
    joint_names: tuple[str, ...]
    per_joint_status_code: tuple[int, ...]
    error_codes: tuple[str, ...]
