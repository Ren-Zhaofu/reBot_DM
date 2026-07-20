"""Non-hardware SDK substitute used by the ROS node by default."""

from __future__ import annotations

import numpy as np

from .hardware_config import HardwareConfig


class FakeRobot:
    def __init__(self, joint_count: int) -> None:
        self._connected = False
        self.disable_calls = 0
        self.enable_calls = 0
        self.hold_target = None
        self.hold_max_velocity = None
        self._zeros = np.zeros(joint_count, dtype=np.float64)

    def connect(self) -> None:
        self._connected = True

    def disconnect(self) -> None:
        self._connected = False

    def get_state(self):
        if not self._connected:
            raise RuntimeError("fake robot is not connected")
        return self._zeros.copy(), self._zeros.copy(), self._zeros.copy()

    def get_joint_status_codes(self, names: tuple[str, ...]) -> tuple[int, ...]:
        return (0,) * len(names)

    def disable_all(self) -> None:
        if not self._connected:
            raise RuntimeError("fake robot is not connected")
        self.disable_calls += 1

    def enable_all(self) -> None:
        if not self._connected:
            raise RuntimeError("fake robot is not connected")
        self.enable_calls += 1

    def enable_position_hold(
        self, positions: tuple[float, ...], max_velocity: float
    ) -> None:
        self.enable_all()
        self.hold_target = tuple(positions)
        self.hold_max_velocity = max_velocity

    def send_position_target(
        self, positions: tuple[float, ...], max_velocity: float
    ) -> None:
        self._zeros = np.asarray(positions, dtype=np.float64)
        self.hold_target = tuple(positions)
        self.hold_max_velocity = max_velocity

    def set_zero(self, joint_name: str) -> None:
        index = int(joint_name.removeprefix("joint")) - 1
        self._zeros[index] = 0.0

    def enter_gravity_compensation(self, positions, kp, kd, torque) -> None:
        self.enable_all()
        self.hold_target = tuple(positions)
        self.gravity_torque = tuple(torque)

    def send_gravity_command(self, positions, kp, kd, torque) -> None:
        self.hold_target = tuple(positions)
        self.gravity_torque = tuple(torque)


def fake_robot_factory(config: HardwareConfig) -> FakeRobot:
    return FakeRobot(len(config.arm_joints))
