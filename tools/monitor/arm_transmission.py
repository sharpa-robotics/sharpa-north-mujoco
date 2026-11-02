"""Production-equivalent left/right arm position transmission."""

from __future__ import annotations

from typing import Iterable, Tuple


AXIS_COUNT = 7
CABLE_RATIOS = (1.0, 1.0, 1.0, 2.34375, 2.34375, 1.0, 1.0)


def _parameters(side: str):
    if side == "left":
        return (
            (-1.0, -1.0, 1.0, -1.0, 1.0, 1.0, -1.0),
            (1.0, 1.0, 1.0, 1.0, 0.5, 1.0, -1.0),
        )
    if side == "right":
        return (
            (1.0, -1.0, -1.0, -1.0, 1.0, 1.0, -1.0),
            (1.0, 1.0, 1.0, 1.0, -0.5, 1.0, 1.0),
        )
    raise ValueError(f"invalid arm side: {side!r}")


def _axes(values: Iterable[float]) -> Tuple[float, ...]:
    result = tuple(values)
    if len(result) != AXIS_COUNT:
        raise ValueError("arm transmission requires exactly seven axes")
    return result


class ArmTransmission:
    def __init__(self, side: str):
        self._directions, self._joint_ratios = _parameters(side)

    def joint_to_motor(self, joint_positions: Iterable[float]) -> Tuple[float, ...]:
        joints = _axes(joint_positions)
        direction = self._directions
        ratio = self._joint_ratios
        cable = CABLE_RATIOS
        motors = [0.0] * AXIS_COUNT
        for axis in range(3):
            motors[axis] = joints[axis] * ratio[axis] * direction[axis] * cable[axis]
        motors[3] = (
            (joints[3] * ratio[3] + joints[4] * ratio[4])
            * direction[3]
            * cable[3]
        )
        motors[4] = (
            (joints[3] * ratio[3] - joints[4] * ratio[4])
            * direction[4]
            * cable[4]
        )
        motors[5] = (
            (joints[5] * ratio[5] + joints[6] * ratio[6])
            * direction[5]
            * cable[5]
        )
        motors[6] = (
            (joints[5] * ratio[5] - joints[6] * ratio[6])
            * direction[6]
            * cable[6]
        )
        return tuple(motors)

    def motor_to_joint(self, motor_positions: Iterable[float]) -> Tuple[float, ...]:
        motors = _axes(motor_positions)
        direction = self._directions
        ratio = self._joint_ratios
        cable = CABLE_RATIOS
        joints = [0.0] * AXIS_COUNT
        for axis in range(3):
            joints[axis] = motors[axis] / (direction[axis] * cable[axis]) / ratio[axis]
        motor3 = motors[3] / (direction[3] * cable[3])
        motor4 = motors[4] / (direction[4] * cable[4])
        joints[3] = 0.5 * (motor3 + motor4) / ratio[3]
        joints[4] = 0.5 * (motor3 - motor4) / ratio[4]
        motor5 = motors[5] / (direction[5] * cable[5])
        motor6 = motors[6] / (direction[6] * cable[6])
        joints[5] = 0.5 * (motor5 + motor6) / ratio[5]
        joints[6] = 0.5 * (motor5 - motor6) / ratio[6]
        return tuple(joints)
