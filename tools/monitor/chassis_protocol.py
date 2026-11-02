"""Decode the fixed Kinco CANopen mapping used by the chassis simulation."""

from dataclasses import dataclass
import math
import struct

from chassis_config import CAN_INTERFACES, MOTOR_IDS
from torso_protocol import CanFrame, DecodeError


def position_radians(raw: int, encoder_resolution: int) -> float:
    """Encoder counts -> motor-shaft radians; preserve sign and multi-turn angle."""
    return raw * math.tau / encoder_resolution


def velocity_radians_per_second(raw: int, encoder_resolution: int) -> float:
    """Kinco velocity DEC -> motor-shaft rad/s, without gearing or body mapping."""
    motor_rpm = raw * 1875.0 / (512.0 * encoder_resolution)
    return motor_rpm * math.tau / 60.0


@dataclass(frozen=True)
class MotorEvent:
    motor_id: int
    kind: str
    value: int = 0
    control_word: int | None = None


def decode_frame(frame: CanFrame, channel: str) -> list[MotorEvent]:
    """Ignore stretch/unrelated traffic; reject malformed recognized frames.

    RPDO1: controlword + target position (steer) / velocity (wheel).
    RPDO2: current + controlword. TPDO1: actual position + velocity.
    Expedited SDO covers startup/fallback target, mode and feedback objects.
    """
    if channel not in ("wire_action", "wire_state"):
        raise ValueError(f"invalid chassis wire channel: {channel}")
    can_id, data = frame.can_id, frame.data
    if can_id == 0 and channel == "wire_action":
        if len(data) != 2:
            raise DecodeError("Kinco NMT must have DLC 2")
        if data[0] in (2, 128, 129, 130):
            return [MotorEvent(node, "reset") for node in MOTOR_IDS
                    if (data[1] == 0 or data[1] == node)
                    and CAN_INTERFACES[node - 1] == frame.can_interface]
        return []
    base = can_id & ~0x7F
    node = can_id - base
    if node not in MOTOR_IDS or base not in (0x180, 0x200, 0x280, 0x300, 0x580, 0x600, 0x700):
        return []
    if frame.can_interface != CAN_INTERFACES[node - 1]:
        raise DecodeError(f"Kinco motor {node} on wrong CAN interface")
    if frame.flags & 0x02:  # Node guarding RTR carries no measurement.
        return []
    action = base in (0x200, 0x300, 0x600)
    if action != (channel == "wire_action"):
        raise DecodeError("Kinco frame arrived in the wrong MCU direction")
    if base == 0x200:
        if len(data) != 6:
            raise DecodeError("Kinco RPDO1 must have DLC 6")
        control, raw = struct.unpack("<Hi", data)
        return [MotorEvent(node, "target", raw, control)]
    if base == 0x300:
        if len(data) != 4:
            raise DecodeError("Kinco RPDO2 must have DLC 4")
        return [MotorEvent(node, "current")]
    if base == 0x180:
        if len(data) != 8:
            raise DecodeError("Kinco TPDO1 must have DLC 8")
        position, velocity = struct.unpack("<ii", data)
        return [MotorEvent(node, "feedback", position if node <= 4 else velocity)]
    if base == 0x280:
        if len(data) != 4:
            raise DecodeError("Kinco TPDO2 must have DLC 4")
        return []
    if base == 0x700:
        if len(data) != 1:
            raise DecodeError("Kinco heartbeat must have DLC 1")
        return [MotorEvent(node, "reset")] if data[0] == 0 else []
    if len(data) != 8:
        raise DecodeError("Kinco SDO must have DLC 8")
    command, index, sub = struct.unpack_from("<BHB", data)
    widths = ({0x2F: 1, 0x2B: 2, 0x27: 3, 0x23: 4} if action else
              {0x4F: 1, 0x4B: 2, 0x47: 3, 0x43: 4})
    if command not in widths:  # read request, write acknowledgement, abort, etc.
        return []
    kind = None
    width = 0
    if index == 0x6060 and sub == 0:
        kind, width = "mode", 1
    elif action and index == 0x6040 and sub == 0:
        kind, width = "control", 2
    elif action and index == 0x60F6 and sub == 8:
        kind, width = "current", 2
    elif action and sub == 0 and index == (0x607A if node <= 4 else 0x60FF):
        kind, width = "target", 4
    elif not action and sub == 0 and index == (0x6064 if node <= 4 else 0x606C):
        kind, width = "feedback", 4
    if kind is None:
        return []
    if widths[command] != width:
        raise DecodeError(f"wrong Kinco SDO width for 0x{index:04x}:{sub}")
    value = int.from_bytes(data[4:4 + width], "little", signed=kind != "control")
    return [MotorEvent(node, kind, value)]
