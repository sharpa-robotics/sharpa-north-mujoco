"""Strict, side-effect-free decoders for the arm realtime UDP protocol."""

from __future__ import annotations

import socket
import struct
from dataclasses import dataclass
from typing import Optional, Tuple


CONTROL_COMMAND = 0x20
FEEDBACK_COMMAND = 0x18
CONTROL_AXIS_SIZE = 17
FEEDBACK_AXIS_SIZE = 21
EXTENDED_FEEDBACK_AXIS_SIZE = 30
HEADER_SIZE = 4
CRC_SIZE = 2
TIMESTAMP_SIZE = 8


class DecodeError(ValueError):
    pass


@dataclass(frozen=True)
class AxisControl:
    node_id: int
    mode: int
    control_word: int
    position_rad: float
    velocity_rad_s: float
    torque_nm: float
    kp: float
    kd: float


@dataclass(frozen=True)
class AxisFeedback:
    node_id: int
    mode: int
    status_word: int
    position_rad: float
    velocity_rad_s: float
    torque_nm: float
    fault_code: int
    low_speed_position_rad: float
    low_speed_velocity_rad_s: float


@dataclass(frozen=True)
class ArmControlFrame:
    sequence: int
    axes: Tuple[AxisControl, ...]


@dataclass(frozen=True)
class ArmFeedbackFrame:
    sequence: int
    axes: Tuple[AxisFeedback, ...]
    timestamp_ns: int


@dataclass(frozen=True)
class UdpDatagram:
    source_ip: str
    destination_ip: str
    source_port: int
    destination_port: int
    payload: bytes


def crc16_ccitt_false(data: bytes) -> int:
    crc = 0xFFFF
    for value in data:
        crc ^= value << 8
        for _bit in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


def _axis_count(size: int, axis_size: int, trailer_size: int) -> int:
    for count in (1, 2):
        if size == HEADER_SIZE + count * axis_size + trailer_size:
            return count
    return 0


def decode_control(data: bytes) -> ArmControlFrame:
    axis_count = _axis_count(len(data), CONTROL_AXIS_SIZE, CRC_SIZE)
    if not axis_count:
        raise DecodeError("arm control frame must be 23 or 40 bytes")
    expected_crc = struct.unpack_from("<H", data, len(data) - CRC_SIZE)[0]
    if expected_crc != crc16_ccitt_false(data[:-CRC_SIZE]):
        raise DecodeError("arm control frame CRC16 mismatch")
    axes = []
    for slot in range(axis_count):
        offset = HEADER_SIZE + slot * CONTROL_AXIS_SIZE
        command, node_id, mode, control_word, position, velocity, torque, kp, kd = (
            struct.unpack_from("<BBbHihhHH", data, offset)
        )
        if command != CONTROL_COMMAND:
            raise DecodeError(f"unexpected arm control command 0x{command:02x}")
        axes.append(
            AxisControl(
                node_id=node_id,
                mode=mode,
                control_word=control_word,
                position_rad=position / 1_000_000.0,
                velocity_rad_s=velocity / 1_000.0,
                torque_nm=torque / 100.0,
                kp=kp / 100.0,
                kd=kd / 1_000.0,
            )
        )
    return ArmControlFrame(
        sequence=struct.unpack_from("<I", data)[0], axes=tuple(axes)
    )


def decode_feedback(data: bytes) -> ArmFeedbackFrame:
    trailer_size = CRC_SIZE + TIMESTAMP_SIZE
    axis_size = FEEDBACK_AXIS_SIZE
    axis_count = _axis_count(len(data), axis_size, trailer_size)
    if not axis_count:
        axis_size = EXTENDED_FEEDBACK_AXIS_SIZE
        axis_count = _axis_count(len(data), axis_size, trailer_size)
    if not axis_count:
        raise DecodeError(
            "arm feedback frame must be 35 or 56 bytes (legacy), "
            "or 44 or 74 bytes (extended)"
        )
    crc_offset = HEADER_SIZE + axis_count * axis_size
    expected_crc = struct.unpack_from("<H", data, crc_offset)[0]
    if expected_crc != crc16_ccitt_false(data[:crc_offset]):
        raise DecodeError("arm feedback frame CRC16 mismatch")
    axes = []
    for slot in range(axis_count):
        # Extended feedback appends nine diagnostic bytes to each axis;
        # positions keep the legacy layout, but the axis stride and CRC move.
        offset = HEADER_SIZE + slot * axis_size
        (
            command,
            node_id,
            mode,
            status_word,
            position,
            velocity,
            torque,
            fault_code,
            low_speed_position,
            low_speed_velocity,
        ) = struct.unpack_from("<BBbHihhHih", data, offset)
        if command != FEEDBACK_COMMAND:
            raise DecodeError(f"unexpected arm feedback command 0x{command:02x}")
        axes.append(
            AxisFeedback(
                node_id=node_id,
                mode=mode,
                status_word=status_word,
                position_rad=position / 1_000_000.0,
                velocity_rad_s=velocity / 1_000.0,
                torque_nm=torque / 100.0,
                fault_code=fault_code,
                low_speed_position_rad=low_speed_position / 1_000_000.0,
                low_speed_velocity_rad_s=low_speed_velocity / 1_000.0,
            )
        )
    return ArmFeedbackFrame(
        sequence=struct.unpack_from("<I", data)[0],
        axes=tuple(axes),
        timestamp_ns=struct.unpack_from("<Q", data, crc_offset + CRC_SIZE)[0],
    )


def extract_ipv4_udp(packet: bytes) -> Optional[UdpDatagram]:
    """Extract a UDP datagram from an AF_PACKET/SOCK_DGRAM IPv4 packet."""

    if len(packet) < 28 or packet[0] >> 4 != 4:
        return None
    header_size = (packet[0] & 0x0F) * 4
    if header_size < 20 or len(packet) < header_size + 8 or packet[9] != 17:
        return None
    fragment = struct.unpack_from(">H", packet, 6)[0]
    if fragment & 0x3FFF:
        return None
    source_port, destination_port, udp_size = struct.unpack_from(
        ">HHH", packet, header_size
    )
    if udp_size < 8 or len(packet) < header_size + udp_size:
        raise DecodeError("truncated UDP packet")
    return UdpDatagram(
        source_ip=socket.inet_ntoa(packet[12:16]),
        destination_ip=socket.inet_ntoa(packet[16:20]),
        source_port=source_port,
        destination_port=destination_port,
        payload=packet[header_size + 8 : header_size + udp_size],
    )
