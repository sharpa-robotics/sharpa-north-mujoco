"""Strict, side-effect-free decoders for torso wire protocols."""

from __future__ import annotations

import socket
import struct
import zlib
from dataclasses import dataclass
from typing import Dict, Optional, Set


MCU_ENVELOPE_SIZE = 32
MCU_CRC_SIZE = 4
MCU_GPTP_SIZE = 8
MCU_VERSION = 1
MCU_COMMAND_SOURCE = (0x02 << 24) | (0x05 << 16)
MCU_SENSOR_SOURCE = (0x01 << 24) | (0x05 << 16)
CAN_EFF_MASK = 0x1FFFFFFF
CSP_ACTION_CAN_ID = 0x200
MIT_ACTION_CAN_ID = 0x210
CONTROL_MODE_POSITION = "Position/CSP"
CONTROL_MODE_MIT = "MIT"


class DecodeError(ValueError):
    pass


@dataclass(frozen=True)
class CanFrame:
    can_id: int
    data: bytes
    can_interface: int = 0
    flags: int = 0


@dataclass(frozen=True)
class McuEnvelope:
    source_code: int
    sequence: int
    timestamp_ns: int
    fragment_index: int
    fragment_count: int
    payload: bytes
    gptp_ns: Optional[int]


@dataclass(frozen=True)
class UdpDatagram:
    source_ip: str
    destination_ip: str
    source_port: int
    destination_port: int
    payload: bytes


def _i16_be(data: bytes, offset: int = 0) -> int:
    return struct.unpack_from(">h", data, offset)[0]


def decode_socketcan_frame(packet: bytes) -> CanFrame:
    if len(packet) not in (16, 72):
        raise DecodeError(f"unexpected SocketCAN frame size: {len(packet)}")
    can_id, length, flags = struct.unpack_from("=IBB", packet, 0)
    max_length = 8 if len(packet) == 16 else 64
    if length > max_length:
        raise DecodeError("SocketCAN data length exceeds frame capacity")
    return CanFrame(can_id & CAN_EFF_MASK, packet[8 : 8 + length], flags=flags)


def decode_mcu_envelope(datagram: bytes) -> McuEnvelope:
    if len(datagram) < MCU_ENVELOPE_SIZE + MCU_CRC_SIZE:
        raise DecodeError("short MCU envelope")
    if datagram[4] != MCU_VERSION:
        raise DecodeError("unsupported MCU envelope version")
    payload_size = struct.unpack_from(">H", datagram, 22)[0]
    base_size = MCU_ENVELOPE_SIZE + payload_size + MCU_CRC_SIZE
    if len(datagram) not in (base_size, base_size + MCU_GPTP_SIZE):
        raise DecodeError("MCU envelope length mismatch")
    expected_crc = struct.unpack_from(">I", datagram, MCU_ENVELOPE_SIZE + payload_size)[0]
    actual_crc = zlib.crc32(datagram[4 : MCU_ENVELOPE_SIZE + payload_size]) & 0xFFFFFFFF
    if expected_crc != actual_crc:
        raise DecodeError("MCU envelope CRC32 mismatch")
    fragment_index, fragment_count = struct.unpack_from(">HH", datagram, 24)
    if fragment_index != 0 or fragment_count != 1:
        raise DecodeError("fragmented torso MCU payload is unsupported")
    gptp_ns = None
    if len(datagram) == base_size + MCU_GPTP_SIZE:
        gptp_ns = struct.unpack_from("<Q", datagram, base_size)[0]
    return McuEnvelope(
        source_code=struct.unpack_from(">I", datagram, 6)[0],
        sequence=struct.unpack_from(">I", datagram, 10)[0],
        timestamp_ns=struct.unpack_from(">Q", datagram, 14)[0],
        fragment_index=fragment_index,
        fragment_count=fragment_count,
        payload=datagram[MCU_ENVELOPE_SIZE : MCU_ENVELOPE_SIZE + payload_size],
        gptp_ns=gptp_ns,
    )


def decode_embedded_can(payload: bytes) -> CanFrame:
    if len(payload) < 10:
        raise DecodeError("short embedded CAN payload")
    if payload[2:4] != b"\0\0":
        raise DecodeError("embedded CAN reserved bytes are nonzero")
    length = payload[8]
    if length > 64 or len(payload) != 10 + length:
        raise DecodeError("embedded CAN payload length mismatch")
    return CanFrame(
        can_id=struct.unpack_from("<I", payload, 4)[0] & CAN_EFF_MASK,
        data=payload[10:],
        can_interface=payload[0],
        flags=payload[9],
    )


def decode_position_action(
    frame: CanFrame,
    valid_motor_ids: Set[int],
    mit_position_ranges: Dict[int, float],
) -> Optional[Dict[int, float]]:
    """Decode motor-space targets in radians from CSP or MIT frames."""

    result: Dict[int, float] = {}
    if frame.can_id == CSP_ACTION_CAN_ID:
        if len(frame.data) != 64:
            raise DecodeError("CSP frame must have DLC 64")
        for slot in range(8):
            motor_id = frame.data[56 + slot]
            if motor_id not in valid_motor_ids:
                continue
            result[motor_id] = _i16_be(frame.data, slot * 7 + 1) * 3.141592653589793 / 32768.0
        return result
    if frame.can_id == MIT_ACTION_CAN_ID:
        if len(frame.data) != 64:
            raise DecodeError("MIT frame must have DLC 64")
        for slot in range(6):
            motor_id = frame.data[56 + slot]
            if motor_id not in valid_motor_ids:
                continue
            raw = struct.unpack_from(">H", frame.data, slot * 9 + 1)[0]
            position_range = mit_position_ranges.get(motor_id, 3.14)
            result[motor_id] = raw / 65535.0 * (2.0 * position_range) - position_range
        return result
    return None


def position_action_control_mode(frame: CanFrame) -> Optional[str]:
    """Return the per-motor control mode encoded by a periodic action frame."""

    if frame.can_id == CSP_ACTION_CAN_ID:
        return CONTROL_MODE_POSITION
    if frame.can_id == MIT_ACTION_CAN_ID:
        return CONTROL_MODE_MIT
    return None


def decode_position_feedback(
    frame: CanFrame, valid_motor_ids: Set[int]
) -> Optional[Dict[int, float]]:
    motor_id = frame.can_id - 0x300
    if motor_id not in valid_motor_ids:
        return None
    if len(frame.data) not in (12, 16):
        raise DecodeError("torso feedback frame must have DLC 12 or 16")
    radians = _i16_be(frame.data) * 3.141592653589793 / 32768.0
    return {motor_id: radians}


def decode_mit_position_range(frame: CanFrame) -> Optional[tuple[int, float]]:
    """Passively learn object 0x2542 from an expedited SDO response."""

    if not (0x581 <= frame.can_id <= 0x5FF) or len(frame.data) < 8:
        return None
    command = frame.data[0]
    index = frame.data[1] | (frame.data[2] << 8)
    if index != 0x2542 or command not in (0x43, 0x47, 0x4B, 0x4F):
        return None
    value = struct.unpack_from("<I", frame.data, 4)[0]
    position_range = value * 0.01
    if position_range <= 0.0:
        raise DecodeError("invalid MIT position range in SDO response")
    return frame.can_id - 0x580, position_range


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
    source_port, destination_port, udp_size = struct.unpack_from(">HHH", packet, header_size)
    if udp_size < 8 or len(packet) < header_size + udp_size:
        raise DecodeError("truncated UDP packet")
    return UdpDatagram(
        source_ip=socket.inet_ntoa(packet[12:16]),
        destination_ip=socket.inet_ntoa(packet[16:20]),
        source_port=source_port,
        destination_port=destination_port,
        payload=packet[header_size + 8 : header_size + udp_size],
    )
