"""Strict, side-effect-free decoders for monitored Wave joint UDP packets."""

from __future__ import annotations

import math
import socket
import struct
import zlib
from dataclasses import dataclass
from typing import Optional, Tuple


JOINT_COUNT = 22
JOINT_PACKET_SIZE = 348
JOINT_DATA_OFFSET = 72
KNOWN_TYPE_FLAG_MASK = 0xE07F


class DecodeError(ValueError):
    pass


@dataclass(frozen=True)
class WaveJointPacket:
    protocol_version: int
    side: str
    control_source: int
    control_mode: int
    running: bool
    sequence: int
    positions: Tuple[float, ...]
    velocities: Tuple[float, ...]
    torques: Tuple[float, ...]


@dataclass(frozen=True)
class UdpDatagram:
    source_ip: str
    destination_ip: str
    source_port: int
    destination_port: int
    payload: bytes


def decode_wave_joint_packet(
    data: bytes, expected_protocol_version: Optional[int] = None
) -> WaveJointPacket:
    """Decode a 22-joint Wave V0 packet from either UDP direction."""

    if len(data) != JOINT_PACKET_SIZE:
        raise DecodeError(f"Wave joint packet must be {JOINT_PACKET_SIZE} bytes")
    expected_crc = struct.unpack_from("<I", data, len(data) - 4)[0]
    actual_crc = zlib.crc32(data[:-4]) & 0xFFFFFFFF
    if expected_crc != actual_crc:
        raise DecodeError("Wave joint packet CRC32 mismatch")
    if data[0:2] != b"\xbb\xee" or data[3] != 0:
        raise DecodeError("invalid Wave joint pre-header")
    protocol_version = data[2]
    if protocol_version not in (0, 1):
        raise DecodeError(f"unsupported Wave protocol version {protocol_version}")
    if expected_protocol_version is not None and protocol_version != expected_protocol_version:
        raise DecodeError(
            f"expected Wave protocol version {expected_protocol_version}, got {protocol_version}"
        )
    if data[4] != 0 or data[5] != 0 or data[6] != 0 or data[7] != JOINT_COUNT:
        raise DecodeError("invalid Wave joint data header")

    type_flag = struct.unpack_from("<H", data, 24)[0]
    if type_flag & (~KNOWN_TYPE_FLAG_MASK & 0xFFFF):
        raise DecodeError("Wave joint type flag contains reserved bits")
    side = "right" if type_flag & 0x8000 else "left"
    control_source = type_flag & 0x07
    control_mode = (type_flag >> 3) & 0x0F
    running = bool(type_flag & 0x2000)
    sequence = struct.unpack_from("<H", data, 26)[0]

    positions = []
    velocities = []
    torques = []
    offset = JOINT_DATA_OFFSET
    for _joint in range(JOINT_COUNT):
        position, velocity, torque = struct.unpack_from("<fff", data, offset)
        positions.append(position)
        velocities.append(velocity)
        torques.append(torque)
        offset += 12
    if not all(math.isfinite(value) for values in (positions, velocities, torques) for value in values):
        raise DecodeError("Wave joint packet contains a non-finite value")
    return WaveJointPacket(
        protocol_version=protocol_version,
        side=side,
        control_source=control_source,
        control_mode=control_mode,
        running=running,
        sequence=sequence,
        positions=tuple(positions),
        velocities=tuple(velocities),
        torques=tuple(torques),
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
