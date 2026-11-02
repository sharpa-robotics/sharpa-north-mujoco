import socket
import struct
import sys
import unittest
import zlib
from pathlib import Path


TOOL_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(TOOL_DIR))

from torso_protocol import (
    MCU_COMMAND_SOURCE,
    MCU_SENSOR_SOURCE,
    CONTROL_MODE_MIT,
    CONTROL_MODE_POSITION,
    CanFrame,
    DecodeError,
    decode_embedded_can,
    decode_mcu_envelope,
    decode_mit_position_range,
    decode_position_action,
    decode_position_feedback,
    decode_socketcan_frame,
    extract_ipv4_udp,
    position_action_control_mode,
)


def make_embedded_can(can_id, data, can_interface=0):
    return (
        bytes((can_interface, 0, 0, 0))
        + struct.pack("<I", can_id)
        + bytes((len(data), 1))
        + data
    )


def make_envelope(source_code, payload, sequence=5, gptp=None):
    header = bytearray(32)
    header[0:4] = b"POC2"
    header[4] = 1
    struct.pack_into(">I", header, 6, source_code)
    struct.pack_into(">I", header, 10, sequence)
    struct.pack_into(">Q", header, 14, 123456789)
    struct.pack_into(">H", header, 22, len(payload))
    struct.pack_into(">HH", header, 24, 0, 1)
    packet = bytes(header) + payload
    packet += struct.pack(">I", zlib.crc32(packet[4:]) & 0xFFFFFFFF)
    if gptp is not None:
        packet += struct.pack("<Q", gptp)
    return packet


class ProtocolTest(unittest.TestCase):
    def test_socketcan_fd_and_embedded_can(self):
        data = bytes(range(16))
        packet = struct.pack("=IBBBB64s", 0x305, len(data), 1, 0, 0, data.ljust(64, b"\0"))
        decoded = decode_socketcan_frame(packet)
        self.assertEqual(decoded.can_id, 0x305)
        self.assertEqual(decoded.data, data)
        embedded = decode_embedded_can(make_embedded_can(0x305, data, 2))
        self.assertEqual(embedded.can_interface, 2)
        self.assertEqual(embedded.data, data)

    def test_mcu_downlink_and_uplink(self):
        payload = make_embedded_can(0x200, bytes(64))
        downlink = decode_mcu_envelope(make_envelope(MCU_COMMAND_SOURCE, payload))
        self.assertIsNone(downlink.gptp_ns)
        uplink = decode_mcu_envelope(make_envelope(MCU_SENSOR_SOURCE, payload, gptp=42))
        self.assertEqual(uplink.gptp_ns, 42)
        broken = bytearray(make_envelope(MCU_COMMAND_SOURCE, payload))
        broken[33] ^= 0x80
        with self.assertRaisesRegex(DecodeError, "CRC"):
            decode_mcu_envelope(bytes(broken))

    def test_csp_mit_and_feedback(self):
        csp = bytearray(64)
        csp[56] = 1
        struct.pack_into(">h", csp, 1, 16384)
        action = decode_position_action(CanFrame(0x200, bytes(csp)), {1}, {})
        self.assertAlmostEqual(action[1], 3.141592653589793 / 2.0)
        self.assertEqual(
            position_action_control_mode(CanFrame(0x200, bytes(csp))),
            CONTROL_MODE_POSITION,
        )

        mit = bytearray(64)
        mit[56] = 4
        struct.pack_into(">H", mit, 1, 65535)
        action = decode_position_action(CanFrame(0x210, bytes(mit)), {4}, {4: 2.5})
        self.assertAlmostEqual(action[4], 2.5)
        self.assertEqual(
            position_action_control_mode(CanFrame(0x210, bytes(mit))),
            CONTROL_MODE_MIT,
        )

        feedback = bytearray(16)
        struct.pack_into(">h", feedback, 0, -8192)
        state = decode_position_feedback(CanFrame(0x304, bytes(feedback)), {4})
        self.assertAlmostEqual(state[4], -3.141592653589793 / 4.0)

    def test_sdo_mit_range(self):
        data = bytearray(8)
        data[0] = 0x43
        data[1:3] = struct.pack("<H", 0x2542)
        data[4:8] = struct.pack("<I", 400)
        self.assertEqual(decode_mit_position_range(CanFrame(0x584, bytes(data))), (4, 4.0))

    def test_extract_ipv4_udp(self):
        payload = b"torso"
        udp = struct.pack(">HHHH", 45120, 45520, 8 + len(payload), 0) + payload
        ip = bytearray(20)
        ip[0] = 0x45
        ip[9] = 17
        ip[12:16] = socket.inet_aton("127.0.0.1")
        ip[16:20] = socket.inet_aton("127.0.0.2")
        packet = extract_ipv4_udp(bytes(ip) + udp)
        self.assertEqual(packet.source_port, 45120)
        self.assertEqual(packet.destination_ip, "127.0.0.2")
        self.assertEqual(packet.payload, payload)


if __name__ == "__main__":
    unittest.main()
