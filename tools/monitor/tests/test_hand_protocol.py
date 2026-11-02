import math
import socket
import struct
import sys
import unittest
import zlib
from pathlib import Path


TOOL_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(TOOL_DIR))

from hand_protocol import DecodeError, decode_wave_joint_packet, extract_ipv4_udp


def make_joint_packet(protocol_version=0, side="left"):
    packet = bytearray(348)
    packet[0:4] = bytes((0xBB, 0xEE, protocol_version, 0))
    packet[4:8] = bytes((0, 0, 0, 22))
    type_flag = 4 | (13 << 3) | (1 << 13)
    if side == "right":
        type_flag |= 1 << 15
    struct.pack_into("<HHQ", packet, 24, type_flag, 42, 123456789)
    for joint in range(22):
        struct.pack_into(
            "<fff", packet, 72 + joint * 12, joint * 0.1, joint * 0.2, joint * 0.3
        )
    struct.pack_into("<I", packet, 344, zlib.crc32(packet[:-4]) & 0xFFFFFFFF)
    return bytes(packet)


class ProtocolTest(unittest.TestCase):
    def test_decode_downlink_and_uplink(self):
        action = decode_wave_joint_packet(make_joint_packet(0, "left"), 0)
        self.assertEqual(action.side, "left")
        self.assertEqual(action.control_source, 4)
        self.assertEqual(action.control_mode, 13)
        self.assertTrue(action.running)
        self.assertEqual(action.sequence, 42)
        self.assertAlmostEqual(action.positions[10], 1.0)

        state = decode_wave_joint_packet(make_joint_packet(1, "right"), 1)
        self.assertEqual(state.side, "right")
        self.assertAlmostEqual(state.velocities[5], 1.0)

    def test_version_crc_and_finite_validation(self):
        with self.assertRaisesRegex(DecodeError, "version"):
            decode_wave_joint_packet(make_joint_packet(0), 1)
        broken = bytearray(make_joint_packet())
        broken[80] ^= 0x40
        with self.assertRaisesRegex(DecodeError, "CRC"):
            decode_wave_joint_packet(bytes(broken))
        nonfinite = bytearray(make_joint_packet())
        struct.pack_into("<f", nonfinite, 72, math.nan)
        struct.pack_into("<I", nonfinite, 344, zlib.crc32(nonfinite[:-4]) & 0xFFFFFFFF)
        with self.assertRaisesRegex(DecodeError, "non-finite"):
            decode_wave_joint_packet(bytes(nonfinite))

    def test_extract_ipv4_udp(self):
        payload = make_joint_packet()
        udp = struct.pack(">HHHH", 40000, 50020, 8 + len(payload), 0) + payload
        ip = bytearray(20)
        ip[0] = 0x45
        ip[9] = 17
        ip[12:16] = socket.inet_aton("127.0.0.1")
        ip[16:20] = socket.inet_aton("127.0.50.10")
        decoded = extract_ipv4_udp(bytes(ip) + udp)
        self.assertEqual(decoded.destination_port, 50020)
        self.assertEqual(decoded.destination_ip, "127.0.50.10")
        self.assertEqual(decoded.payload, payload)


if __name__ == "__main__":
    unittest.main()
