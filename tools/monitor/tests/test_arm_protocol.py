import socket
import struct
import sys
import unittest
from pathlib import Path


TOOL_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(TOOL_DIR))

from arm_protocol import (
    CONTROL_COMMAND,
    FEEDBACK_COMMAND,
    DecodeError,
    crc16_ccitt_false,
    decode_control,
    decode_feedback,
    extract_ipv4_udp,
)


def make_control(axes, sequence=0x01020304):
    frame = bytearray(struct.pack("<I", sequence))
    for node_id, position, velocity, torque, kp, kd in axes:
        frame.extend(
            struct.pack(
                "<BBbHihhHH",
                CONTROL_COMMAND,
                node_id,
                11,
                0x000F,
                round(position * 1_000_000),
                round(velocity * 1_000),
                round(torque * 100),
                round(kp * 100),
                round(kd * 1_000),
            )
        )
    frame.extend(struct.pack("<H", crc16_ccitt_false(frame)))
    return bytes(frame)


def make_feedback(axes, sequence=7, timestamp=123456789, extended=False):
    frame = bytearray(struct.pack("<I", sequence))
    for node_id, position, velocity, torque in axes:
        frame.extend(
            struct.pack(
                "<BBbHihhHih",
                FEEDBACK_COMMAND,
                node_id,
                8,
                0x0027,
                round(position * 1_000_000),
                round(velocity * 1_000),
                round(torque * 100),
                0,
                round(position * 1_000_000),
                round(velocity * 1_000),
            )
        )
        if extended:
            frame.extend(struct.pack("<bbbh4x", 48, 25, 30, round(torque * 100)))
    frame.extend(struct.pack("<H", crc16_ccitt_false(frame)))
    frame.extend(struct.pack("<Q", timestamp))
    return bytes(frame)


class ProtocolTest(unittest.TestCase):
    def test_crc_known_vector_and_control(self):
        self.assertEqual(crc16_ccitt_false(b"123456789"), 0x29B1)
        decoded = decode_control(
            make_control([(0, 1.25, -0.5, 2.5, 12.0, 0.8)])
        )
        self.assertEqual(decoded.sequence, 0x01020304)
        self.assertEqual(len(decoded.axes), 1)
        self.assertEqual(decoded.axes[0].mode, 11)
        self.assertAlmostEqual(decoded.axes[0].position_rad, 1.25)
        self.assertAlmostEqual(decoded.axes[0].velocity_rad_s, -0.5)
        self.assertAlmostEqual(decoded.axes[0].torque_nm, 2.5)
        self.assertAlmostEqual(decoded.axes[0].kp, 12.0)
        self.assertAlmostEqual(decoded.axes[0].kd, 0.8)

    def test_dual_feedback_and_validation(self):
        frame = make_feedback([(0, 0.25, 0.5, -1.5), (1, -0.75, -0.2, 2.0)])
        decoded = decode_feedback(frame)
        self.assertEqual(decoded.sequence, 7)
        self.assertEqual(decoded.timestamp_ns, 123456789)
        self.assertEqual(len(decoded.axes), 2)
        self.assertAlmostEqual(decoded.axes[1].position_rad, -0.75)
        self.assertAlmostEqual(decoded.axes[0].low_speed_velocity_rad_s, 0.5)

        corrupt = bytearray(frame)
        corrupt[9] ^= 1
        with self.assertRaisesRegex(DecodeError, "CRC16"):
            decode_feedback(bytes(corrupt))
        with self.assertRaisesRegex(DecodeError, "35 or 56"):
            decode_feedback(frame[:-1])

    def test_legacy_and_extended_feedback_layouts(self):
        axes = [(0, 0.25, 0.5, -1.5), (1, -0.75, -0.2, 2.0)]
        timestamp = 0x0102030405060708
        for extended, sizes in ((False, (35, 56)), (True, (44, 74))):
            for count, size in enumerate(sizes, start=1):
                with self.subTest(extended=extended, axes=count):
                    frame = make_feedback(
                        axes[:count], timestamp=timestamp, extended=extended
                    )
                    self.assertEqual(len(frame), size)
                    decoded = decode_feedback(frame)
                    self.assertEqual(decoded.sequence, 7)
                    self.assertEqual(decoded.timestamp_ns, timestamp)
                    self.assertEqual(len(decoded.axes), count)
                    for actual, (node, position, velocity, torque) in zip(decoded.axes, axes):
                        self.assertEqual(actual.node_id, node)
                        self.assertEqual(actual.mode, 8)
                        self.assertEqual(actual.status_word, 0x0027)
                        self.assertEqual(actual.fault_code, 0)
                        self.assertAlmostEqual(actual.position_rad, position)
                        self.assertAlmostEqual(actual.velocity_rad_s, velocity)
                        self.assertAlmostEqual(actual.torque_nm, torque)
                        self.assertAlmostEqual(actual.low_speed_position_rad, position)
                        self.assertAlmostEqual(actual.low_speed_velocity_rad_s, velocity)

                    corrupt = bytearray(frame)
                    # The CRC covers the extended diagnostics too.
                    corrupt[-11] ^= 1
                    with self.assertRaisesRegex(DecodeError, "CRC16"):
                        decode_feedback(bytes(corrupt))
                    with self.assertRaisesRegex(DecodeError, "frame must be"):
                        decode_feedback(frame[:-1])

                    invalid_command = bytearray(frame)
                    invalid_command[4] = 0x20
                    crc_offset = len(frame) - 10
                    struct.pack_into(
                        "<H", invalid_command, crc_offset,
                        crc16_ccitt_false(invalid_command[:crc_offset]),
                    )
                    with self.assertRaisesRegex(DecodeError, "feedback command"):
                        decode_feedback(bytes(invalid_command))

    def test_extract_ipv4_udp(self):
        payload = make_control([(0, 0.0, 0.0, 0.0, 0.0, 0.0)])
        udp = struct.pack(">HHHH", 8080, 21000, 8 + len(payload), 0) + payload
        ip = bytearray(20)
        ip[0] = 0x45
        ip[9] = 17
        ip[12:16] = socket.inet_aton("127.0.0.1")
        ip[16:20] = socket.inet_aton("127.0.40.101")
        packet = extract_ipv4_udp(bytes(ip) + udp)
        self.assertEqual(packet.source_port, 8080)
        self.assertEqual(packet.destination_port, 21000)
        self.assertEqual(packet.payload, payload)


if __name__ == "__main__":
    unittest.main()
