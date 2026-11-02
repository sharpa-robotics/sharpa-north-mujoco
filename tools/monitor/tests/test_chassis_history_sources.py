import math
from pathlib import Path
import socket
import struct
import sys
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch
import zlib


TOOL_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(TOOL_DIR))

from chassis_config import CAN_INTERFACES, ChassisMonitorConfig
from chassis_history import MonitorHistory
from chassis_protocol import MotorEvent
from chassis_sources import WireCapture, ZenohCapture
from test_chassis_protocol import sdo


RAW_VALUES = (9 * 65536, -32768, 3 * 65536, -5 * 65536,
              -16777216, 33554432, 1234567, -7654321)


def packet(node, base, data, action=True, source=0x02050000):
    # Same on-wire packing as common::encode_can_fd_payload/encode_mcu_uplink.
    payload = bytes((CAN_INTERFACES[node - 1] if node <= 8 else 0, 0, 0, 0))
    payload += struct.pack("<IBB", base + node, len(data), 0) + data
    header = bytearray(32)
    header[:4] = b"POC2"
    header[4] = 1
    struct.pack_into(">I", header, 6, source)
    struct.pack_into(">I", header, 10, 5)
    struct.pack_into(">Q", header, 14, 1234)
    struct.pack_into(">HHH", header, 22, len(payload), 0, 1)
    envelope = bytes(header) + payload
    envelope += struct.pack(">I", zlib.crc32(envelope[4:]) & 0xFFFFFFFF)
    if not action:
        envelope += struct.pack("<Q", 1234)
    ports = (45220, 45620) if action else (45620, 45220)
    udp = struct.pack(">HHHH", *ports, 8 + len(envelope), 0) + envelope
    ip = bytearray(20)
    ip[0], ip[9] = 0x45, 17
    struct.pack_into(">H", ip, 2, len(ip) + len(udp))
    ip[12:16] = ip[16:20] = socket.inet_aton("127.0.0.1")
    return bytes(ip) + udp


class HistoryAndWireTest(unittest.TestCase):
    def setUp(self):
        self.now = [0.0]
        self.config = ChassisMonitorConfig(Path("test"))
        self.history = MonitorHistory(self.config, history_seconds=2.0, clock=lambda: self.now[0])
        self.capture = WireCapture(self.config, self.history)

    def feed(self, action=True, nodes=range(1, 9), values=RAW_VALUES):
        for node in nodes:
            data = (struct.pack("<Hi", 0x103F if node <= 4 else 15, values[node - 1]) if action else
                    struct.pack("<ii", values[node - 1] if node <= 4 else 0,
                                values[node - 1] if node > 4 else 0))
            self.capture.process_packet(packet(node, 0x200 if action else 0x180, data, action))

    def test_one_motor_packet_immediately_appends_only_that_motor(self):
        self.feed(nodes=(3,))
        self.feed(action=False, nodes=(3,))
        snap = self.history.snapshot()
        for channel in ("wire_action", "wire_state"):
            self.assertEqual(snap.counts[channel], 1)
            self.assertEqual(snap.unavailable[channel][2], "")
            self.assertAlmostEqual(snap.samples[channel][2][-1][1], 6 * math.pi)
            for axis in (0, 1, 3, 4, 5, 6, 7):
                self.assertEqual(snap.samples[channel][axis], [])
                self.assertIn("waiting", snap.unavailable[channel][axis])
        self.assertEqual(sum(snap.errors.values()), 0)

    def test_full_pipeline_preserves_motor_units_instead_of_chassis_twist(self):
        self.feed()
        self.feed(action=False)
        snap = self.history.snapshot()
        for channel in ("wire_action", "wire_state"):
            self.assertEqual(len(snap.samples[channel]), 8)
            self.assertEqual(snap.counts[channel], 8)
            self.assertEqual(snap.unavailable[channel], [""] * 8)
            self.assertAlmostEqual(snap.samples[channel][0][-1][1], 18 * math.pi)
            self.assertAlmostEqual(snap.samples[channel][1][-1][1], -math.pi)
            self.assertAlmostEqual(snap.samples[channel][4][-1][1], -31.25 * math.pi)
            self.assertAlmostEqual(snap.samples[channel][5][-1][1], 62.5 * math.pi)
        self.now[0] = 0.01
        self.feed(nodes=(1,))
        snap = self.history.snapshot()
        self.assertEqual([len(axis) for axis in snap.samples["wire_action"]], [2] + [1] * 7)
        self.assertEqual([len(axis) for axis in snap.samples["wire_state"]], [1] * 8)

    def test_current_mode_breaks_only_affected_motor_target_and_keeps_its_feedback(self):
        self.feed()
        self.feed(action=False)
        self.history.add("zenoh_action", (1, 2, 3))
        before = self.history.snapshot()
        self.now[0] = 0.01
        self.capture.process_packet(packet(5, 0x300, struct.pack("<hH", 100, 15)))
        self.feed(nodes=(5,))  # RPDO1 does not change an observed current mode.
        self.feed(action=False, nodes=(5,))
        snap = self.history.snapshot()
        self.assertIn("current", snap.unavailable["wire_action"][4])
        self.assertTrue(math.isnan(snap.samples["wire_action"][4][-1][1]))
        self.assertEqual(snap.unavailable["wire_state"], [""] * 8)
        self.assertEqual(snap.unavailable["zenoh_action"], [""] * 3)
        for axis in (0, 1, 2, 3, 5, 6, 7):
            self.assertEqual(snap.samples["wire_action"][axis], before.samples["wire_action"][axis])
            self.assertEqual(snap.unavailable["wire_action"][axis], "")
        self.capture.process_packet(packet(5, 0x600, sdo(0x6060, -3, 1)))
        self.assertIn("waiting", self.history.snapshot().unavailable["wire_action"][4])
        self.feed(nodes=(5,))  # No new targets from the other seven motors needed.
        snap = self.history.snapshot()
        self.assertEqual(snap.unavailable["wire_action"], [""] * 8)
        self.assertTrue(any(math.isnan(value) for _, value in snap.samples["wire_action"][4]))

    def test_mode_write_invalidates_only_its_motor_before_current_command(self):
        self.feed()
        self.capture.process_packet(packet(1, 0x600, sdo(0x6060, 4, 1)))
        self.feed()
        snap = self.history.snapshot()
        self.assertIn("current", snap.unavailable["wire_action"][0])
        self.assertEqual(snap.unavailable["wire_action"][1:], [""] * 7)

    def test_feedback_expiry_and_recovery_are_independent(self):
        self.feed(action=False)
        self.now[0] = 0.15
        self.feed(action=False, nodes=range(1, 8))
        snap = self.history.snapshot()
        self.assertEqual(snap.unavailable["wire_state"], [""] * 7 + ["stale"])
        self.assertTrue(math.isnan(snap.samples["wire_state"][7][-1][1]))
        before = snap.samples["wire_state"][:7]
        self.feed(action=False, nodes=(8,))
        snap = self.history.snapshot()
        self.assertEqual(snap.unavailable["wire_state"], [""] * 8)
        self.assertEqual(snap.samples["wire_state"][:7], before)
        self.assertTrue(any(math.isnan(v) for _, v in snap.samples["wire_state"][7]))

    def test_disabled_relative_and_reset_events_are_local(self):
        self.feed()
        self.feed(action=False)
        for control, reason in ((6, "disabled"), (0x107F, "relative")):
            self.capture.process_packet(packet(1, 0x200, struct.pack("<Hi", control, 100)))
            snap = self.history.snapshot()
            self.assertIn(reason, snap.unavailable["wire_action"][0])
            self.assertEqual(snap.unavailable["wire_action"][1:], [""] * 7)
            self.assertEqual(snap.unavailable["wire_state"], [""] * 8)
        self.history.add_wire(MotorEvent(2, "reset"))
        snap = self.history.snapshot()
        self.assertIn("reset", snap.unavailable["wire_action"][1])
        self.assertIn("reset", snap.unavailable["wire_state"][1])
        self.assertEqual(snap.unavailable["wire_action"][2:], [""] * 6)
        self.assertEqual(snap.unavailable["wire_state"][2:], [""] * 6)
        self.feed(nodes=(2,))
        self.feed(action=False, nodes=(2,))
        self.assertEqual(self.history.snapshot().unavailable["wire_action"][1], "")
        self.assertEqual(self.history.snapshot().unavailable["wire_state"][1], "")

    def test_trimming_and_nonfinite_zenoh_data(self):
        self.history.add("zenoh_action", (0.1, -0.2, 0.3))
        self.assertFalse(self.history.add("zenoh_action", (math.nan, 0, 0)))
        self.now[0] = 3.1
        snap = self.history.snapshot()
        self.assertTrue(all(math.isnan(v) for _, v in snap.samples["zenoh_action"][0]))
        self.assertEqual(snap.errors["zenoh_decode"], 1)
        self.assertEqual(snap.unavailable["zenoh_action"], ["stale"] * 3)

    def test_bad_crc_and_source_are_counted_without_samples_and_stretch_is_ignored(self):
        bad = bytearray(packet(1, 0x200, struct.pack("<Hi", 0x103F, 0)))
        bad[-1] ^= 1
        self.capture.process_packet(bytes(bad))
        self.capture.process_packet(packet(1, 0x180, bytes(8), False, source=0x01050000))
        self.capture.process_packet(packet(9, 0x200, bytes(6)))
        snap = self.history.snapshot()
        self.assertEqual(snap.errors["wire_decode"], 2)
        self.assertEqual(sum(snap.counts.values()), 0)


class ZenohTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        sys.path.insert(0, str(TOOL_DIR.parents[2] / "sharpa_platform" / "proto"))
        import north_pb2
        cls.proto = north_pb2

    def setUp(self):
        self.now = [0.0]
        self.history = MonitorHistory(ChassisMonitorConfig(Path("test")), clock=lambda: self.now[0])
        self.capture = ZenohCapture(Mock(), self.proto, self.history)

    def test_protobuf_units_and_repeated_feedback_timestamp_goes_stale(self):
        command = self.proto.ChassisCommand(vx=0.2, vy=-0.3, vtheta=0.4, v_l=100)
        self.capture._on_action(SimpleNamespace(payload=command.SerializeToString()))
        state = self.proto.ChassisState()
        state.header.stamp.seconds = 1
        state.velocity.linear.x, state.velocity.linear.y = 0.1, -0.2
        state.velocity.angular.z = 0.3
        sample = SimpleNamespace(payload=state.SerializeToString())
        self.capture._on_state(sample)
        snap = self.history.snapshot()
        self.assertAlmostEqual(snap.samples["zenoh_action"][1][-1][1], -0.3, places=6)
        self.assertAlmostEqual(snap.samples["zenoh_state"][2][-1][1], 0.3, places=6)
        self.now[0] = 1.1
        self.capture._on_state(sample)
        snap = self.history.snapshot()
        self.assertEqual(snap.counts["zenoh_state"], 1)
        self.assertEqual(snap.unavailable["zenoh_state"], ["stale"] * 3)

    def test_missing_feedback_or_invalid_protobuf_is_not_zero_speed(self):
        state = self.proto.ChassisState()
        self.capture._on_state(SimpleNamespace(payload=state.SerializeToString()))
        self.capture._on_action(SimpleNamespace(payload=b"\xff"))
        state.velocity.linear.x = 0.1
        self.capture._on_state(SimpleNamespace(payload=state.SerializeToString()))
        snap = self.history.snapshot()
        self.assertEqual(sum(snap.counts.values()), 0)
        self.assertEqual(snap.errors["zenoh_decode"], 2)
        self.assertIn("timestamp", snap.unavailable["zenoh_state"][0])

    def test_partial_subscription_startup_is_cleaned_up(self):
        session, subscriber = Mock(), Mock()
        self.capture._zenoh.open.return_value = session
        session.declare_subscriber.side_effect = [subscriber, RuntimeError("subscription failed")]
        with patch("chassis_sources.client_config", return_value=object()):
            with self.assertRaises(RuntimeError):
                self.capture.start()
        self.capture.stop()
        subscriber.undeclare.assert_called_once()
        session.close.assert_called_once()


if __name__ == "__main__":
    unittest.main()
