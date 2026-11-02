import math
from pathlib import Path
import struct
import sys
import tempfile
import unittest


TOOL_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(TOOL_DIR))

from chassis_config import CAN_INTERFACES, load_chassis_config
from chassis_protocol import decode_frame, position_radians, velocity_radians_per_second
from torso_protocol import CanFrame, DecodeError


def frame(node, base, payload):
    return CanFrame(base + node, payload, CAN_INTERFACES[node - 1])


def sdo(index, value, width=4, sub=0, response=False):
    command = ({1: 0x4F, 2: 0x4B, 4: 0x43} if response else
               {1: 0x2F, 2: 0x2B, 4: 0x23})[width]
    return struct.pack("<BHB", command, index, sub) + int(value).to_bytes(
        width, "little", signed=True).ljust(4, b"\0")


class ConfigTest(unittest.TestCase):
    def test_repository_profile(self):
        config = load_chassis_config(TOOL_DIR.parents[1] / "sharpa_mujoco.conf")
        self.assertEqual((config.bind_port, config.peer_port), (45620, 45220))
        self.assertEqual((config.encoder_resolution, config.sample_timeout), (65536, 0.1))

    def test_invalid_or_unsupported_config_is_rejected(self):
        for setting in ("enabled=false", "driver=udp_mcu_jiechang",
                        "kinco_bind_port=0", "kinco_peer_ip=::1",
                        "kinco_encoder_resolution=0", "command_timeout_s=inf"):
            with self.subTest(setting=setting), tempfile.TemporaryDirectory() as directory:
                path = Path(directory) / "sim.conf"
                driver = "" if setting.startswith("driver=") else "driver=udp_mcu_kinco\n"
                path.write_text("sdk=ignored\n[devices.chassis]\n" + driver + setting)
                with self.assertRaises(ValueError):
                    load_chassis_config(path)


    def test_gearing_and_motor_mapping_are_not_used(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "sim.conf"
            path.write_text("[devices.chassis]\ndriver=udp_mcu_kinco\n"
                            "kinco_steer_gear_ratio=nan\nkinco_wheel_gear_ratio=nan\n"
                            "motor_signs=irrelevant\n")
            config = load_chassis_config(path)
            self.assertEqual(config.encoder_resolution, 65536)


class MotorUnitsTest(unittest.TestCase):
    def test_motor_shaft_units_preserve_sign_and_do_not_apply_gear_ratios(self):
        self.assertAlmostEqual(position_radians(65536 * 9 // 4, 65536), 9 * math.pi / 2)
        # A negative motor speed of 600 RPM stays -600 RPM, not +60 RPM.
        raw = round(-60 * 10 * 512 * 65536 / 1875)
        # Half of one raw velocity count is about 2.93e-6 rad/s.
        self.assertAlmostEqual(velocity_radians_per_second(raw, 65536), -10 * math.tau, delta=3e-6)

    def test_multi_turn_position_is_not_wrapped_or_zeroed(self):
        self.assertAlmostEqual(position_radians(-3 * 65536, 65536), -6 * math.pi)
        self.assertAlmostEqual(position_radians(4096, 8192), math.pi)


class ProtocolTest(unittest.TestCase):
    def test_pdo_targets_feedback_and_current(self):
        for node in (1, 5):
            target = decode_frame(frame(node, 0x200, struct.pack("<Hi", 0x103F, -12345)), "wire_action")[0]
            self.assertEqual((target.kind, target.value, target.control_word), ("target", -12345, 0x103F))
            state = decode_frame(frame(node, 0x180, struct.pack("<ii", -111, 222)), "wire_state")[0]
            self.assertEqual((state.kind, state.value), ("feedback", -111 if node == 1 else 222))
            current = decode_frame(frame(node, 0x300, struct.pack("<hH", -5, 15)), "wire_action")[0]
            self.assertEqual(current.kind, "current")

    def test_sdo_modes_targets_and_feedback(self):
        for node, index, width, raw, kind, response in (
            (5, 0x6060, 1, -3, "mode", False), (1, 0x6060, 1, 4, "mode", True),
            (1, 0x607A, 4, -123456, "target", False),
            (5, 0x60FF, 4, 654321, "target", False),
            (1, 0x6064, 4, -23456, "feedback", True),
            (5, 0x606C, 4, 23456, "feedback", True),
            (5, 0x6040, 2, 15, "control", False),
        ):
            with self.subTest(node=node, index=index):
                event = decode_frame(frame(node, 0x580 if response else 0x600,
                                           sdo(index, raw, width, response=response)),
                                     "wire_state" if response else "wire_action")[0]
                self.assertEqual((event.kind, event.value), (kind, raw))
        event = decode_frame(frame(5, 0x600, sdo(0x60F6, 12, 2, sub=8)), "wire_action")[0]
        self.assertEqual(event.kind, "current")

    def test_invalid_frames_direction_and_interface(self):
        for packet, channel in (
            (frame(1, 0x200, bytes(5)), "wire_action"),
            (frame(5, 0x180, bytes(7)), "wire_state"),
            (frame(5, 0x300, bytes(3)), "wire_action"),
            (frame(1, 0x200, bytes(6)), "wire_state"),
            (CanFrame(0x181, bytes(8), 0), "wire_state"),
            (frame(5, 0x600, sdo(0x60FF, 1, 2)), "wire_action"),
        ):
            with self.subTest(packet=packet), self.assertRaises(DecodeError):
                decode_frame(packet, channel)

    def test_stretch_unrelated_sdo_and_guard_requests_are_ignored(self):
        self.assertEqual(decode_frame(CanFrame(0x209, bytes(6)), "wire_action"), [])
        self.assertEqual(decode_frame(frame(1, 0x600, bytes.fromhex("4041600000000000")), "wire_action"), [])
        self.assertEqual(decode_frame(frame(1, 0x580, bytes.fromhex("607a600000000000")), "wire_state"), [])
        self.assertEqual(decode_frame(CanFrame(0x701, b"", 1, 2), "wire_action"), [])
        resets = decode_frame(CanFrame(0, bytes((129, 0)), 1), "wire_action")
        self.assertEqual([event.motor_id for event in resets], [1, 4, 5, 8])


if __name__ == "__main__":
    unittest.main()
