import tempfile
import sys
import unittest
from pathlib import Path


TOOL_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(TOOL_DIR))

from hand_config import load_hand_config


class ConfigTest(unittest.TestCase):
    def _load(self, text, sides=("left", "right")):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "test.conf"
            path.write_text(text, encoding="utf-8")
            return load_hand_config(path, sides)

    def test_dual_hand_config_and_defaults(self):
        config = self._load(
            """
mujoco_root=/opt/mujoco

[devices.hand_left]
driver=wave_sdk_loopback

[devices.hand_right]
driver=wave_sdk_loopback
device_ip=127.0.60.20
sdk_joint_port=50110
"""
        )
        self.assertEqual([endpoint.side for endpoint in config.endpoints], ["left", "right"])
        self.assertEqual(config.endpoints[0].command_udp_port, 50020)
        self.assertEqual(config.endpoints[1].device_ip, "127.0.60.20")
        self.assertEqual(config.endpoints[1].sdk_joint_port, 50110)

    def test_single_side_does_not_require_other_section(self):
        config = self._load(
            "[devices.hand_left]\ndriver=wave_sdk_loopback\n", ("left",)
        )
        self.assertEqual(len(config.endpoints), 1)
        self.assertEqual(config.endpoints[0].serial_number, "SIM-WAVE-L-0001")

    def test_disabled_or_wrong_driver_is_rejected(self):
        with self.assertRaisesRegex(ValueError, "disabled"):
            self._load("[devices.hand_left]\nenabled=false\n", ("left",))
        with self.assertRaisesRegex(ValueError, "wave_sdk_loopback"):
            self._load("[devices.hand_left]\ndriver=udp_mcu\n", ("left",))


if __name__ == "__main__":
    unittest.main()
