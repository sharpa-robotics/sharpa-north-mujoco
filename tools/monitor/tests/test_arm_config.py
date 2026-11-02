import sys
import tempfile
import unittest
from pathlib import Path


TOOL_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(TOOL_DIR))

from arm_config import load_arm_config


class ConfigTest(unittest.TestCase):
    def _load(self, text, side="left"):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "test.conf"
            path.write_text(text, encoding="utf-8")
            return load_arm_config(path, side)

    def test_left_defaults_match_plugin_layout(self):
        config = self._load("[devices.arm_left]\ndriver=udp_arm\n")
        self.assertEqual(config.side, "left")
        self.assertEqual(len(config.endpoints), 5)
        self.assertEqual(config.endpoints[0].bind_ip, "127.0.40.101")
        self.assertEqual(config.endpoints[0].peer_port, 8080)
        self.assertEqual(
            [endpoint.motor_indices for endpoint in config.endpoints],
            [(0,), (1,), (2,), (3, 4), (5, 6)],
        )

    def test_right_defaults_and_override(self):
        config = self._load(
            """
mujoco_root=/opt/mujoco

[devices.arm_right]
driver=udp_arm
endpoint4_bind_ip=192.168.5.10
endpoint4_peer_port=9001
""",
            "right",
        )
        self.assertEqual(config.endpoints[0].bind_ip, "127.0.40.106")
        self.assertEqual(config.endpoints[0].peer_port, 8081)
        self.assertEqual(config.endpoints[4].bind_ip, "192.168.5.10")
        self.assertEqual(config.endpoints[4].peer_port, 9001)

    def test_disabled_wrong_driver_and_bad_coverage_are_rejected(self):
        with self.assertRaisesRegex(ValueError, "disabled"):
            self._load("[devices.arm_left]\nenabled=false\n")
        with self.assertRaisesRegex(ValueError, "udp_arm"):
            self._load("[devices.arm_left]\ndriver=socketcan\n")
        with self.assertRaisesRegex(ValueError, "cover 0..6"):
            self._load(
                """
[devices.arm_left]
driver=udp_arm
endpoint4_motor_indices=4,6
"""
            )


if __name__ == "__main__":
    unittest.main()
