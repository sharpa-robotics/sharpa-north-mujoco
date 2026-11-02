import sys
import tempfile
import unittest
from pathlib import Path


TOOL_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(TOOL_DIR))

from torso_config import DEFAULT_MOTOR_IDS, load_torso_config


class ConfigTest(unittest.TestCase):
    def _load(self, text):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "test.conf"
            path.write_text(text, encoding="utf-8")
            return load_torso_config(path)

    def test_can_config_and_defaults(self):
        config = self._load(
            """
mujoco_root=/opt/mujoco

[devices.torso]
enabled=true
driver=can
lower_can_interface=vcan10
upper_can_interface=vcan11
lower_motor_ids=1,2,3,4,5
upper_motor_ids=6,7,8
"""
        )
        self.assertEqual(config.driver, "can")
        self.assertEqual(config.motor_ids, DEFAULT_MOTOR_IDS)
        self.assertEqual(config.endpoints[0].socketcan_interface, "vcan10")
        self.assertEqual(config.endpoints[1].socketcan_interface, "vcan11")
        self.assertEqual(config.endpoints[0].bind_port, 45520)

    def test_udp_config(self):
        config = self._load(
            """
[devices.torso]
driver=udp
motor_ids=1,2,4,3,5,6,7,8
motor_signs=-1,-1,1,1,-1,1,1,1
lower_bind_ip=127.0.0.2
lower_bind_port=50001
lower_peer_ip=127.0.0.3
lower_peer_port=50002
lower_can_if=7
lower_motor_ids=1,2,3,4,5
upper_motor_ids=6,7,8
"""
        )
        lower = config.endpoints[0]
        self.assertEqual(lower.bind_ip, "127.0.0.2")
        self.assertEqual(lower.bind_port, 50001)
        self.assertEqual(lower.peer_port, 50002)
        self.assertEqual(lower.can_interface_id, 7)

    def test_invalid_motor_coverage_is_rejected(self):
        with self.assertRaisesRegex(ValueError, "cover"):
            self._load(
                """
[devices.torso]
driver=can
lower_motor_ids=1,2,3,4
upper_motor_ids=6,7,8
"""
            )

    def test_disabled_torso_is_rejected(self):
        with self.assertRaisesRegex(ValueError, "disabled"):
            self._load("[devices.torso]\nenabled=false\n")


if __name__ == "__main__":
    unittest.main()
