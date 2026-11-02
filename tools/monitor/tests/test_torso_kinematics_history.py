import math
import sys
import unittest
from pathlib import Path


TOOL_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(TOOL_DIR))

from torso_history import MonitorHistory
from torso_kinematics import ParallelWaistKinematics
from torso_protocol import CONTROL_MODE_MIT, CONTROL_MODE_POSITION


class FakeClock:
    def __init__(self):
        self.value = 100.0

    def __call__(self):
        return self.value


class KinematicsHistoryTest(unittest.TestCase):
    def test_parallel_waist_round_trip(self):
        kinematics = ParallelWaistKinematics()
        for pitch_deg, roll_deg in ((0, 0), (20, 10), (80, -15), (115, 5)):
            motor = kinematics.joint_to_motor(
                math.radians(pitch_deg), math.radians(roll_deg)
            )
            self.assertIsNotNone(motor)
            joint = kinematics.motor_to_joint(*motor)
            self.assertIsNotNone(joint)
            self.assertAlmostEqual(math.degrees(joint[0]), pitch_deg, places=4)
            self.assertAlmostEqual(math.degrees(joint[1]), roll_deg, places=4)

    def test_rate_limit_and_history_trim(self):
        clock = FakeClock()
        history = MonitorHistory(
            (1, 2, 4, 3, 5, 6, 7, 8),
            (-1, -1, 1, 1, -1, 1, 1, 1),
            history_seconds=1.0,
            plot_rate_hz=20.0,
            clock=clock,
        )
        history.add_zenoh("zenoh_action", [0.1] * 8)
        clock.value += 0.01
        history.add_zenoh("zenoh_action", [0.2] * 8)
        self.assertEqual(len(history.snapshot().samples["zenoh_action"][0]), 1)
        clock.value += 0.05
        history.add_zenoh("zenoh_action", [0.3] * 8)
        self.assertEqual(len(history.snapshot().samples["zenoh_action"][0]), 2)
        clock.value += 1.1
        self.assertEqual(len(history.snapshot().samples["zenoh_action"][0]), 0)

    def test_wire_sign_and_waist_conversion(self):
        clock = FakeClock()
        motor_ids = (1, 2, 4, 3, 5, 6, 7, 8)
        history = MonitorHistory(
            motor_ids,
            (-1, -1, 1, 1, -1, 1, 1, 1),
            clock=clock,
        )
        waist = ParallelWaistKinematics()
        motors = waist.joint_to_motor(math.radians(30), math.radians(-10))
        history.add_wire(
            "wire_state",
            {1: math.radians(5), motor_ids[2]: motors[0], motor_ids[3]: motors[1]},
        )
        snapshot = history.snapshot()
        self.assertAlmostEqual(snapshot.samples["wire_state"][0][-1][1], -5.0)
        self.assertAlmostEqual(snapshot.samples["wire_state"][2][-1][1], 30.0, places=4)
        self.assertAlmostEqual(snapshot.samples["wire_state"][3][-1][1], -10.0, places=4)

    def test_each_motor_mode_selects_its_own_wire_view(self):
        motor_ids = (1, 2, 4, 3, 5, 6, 7, 8)
        history = MonitorHistory(
            motor_ids,
            (-1, -1, 1, 1, -1, 1, 1, 1),
            plot_rate_hz=math.inf,
        )

        history.add_wire(
            "wire_action", {1: 0.25}, control_mode=CONTROL_MODE_MIT
        )
        history.add_wire("wire_state", {1: 0.20})
        history.add_wire(
            "wire_action", {2: 0.40}, control_mode=CONTROL_MODE_POSITION
        )
        history.add_wire("wire_state", {2: 0.30})

        snapshot = history.snapshot()
        self.assertEqual(snapshot.control_modes[0], CONTROL_MODE_MIT)
        self.assertEqual(snapshot.display_spaces[0], "motor")
        self.assertAlmostEqual(snapshot.samples["wire_action"][0][-1][1], 0.25)
        self.assertAlmostEqual(snapshot.samples["wire_state"][0][-1][1], 0.20)

        self.assertEqual(snapshot.control_modes[1], CONTROL_MODE_POSITION)
        self.assertEqual(snapshot.display_spaces[1], "joint")
        self.assertAlmostEqual(
            snapshot.samples["wire_action"][1][-1][1], math.degrees(-0.40)
        )
        self.assertAlmostEqual(
            snapshot.samples["wire_state"][1][-1][1], math.degrees(-0.30)
        )

    def test_parallel_waist_motors_are_selected_independently(self):
        motor_ids = (1, 2, 4, 3, 5, 6, 7, 8)
        history = MonitorHistory(
            motor_ids,
            (-1, -1, 1, 1, -1, 1, 1, 1),
            plot_rate_hz=math.inf,
        )
        history.add_wire(
            "wire_action",
            {4: 0.50},
            control_mode=CONTROL_MODE_MIT,
        )
        history.add_wire(
            "wire_action",
            {3: -0.40},
            control_mode=CONTROL_MODE_POSITION,
        )
        # A waist mode transition invalidates the coupled FK cache. The next
        # packet from the other motor completes a fresh pair.
        history.add_wire(
            "wire_action",
            {4: 0.50},
            control_mode=CONTROL_MODE_MIT,
        )
        history.add_wire("wire_state", {4: 0.45, 3: -0.35})

        snapshot = history.snapshot()
        self.assertEqual(
            snapshot.control_modes[2:4],
            (CONTROL_MODE_MIT, CONTROL_MODE_POSITION),
        )
        self.assertEqual(snapshot.display_spaces[2:4], ("motor", "joint"))
        self.assertAlmostEqual(snapshot.samples["wire_action"][2][-1][1], 0.50)
        self.assertTrue(snapshot.samples["wire_action"][3])


if __name__ == "__main__":
    unittest.main()
