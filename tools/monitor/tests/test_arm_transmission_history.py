import math
import sys
import unittest
from pathlib import Path


TOOL_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(TOOL_DIR))

from arm_history import MonitorHistory
from arm_transmission import ArmTransmission


class FakeClock:
    def __init__(self):
        self.value = 100.0

    def __call__(self):
        return self.value


class TransmissionHistoryTest(unittest.TestCase):
    def test_left_and_right_transmission_round_trip(self):
        joints = (0.1, -0.2, 0.3, -0.4, 0.5, -0.6, 0.7)
        for side in ("left", "right"):
            transmission = ArmTransmission(side)
            actual = transmission.motor_to_joint(
                transmission.joint_to_motor(joints)
            )
            for expected, result in zip(joints, actual):
                self.assertAlmostEqual(result, expected)

    def test_wire_cache_conversion_rate_limit_and_trim(self):
        clock = FakeClock()
        history = MonitorHistory(
            "left", history_seconds=1.0, plot_rate_hz=20.0, clock=clock
        )
        joints = (0.1, -0.2, 0.3, -0.4, 0.5, -0.6, 0.7)
        motors = ArmTransmission("left").joint_to_motor(joints)
        endpoint_groups = ((0,), (1,), (2,), (3, 4), (5, 6))
        for group in endpoint_groups:
            self.assertTrue(
                history.add_wire(
                    "wire_state", {index: motors[index] for index in group}
                )
            )
        snapshot = history.snapshot()
        self.assertEqual(snapshot.counts["wire_state"], 5)
        for axis, expected in enumerate(joints):
            self.assertEqual(len(snapshot.samples["wire_state"][axis]), 1)
            self.assertAlmostEqual(
                snapshot.samples["wire_state"][axis][0][1],
                math.degrees(expected),
            )

        clock.value += 0.01
        history.add_wire("wire_state", dict(enumerate(motors)))
        self.assertEqual(len(history.snapshot().samples["wire_state"][0]), 1)
        clock.value += 0.05
        history.add_wire("wire_state", dict(enumerate(motors)))
        self.assertEqual(len(history.snapshot().samples["wire_state"][0]), 2)
        clock.value += 1.1
        self.assertEqual(len(history.snapshot().samples["wire_state"][0]), 0)

    def test_wire_packet_only_appends_its_dependent_joints(self):
        clock = FakeClock()
        history = MonitorHistory(
            "right", history_seconds=1.0, plot_rate_hz=math.inf, clock=clock
        )
        transmission = ArmTransmission("right")
        joints = (0.1, -0.2, 0.3, -0.4, 0.5, -0.6, 0.7)
        motors = transmission.joint_to_motor(joints)
        endpoint_groups = ((0,), (1,), (2,), (3, 4), (5, 6))
        for group in endpoint_groups:
            history.add_wire(
                "wire_action", {index: motors[index] for index in group}
            )

        clock.value += 0.001
        updated_joints = list(joints)
        updated_joints[0] += 0.01
        updated_motors = transmission.joint_to_motor(updated_joints)
        history.add_wire("wire_action", {0: updated_motors[0]})

        samples = history.snapshot().samples["wire_action"]
        self.assertEqual([len(axis) for axis in samples], [2, 1, 1, 1, 1, 1, 1])
        self.assertAlmostEqual(samples[0][-1][1], math.degrees(updated_joints[0]))

    def test_wire_differential_joint_requires_both_fresh_motors(self):
        history = MonitorHistory("left", plot_rate_hz=math.inf)
        motors = ArmTransmission("left").joint_to_motor([0.1] * 7)

        history.add_wire("wire_action", {3: motors[3]})
        history.add_wire("wire_action", {4: motors[4]})

        samples = history.snapshot().samples["wire_action"]
        self.assertEqual(len(samples[3]), 0)
        self.assertEqual(len(samples[4]), 0)

    def test_invalid_zenoh_and_wire_values_are_rejected(self):
        history = MonitorHistory("right")
        self.assertFalse(history.add_zenoh("zenoh_action", [0.0] * 6))
        self.assertFalse(history.add_wire("wire_action", {0: math.nan}))
        snapshot = history.snapshot()
        self.assertEqual(snapshot.errors["zenoh_decode"], 1)
        self.assertEqual(snapshot.errors["wire_decode"], 1)

    def test_motor_pd_selects_raw_motor_radians_and_position_selects_joints(self):
        history = MonitorHistory("left", plot_rate_hz=math.inf)
        joints = (0.1, -0.2, 0.3, -0.4, 0.5, -0.6, 0.7)
        motors = ArmTransmission("left").joint_to_motor(joints)

        history.set_control_mode(5)
        history.add_wire("wire_action", dict(enumerate(motors)))
        snapshot = history.snapshot()
        self.assertEqual(snapshot.display_space, "motor")
        self.assertEqual(snapshot.control_mode_name, "MotorPd")
        self.assertAlmostEqual(snapshot.samples["wire_action"][0][0][1], motors[0])

        history.set_control_mode(0)
        history.add_wire("wire_action", dict(enumerate(motors)))
        snapshot = history.snapshot()
        self.assertEqual(snapshot.display_space, "joint")
        self.assertEqual(snapshot.control_mode_name, "JointPosition")
        self.assertAlmostEqual(
            snapshot.samples["wire_action"][0][0][1], math.degrees(joints[0])
        )

    def test_exact_heartbeat_mode_overrides_udp_inference(self):
        history = MonitorHistory("right")
        self.assertTrue(history.observe_udp_mode(11, has_mit_gains=True))
        self.assertEqual(history.snapshot().control_mode, 5)
        self.assertEqual(history.snapshot().control_mode_source, "UDP inference")

        self.assertTrue(history.set_control_mode(1, source="heartbeat"))
        self.assertFalse(history.observe_udp_mode(8, has_mit_gains=False))
        snapshot = history.snapshot()
        self.assertEqual(snapshot.control_mode, 1)
        self.assertEqual(snapshot.control_mode_name, "JointPd")
        self.assertEqual(snapshot.control_mode_source, "heartbeat")


if __name__ == "__main__":
    unittest.main()
