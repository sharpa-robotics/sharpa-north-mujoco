"""Check the arm driver's published motion without a running Platform."""

import contextlib
import io
import math
from pathlib import Path
import sys
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "examples" / "motion"))

import arm_motion_driver as arm
from kinematic_motion import ChannelSpec, JointStateListener, PROFILE_NAMES


class Clock:
    def __init__(self):
        self.now = 0.0

    def monotonic(self):
        return self.now

    def sleep(self, duration):
        self.now += max(duration, 0.000001)


class ArmMotionDriverTest(unittest.TestCase):
    def setUp(self):
        with patch.object(sys, "argv", ["arm_motion_driver.py"]):
            args = arm.parse_args()
        self.runner = arm.ArmMotionRunner(
            session=Mock(),
            proto=SimpleNamespace(ArmState=object),
            channels=tuple(
                ChannelSpec(
                    f"{side}_arm", f"action/{side}_arm", f"state/{side}_arm",
                    "ArmCommand", "ArmState", side=side,
                )
                for side in ("left", "right")
            ),
            joint_names=arm.JOINT_NAMES,
            limits_deg=arm.JOINT_LIMITS_DEG,
            amplitudes_deg=arm.JOINT_AMPLITUDES_DEG,
            preferred_directions=arm.PREFERRED_DIRECTIONS,
            args=args,
            frame_id="test",
            enforce_bidirectional_limit_margin=True,
        )
        self.runner._baselines = {f"{side}_arm": [0.0] * 7 for side in ("left", "right")}
        self.runner._last_targets = self.runner._baselines
        self.targets = []

        def publish(targets):
            saved = {label: list(values) for label, values in targets.items()}
            self.targets.append(saved)
            self.runner._last_targets = saved
            self.runner._has_published = True

        self.runner._publish = publish
        for label in self.runner._listeners:
            listener = Mock()
            listener.is_fresh.return_value = True
            listener.snapshot.side_effect = lambda label=label: (
                list(self.runner._last_targets[label]), len(self.targets)
            )
            self.runner._listeners[label] = listener

        clock = Clock()
        self.clock = clock
        contexts = contextlib.ExitStack()
        self.addCleanup(contexts.close)
        contexts.enter_context(patch("kinematic_motion.time.monotonic", clock.monotonic))
        contexts.enter_context(patch("kinematic_motion.time.sleep", clock.sleep))
        contexts.enter_context(contextlib.redirect_stdout(io.StringIO()))

    def test_shoulders_open_before_other_joints_move(self):
        self.runner.prepare()
        pose = [math.radians(value) for value in arm.EXERCISE_POSE_DEG]
        for targets in self.targets:
            self.assertEqual(targets["left_arm"], targets["right_arm"])
            values = targets["left_arm"]
            if abs(values[1] - pose[1]) > 1e-9:
                self.assertEqual(values[:1] + values[2:], [0.0] * 6)
        self.assertEqual(self.runner._baselines["left_arm"], pose)
        self.assertEqual(self.targets[-1]["right_arm"], pose)

    def test_all_profiles_stay_in_the_spread_envelope(self):
        self.runner.prepare()
        self.targets.clear()
        self.runner.run(tuple(range(7)), PROFILE_NAMES)
        deltas = self.runner._calculate_deltas(tuple(range(7)))
        self.assertEqual(deltas["left_arm"], deltas["right_arm"])
        self.assertLess(deltas["left_arm"][1], 0.0)
        for targets in self.targets:
            for values in targets.values():
                degrees = [math.degrees(value) for value in values]
                self.assertTrue(-90.000001 <= degrees[1] <= -59.999999)
                self.assertTrue(4.999999 <= degrees[3] <= 45.000001)
                for axis in (0, 2, 4, 5, 6):
                    self.assertLessEqual(abs(degrees[axis]), 15.000001)

    def test_continuous_swings_slow_down_without_changing_paths_or_other_profiles(self):
        self.runner._args.speed = 2.0
        self.runner._args.rate = 100.0
        self.runner.prepare()
        axes = tuple(range(7))
        for profile in PROFILE_NAMES:
            with self.subTest(profile=profile):
                with patch.object(self.runner, "_profile_speed", return_value=2.0):
                    original = self.runner._plan_segment(profile, axes)
                slowed = self.runner._plan_segment(profile, axes)
                factor = 2.0 if profile in ("sine", "chirp") else 1.0
                self.assertEqual(slowed.targets, original.targets)
                self.assertAlmostEqual(slowed.duration, original.duration * factor)

    def test_extra_j1_sweep_opens_shoulders_before_fixed_endpoints(self):
        for speed, scale in ((1.0, 1.0), (2.0, 10.0)):
            with self.subTest(speed=speed, scale=scale):
                self.runner._args.speed = speed
                self.runner._args.amplitude_scale = scale
                self.runner.prepare()
                self.assertTrue(self.runner.return_to_zero())
                self.targets.clear()
                self.runner.sweep_j1()
                angles = []
                previous = {label: [0.0] * 7 for label in self.runner._baselines}
                shoulder_target = math.radians(arm.J1_SWEEP_SHOULDER_OPEN_DEG)
                first_j1_motion = None
                for targets in self.targets:
                    self.assertEqual(targets["left_arm"], targets["right_arm"])
                    for label, values in targets.items():
                        self.assertEqual(values[2:], [0.0] * 5)
                        self.assertLessEqual(abs(values[0] - previous[label][0]), math.radians(5) + 1e-9)
                    angles.append(targets["left_arm"][0])
                    if first_j1_motion is None and abs(angles[-1]) > 1e-9:
                        first_j1_motion = len(angles) - 1
                    previous = targets
                self.assertIsNotNone(first_j1_motion)
                self.assertAlmostEqual(
                    self.targets[first_j1_motion]["left_arm"][1], shoulder_target
                )
                self.assertAlmostEqual(min(angles), math.radians(-45))
                self.assertAlmostEqual(max(angles), math.radians(135))
                self.assertLess(angles.index(min(angles)), angles.index(max(angles)))
                expected = [0.0, shoulder_target, 0.0, 0.0, 0.0, 0.0, 0.0]
                self.assertEqual(
                    self.targets[-1], {label: list(expected) for label in previous}
                )

    def test_interrupting_extra_j1_sweep_still_returns_to_zero(self):
        self.runner._has_published = True
        original_publish = self.runner._publish

        def publish(targets):
            original_publish(targets)
            if targets["left_arm"][0] < math.radians(-20):
                raise KeyboardInterrupt

        with patch.object(self.runner, "_publish", side_effect=publish):
            with self.assertRaises(KeyboardInterrupt):
                self.runner.sweep_j1()
        self.assertTrue(self.runner.return_to_zero())
        self.assertEqual(self.targets[-1], {label: [0.0] * 7 for label in self.runner._baselines})

    def test_profile_return_holds_exercise_pose_until_exit(self):
        self.runner.prepare()
        displaced = [math.radians(value) for value in (15, -60, 15, 45, 15, 15, 15)]
        self.runner._publish({label: list(displaced) for label in self.runner._baselines})
        self.assertTrue(self.runner.return_to_baseline())
        expected = [math.radians(value) for value in arm.EXERCISE_POSE_DEG]
        self.assertEqual(self.targets[-1]["left_arm"], expected)
        self.assertEqual(self.targets[-1]["right_arm"], expected)

    def test_exit_returns_to_zero_from_nonzero_start_and_closes_shoulders_last(self):
        starts = {
            "left_arm": [math.radians(x) for x in (-0.1, -0.2, 0.3, 0.4, -0.5, 0.6, -0.7)],
            "right_arm": [math.radians(x) for x in (0.2, -0.3, 0.4, 0.5, -0.6, 0.7, -0.8)],
        }
        self.runner._baselines = {label: list(values) for label, values in starts.items()}
        self.runner._last_targets = {label: list(values) for label, values in starts.items()}
        self.runner.prepare()
        displaced = [math.radians(value) for value in (15, -60, 15, 45, 15, 15, 15)]
        self.runner._publish({label: list(displaced) for label in starts})
        self.targets.clear()
        self.assertTrue(self.runner.return_to_zero())
        for targets in self.targets:
            for label, values in targets.items():
                if abs(values[1] - displaced[1]) > 1e-9:
                    for axis in (0, 2, 3, 4, 5, 6):
                        self.assertAlmostEqual(values[axis], 0.0)
        self.assertEqual(self.targets[-1], {label: [0.0] * 7 for label in starts})

    def test_exit_recovers_from_interrupted_shoulder_preparation(self):
        original_publish = self.runner._publish
        count = 0

        def publish(targets):
            nonlocal count
            original_publish(targets)
            count += 1
            if count == 10:
                raise KeyboardInterrupt

        with patch.object(self.runner, "_publish", side_effect=publish):
            with self.assertRaises(KeyboardInterrupt):
                self.runner.prepare()
        self.assertTrue(self.runner.return_to_zero())
        for targets in self.targets:
            for values in targets.values():
                self.assertEqual(values[:1] + values[2:], [0.0] * 6)
        self.assertEqual(self.targets[-1], {label: [0.0]*7 for label in self.runner._baselines})

    def test_exit_allows_known_wrist_holding_error_without_changing_test_tolerance(self):
        self.runner.prepare()
        for label, listener in self.runner._listeners.items():
            def snapshot(label=label):
                positions = list(self.runner._last_targets[label])
                positions[5] += math.radians(1.55)
                return positions, len(self.targets)
            listener.snapshot.side_effect = snapshot
        self.assertTrue(self.runner.return_to_zero())
        self.assertEqual(self.runner._args.return_tolerance_deg, 1.0)
        self.assertEqual(self.targets[-1], {label: [0.0]*7 for label in self.runner._baselines})

    def test_exit_keeps_shoulders_open_if_other_joints_do_not_arrive(self):
        self.runner.prepare()
        displaced = [math.radians(value) for value in (0, -60, 0, 45, 0, 0, 0)]
        self.runner._publish({label: list(displaced) for label in self.runner._baselines})
        self.runner._args.return_timeout = 0.2
        self.runner._listeners["left_arm"].snapshot.side_effect = lambda: (displaced, 10)
        self.targets.clear()
        with contextlib.redirect_stderr(io.StringIO()):
            self.assertFalse(self.runner.return_to_zero())
        self.assertTrue(all(target["left_arm"][1] == displaced[1] for target in self.targets))

    def test_ctrl_c_returns_all_groups_to_zero_during_preparation_and_motion(self):
        for phase in ("prepare", "run"):
            with self.subTest(phase=phase):
                with patch.object(sys, "argv", ["arm_motion_driver.py"]):
                    args = arm.parse_args()
                runner = Mock()
                getattr(runner, phase).side_effect = KeyboardInterrupt
                session = Mock()
                motion = Mock()
                motion.return_to_zero.return_value = True
                with patch.object(arm, "ArmMotionRunner", return_value=runner), \
                        patch.object(arm, "load_dependencies", return_value=(None, None)), \
                        patch.object(arm, "open_session", return_value=session), \
                        patch.object(arm, "MotionSession", return_value=motion):
                    self.assertEqual(arm.run(args), 130)
                motion.return_to_zero.assert_called_once_with()
                motion.close.assert_called_once_with()
                runner.hold_last_targets.assert_not_called()
                session.close.assert_called_once_with()

    def test_j1_sweep_only_skips_preparation_and_profiles_and_always_cleans_up(self):
        for side, outcome in (("both", None), ("left", KeyboardInterrupt),
                              ("right", RuntimeError("sweep failed"))):
            with self.subTest(side=side, outcome=outcome):
                with patch.object(sys, "argv", [
                    "arm_motion_driver.py", "--j1-sweep-only", "--side", side,
                    "--axes", "2", "--speed", "0.5",
                ]):
                    args = arm.parse_args()
                runner = Mock()
                runner.sweep_j1.side_effect = outcome
                session = Mock()
                motion = Mock()
                motion.return_to_zero.return_value = True
                with patch.object(arm, "ArmMotionRunner", return_value=runner) as factory, \
                        patch.object(arm, "load_dependencies", return_value=(None, None)), \
                        patch.object(arm, "open_session", return_value=session), \
                        patch.object(arm, "MotionSession", return_value=motion):
                    if isinstance(outcome, RuntimeError):
                        with self.assertRaisesRegex(RuntimeError, "sweep failed"):
                            arm.run(args)
                    else:
                        self.assertEqual(arm.run(args), 130 if outcome else 0)
                runner.prepare.assert_not_called()
                runner.run.assert_not_called()
                runner.sweep_j1.assert_called_once_with()
                motion.start.assert_called_once_with()
                motion.return_to_zero.assert_called_once_with()
                motion.close.assert_called_once_with()
                session.close.assert_called_once_with()
                self.assertEqual(args.speed, 0.5)
                expected_sides = ("left", "right") if side == "both" else (side,)
                self.assertEqual(
                    tuple(channel.side for channel in factory.call_args.kwargs["channels"]),
                    expected_sides,
                )

    def test_stale_state_prevents_return_commands(self):
        self.runner.prepare()
        self.runner._listeners["left_arm"].is_fresh.return_value = False
        with patch.object(arm.MotionRunner, "_publish") as publish:
            with self.assertRaisesRegex(RuntimeError, "state became stale"):
                arm.ArmMotionRunner._publish(self.runner, self.runner._baselines)
            publish.assert_not_called()

    def test_timeout_identifies_joint_and_persistent_offset_with_feedback_fields(self):
        self.runner._publish(self.runner._baselines)
        self.runner._args.return_timeout = 3.0
        for label, listener in self.runner._listeners.items():
            def snapshot(label=label):
                positions = list(self.runner._last_targets[label])
                if label == "left_arm":
                    positions[5] += math.radians(1.65)
                return positions, len(self.targets)
            listener.snapshot.side_effect = snapshot
            listener.diagnostics.return_value = {
                "age": 0.012, "velocity": [0.0] * 7, "torque": [1.2] * 7,
            }
        output = io.StringIO()
        with contextlib.redirect_stderr(output):
            self.assertFalse(self.runner.return_to_baseline())
        log = output.getvalue()
        self.assertIn("worst=J6(arm_joint_6)", log)
        self.assertIn("actual=1.65deg error=+1.65deg OVER", log)
        self.assertIn("velocity=+0.000deg/s torque=+1.200Nm", log)
        self.assertIn("range=0.000deg/", log)
        self.assertIn("persistent position offset", log)
        self.assertIn("[Arrival][right_arm J7 arm_joint_7]", log)
        self.assertEqual(self.runner._args.return_tolerance_deg, 1.0)

    def test_timeout_distinguishes_changing_feedback_from_stationary_offset(self):
        self.runner._publish(self.runner._baselines)
        self.runner._args.return_timeout = 3.0
        listener = self.runner._listeners["left_arm"]
        def snapshot():
            positions = list(self.runner._last_targets["left_arm"])
            positions[5] += math.radians(2.0 + 0.3 * math.sin(self.clock.now * 3.0))
            return positions, len(self.targets)
        listener.snapshot.side_effect = snapshot
        listener.diagnostics.return_value = {"age": 0.01, "velocity": [], "torque": []}
        output = io.StringIO()
        with contextlib.redirect_stderr(output):
            self.assertFalse(self.runner.return_to_baseline())
        self.assertIn("position still varies", output.getvalue())
        self.assertIn("velocity=n/a torque=n/a", output.getvalue())
        self.assertNotIn("persistent position offset", output.getvalue())

    def test_repeated_frame_is_not_classified_as_stationary_feedback(self):
        self.runner._publish(self.runner._baselines)
        self.runner._args.return_timeout = 1.5
        positions = [0.0] * 7
        positions[5] = math.radians(1.65)
        listener = self.runner._listeners["left_arm"]
        listener.snapshot.side_effect = lambda: (positions, 10)
        output = io.StringIO()
        with contextlib.redirect_stderr(output):
            self.assertFalse(self.runner.return_to_baseline())
        self.assertIn("insufficient fresh history", output.getvalue())
        self.assertNotIn("persistent position offset", output.getvalue())

    def test_wait_logs_once_per_second_instead_of_each_command_frame(self):
        self.runner._publish(self.runner._baselines)
        self.runner._args.return_timeout = 3.0
        listener = self.runner._listeners["left_arm"]
        listener.snapshot.side_effect = lambda: ([math.radians(1.65)] * 7, len(self.targets))
        output = io.StringIO()
        with contextlib.redirect_stdout(output), contextlib.redirect_stderr(io.StringIO()):
            self.assertFalse(self.runner.return_to_baseline())
        self.assertEqual(output.getvalue().count("waiting after"), 3)

    def test_listener_preserves_optional_feedback_without_changing_snapshot(self):
        joints = SimpleNamespace(position=[0.1] * 7, velocity=[0.2] * 7, torque=[1.3] * 7)
        state = SimpleNamespace(joints=joints, ParseFromString=lambda payload: None)
        listener = JointStateListener(lambda: state, 7)
        listener.on_sample(SimpleNamespace(payload=b"sample"))
        self.assertEqual(listener.snapshot(), ([0.1] * 7, 1))
        self.assertEqual(listener.diagnostics()["velocity"], [0.2] * 7)
        self.assertEqual(listener.diagnostics()["torque"], [1.3] * 7)
        joints.velocity = []
        joints.torque = []
        listener.on_sample(SimpleNamespace(payload=b"next"))
        self.assertEqual(listener.diagnostics()["velocity"], [])
        self.assertEqual(listener.diagnostics()["torque"], [])


if __name__ == "__main__":
    unittest.main()
