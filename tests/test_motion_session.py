"""Verify coordinated holding, zero cleanup and failure ordering offline."""

import contextlib
import io
import json
import math
import runpy
from pathlib import Path
import sys
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "examples" / "motion"))

import arm_motion_driver as arm
import hand_motion_driver as hand
import torso_motion_driver as torso
from kinematic_motion import ChannelSpec, MotionRunner, PROFILE_NAMES
from motion_session import MotionSession


class Command:
    def __init__(self):
        self.header = SimpleNamespace(stamp=SimpleNamespace(seconds=0, nanos=0))
        self.joints = SimpleNamespace(position=[], velocity=[], torque=[])

    def SerializeToString(self):
        return json.dumps(self.joints.position).encode()


class Listener:
    def __init__(self, positions):
        self.positions = list(positions)
        self.count = 1
        self.fresh = True

    def snapshot(self):
        return list(self.positions), self.count

    def diagnostics(self):
        return {"age": 0.0, "velocity": [], "torque": []}

    def is_fresh(self, timeout):
        return self.fresh

    def wait_for_newer_than(self, count, timeout):
        self.count += 1
        return True


class MotionSessionTest(unittest.TestCase):
    def setUp(self):
        with patch.object(sys, "argv", ["motion", "--tolerance", "2"]):
            self.args = arm.parse_args()
        self.proto = SimpleNamespace(
            ArmState=object, TorsoState=object, HandState=object,
            ArmCommand=Command, TorsoCommand=Command, HandCommand=Command,
            EnableRequest=lambda: SimpleNamespace(SerializeToString=lambda: b"enabled"),
            SPACE_MODE_JOINT_SPACE=1, SIDE_LEFT=1, SIDE_RIGHT=2,
        )
        self.session = Mock()
        self.trace = []
        self.now = 0.0
        contexts = contextlib.ExitStack()
        self.addCleanup(contexts.close)
        contexts.enter_context(contextlib.redirect_stdout(io.StringIO()))
        contexts.enter_context(contextlib.redirect_stderr(io.StringIO()))
        contexts.enter_context(patch("kinematic_motion.time.monotonic", lambda: self.now))
        contexts.enter_context(patch("kinematic_motion.time.sleep", self.sleep))
        self.enable = contexts.enter_context(patch.object(arm, "enable_with_recovery"))
        self.enable_service = contexts.enter_context(patch.object(arm, "call_enable_service"))

    def sleep(self, duration):
        self.now += max(0.000001, duration)

    def make_runner(self, module, part, sides=(None,), runner_type=MotionRunner, **kwargs):
        return runner_type(
            session=self.session, proto=self.proto,
            channels=tuple(ChannelSpec(
                label=f"{side}_{part}" if side else part,
                action_topic=f"action/{side}_{part}" if side else f"action/{part}",
                state_topic=f"state/{side}_{part}" if side else f"state/{part}",
                command_type=f"{part.title()}Command", state_type=f"{part.title()}State",
                side=side or "",
            ) for side in sides),
            joint_names=module.JOINT_NAMES, limits_deg=module.JOINT_LIMITS_DEG,
            amplitudes_deg=module.JOINT_AMPLITUDES_DEG,
            preferred_directions=module.PREFERRED_DIRECTIONS,
            args=self.args, frame_id="test", **kwargs,
        )

    def initialize(self, runners):
        motion = MotionSession(self.session, self.proto, self.args, runners)
        self.addCleanup(motion.close)
        self.listeners = {}
        for runner in motion.runners:
            for channel in runner._channels:
                values = [math.radians(3)] * len(runner._joint_names)
                if channel.label == "torso":
                    values[1] *= -1.0
                listener = Listener(values)
                runner._listeners[channel.label] = listener
                self.listeners[channel.label] = listener

        def publish(topic, payload):
            if topic.startswith("action/"):
                label = topic.removeprefix("action/")
                values = json.loads(payload)
                self.trace.append((label, list(values)))
                self.listeners[label].positions = list(values)
                self.listeners[label].count += 1

        self.session.put.side_effect = publish
        motion.start()
        return motion

    def test_every_driver_covers_all_five_groups_and_enables_both_arms(self):
        for module, part, sides in (
            (arm, "arm", ("left",)), (hand, "hand", ("right",)),
            (torso, "torso", (None,)),
        ):
            with self.subTest(part=part):
                moving = self.make_runner(module, part, sides)
                self.initialize((moving,))
                self.assertEqual(set(self.listeners),
                                 {"torso", "left_arm", "right_arm", "left_hand", "right_hand"})
                self.enable.assert_any_call(self.session, self.proto, "left", self.args.state_timeout)
                self.enable.assert_any_call(self.session, self.proto, "right", self.args.state_timeout)
                self.enable_service.assert_any_call(
                    self.session, self.proto, "service/torso/enable", True, self.args.state_timeout
                )

    def test_each_active_tick_holds_all_other_groups_at_fixed_targets(self):
        moving = self.make_runner(arm, "arm", ("left",), arm.ArmMotionRunner)
        motion = self.initialize((moving,))
        fixed = {label: list(listener.positions) for label, listener in self.listeners.items()}
        # A disturbance must not be recaptured as the new hold target.
        self.listeners["torso"].positions[3] += math.radians(4)
        self.trace.clear()
        target = list(fixed["left_arm"])
        target[3] += math.radians(10)
        moving._publish({"left_arm": target})
        self.assertEqual(len(self.trace), 5)
        for label, values in self.trace:
            self.assertEqual(values, target if label == "left_arm" else fixed[label])
        self.assertTrue(motion.ready)

    def test_nonzero_start_returns_every_group_to_zero_with_shoulders_last(self):
        moving = self.make_runner(arm, "arm", ("left", "right"), arm.ArmMotionRunner)
        motion = self.initialize((moving,))
        moving.prepare()
        self.trace.clear()
        self.assertTrue(motion.return_to_zero())
        final = dict(self.trace)
        self.assertEqual(set(final), set(self.listeners))
        self.assertTrue(all(all(value == 0.0 for value in values) for values in final.values()))
        torso_zero = next(index for index, (label, values) in enumerate(self.trace)
                          if label == "torso" and all(value == 0.0 for value in values))
        for index, (label, values) in enumerate(self.trace):
            if label.endswith("_arm") and values[1] != math.radians(-75):
                self.assertGreater(index, torso_zero)
                self.assertTrue(all(value == 0.0 for axis, value in enumerate(values) if axis != 1))
        # Cleanup never requests disable or reset on any group.
        services = [call.args for call in self.session.put.call_args_list
                    if call.args[0].startswith("service/")]
        self.assertEqual(services, [])
        self.assertEqual(self.enable_service.call_count, 1)
        self.assertTrue(all(call.args[3] is True for call in self.enable_service.call_args_list))

    def test_torso_keeps_clearance_arms_and_hands_commanded(self):
        arms = self.make_runner(arm, "arm", ("left", "right"), torso.ArmClearanceRunner)
        moving = self.make_runner(torso, "torso", runner_type=torso.TorsoMotionRunner, arms=arms)
        motion = self.initialize((moving, arms))
        arms.spread()
        self.trace.clear()
        moving._publish({"torso": list(moving._baselines["torso"])})
        targets = dict(self.trace)
        self.assertEqual(targets["left_arm"], [math.radians(x) for x in torso.ARM_CLEARANCE_POSE_DEG])
        self.assertEqual(len(self.trace), 5)
        self.assertTrue(motion.return_to_zero())

    def test_stale_companion_blocks_active_commands_before_any_publish(self):
        moving = self.make_runner(hand, "hand", ("left",))
        self.initialize((moving,))
        self.listeners["right_arm"].fresh = False
        self.trace.clear()
        with self.assertRaisesRegex(RuntimeError, "right_arm state became stale"):
            moving._publish(moving._last_targets)
        self.assertEqual(self.trace, [])

    def test_direct_script_class_also_returns_active_arms_to_zero(self):
        namespace = runpy.run_path(str(Path(arm.__file__)), run_name="standalone_arm_driver")
        driver_class = namespace["ArmMotionRunner"]
        self.assertIsNot(driver_class, arm.ArmMotionRunner)
        moving = self.make_runner(arm, "arm", ("left", "right"), driver_class)
        motion = self.initialize((moving,))
        moving.prepare()
        self.assertTrue(motion.return_to_zero())
        self.assertEqual(moving._last_targets, {"left_arm": [0.0] * 7, "right_arm": [0.0] * 7})

    def test_failed_torso_home_keeps_shoulders_spread_but_reports_failure(self):
        moving = self.make_runner(arm, "arm", ("left", "right"), arm.ArmMotionRunner)
        motion = self.initialize((moving,))
        moving.prepare()
        body = next(runner for runner in motion.runners if runner._channels[0].label == "torso")
        with patch.object(body, "return_to_zero", return_value=False):
            self.assertFalse(motion.return_to_zero())
        self.assertEqual(moving._last_targets["left_arm"][1], math.radians(-75))

    def test_rejected_torso_enable_prevents_exercise_and_position_commands(self):
        moving = self.make_runner(hand, "hand", ("left",))
        self.enable_service.side_effect = RuntimeError("torso enable rejected")
        with self.assertRaisesRegex(RuntimeError, "torso enable rejected"):
            self.initialize((moving,))
        self.assertEqual(self.trace, [])

    def test_hand_exercise_spreads_only_shoulders_and_holds_clearance(self):
        arms = self.make_runner(arm, "arm", ("left", "right"), torso.ArmClearanceRunner)
        moving = self.make_runner(hand, "hand", ("left",), hand.HandMotionRunner, arms=arms)
        motion = self.initialize((moving, arms))
        # A shoulder already farther out must stay there during preparation.
        arms._last_targets["right_arm"][1] = math.radians(-40)
        self.listeners["right_arm"].positions[1] = math.radians(-40)
        starts = {label: list(values) for label, values in arms._last_targets.items()}
        self.trace.clear()
        arms.spread(shoulder_deg=hand.HAND_SHOULDER_CLEARANCE_DEG, motion_name="hand")
        for label, values in self.trace:
            if label.endswith("_arm"):
                self.assertEqual([value for axis, value in enumerate(values) if axis != 1],
                                 [value for axis, value in enumerate(starts[label]) if axis != 1])
                if label == "right_arm":
                    self.assertEqual(values[1], math.radians(-40))
        self.assertEqual(arms._last_targets["left_arm"][1], math.radians(-20))
        clearance = {label: list(values) for label, values in arms._last_targets.items()}
        self.trace.clear()
        moving.run((5,), ("minjerk",))
        self.assertTrue(any(label == "left_hand" and values[5] > math.radians(3)
                            for label, values in self.trace))
        for label, values in self.trace:
            if label.endswith("_arm"):
                self.assertEqual(values, clearance[label])
        # Simulate Ctrl+C during a fist: fingers must open before shoulder closure.
        fist = [math.radians(3)] * len(hand.JOINT_NAMES)
        fist[5] = math.radians(60)
        moving._publish({"left_hand": fist})
        self.trace.clear()
        self.assertTrue(motion.return_to_zero())
        open_hand = next(index for index, (label, values) in enumerate(self.trace)
                         if label == "left_hand" and all(value == 0.0 for value in values))
        for index, (label, values) in enumerate(self.trace):
            if label.endswith("_arm") and values[1] != clearance[label][1]:
                self.assertGreater(index, open_hand)

    def test_all_hand_profiles_hold_side_swing_at_zero_even_with_large_scale(self):
        self.args.amplitude_scale = 10.0
        self.args.speed = 2.0
        arms = self.make_runner(arm, "arm", ("left", "right"), torso.ArmClearanceRunner)
        moving = self.make_runner(
            hand, "hand", ("left", "right"), hand.HandMotionRunner,
            arms=arms, unidirectional_continuous_profiles=True,
        )
        self.initialize((moving, arms))
        arms.spread(shoulder_deg=hand.HAND_SHOULDER_CLEARANCE_DEG, motion_name="hand")
        # Initial AA feedback is nonzero: verify the entry transition zeros it,
        # then every sweep and coordinated trajectory keeps the zero targets.
        traces = {}
        drive_segment = moving._drive_segment

        def drive(profile, axes, *args):
            self.assertTrue(set(axes).isdisjoint(hand.SIDE_SWING_AXES))
            if profile != "sweep":
                self.assertNotIn(hand.PINKY_CMC_AXIS, axes)
            self.trace.clear()
            drive_segment(profile, axes, *args)
            traces.setdefault(profile, []).extend(self.trace)

        with patch.object(moving, "_drive_segment", side_effect=drive):
            moving.run(tuple(range(22)), PROFILE_NAMES, sweep_groups=hand.FINGER_JOINT_GROUPS)
        self.assertEqual(set(traces), set(PROFILE_NAMES))
        self.assertEqual(hand.SIDE_SWING_AXES, (1, 3, 6, 10, 14, 19))
        for profile, trace in traces.items():
            with self.subTest(profile=profile):
                hands = [values for label, values in trace if label.endswith("_hand")]
                self.assertTrue(hands)
                for values in hands:
                    self.assertEqual([values[axis] for axis in hand.SIDE_SWING_AXES], [0.0] * 6)
                    if profile != "sweep":
                        self.assertEqual(values[hand.PINKY_CMC_AXIS], 0.0)
                self.assertTrue(any(values[5] > math.radians(3) for values in hands))
        self.assertTrue(any(
            values[hand.PINKY_CMC_AXIS] > math.radians(3)
            for label, values in traces["sweep"] if label.endswith("_hand")
        ))

    def test_coordinated_hand_motion_zeros_pinky_cmc_without_initial_sweep(self):
        arms = self.make_runner(arm, "arm", ("left", "right"), torso.ArmClearanceRunner)
        moving = self.make_runner(hand, "hand", ("left",), hand.HandMotionRunner, arms=arms)
        self.initialize((moving, arms))
        arms.spread(shoulder_deg=hand.HAND_SHOULDER_CLEARANCE_DEG, motion_name="hand")
        drive_segment = moving._drive_segment

        def drive(profile, axes, *args):
            self.assertNotIn(hand.PINKY_CMC_AXIS, axes)
            self.trace.clear()
            drive_segment(profile, axes, *args)
            hands = [values for label, values in self.trace if label == "left_hand"]
            self.assertTrue(hands)
            self.assertTrue(all(values[hand.PINKY_CMC_AXIS] == 0.0 for values in hands))

        with patch.object(moving, "_drive_segment", side_effect=drive):
            moving.run((5, hand.PINKY_CMC_AXIS), ("minjerk",))

    def test_selecting_only_hand_side_swing_joints_fails_before_connecting(self):
        self.args.axes = "2,4,7,11,15,20"
        with patch.object(hand, "load_dependencies") as load:
            with self.assertRaisesRegex(ValueError, "side-swing motion is disabled"):
                hand.run(self.args)
            load.assert_not_called()

    def test_arm_clearance_loss_stops_hand_targets(self):
        arms = self.make_runner(arm, "arm", ("left", "right"), torso.ArmClearanceRunner)
        moving = self.make_runner(hand, "hand", ("left",), hand.HandMotionRunner, arms=arms)
        self.initialize((moving, arms))
        arms.spread(shoulder_deg=hand.HAND_SHOULDER_CLEARANCE_DEG, motion_name="hand")
        self.listeners["left_arm"].positions[1] = math.radians(-10)
        self.trace.clear()
        with self.assertRaisesRegex(RuntimeError, "left_arm left clearance pose"):
            moving._publish(moving._last_targets)
        self.assertEqual(self.trace, [])

    def test_hand_preparation_failure_skips_exercise_and_still_cleans_up(self):
        with patch.object(hand, "HandMotionRunner", return_value=Mock()) as factory, \
                patch.object(hand, "ArmClearanceRunner", return_value=Mock()) as arms_factory, \
                patch.object(hand, "MotionSession", return_value=Mock()) as motion_factory, \
                patch.object(hand, "load_dependencies", return_value=(None, self.proto)), \
                patch.object(hand, "open_session", return_value=self.session):
            arms_factory.return_value.spread.side_effect = RuntimeError("clearance failed")
            with self.assertRaisesRegex(RuntimeError, "clearance failed"):
                hand.run(self.args)
            factory.return_value.run.assert_not_called()
            motion_factory.return_value.return_to_zero.assert_called_once_with()
            motion_factory.return_value.close.assert_called_once_with()

    def test_failed_finger_zero_keeps_shoulders_out(self):
        arms = self.make_runner(arm, "arm", ("left", "right"), torso.ArmClearanceRunner)
        moving = self.make_runner(hand, "hand", ("left",), hand.HandMotionRunner, arms=arms)
        motion = self.initialize((moving, arms))
        arms.spread(shoulder_deg=hand.HAND_SHOULDER_CLEARANCE_DEG, motion_name="hand")
        with patch.object(moving, "return_to_zero", return_value=False):
            self.assertFalse(motion.return_to_zero())
        self.assertEqual(arms._last_targets["left_arm"][1], math.radians(-20))

    def test_small_hand_zero_offset_does_not_leave_shoulders_spread(self):
        arms = self.make_runner(arm, "arm", ("left", "right"), torso.ArmClearanceRunner)
        moving = self.make_runner(hand, "hand", ("left",), hand.HandMotionRunner, arms=arms)
        self.args.return_tolerance_deg = 0.5
        motion = self.initialize((moving, arms))
        arms.spread(shoulder_deg=hand.HAND_SHOULDER_CLEARANCE_DEG, motion_name="hand")
        for label, listener in self.listeners.items():
            if label.endswith("_hand"):
                original = listener.snapshot
                def snapshot(original=original):
                    values, count = original()
                    values[5] += math.radians(0.51)
                    return values, count
                listener.snapshot = snapshot
        self.assertTrue(motion.return_to_zero())
        self.assertEqual(arms._last_targets["left_arm"], [0.0] * 7)
        self.assertEqual(arms._last_targets["right_arm"], [0.0] * 7)
        self.assertTrue(all(all(value == 0.0 for values in runner._last_targets.values() for value in values)
                            for runner in motion.runners))
        self.assertEqual(self.args.return_tolerance_deg, 0.5)

    def test_drivers_home_on_normal_completion_ctrl_c_and_exercise_failure(self):
        for module, runner_name in (
            (arm, "ArmMotionRunner"), (torso, "TorsoMotionRunner"), (hand, "HandMotionRunner"),
        ):
            for failure in (None, KeyboardInterrupt(), RuntimeError("exercise failed")):
                with self.subTest(module=module.__name__, failure=type(failure).__name__):
                    runner, motion = Mock(), Mock()
                    runner.run.side_effect = failure
                    motion.return_to_zero.return_value = True
                    with patch.object(module, runner_name, return_value=runner), \
                            patch.object(torso, "ArmClearanceRunner", return_value=Mock()), \
                            patch.object(hand, "ArmClearanceRunner", return_value=Mock()), \
                            patch.object(module, "MotionSession", return_value=motion), \
                            patch.object(module, "load_dependencies", return_value=(None, self.proto)), \
                            patch.object(module, "open_session", return_value=self.session):
                        if isinstance(failure, RuntimeError):
                            with self.assertRaisesRegex(RuntimeError, "exercise failed"):
                                module.run(self.args)
                        else:
                            self.assertEqual(module.run(self.args), 130 if failure else 0)
                    motion.return_to_zero.assert_called_once_with()
                    motion.close.assert_called_once_with()
                    runner.hold_last_targets.assert_not_called()


if __name__ == "__main__":
    unittest.main()
