# Copyright 2026 Sharpa
# SPDX-License-Identifier: Apache-2.0
"""Exercise task ownership and read-only behavior without a robot or SDK build.

Run: python3 -m unittest discover -s examples/sdk/tests -v
"""

import ast
import contextlib
from enum import Enum
import importlib.util
import io
import json
import math
from pathlib import Path
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import MagicMock, patch


SPEC = importlib.util.spec_from_file_location(
    "sdk_motion", Path(__file__).resolve().parents[1] / "whole_body_motion.py"
)
MOTION = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MOTION)


class Phase(Enum):
    STANDBY = 1
    RUNNING = 2


class Mode(Enum):
    AUTOMATION = 1
    INFERENCE = 2


class SDKError(Exception):
    pass


class MotionLifecycleTests(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.addCleanup(self.temporary.cleanup)
        self.output = Path(self.temporary.name) / "report.json"
        self.state = SimpleNamespace(
            phase=Phase.STANDBY, mode=Mode.AUTOMATION, task_id=""
        )
        self.master = MagicMock()
        self.master.latest_status.side_effect = lambda age: self.state
        self.master.set_mode.side_effect = self.set_mode
        self.master.stop_task.side_effect = self.stop_task
        self.part = MagicMock()
        self.part.latest_state.return_value = SimpleNamespace(
            joints=SimpleNamespace(position=[0.0] * 7)
        )
        self.sdk = MagicMock()
        self.sdk.__enter__.return_value = self.sdk
        self.sdk.master.return_value = self.master
        self.sdk.arm.return_value = self.part
        self.module = SimpleNamespace(
            CommandChannel=SimpleNamespace(INFERENCE=1),
            ErrorCode=SimpleNamespace(STALE_STATE=1),
            MasterPhase=Phase,
            MasterResult=SimpleNamespace(OK=1),
            NorthSDK=MagicMock(return_value=self.sdk),
            NorthSDKError=SDKError,
            OperationMode=Mode,
            SdkConfig=lambda: SimpleNamespace(time_sync=SimpleNamespace(enabled=True)),
            Side=SimpleNamespace(LEFT=0, RIGHT=1),
        )

    def set_mode(self, mode):
        self.state.mode = mode
        return SimpleNamespace(result=1)

    def stop_task(self):
        self.state.phase = Phase.STANDBY
        self.state.task_id = ""
        return SimpleNamespace(result=1)

    def run_example(self, execute):
        options = ["--arms-only", "--side", "right", "--output", str(self.output)]
        if not execute:
            options.append("--preview")
        args = MOTION.arguments(options)
        with patch.dict("sys.modules", {"north_sdk": self.module}):
            with contextlib.redirect_stdout(io.StringIO()):
                with contextlib.redirect_stderr(io.StringIO()):
                    return MOTION.main(args)

    def test_unknown_start_outcome_stops_matching_task_once(self):
        # The server starts the task, but the RPC response is lost.
        def start(task):
            self.state.phase = Phase.RUNNING
            self.state.task_id = task
            raise SDKError("OUTCOME_UNKNOWN")

        self.master.start_task.side_effect = start
        with self.assertRaisesRegex(SDKError, "OUTCOME_UNKNOWN"):
            self.run_example(execute=True)
        self.master.start_task.assert_called_once_with("inference")
        self.master.stop_task.assert_called_once_with()
        report = json.loads(self.output.read_text())
        self.assertFalse(report["passed"])
        self.assertEqual(report["sent"], 0)
        self.assertEqual(report["final_phase"], "STANDBY")

    def test_unknown_start_outcome_never_stops_unrelated_task(self):
        def start(task):
            self.state.phase = Phase.RUNNING
            self.state.task_id = "another_task"
            raise SDKError("OUTCOME_UNKNOWN")

        self.master.start_task.side_effect = start
        with self.assertRaisesRegex(RuntimeError, "Another task"):
            self.run_example(execute=True)
        self.master.start_task.assert_called_once_with("inference")
        self.master.stop_task.assert_not_called()
        report = json.loads(self.output.read_text())
        self.assertFalse(report["passed"])
        self.assertIn("Another task", report["cleanup_error"])

    def test_preview_does_not_mutate_robot_state(self):
        result = self.run_example(execute=False)
        self.assertTrue(result["passed"])
        self.assertEqual(result["sent"], 0)
        self.master.set_mode.assert_not_called()
        self.master.start_task.assert_not_called()
        self.master.stop_task.assert_not_called()
        self.sdk.whole_body.assert_not_called()
        self.part.enable.assert_not_called()
        self.part.set_mode.assert_not_called()
        self.part.send_joint_command.assert_not_called()
        self.master.on_status.return_value.close.assert_called_once_with()
        self.part.on_state.return_value.close.assert_called_once_with()


    def simulate_motion(self, *, interrupts=(), failure=None, hold=0.1, feedback=True, return_offset=0.0, send_hook=None):
        now = 0.0
        stage = ""
        pending = list(interrupts)
        commands = []
        self.positions = [0.0] * 7
        self.part.latest_state.side_effect = lambda age: SimpleNamespace(
            joints=SimpleNamespace(position=self.positions.copy()))

        def start(task):
            self.state.phase, self.state.task_id = Phase.RUNNING, task
            return SimpleNamespace(result=1)

        def send(channel, targets):
            commands.append(targets["right_arm"].copy())
            if feedback:
                self.positions = [q + (return_offset if stage == "Return to captured starting pose" else 0)
                                  for q in commands[-1]]
            if send_hook:
                send_hook(len(commands))

        def sleep(seconds):
            nonlocal now
            now += max(seconds, 0.001)
            if pending and pending[0] == stage and (stage != "Raise arms" or now >= 0.5):
                pending.pop(0)
                raise KeyboardInterrupt()
            if failure and stage == failure:
                raise RuntimeError("stale feedback")

        def output(*values, **kwargs):
            nonlocal stage
            message = str(values[0])
            for name in ("Raise arms", "Hold thumbs up", "Return to captured starting pose"):
                if message.startswith(name):
                    stage = name

        self.master.start_task.side_effect = start
        self.sdk.whole_body.return_value.send.side_effect = send
        args = MOTION.arguments(["--arms-only", "--side", "right",
                                 "--hold-duration", str(hold), "--output", str(self.output)])
        with patch.dict("sys.modules", {"north_sdk": self.module}), \
             patch.object(MOTION, "make_command", side_effect=lambda targets: targets), \
             patch.object(MOTION.time, "monotonic", side_effect=lambda: now), \
             patch.object(MOTION.time, "sleep", side_effect=sleep), \
             patch("builtins.print", side_effect=output):
            try:
                report = MOTION.main(args)
                error = None
            except BaseException as exc:
                report = json.loads(self.output.read_text())
                error = exc
        return report, commands, error

    def test_timed_motion_returns_before_stopping_owned_task(self):
        report, commands, error = self.simulate_motion()
        self.assertIsNone(error)
        self.assertTrue(report["passed"])
        self.assertTrue(report["sequence_completed"])
        self.assertTrue(report["motion_verified"])
        self.assertEqual(report["outcome"], "completed")
        self.assertEqual(commands[-1], [0.0] * 7)
        self.assertEqual(max(commands, key=lambda q: q[0]), [math.radians(x) for x in MOTION.ARM_DEG])
        self.master.stop_task.assert_called_once_with()

    def test_default_infinite_hold_returns_on_first_interrupt(self):
        report, commands, error = self.simulate_motion(interrupts=("Hold thumbs up",), hold=0)
        self.assertIsNone(error)
        self.assertTrue(report["passed"])
        self.assertTrue(report["interrupted"])
        self.assertTrue(report["return_verified"])
        self.assertFalse(report["sequence_completed"])
        self.assertEqual(report["outcome"], "interrupted_returned")
        self.assertEqual(commands[-1], [0.0] * 7)
        self.master.stop_task.assert_called_once_with()

    def test_interrupt_during_raise_returns_from_partial_trajectory(self):
        report, commands, error = self.simulate_motion(interrupts=("Raise arms",))
        self.assertIsNone(error)
        self.assertTrue(report["passed"])
        self.assertTrue(report["return_verified"])
        self.assertFalse(report["motion_verified"])
        self.assertGreater(max(q[0] for q in commands), 0.001)
        self.assertEqual(commands[-1], [0.0] * 7)
        for before, after in zip(commands, commands[1:]):
            self.assertLessEqual(max(abs(a-b) for a,b in zip(before, after)), MOTION.MAX_STEP)

    def test_second_interrupt_aborts_return_and_cleans_up(self):
        report, _, error = self.simulate_motion(
            interrupts=("Hold thumbs up", "Return to captured starting pose"), hold=0)
        self.assertIsInstance(error, KeyboardInterrupt)
        self.assertTrue(report["return_interrupted"])
        self.assertFalse(report["return_verified"])
        self.assertFalse(report["passed"])
        self.master.stop_task.assert_called_once_with()

    def test_runtime_failure_does_not_send_return_commands(self):
        report, commands, error = self.simulate_motion(failure="Hold thumbs up")
        self.assertIsInstance(error, RuntimeError)
        self.assertFalse(report["return_verified"])
        self.assertFalse(report["passed"])
        self.assertEqual(report["stage"], "Hold thumbs up")
        self.assertEqual(commands[-1], [math.radians(x) for x in MOTION.ARM_DEG])
        self.master.stop_task.assert_called_once_with()

    def test_task_changed_during_motion_is_never_stopped(self):
        def takeover(count):
            if count == 3:
                self.state.task_id = "another_task"
        report, commands, error = self.simulate_motion(send_hook=takeover)
        self.assertIsInstance(error, RuntimeError)
        self.assertEqual(len(commands), 3)
        self.assertIn("Another task", report["cleanup_error"])
        self.assertFalse(report["passed"])
        self.master.stop_task.assert_not_called()

    def test_stale_feedback_stops_commands_and_owned_task(self):
        def stale(count):
            if count == 3:
                self.part.latest_state.side_effect = SDKError("stale feedback")
        report, commands, error = self.simulate_motion(send_hook=stale)
        self.assertIsInstance(error, SDKError)
        self.assertEqual(len(commands), 3)
        self.assertIn("stale feedback", report["error"])
        self.assertFalse(report["return_verified"])
        self.master.stop_task.assert_called_once_with()

    def test_feedback_outside_tolerance_times_out(self):
        report, _, error = self.simulate_motion(return_offset=0.1)
        self.assertIsInstance(error, RuntimeError)
        self.assertIn("Timed out waiting for joints to return", str(error))
        self.assertFalse(report["return_verified"])
        self.assertFalse(report["passed"])
        self.assertTrue(all(q > math.radians(1) for q in report["return_error_rad"].values()))
        self.master.stop_task.assert_called_once_with()


class TrajectoryAndArgumentsTests(unittest.TestCase):
    def test_defaults_and_legacy_aliases(self):
        args = MOTION.arguments([])
        self.assertTrue(args.execute)
        self.assertFalse(MOTION.arguments(["--preview"]).execute)
        self.assertTrue(MOTION.arguments(["--execute"]).execute)
        self.assertEqual((args.rate, args.transition_duration, args.hold_duration), (50, 4, 0))
        self.assertEqual((args.return_timeout, args.return_tolerance_deg), (8, 1))
        aliases = MOTION.arguments(["--hz", "60", "--move-duration", "5"])
        canonical = MOTION.arguments(["--rate", "60", "--transition-duration", "5"])
        self.assertEqual(vars(aliases), vars(canonical))

    def test_invalid_parameters_are_rejected(self):
        for option, value in (("--rate", "0"), ("--rate", "nan"),
                              ("--transition-duration", "-1"), ("--hold-duration", "-1"),
                              ("--hold-duration", "inf"), ("--return-timeout", "0"),
                              ("--tolerance", "0")):
            with self.subTest(option=option, value=value), contextlib.redirect_stderr(io.StringIO()):
                with self.assertRaises(SystemExit):
                    MOTION.arguments([option, value])

    def test_fixed_poses_match_ordinary_python_example(self):
        path = Path(__file__).resolve().parents[2] / "whole_body_motion.py"
        tree = ast.parse(path.read_text())
        poses = {node.targets[0].id: ast.literal_eval(node.value)
                 for node in tree.body if isinstance(node, ast.Assign) and
                 isinstance(node.targets[0], ast.Name) and
                 node.targets[0].id in ("RAISED_ARM_POSE_DEG", "SQUAT_POSE_DEG", "THUMBS_UP_POSE_DEG")}
        self.assertEqual(MOTION.ARM_DEG, poses["RAISED_ARM_POSE_DEG"])
        self.assertEqual(MOTION.TORSO_DEG, poses["SQUAT_POSE_DEG"])
        self.assertEqual(MOTION.HAND_DEG, poses["THUMBS_UP_POSE_DEG"])

    def test_minimum_jerk_endpoints_and_safe_per_frame_steps(self):
        self.assertEqual(MOTION.minimum_jerk(-1), 0)
        self.assertEqual(MOTION.minimum_jerk(2), 1)
        for distance in (math.radians(75), math.pi * 2):
            duration = MOTION.transition_duration({"arm": [0]}, {"arm": [distance]}, 4, 50)
            samples = [distance * MOTION.minimum_jerk(t / 50 / duration)
                       for t in range(math.ceil(duration * 50) + 1)]
            self.assertTrue(all(-1e-12 <= b-a <= MOTION.MAX_STEP for a,b in zip(samples, samples[1:])))


if __name__ == "__main__":
    unittest.main()
