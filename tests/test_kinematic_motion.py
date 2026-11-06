"""Verify complete trajectory playback and amplitude scaling without Platform."""

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
import hand_motion_driver as hand
import torso_motion_driver as torso
from kinematic_motion import (
    CHIRP_OSCILLATION_FRACTION, ChannelSpec, MotionRunner, PROFILE_NAMES,
    motion_offset, profile_duration, validate_args,
)


class Clock:
    def __init__(self):
        self.now = 0.0

    def monotonic(self):
        return self.now

    def sleep(self, duration):
        self.now += max(duration, 0.000001)


class MotionPlaybackTest(unittest.TestCase):
    def args(self, module=torso, *options):
        with patch.object(sys, "argv", [module.__name__, *options]):
            return module.parse_args()

    def instrument(self, runner, baselines):
        runner._baselines = {label: list(values) for label, values in baselines.items()}
        runner._last_targets = {label: list(values) for label, values in baselines.items()}
        clock = Clock()
        trace = []

        def publish(targets):
            saved = {label: list(values) for label, values in targets.items()}
            trace.append((clock.now, saved))
            runner._last_targets = saved
            runner._has_published = True

        runner._publish = publish
        for label in runner._listeners:
            listener = Mock()
            listener.is_fresh.return_value = True
            listener.snapshot.side_effect = lambda label=label: (
                list(runner._last_targets[label]), len(trace)
            )
            runner._listeners[label] = listener
        return clock, trace

    @contextlib.contextmanager
    def clock_context(self, clock):
        with patch("kinematic_motion.time.monotonic", clock.monotonic), \
                patch("kinematic_motion.time.sleep", clock.sleep), \
                contextlib.redirect_stdout(io.StringIO()):
            yield

    def runner(self, speed=1.0, scale=1.0, **kwargs):
        args = self.args(torso, "--speed", str(speed), "--scale", str(scale))
        # Equal trajectory-clock sampling lets us compare every commanded pose
        # across playback speeds, including sine and chirp cycle counts.
        args.rate = 80.0 * speed
        return MotionRunner(
            session=Mock(), proto=SimpleNamespace(TorsoState=object),
            channels=(ChannelSpec("test", "action/test", "state/test",
                                  "TorsoCommand", "TorsoState"),),
            joint_names=("test_joint",), limits_deg=((-90.0, 90.0),),
            amplitudes_deg=(15.0,), preferred_directions=(1.0,),
            args=args, frame_id="test", **kwargs,
        )

    def trace_profile(self, profile, speed, **kwargs):
        runner = self.runner(speed=speed, **kwargs)
        clock, trace = self.instrument(runner, {"test": [0.0]})
        with self.clock_context(clock):
            runner._drive_segment(profile, (0,), {"test": [0.0]}, {"test": [0.0]})
        return trace

    def test_cli_accepts_scale_above_one_and_replaces_duration(self):
        for module in (torso, arm, hand):
            with self.subTest(module=module.__name__):
                args = self.args(module, "--scale", "2", "--speed", "1.5", "--tolerance", "2")
                validate_args(args, len(module.JOINT_NAMES))
                self.assertEqual(args.amplitude_scale, 2.0)
                self.assertEqual(args.speed, 1.5)
                self.assertEqual(args.return_tolerance_deg, 2.0)
                self.assertFalse(hasattr(args, "segment_duration"))
                self.assertEqual(self.args(module).speed, 1.0)
                with contextlib.redirect_stderr(io.StringIO()):
                    with self.assertRaises(SystemExit):
                        self.args(module, "--duration", "4")

    def test_nonpositive_or_nonfinite_speed_and_scale_are_rejected(self):
        for attribute in ("speed", "amplitude_scale"):
            for value in (0.0, -1.0, math.inf, math.nan):
                with self.subTest(attribute=attribute, value=value):
                    args = self.args()
                    setattr(args, attribute, value)
                    with self.assertRaises(ValueError):
                        validate_args(args, len(torso.JOINT_NAMES))

    def test_segment_timing_depends_on_travel_and_profile(self):
        short = math.radians(5.0)
        long = math.radians(20.0)
        duration = lambda profile, travel: profile_duration(profile, travel, 0.25, 1.0)
        self.assertLess(duration("sweep", short), duration("sweep", long))
        self.assertLess(duration("trapezoid", long), duration("minjerk", long))
        self.assertGreater(duration("sine", long), duration("sweep", long))
        self.assertGreaterEqual(duration("sine", short) * 0.25, 2.0)

    def test_speed_preserves_every_profile_and_continuous_cycle_count(self):
        variants = ({}, {"smooth_chirp_return": True},
                    {"unidirectional_continuous_profiles": True})
        for options in variants:
            for profile in PROFILE_NAMES:
                normal = self.trace_profile(profile, 1.0, **options)
                for speed in (0.5, 2.0):
                    with self.subTest(profile=profile, speed=speed, options=options):
                        scaled = self.trace_profile(profile, speed, **options)
                        self.assertEqual(len(normal), len(scaled))
                        for (original_time, original), (scaled_time, target) in zip(normal, scaled):
                            self.assertAlmostEqual(scaled_time * speed, original_time)
                            self.assertAlmostEqual(target["test"][0], original["test"][0])
                        self.assertAlmostEqual(scaled[-1][1]["test"][0], 0.0)

    def test_chirp_return_preserves_join_velocity_and_acceleration(self):
        for duration, frequency, end_frequency in ((5.0, 0.25, 1.0),
                                                   (8.0, 1.0, 0.25),
                                                   (4.0, 2.0, 4.0)):
            for axis in range(8):
                with self.subTest(duration=duration, frequency=frequency, axis=axis):
                    phase = 2.0 * math.pi * axis / 8.0
                    join = duration * CHIRP_OSCILLATION_FRACTION
                    slope = (end_frequency - frequency) / join
                    angle = 2.0 * math.pi * (
                        frequency * join + 0.5 * slope * join**2
                    ) + phase
                    omega = 2.0 * math.pi * end_frequency
                    value = lambda elapsed: motion_offset(
                        "chirp", elapsed, duration, phase=phase,
                        frequency=frequency, chirp_end_frequency=end_frequency,
                        smooth_chirp_return=True,
                    )
                    h = duration * 1e-5
                    center = value(join)
                    self.assertAlmostEqual(center, math.sin(angle))
                    expected_velocity = omega * math.cos(angle)
                    expected_acceleration = (
                        -omega**2 * math.sin(angle)
                        + 2.0 * math.pi * slope * math.cos(angle)
                    )
                    for direction in (-1, 1):
                        near = value(join + direction * h)
                        far = value(join + direction * 2.0 * h)
                        farther = value(join + direction * 3.0 * h)
                        velocity = direction * (-3.0 * center + 4.0 * near - far) / (2.0 * h)
                        acceleration = (2.0 * center - 5.0 * near + 4.0 * far - farther) / h**2
                        self.assertAlmostEqual(velocity, expected_velocity, delta=1e-5)
                        self.assertAlmostEqual(acceleration, expected_acceleration, delta=0.02)
                    for index in range(501):
                        elapsed = duration * index / 500.0
                        self.assertLessEqual(abs(value(elapsed)), 1.0 + 1e-12)
                    self.assertAlmostEqual(value(duration), 0.0)
                    self.assertAlmostEqual(value(duration - h) / h, 0.0, delta=1e-5)
                    self.assertAlmostEqual(
                        (value(duration - 2.0 * h) - 2.0 * value(duration - h)) / h**2,
                        0.0, delta=0.02,
                    )

    def test_chirp_keeps_oscillating_at_full_amplitude_in_late_sweep(self):
        duration = 6.0
        join = duration * CHIRP_OSCILLATION_FRACTION
        frequency, end_frequency = 0.25, 1.0
        slope = (end_frequency - frequency) / join
        values = []
        for index in range(1001):
            elapsed = duration / 2.0 + (join - duration / 2.0) * index / 1000.0
            actual = motion_offset("chirp", elapsed, duration,
                                   frequency=frequency, chirp_end_frequency=end_frequency,
                                   smooth_chirp_return=True)
            expected = math.sin(2.0 * math.pi * (
                frequency * elapsed + 0.5 * slope * elapsed**2
            ))
            self.assertAlmostEqual(actual, expected)
            values.append(actual)
        crossings = sum(start * end < 0.0 for start, end in zip(values, values[1:]))
        self.assertGreaterEqual(crossings, 3)
        self.assertGreater(max(values), 0.99)
        self.assertLess(min(values), -0.99)

    def test_enlarged_scale_increases_travel_but_respects_joint_limits(self):
        for scale, expected in ((1.0, 15.0), (2.0, 30.0), (10.0, 89.0)):
            with self.subTest(scale=scale):
                runner = self.runner(scale=scale)
                clock, trace = self.instrument(runner, {"test": [0.0]})
                with self.clock_context(clock):
                    runner._drive_segment("sweep", (0,), {"test": [0.0]}, {"test": [0.0]})
                degrees = [math.degrees(targets["test"][0]) for _, targets in trace]
                self.assertAlmostEqual(max(degrees), expected, places=3)
                self.assertTrue(all(0.0 <= value <= 89.0 for value in degrees))

    def test_command_rate_limits_speed_without_reducing_amplitude(self):
        for profile in PROFILE_NAMES:
            with self.subTest(profile=profile):
                runner = self.runner(speed=10.0, scale=2.0)
                runner._args.rate = 10.0
                clock, trace = self.instrument(runner, {"test": [0.0]})
                plan = runner._plan_segment(profile, (0,))
                self.assertGreater(plan.limited_seconds, 0.0)
                with self.clock_context(clock):
                    runner._drive_segment(profile, (0,), {"test": [0.0]}, {"test": [0.0]})
                values = [targets["test"][0] for _, targets in trace]
                self.assertLessEqual(max(abs(b-a) for a,b in zip(values, values[1:])),
                                     math.radians(5.0) + 1e-12)
                self.assertAlmostEqual(values[-1], 0.0)
                if profile in ("sweep", "minjerk", "trapezoid"):
                    self.assertAlmostEqual(
                        max(math.degrees(target["test"][0]) for target in plan.targets),
                        30.0, places=2,
                    )
                    # A discrete command tick can straddle an extremum by up
                    # to one frame's travel; the planned amplitude is intact.
                    self.assertGreater(math.degrees(max(values)), 25.0)
                self.assertGreaterEqual(trace[-1][0] + 1e-9, plan.duration)

    def test_torso_profiles_keep_sweeps_faster_than_coordinated_moves(self):
        for speed in (0.5, 2.0):
            with self.subTest(speed=speed):
                args = self.args(torso, "--scale", "2.5", "--speed", str(speed))
                args.rate = 200.0
                runner = torso.TorsoMotionRunner(
                    arms=Mock(), session=Mock(), proto=SimpleNamespace(TorsoState=object),
                    channels=(ChannelSpec("torso", "action/torso", "state/torso",
                                          "TorsoCommand", "TorsoState"),),
                    joint_names=torso.JOINT_NAMES, limits_deg=torso.JOINT_LIMITS_DEG,
                    amplitudes_deg=torso.JOINT_AMPLITUDES_DEG,
                    preferred_directions=torso.PREFERRED_DIRECTIONS, args=args,
                    frame_id="test", smooth_chirp_return=True,
                )
                runner._baselines = {"torso": [0.0] * 8}
                for profile in PROFILE_NAMES:
                    with self.subTest(profile=profile):
                        with patch.object(runner, "_profile_speed", return_value=speed):
                            original = runner._plan_segment(profile, tuple(range(8)))
                        adjusted = runner._plan_segment(profile, tuple(range(8)))
                        if profile == "sweep":
                            # A sweep containing J1/J2 shares their slower timing.
                            factor = 1.0
                        else:
                            factor = 2.0
                        self.assertEqual(adjusted.targets, original.targets)
                        self.assertAlmostEqual(adjusted.duration, original.duration * factor)
                        self.assertEqual(args.speed, speed)

                for axis in range(len(torso.JOINT_NAMES)):
                    with self.subTest(sweep_axis=axis + 1):
                        with patch.object(runner, "_profile_speed", return_value=2.0 * speed):
                            original = runner._plan_segment("sweep", (axis,))
                        adjusted = runner._plan_segment("sweep", (axis,))
                        factor = 2.0 if axis in (0, 1) else 1.0
                        self.assertEqual(adjusted.targets, original.targets)
                        pause = torso.J1_SWEEP_PAUSE_SECONDS if axis == 0 else 0.0
                        self.assertAlmostEqual(
                            adjusted.duration,
                            (original.duration - pause) * factor + pause,
                        )
                        self.assertEqual(args.rate, 200.0)

    def test_torso_j1_sweep_holds_endpoint_for_one_second_before_return(self):
        args = self.args(torso, "--axes", "1", "--profiles", "sweep")
        args.rate = 100.0
        runner = torso.TorsoMotionRunner(
            arms=Mock(), session=Mock(), proto=SimpleNamespace(TorsoState=object),
            channels=(ChannelSpec("torso", "action/torso", "state/torso",
                                  "TorsoCommand", "TorsoState"),),
            joint_names=torso.JOINT_NAMES, limits_deg=torso.JOINT_LIMITS_DEG,
            amplitudes_deg=torso.JOINT_AMPLITUDES_DEG,
            preferred_directions=torso.PREFERRED_DIRECTIONS, args=args,
            frame_id="test", smooth_chirp_return=True,
        )
        runner._baselines = {"torso": [0.0] * len(torso.JOINT_NAMES)}
        unpaused = MotionRunner._plan_segment(runner, "sweep", (0,))
        paused = runner._plan_segment("sweep", (0,))
        midpoint = len(unpaused.times) // 2

        self.assertEqual(paused.targets[midpoint], paused.targets[midpoint + 1])
        self.assertAlmostEqual(
            paused.times[midpoint + 1] - paused.times[midpoint],
            torso.J1_SWEEP_PAUSE_SECONDS,
        )
        self.assertAlmostEqual(
            paused.duration - unpaused.duration,
            torso.J1_SWEEP_PAUSE_SECONDS,
        )
        self.assertEqual(
            paused.at(paused.times[midpoint] + 0.5), paused.targets[midpoint]
        )

    def test_torso_chirp_return_uses_requested_speed_without_global_slowdown(self):
        args = self.args(torso, "--scale", "1.5", "--speed", "2")
        runner = MotionRunner(
            session=Mock(), proto=SimpleNamespace(TorsoState=object),
            channels=(ChannelSpec("torso", "action/torso", "state/torso",
                                  "TorsoCommand", "TorsoState"),),
            joint_names=torso.JOINT_NAMES, limits_deg=torso.JOINT_LIMITS_DEG,
            amplitudes_deg=torso.JOINT_AMPLITUDES_DEG,
            preferred_directions=torso.PREFERRED_DIRECTIONS, args=args,
            frame_id="test", smooth_chirp_return=True,
        )
        clock, trace = self.instrument(runner, {"torso": [0.0] * 8})
        fast = runner._plan_segment("chirp", (0, 1, 2))
        self.assertAlmostEqual(
            fast.duration - fast.return_start_time,
            fast.nominal_duration * (1.0 - CHIRP_OSCILLATION_FRACTION) / args.speed,
        )
        # The old whole-segment worst-case estimate made this 17.3 seconds.
        self.assertLess(fast.duration, 8.0)
        args.speed = 1.0
        normal = runner._plan_segment("chirp", (0, 1, 2))
        self.assertLess(fast.duration, normal.duration)
        self.assertEqual(fast.targets, normal.targets)
        args.speed = 2.0
        with self.clock_context(clock):
            runner._drive_segment("chirp", (0, 1, 2),
                                  {"torso": [0.0]*8}, {"torso": [0.0]*8})
        self.assertLess(clock.now, 8.0)

    def test_torso_chirp_return_keeps_limits_and_bounded_command_steps(self):
        for speed, scale, rate in ((1.0, 1.0, 100.0), (10.0, 3.0, 5.0)):
            with self.subTest(speed=speed, scale=scale, rate=rate):
                args = self.args(torso, "--speed", str(speed), "--scale", str(scale),
                                 "--rate", str(rate))
                runner = torso.TorsoMotionRunner(
                    arms=Mock(), session=Mock(), proto=SimpleNamespace(TorsoState=object),
                    channels=(ChannelSpec("torso", "action/torso", "state/torso",
                                          "TorsoCommand", "TorsoState"),),
                    joint_names=torso.JOINT_NAMES, limits_deg=torso.JOINT_LIMITS_DEG,
                    amplitudes_deg=torso.JOINT_AMPLITUDES_DEG,
                    preferred_directions=torso.PREFERRED_DIRECTIONS, args=args,
                    frame_id="test", smooth_chirp_return=True,
                )
                baseline = [0.0] * 7 + [math.radians(0.4)]
                clock, trace = self.instrument(runner, {"torso": baseline})
                axes = tuple(range(8))
                plan = runner._plan_segment("chirp", axes)
                if speed == 1.0:
                    self.assertAlmostEqual(plan.duration, 12.0)
                    self.assertAlmostEqual(plan.return_start_time, 10.0)
                output = io.StringIO()
                with self.clock_context(clock), contextlib.redirect_stdout(output):
                    runner._drive_segment("chirp", axes,
                                          {"torso": list(baseline)}, {"torso": list(baseline)})
                self.assertEqual(output.getvalue().count("starting final smooth return"), 1)
                if speed == 1.0:
                    self.assertIn("continuous oscillation: 10.00s; final smooth return: 2.00s",
                                  output.getvalue())
                for targets in plan.targets + tuple(targets for _, targets in trace):
                    for value, (lower, upper) in zip(targets["torso"], runner._limits):
                        self.assertGreaterEqual(value, lower)
                        self.assertLessEqual(value, upper)
                for (_, previous), (_, current) in zip(trace, trace[1:]):
                    for start, end in zip(previous["torso"], current["torso"]):
                        self.assertLessEqual(abs(end - start), math.radians(5.0) + 1e-12)
                for actual, expected in zip(trace[-1][1]["torso"], baseline):
                    self.assertAlmostEqual(actual, expected)
                self.assertEqual(args.rate, rate)

    def test_scheduler_delay_does_not_skip_across_command_steps(self):
        runner = self.runner(speed=2.0, scale=2.0)
        runner._args.rate = 50.0
        clock, trace = self.instrument(runner, {"test": [0.0]})
        original_sleep = clock.sleep
        delayed = False

        def sleep(duration):
            nonlocal delayed
            if not delayed:
                delayed = True
                duration += 0.5
            original_sleep(duration)

        clock.sleep = sleep
        with self.clock_context(clock):
            runner._drive_segment("sine", (0,), {"test": [0.0]}, {"test": [0.0]})
        positions = [target["test"][0] for _, target in trace]
        self.assertLessEqual(max(abs(b-a) for a,b in zip(positions, positions[1:])),
                             math.radians(5.0) + 1e-12)
        self.assertAlmostEqual(positions[-1], 0.0)

    def test_return_speed_preserves_target_and_limits_command_steps(self):
        for speed, rate in ((0.5, 50.0), (2.0, 50.0), (10.0, 5.0)):
            with self.subTest(speed=speed, rate=rate):
                runner = self.runner(speed=speed)
                runner._args.rate = rate
                start = math.radians(37.5)
                clock, trace = self.instrument(runner, {"test": [0.0]})
                runner._last_targets = {"test": [start]}
                runner._has_published = True
                with self.clock_context(clock):
                    self.assertTrue(runner.return_to_baseline(duration=4.0))
                arrivals = [t for t, targets in trace if targets["test"] == [0.0]]
                expected = max(4.0 / speed, 1.875 * 37.5 / (5.0 * rate))
                self.assertAlmostEqual(arrivals[0], math.ceil(expected * rate) / rate)
                positions = [start] + [targets["test"][0] for _, targets in trace]
                self.assertLessEqual(max(abs(b-a) for a,b in zip(positions, positions[1:])),
                                     math.radians(5.0))

    def test_torso_arm_spread_and_lower_follow_speed_once(self):
        for speed in (0.5, 2.0):
            with self.subTest(speed=speed):
                args = self.args(torso, "--speed", str(speed))
                runner = torso.ArmClearanceRunner(
                    session=Mock(), proto=SimpleNamespace(ArmState=object),
                    channels=(ChannelSpec("left_arm", "action/left_arm", "state/left_arm",
                                          "ArmCommand", "ArmState", side="left"),),
                    joint_names=arm.JOINT_NAMES, limits_deg=arm.JOINT_LIMITS_DEG,
                    amplitudes_deg=arm.JOINT_AMPLITUDES_DEG,
                    preferred_directions=arm.PREFERRED_DIRECTIONS, args=args, frame_id="test",
                )
                clock, trace = self.instrument(runner, {"left_arm": [0.0] * 7})
                pose = [math.radians(value) for value in torso.ARM_CLEARANCE_POSE_DEG]
                with self.clock_context(clock):
                    runner.spread()
                    self.assertEqual(trace[-1][1]["left_arm"], pose)
                    arrival = next(t for t, targets in trace if targets["left_arm"] == pose)
                    self.assertAlmostEqual(arrival, 4.0 / speed)
                    started = clock.now
                    trace.clear()
                    self.assertTrue(runner.lower())
                    arrival = next(t for t, targets in trace if targets["left_arm"] == [0.0]*7)
                    self.assertGreaterEqual(arrival - started, 4.0 / speed - 1e-9)
                    self.assertLessEqual(arrival - started, 4.0 / speed + 1.0/args.rate + 1e-9)


if __name__ == "__main__":
    unittest.main()
