#!/usr/bin/env python3
"""Check motion bounds and stop-on-failure without a running robot stack."""

import json
import math
from pathlib import Path
import sys
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "examples/motion"))
import chassis_motion_driver as driver


class Command:
    def __init__(self):
        self.header = SimpleNamespace(stamp=SimpleNamespace())

    def SerializeToString(self):
        return json.dumps([self.vx, self.vy, self.vtheta, self.v_l]).encode()


class State:
    def __init__(self):
        self.header = SimpleNamespace(stamp=SimpleNamespace(seconds=0, nanos=0), seq=0, frame_id="chassis")
        self.joints = SimpleNamespace(name=list(driver.JOINT_NAMES), position=[0.] * 8,
                                      velocity=[0.] * 8, torque=[0.] * 8)
        self.joint_mode = []
        self.electrical = SimpleNamespace(name=[], current=[], bus_voltage=[], temperature_celsius=[])
        self.velocity = SimpleNamespace(linear=SimpleNamespace(x=0., y=0.), angular=SimpleNamespace(z=0.))

    def ParseFromString(self, payload):
        data = json.loads(payload)
        self.header.stamp.nanos = data["stamp"]
        self.joints.position = data.get("position", [0.] * 8)
        self.joints.velocity = data.get("velocity", [0.] * 8)
        self.joints.name = data.get("names", list(driver.JOINT_NAMES))


class Session:
    def __init__(self, fault=None):
        self.commands = []
        self.fault = fault
        self.closed = False
        self.undeclared = False
        self.stamp = 0

    def emit(self, command):
        if self.fault != "frozen" or self.stamp == 0:
            self.stamp += 1
        vx, vy, wz, _ = command
        positions, velocities = [], []
        for i, (rx, ry) in enumerate(((.19, .18), (.19, -.18), (-.2, -.18), (-.2, .18))):
            x, y = vx - wz * ry, vy + wz * rx
            angle = math.atan2(y, x)
            if self.fault == "stuck-steer" or (self.fault == "one-steer" and i == 2):
                angle = 0.
            speed = math.hypot(x, y) / .065
            if self.fault == "unresponsive" or (self.fault == "one-wheel" and i == 2):
                speed = 0.
            positions.extend((0., angle))
            velocities.extend((speed, 0.))
        self.callback(SimpleNamespace(payload=json.dumps(
            dict(stamp=self.stamp, position=positions, velocity=velocities)).encode()))

    def declare_subscriber(self, topic, callback):
        assert topic == "state/chassis"
        self.callback = callback
        self.emit([0, 0, 0, 0])
        return SimpleNamespace(undeclare=self.undeclare)

    def undeclare(self):
        self.undeclared = True

    def put(self, topic, payload):
        assert topic == "action/chassis"
        command = json.loads(payload)
        self.commands.append(command)
        if any(command) and self.fault == "interrupt":
            self.fault = None
            raise KeyboardInterrupt
        if self.fault != "stale":
            self.emit(command)

    def close(self):
        self.closed = True


class ChassisMotionTest(unittest.TestCase):
    def test_velocity_bounds_and_directions(self):
        for name, direction in driver.MOTIONS.items():
            self.assertEqual(driver.velocity_at(name, 0, 4, .15, .3), (0, 0, 0))
            self.assertEqual(driver.velocity_at(name, 4, 4, .15, .3), (0, 0, 0))
            expected = tuple(d * s for d, s in zip(direction, (.15, .15, .3)))
            self.assertEqual(driver.velocity_at(name, 2, 4, .15, .3), expected)
            for step in range(101):
                values = driver.velocity_at(name, step / 25, 4, .15, .3)
                for value, limit in zip(values, expected):
                    self.assertLessEqual(abs(value), abs(limit))
                    self.assertGreaterEqual(value * limit, 0)

    def test_invalid_arguments(self):
        for option, value in (("--rate", "1"), ("--linear-speed", "nan"),
                              ("--angular-speed", "inf"), ("--stop-duration", "0"),
                              ("--motions", "forward,typo"), ("--motions", ""),
                              ("--steer-angle-deg", "81"), ("--steer-cycles", "0"),
                              ("--repeat", "0"), ("--minimum-steer-travel-deg", "nan")):
            with self.subTest(option=option), self.assertRaises(ValueError):
                driver.validate_args(driver.parse_args([option, value]))

    def test_circle_integrates_to_one_turn_at_different_speeds(self):
        for angular_speed in (0.3, 0.6, 1.2):
            duration = driver.motion_duration("arc-left", 4.0, angular_speed)
            dt = duration / 2000
            heading = x = y = 0.0
            for step in range(2000):
                vx, vy, wz = driver.velocity_at(
                    "arc-left", (step + 0.5) * dt, duration, 0.3, angular_speed,
                )
                self.assertEqual(vy, 0)
                midpoint_heading = heading + wz * dt / 2
                x += vx * math.cos(midpoint_heading) * dt
                y += vx * math.sin(midpoint_heading) * dt
                heading += wz * dt
            self.assertAlmostEqual(heading, 2 * math.pi, places=6)
            self.assertLess(math.hypot(x, y), 1e-5)
        self.assertEqual(driver.motion_duration("forward", 4.0, 0.6), 4.0)

    def exercise(self, fault=None, options=()):
        args = driver.parse_args(["--segment-duration", "0.4", "--stop-duration", "0.1",
                                  "--stale-timeout", "0.2", "--no-record", *options])
        session = Session(fault)
        clock = [100.0]

        def sleep(seconds):
            clock[0] += max(seconds, 1e-6)

        proto = SimpleNamespace(ChassisCommand=Command, ChassisState=State)
        with patch.object(driver, "load_dependencies", return_value=(None, proto)), \
                patch.object(driver, "open_session", return_value=session), \
                patch.object(driver.time, "monotonic", side_effect=lambda: clock[0]), \
                patch.object(driver.time, "sleep", side_effect=sleep):
            try:
                if fault == "interrupt":
                    with self.assertRaises(KeyboardInterrupt):
                        driver.run(args)
                elif fault:
                    with self.assertRaisesRegex(RuntimeError, "stale|no measured wheel response|no measured steering response"):
                        driver.run(args)
                else:
                    self.assertEqual(driver.run(args), 0)
            finally:
                self.assertTrue(session.closed)
                self.assertTrue(session.undeclared)
                self.assertEqual(session.commands[-1], [0, 0, 0, 0])
                self.assertTrue(all(command[3] == 0 for command in session.commands))
        return session

    def test_complete_sequence(self):
        session = self.exercise(options=("--motions", "all"))
        for axis in range(3):
            self.assertTrue(any(command[axis] > 0 for command in session.commands))
            self.assertTrue(any(command[axis] < 0 for command in session.commands))
        self.assertTrue(any(c[0] > 0 and c[2] > 0 for c in session.commands))

    def test_default_calibration_sequence(self):
        self.assertEqual(driver.validate_args(driver.parse_args([])), driver.CALIBRATION_MOTIONS)
        self.assertNotIn("arc-left", driver.CALIBRATION_MOTIONS)
        self.exercise()

    def test_stop_on_interrupt_and_feedback_failures(self):
        for fault in ("interrupt", "stale", "unresponsive", "frozen", "one-wheel", "stuck-steer", "one-steer"):
            with self.subTest(fault=fault):
                self.exercise(fault)

    def test_malformed_feedback_does_not_refresh_state(self):
        feedback = driver.ChassisFeedback(State)
        for data in (dict(stamp=1, velocity=[0]), dict(stamp=1, velocity=[math.nan] * 8),
                     dict(stamp=1, position=[math.nan] * 8), dict(stamp=0),
                     dict(stamp=1, names=list(reversed(driver.JOINT_NAMES)))):
            feedback.on_sample(SimpleNamespace(payload=json.dumps(data).encode()))
        with self.assertRaisesRegex(RuntimeError, "No state/chassis"):
            feedback.wait(0.001)

    def test_continuous_steering_has_both_signs_at_constant_translation_speed(self):
        for name in driver.STEERING_MOTIONS:
            values = [driver.velocity_at(name, i / 100, 12., .15, .3, 35, 2)
                      for i in range(1201)]
            self.assertEqual(values[0], (0, 0, 0))
            self.assertEqual(values[-1], (0, 0, 0))
            for i, (vx, vy, wz) in enumerate(values):
                self.assertLessEqual(math.hypot(vx, vy), .15 + 1e-12)
                self.assertLessEqual(abs(wz), .3 + 1e-12)
                self.assertGreaterEqual(vx, 0)
                if 180 <= i <= 1020:
                    self.assertAlmostEqual(math.hypot(vx, vy), .15)
            signal = [wz if name == "yaw-sine" else math.atan2(vy, vx)
                      for vx, vy, wz in values]
            self.assertGreater(max(signal), .29)
            self.assertLess(min(signal), -.29)
            if name != "yaw-sine":
                self.assertLessEqual(max(map(abs, signal)), math.radians(35) + 1e-12)
            # Small joins avoid a steering step while speed is nonzero.
            for progress in (.15, .85):
                self.assertAlmostEqual(driver.steering_signal(name, progress, 2), 0)
                self.assertLess(abs(driver.steering_signal(name, progress + 1e-6, 2)), 1e-10)

    def test_hold_reaches_both_plateaus(self):
        for u, expected in ((.21, 1), (.34, 1), (.66, -1), (.79, -1)):
            self.assertAlmostEqual(driver.steering_signal("steer-hold", .15 + .7 * u, 2), expected)

    def test_frozen_or_reordered_source_stamp_does_not_refresh_or_change_peaks(self):
        feedback = driver.ChassisFeedback(State)
        with patch.object(driver.time, "monotonic", return_value=1):
            feedback.on_sample(SimpleNamespace(payload=json.dumps(dict(stamp=10)).encode()))
        with patch.object(driver.time, "monotonic", return_value=2):
            for stamp in (0, 10, 9):
                feedback.on_sample(SimpleNamespace(payload=json.dumps(
                    dict(stamp=stamp, velocity=[100.] * 8)).encode()))
            with self.assertRaisesRegex(RuntimeError, "stale"):
                feedback.check(.5)
        self.assertEqual(feedback.summary()["samples"], 1)
        self.assertEqual(feedback.summary()["wheel_peak_rad_s"], [0.] * 4)

    def test_steering_travel_unwraps_angle_boundary(self):
        feedback = driver.ChassisFeedback(State)
        for stamp, angle in enumerate((179, -179, -177), 1):
            feedback.on_sample(SimpleNamespace(payload=json.dumps(
                dict(stamp=stamp, position=[0, math.radians(angle)] * 4)).encode()))
        for travel in feedback.summary()["steer_travel_deg"]:
            self.assertAlmostEqual(travel, 4.)

    def test_dry_run_does_not_connect_or_create_output(self):
        with patch.object(driver, "load_dependencies") as load, \
                patch.object(driver, "ChassisRecording") as record:
            self.assertEqual(driver.run(driver.parse_args(["--dry-run"])), 0)
        load.assert_not_called()
        record.assert_not_called()

    def test_recording_repetitions_and_partial_failures(self):
        for fault in (None, "interrupt", "stuck-steer"):
            with self.subTest(fault=fault), tempfile.TemporaryDirectory() as directory:
                output = Path(directory) / "capture"
                args = driver.parse_args(["--motions", "steer-hold", "--repeat", "2",
                                          "--segment-duration", "0.4", "--stop-duration", "0.1",
                                          "--label", "real", "--output-dir", str(output)])
                session, clock = Session(fault), [100.]

                def sleep(seconds):
                    clock[0] += max(seconds, 1e-6)

                proto = SimpleNamespace(ChassisCommand=Command, ChassisState=State)
                with patch.object(driver, "load_dependencies", return_value=(None, proto)), \
                        patch.object(driver, "open_session", return_value=session), \
                        patch.object(driver.time, "monotonic", side_effect=lambda: clock[0]), \
                        patch.object(driver.time, "sleep", side_effect=sleep):
                    if fault:
                        with self.assertRaises((KeyboardInterrupt, RuntimeError)):
                            driver.run(args)
                    else:
                        self.assertEqual(driver.run(args), 0)
                self.assertTrue(session.closed)
                self.assertEqual(session.commands[-1], [0, 0, 0, 0])
                metadata = json.loads((output / "metadata.json").read_text())
                self.assertEqual(metadata["args"]["label"], "real")
                events = [json.loads(line) for line in (output / "events.jsonl").read_text().splitlines()]
                self.assertEqual(events[-1]["event"], "result")
                self.assertEqual(events[-1]["outcome"],
                                 "complete" if fault is None else "interrupted" if fault == "interrupt" else "failed")
                self.assertTrue(any(e["event"] == "command" and e["phase"] == "shutdown"
                                    and e["vx"] == e["vy"] == e["vtheta"] == 0 for e in events))
                state = next(e for e in events if e["event"] == "state")
                self.assertEqual(state["joint_names"], list(driver.JOINT_NAMES))
                self.assertEqual(len(state["position"]), 8)
                self.assertIn("current", state)
                self.assertIn("local_time_ns", state)
                self.assertIn("source_stamp_ns", state)
                if fault is None:
                    self.assertEqual({e["repeat"] for e in events if e["phase"] == "motion"}, {1, 2})

    def test_recording_io_failure_stops_and_closes_session(self):
        with tempfile.TemporaryDirectory() as directory:
            args = driver.parse_args(["--motions", "forward", "--segment-duration", ".4",
                                      "--stop-duration", ".1", "--output-dir", str(Path(directory) / "capture")])
            session = Session()
            write = driver.ChassisRecording.write

            def failing_write(recording, event, **data):
                if event == "command" and data["vx"] > 0:
                    recording.error = OSError("disk full")
                else:
                    write(recording, event, **data)

            proto = SimpleNamespace(ChassisCommand=Command, ChassisState=State)
            with patch.object(driver, "load_dependencies", return_value=(None, proto)), \
                    patch.object(driver, "open_session", return_value=session), \
                    patch.object(driver.ChassisRecording, "write", failing_write):
                with self.assertRaisesRegex(RuntimeError, "recording failed"):
                    driver.run(args)
            self.assertTrue(session.closed)
            self.assertEqual(session.commands[-1], [0, 0, 0, 0])

    def test_delayed_callback_keeps_its_original_phase(self):
        with tempfile.TemporaryDirectory() as directory:
            args = driver.parse_args(["--output-dir", str(Path(directory) / "capture")])
            with patch.object(driver.time, "monotonic", return_value=100.):
                recording = driver.ChassisRecording(args, driver.CALIBRATION_MOTIONS)
            with patch.object(driver.time, "monotonic", return_value=101.):
                recording.phase("motion", "steer-sine", 1, 1)
            with patch.object(driver.time, "monotonic", return_value=102.):
                recording.phase("settle", "steer-sine", 1, 1)
                recording.write("state", received=101.9, fresh=True)
            recording.close()
            event = json.loads((recording.directory / "events.jsonl").read_text())
            self.assertEqual(event["phase"], "motion")
            self.assertAlmostEqual(event["phase_elapsed_s"], .9)

    def test_existing_recording_is_not_overwritten(self):
        with tempfile.TemporaryDirectory() as directory:
            existing = Path(directory) / "metadata.json"
            existing.write_text("previous acquisition")
            args = driver.parse_args(["--output-dir", directory])
            with self.assertRaises(FileExistsError):
                driver.ChassisRecording(args, driver.CALIBRATION_MOTIONS)
            self.assertEqual(existing.read_text(), "previous acquisition")

    def test_repository_protobuf_serialization_and_recording(self):
        try:
            import google.protobuf  # noqa: F401
        except ImportError:
            self.skipTest("optional protobuf package is not installed")
        sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "proto/platform"))
        import north_pb2 as proto

        with tempfile.TemporaryDirectory() as directory:
            args = driver.parse_args(["--output-dir", str(Path(directory) / "capture")])
            recording = driver.ChassisRecording(args, driver.CALIBRATION_MOTIONS)
            try:
                feedback = driver.ChassisFeedback(proto.ChassisState, recording)
                state = proto.ChassisState()
                state.header.stamp.seconds = 1
                state.joints.name.extend(driver.JOINT_NAMES)
                state.joints.position.extend([0, .2] * 4)
                state.joints.velocity.extend([1, 0] * 4)
                state.electrical.name.extend(driver.JOINT_NAMES)
                state.electrical.current.extend([.5] * 8)
                for _ in range(2):
                    feedback.on_sample(SimpleNamespace(payload=state.SerializeToString()))
                self.assertEqual(feedback.summary()["samples"], 1)
                self.assertEqual(feedback.summary()["wheel_peak_rad_s"], [1.] * 4)
                commands = []
                session = SimpleNamespace(put=lambda topic, payload: commands.append(
                    proto.ChassisCommand.FromString(payload)))
                driver.ChassisPublisher(session, proto, recording).publish(.1, .02, -.1)
                self.assertAlmostEqual(commands[0].vx, .1)
                self.assertEqual(commands[0].v_l, 0)
            finally:
                recording.close()
            events = [json.loads(line) for line in (recording.directory / "events.jsonl").read_text().splitlines()]
            self.assertEqual([event["fresh"] for event in events[:2]], [True, False])
            self.assertEqual(events[0]["current"], [.5] * 8)
            self.assertEqual(events[2]["event"], "command")
            self.assertEqual(events[2]["source_seq"], 1)


if __name__ == "__main__":
    unittest.main()
