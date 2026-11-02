#!/usr/bin/env python3
"""Check motion bounds and stop-on-failure without a running robot stack."""

import json
import math
from pathlib import Path
import sys
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
    def ParseFromString(self, payload):
        self.joints = SimpleNamespace(velocity=json.loads(payload))


class Session:
    def __init__(self, fault=None):
        self.commands = []
        self.fault = fault
        self.closed = False
        self.undeclared = False

    def emit(self, speed):
        self.callback(SimpleNamespace(payload=json.dumps([speed, 0] * 4).encode()))

    def declare_subscriber(self, topic, callback):
        assert topic == "state/chassis"
        self.callback = callback
        self.emit(0)
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
            self.emit(0 if self.fault == "unresponsive" else max(map(abs, command)) * 10)

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
                              ("--motions", "forward,typo")):
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

    def exercise(self, fault=None):
        args = driver.parse_args(["--segment-duration", "0.4", "--stop-duration", "0.1",
                                  "--stale-timeout", "0.2"])
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
                    with self.assertRaisesRegex(RuntimeError, "stale|no measured wheel response"):
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
        session = self.exercise()
        for axis in range(3):
            self.assertTrue(any(command[axis] > 0 for command in session.commands))
            self.assertTrue(any(command[axis] < 0 for command in session.commands))
        # The final move must publish translation and yaw simultaneously,
        # followed by the existing zero-velocity shutdown sequence.
        last_motion = next(command for command in reversed(session.commands) if any(command))
        self.assertGreater(last_motion[0], 0)
        self.assertEqual(last_motion[1], 0)
        self.assertGreater(last_motion[2], 0)

    def test_stop_on_interrupt_and_feedback_failures(self):
        for fault in ("interrupt", "stale", "unresponsive"):
            with self.subTest(fault=fault):
                self.exercise(fault)

    def test_malformed_feedback_does_not_refresh_state(self):
        feedback = driver.ChassisFeedback(State)
        for values in ([0], [math.nan] * 8):
            feedback.on_sample(SimpleNamespace(payload=json.dumps(values).encode()))
        with self.assertRaisesRegex(RuntimeError, "No state/chassis"):
            feedback.wait(0.001)


if __name__ == "__main__":
    unittest.main()
