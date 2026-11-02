#!/usr/bin/env python3
"""Exercise chassis translation and rotation through Platform's Zenoh interface.

Start the simulator with devices.chassis.enabled=true and the Platform chassis
plugin running. Commands are body-frame velocities (m/s and rad/s). Each move
ramps from and back to zero; this open-loop example does not restore world pose.
The default sequence finishes with a full forward left circle (translation and
yaw commanded together). Its duration accounts for the velocity ramps; actual
travel can differ because this is open-loop control.
"""

import argparse
import math
import os
from pathlib import Path
import sys
import threading
import time

from kinematic_motion import (
    DEFAULT_ZENOH_ENDPOINT,
    default_platform_root,
    load_dependencies,
    open_session,
)


MOTIONS = {
    "forward": (1, 0, 0),
    "backward": (-1, 0, 0),
    "left": (0, 1, 0),
    "right": (0, -1, 0),
    "rotate-left": (0, 0, 1),
    "rotate-right": (0, 0, -1),
    "arc-left": (1, 0, 1),
}
WHEEL_INDICES = (0, 2, 4, 6)


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--platform-root", type=Path,
        default=Path(os.environ.get(
            "SHARPA_PLATFORM_ROOT", str(default_platform_root(__file__))
        )),
        help="Platform checkout containing proto/north_pb2.py",
    )
    parser.add_argument("--connect", default=DEFAULT_ZENOH_ENDPOINT)
    parser.add_argument(
        "--motions", default="all",
        help="Comma-separated " + ",".join(MOTIONS) + ", or all",
    )
    parser.add_argument("--linear-speed", type=float, default=0.3, help="Peak m/s")
    parser.add_argument("--angular-speed", type=float, default=0.6, help="Peak rad/s")
    parser.add_argument(
        "--segment-duration", type=float, default=4.0,
        help="Seconds per basic move; arc-left duration is computed for one full circle",
    )
    parser.add_argument("--rate", type=float, default=50.0, help="Command frequency in Hz")
    parser.add_argument("--state-timeout", type=float, default=8.0)
    parser.add_argument("--stale-timeout", type=float, default=1.0)
    parser.add_argument(
        "--stop-duration", type=float, default=0.3,
        help="Seconds to publish zero velocity between moves and on exit",
    )
    parser.add_argument(
        "--minimum-wheel-speed", type=float, default=0.05,
        help="Minimum measured wheel response per move in rad/s",
    )
    parser.add_argument(
        "--no-verify", action="store_true",
        help="Skip wheel response check; still require fresh state feedback",
    )
    return parser.parse_args(argv)


def validate_args(args):
    for name in (
        "linear_speed", "angular_speed", "segment_duration", "rate",
        "state_timeout", "stale_timeout", "stop_duration", "minimum_wheel_speed",
    ):
        value = getattr(args, name)
        if not math.isfinite(value) or value <= 0:
            raise ValueError(f"--{name.replace('_', '-')} must be finite and positive")
    # The simulator's default chassis motor command timeout is 100 ms.
    if not 20 <= args.rate <= 200:
        raise ValueError("--rate must be between 20 and 200 Hz")
    if args.motions.strip().lower() == "all":
        return tuple(MOTIONS)
    motions = tuple(item.strip().lower() for item in args.motions.split(","))
    if any(name not in MOTIONS for name in motions):
        raise ValueError("--motions must use " + ",".join(MOTIONS) + ", or all")
    return motions


def motion_duration(name, segment_duration, angular_speed):
    if name == "arc-left":
        # Integral of sin^2(pi*t/T) over [0,T] is T/2, so the total
        # commanded yaw is angular_speed*T/2 = 2*pi. The matching linear
        # envelope keeps the circle radius at linear_speed/angular_speed.
        return 4.0 * math.pi / angular_speed
    return segment_duration


def velocity_at(name, elapsed, duration, linear_speed, angular_speed):
    """A sin-squared velocity pulse has zero speed/acceleration at both ends."""
    if elapsed <= 0 or elapsed >= duration:
        return (0.0, 0.0, 0.0)
    scale = math.sin(math.pi * elapsed / duration) ** 2
    x, y, yaw = MOTIONS[name]
    return (x * linear_speed * scale, y * linear_speed * scale,
            yaw * angular_speed * scale)


class ChassisFeedback:
    def __init__(self, message_type):
        self.message_type = message_type
        self.condition = threading.Condition()
        self.last_received = None
        self.peak_wheel_speed = 0.0

    def on_sample(self, sample):
        try:
            state = self.message_type()
            state.ParseFromString(bytes(sample.payload))
            wheels = [state.joints.velocity[index] for index in WHEEL_INDICES]
            if not all(math.isfinite(value) for value in wheels):
                return
        except Exception:
            # Protobuf implementations expose different parse exception classes.
            return
        with self.condition:
            self.last_received = time.monotonic()
            self.peak_wheel_speed = max(self.peak_wheel_speed, *(abs(v) for v in wheels))
            self.condition.notify_all()

    def wait(self, timeout):
        with self.condition:
            if not self.condition.wait_for(lambda: self.last_received is not None, timeout):
                raise RuntimeError(
                    "No state/chassis feedback; check devices.chassis.enabled and "
                    "the Platform chassis plugin"
                )

    def reset_peak(self):
        with self.condition:
            self.peak_wheel_speed = 0.0

    def check(self, max_age):
        with self.condition:
            if self.last_received is None or time.monotonic() - self.last_received > max_age:
                raise RuntimeError("state/chassis feedback became stale; stopping")
            return self.peak_wheel_speed


class ChassisPublisher:
    def __init__(self, session, proto):
        self.session = session
        self.proto = proto
        self.sequence = 0

    def publish(self, vx=0.0, vy=0.0, vtheta=0.0):
        command = self.proto.ChassisCommand()
        self.sequence += 1
        seconds, nanos = divmod(time.time_ns(), 1_000_000_000)
        command.header.stamp.seconds = seconds
        command.header.stamp.nanos = nanos
        command.header.seq = self.sequence
        command.header.frame_id = "sharpa_mujoco_chassis_motion_driver"
        command.vx, command.vy, command.vtheta = vx, vy, vtheta
        command.v_l = 0.0
        self.session.put("action/chassis", command.SerializeToString())

    def stop(self, duration, rate):
        deadline = time.monotonic() + duration
        while True:
            self.publish()
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                return
            time.sleep(min(1.0 / rate, remaining))


def run(args):
    motions = validate_args(args)
    zenoh, proto = load_dependencies(args.platform_root)
    session = open_session(zenoh, args.connect)
    publisher = ChassisPublisher(session, proto)
    feedback = ChassisFeedback(proto.ChassisState)
    subscriber = None
    try:
        subscriber = session.declare_subscriber("state/chassis", feedback.on_sample)
        print("[State] waiting for chassis feedback")
        feedback.wait(args.state_timeout)
        publisher.stop(args.stop_duration, args.rate)
        for name in motions:
            duration = motion_duration(name, args.segment_duration, args.angular_speed)
            print(f"[Motion] {name}: {duration:g} s")
            feedback.reset_peak()
            start = time.monotonic()
            while True:
                peak = feedback.check(args.stale_timeout)
                elapsed = time.monotonic() - start
                publisher.publish(*velocity_at(
                    name, elapsed, duration,
                    args.linear_speed, args.angular_speed,
                ))
                if elapsed >= duration:
                    break
                time.sleep(min(1.0 / args.rate, duration - elapsed))
            publisher.stop(args.stop_duration, args.rate)
            if not args.no_verify and peak < args.minimum_wheel_speed:
                raise RuntimeError(f"{name}: no measured wheel response ({peak:.3f} rad/s)")
            print(f"[State] peak wheel speed: {peak:.3f} rad/s")
        return 0
    finally:
        try:
            print("[Stop] publishing zero chassis velocity")
            publisher.stop(args.stop_duration, args.rate)
        finally:
            try:
                if subscriber is not None:
                    subscriber.undeclare()
            finally:
                session.close()


def main():
    try:
        return run(parse_args())
    except KeyboardInterrupt:
        print("\n[Test] interrupted")
        return 130
    except Exception as exc:
        print(f"[Error] {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    sys.exit(main())
