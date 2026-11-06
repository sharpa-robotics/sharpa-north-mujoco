#!/usr/bin/env python3
"""Collect repeatable chassis drive/steering responses through Platform Zenoh.

Run the same parameters on hardware and simulation. The default calibration
sequence excites drive speed, common steering angle and differential steering.
Commands are body-frame velocities (m/s and rad/s), not direct motor targets.
This is open-loop motion: it does not restore world pose. See chassis_motion.md
for acquisition limits, recording fields and examples. --dry-run prints the plan
without connecting or commanding motion.
"""

import argparse
from bisect import bisect_right
import json
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
STEER_INDICES = (1, 3, 5, 7)
CORNERS = ("lf", "rf", "rr", "lr")
JOINT_NAMES = tuple(f"{corner}_{kind}" for corner in CORNERS for kind in ("drive", "steer"))
STEERING_MOTIONS = ("steer-sine", "steer-hold", "yaw-sine")
CALIBRATION_MOTIONS = ("forward", "backward", *STEERING_MOTIONS)
ALL_MOTIONS = (*MOTIONS, *STEERING_MOTIONS)


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
        "--motions", default="calibration",
        help="Comma-separated " + ",".join(ALL_MOTIONS) + ", calibration (default), or all",
    )
    parser.add_argument("--linear-speed", type=float, default=0.15, help="Peak translation magnitude, m/s")
    parser.add_argument("--angular-speed", type=float, default=0.3, help="Peak yaw rate, rad/s")
    parser.add_argument(
        "--segment-duration", type=float, default=12.0,
        help="Seconds per basic move; arc-left duration is computed for one full circle",
    )
    parser.add_argument("--steer-angle-deg", type=float, default=35.0,
                        help="Translation heading amplitude for steer-sine/steer-hold, degrees (0, 80]")
    parser.add_argument("--steer-cycles", type=int, default=2,
                        help="Oscillations in the middle 70%% of sine segments")
    parser.add_argument("--repeat", type=int, default=1, help="Number of complete sequence repetitions")
    parser.add_argument("--label", default="run", help="Acquisition label, e.g. real or sim")
    parser.add_argument("--output-dir", type=Path,
                        help="New recording directory; default: recordings/chassis/<time_ns>")
    parser.add_argument("--no-record", action="store_true", help="Disable data recording for smoke tests")
    parser.add_argument("--dry-run", action="store_true", help="Print motion plan without connecting")
    parser.add_argument("--rate", type=float, default=50.0, help="Command frequency in Hz")
    parser.add_argument("--state-timeout", type=float, default=8.0)
    parser.add_argument("--stale-timeout", type=float, default=1.0)
    parser.add_argument(
        "--stop-duration", type=float, default=1.0,
        help="Seconds to publish zero velocity between moves and on exit",
    )
    parser.add_argument("--minimum-steer-travel-deg", type=float, default=2.0,
                        help="Minimum measured travel of EACH steer joint in steering segments")
    parser.add_argument(
        "--minimum-wheel-speed", type=float, default=0.05,
        help="Minimum measured wheel response per move in rad/s",
    )
    parser.add_argument(
        "--no-verify", action="store_true",
        help="Skip per-wheel speed and steering travel checks; still require fresh feedback",
    )
    return parser.parse_args(argv)


def validate_args(args):
    for name in (
        "linear_speed", "angular_speed", "segment_duration", "rate",
        "state_timeout", "stale_timeout", "stop_duration", "minimum_wheel_speed",
        "minimum_steer_travel_deg", "steer_angle_deg",
    ):
        value = getattr(args, name)
        if not math.isfinite(value) or value <= 0:
            raise ValueError(f"--{name.replace('_', '-')} must be finite and positive")
    # The simulator's default chassis motor command timeout is 100 ms.
    if not 20 <= args.rate <= 200:
        raise ValueError("--rate must be between 20 and 200 Hz")
    if args.steer_angle_deg > 80:
        raise ValueError("--steer-angle-deg must be at most 80 degrees")
    if args.steer_cycles < 1 or args.repeat < 1:
        raise ValueError("--steer-cycles and --repeat must be positive integers")
    if args.no_record and args.output_dir is not None:
        raise ValueError("--output-dir cannot be combined with --no-record")
    if args.motions.strip().lower() == "calibration":
        return CALIBRATION_MOTIONS
    if args.motions.strip().lower() == "all":
        return ALL_MOTIONS
    motions = tuple(item.strip().lower() for item in args.motions.split(","))
    if any(name not in ALL_MOTIONS for name in motions):
        raise ValueError("--motions must use " + ",".join(ALL_MOTIONS) + ", calibration, or all")
    return motions


def motion_duration(name, segment_duration, angular_speed):
    if name == "arc-left":
        # Integral of sin^2(pi*t/T) over [0,T] is T/2, so the total
        # commanded yaw is angular_speed*T/2 = 2*pi. The matching linear
        # envelope keeps the circle radius at linear_speed/angular_speed.
        return 4.0 * math.pi / angular_speed
    return segment_duration


def smoothstep(value):
    value = max(0.0, min(1.0, value))
    return value**3 * (10.0 - 15.0 * value + 6.0 * value**2)


def steering_signal(name, progress, cycles):
    """Stay centered during ramp-up/down; exercise steering at constant speed."""
    u = max(0.0, min(1.0, (progress - 0.15) / 0.7))
    if u <= 0 or u >= 1:
        return 0.0
    if name != "steer-hold":
        # Complete cycles with zero angle/rate/acceleration at both joins.
        return math.sin(2.0 * math.pi * cycles * u) * smoothstep(u / 0.15) * smoothstep((1 - u) / 0.15)
    if u < 0.2:
        return smoothstep(u / 0.2)
    if u < 0.35:
        return 1.0
    if u < 0.65:
        return 1.0 - 2.0 * smoothstep((u - 0.35) / 0.3)
    if u < 0.8:
        return -1.0
    return -1.0 + smoothstep((u - 0.8) / 0.2)


def velocity_at(name, elapsed, duration, linear_speed, angular_speed,
                steer_angle_deg=35.0, steer_cycles=2):
    """Basic pulses excite drive speed; changing direction excites steering."""
    if elapsed <= 0 or elapsed >= duration:
        return (0.0, 0.0, 0.0)
    progress = elapsed / duration
    if name in STEERING_MOTIONS:
        # Holding translation magnitude separates steering response from the
        # drive acceleration pulse. No zero crossing during the steering sweep:
        # a zero body command may lock/center steering in Platform's IK.
        scale = smoothstep(progress / 0.15) * smoothstep((1 - progress) / 0.15)
        signal = steering_signal(name, progress, steer_cycles)
        if name == "yaw-sine":
            return (linear_speed * scale, 0.0, angular_speed * scale * signal)
        angle = math.radians(steer_angle_deg) * signal
        return (linear_speed * scale * math.cos(angle),
                linear_speed * scale * math.sin(angle), 0.0)
    scale = math.sin(math.pi * progress) ** 2
    x, y, yaw = MOTIONS[name]
    return (x * linear_speed * scale, y * linear_speed * scale,
            yaw * angular_speed * scale)


class ChassisRecording:
    """Timestamped event log: keep asynchronous state at its native arrival rate.

    Buffered writes are serialized with phase changes. Callback errors are saved
    and checked by the command loop so a broken recording stops the acquisition.
    """

    def __init__(self, args, motions):
        self.lock = threading.Lock()
        self.error = None
        self.closed = False
        self.start = time.monotonic()
        self.phase_times = [self.start]
        self.contexts = [{"phase": "startup", "motion": None, "repeat": 0, "segment": 0}]
        self.directory = args.output_dir or Path("recordings/chassis") / str(time.time_ns())
        self.directory.mkdir(parents=True, exist_ok=False)
        metadata = {
            "schema_version": 1,
            "args": {key: str(value) if isinstance(value, Path) else value
                     for key, value in vars(args).items()},
            "motions": list(motions),
            "joint_order_fallback": list(JOINT_NAMES),
            "units": {"position": "rad", "velocity": "rad/s", "current": "A",
                      "body_linear_velocity": "m/s", "body_angular_velocity": "rad/s"},
            "notes": [
                "Commands are body velocities, not post-limiter motor targets.",
                "ChassisTarget is not exposed by the current Platform Zenoh interface.",
                "State stamp is the latest host receive time across available axes, not a device sample time.",
                "fresh=false marks zero, repeated or out-of-order source stamps; do not treat as new measurements.",
                "Current Platform drive position, steer velocity and joint torque are placeholders (zero).",
                "Body feedback velocity is wheel FK, not independent ground truth.",
                "Stretch axes, when present, use Platform's normalized extension values.",
            ],
        }
        (self.directory / "metadata.json").write_text(json.dumps(metadata, indent=2) + "\n")
        self.file = (self.directory / "events.jsonl").open("w", encoding="utf-8")
        print(f"[Record] {self.directory.resolve()}")

    def phase(self, phase, motion=None, repeat=0, segment=0):
        with self.lock:
            self.phase_times.append(time.monotonic())
            self.contexts.append(dict(phase=phase, motion=motion, repeat=repeat, segment=segment))

    def write(self, event, *, received=None, wall_ns=None, **data):
        received = time.monotonic() if received is None else received
        wall_ns = time.time_ns() if wall_ns is None else wall_ns
        with self.lock:
            if self.error is not None or self.closed:
                return
            try:
                # A callback can finish parsing after the next segment starts.
                # Attribute it by its receipt time, not by the writer's time.
                index = max(0, bisect_right(self.phase_times, received) - 1)
                record = dict(event=event, local_time_ns=wall_ns,
                              elapsed_s=received - self.start,
                              phase_elapsed_s=received - self.phase_times[index],
                              **self.contexts[index], **data)
                self.file.write(json.dumps(record, allow_nan=False) + "\n")
            except Exception as exc:
                self.error = exc

    def check(self):
        with self.lock:
            if self.error is not None:
                raise RuntimeError(f"Chassis recording failed: {self.error}") from self.error

    def close(self):
        with self.lock:
            self.closed = True
            self.file.close()
        self.check()


def header_data(message):
    header = message.header
    return dict(source_stamp_ns=header.stamp.seconds * 1_000_000_000 + header.stamp.nanos,
                source_seq=header.seq, frame_id=header.frame_id)


def state_data(state):
    """Keep original names, units and even placeholder fields for offline work."""
    return dict(
        **header_data(state),
        joint_names=list(state.joints.name),
        position=list(state.joints.position), velocity=list(state.joints.velocity),
        torque=list(state.joints.torque), joint_mode=list(state.joint_mode),
        electrical_names=list(state.electrical.name),
        current=list(state.electrical.current),
        bus_voltage=list(state.electrical.bus_voltage),
        temperature_celsius=list(state.electrical.temperature_celsius),
        body_velocity=[state.velocity.linear.x, state.velocity.linear.y, state.velocity.angular.z],
    )


class ChassisFeedback:
    def __init__(self, message_type, recording=None):
        self.message_type = message_type
        self.recording = recording
        self.condition = threading.Condition()
        self.last_received = None
        self.last_stamp = 0
        self.reset_peak()

    def on_sample(self, sample):
        received, wall_ns = time.monotonic(), time.time_ns()
        try:
            state = self.message_type()
            state.ParseFromString(bytes(sample.payload))
            # Fixed schema fallback is allowed only when names are absent.
            if state.joints.name and tuple(state.joints.name[:8]) != JOINT_NAMES:
                return
            wheels = [state.joints.velocity[index] for index in WHEEL_INDICES]
            angles = [state.joints.position[index] for index in STEER_INDICES]
            if not all(math.isfinite(value) for value in (*wheels, *angles)):
                return
            stamp = header_data(state)["source_stamp_ns"]
        except Exception:
            # Protobuf implementations expose different parse exception classes.
            return
        with self.condition:
            fresh = stamp > self.last_stamp
            if fresh:
                # Platform can publish the same cached state at high frequency.
                # Receipt alone is not evidence of live motor feedback.
                self.last_stamp = stamp
                self.last_received = received
                self.sample_count += 1
                for index, (speed, angle) in enumerate(zip(wheels, angles)):
                    self.wheel_peaks[index] = max(self.wheel_peaks[index], abs(speed))
                    previous = self.previous_angles[index]
                    if previous is not None:
                        self.unwrapped_angles[index] += math.remainder(angle - previous, 2 * math.pi)
                    self.previous_angles[index] = angle
                    self.steer_min[index] = min(self.steer_min[index], self.unwrapped_angles[index])
                    self.steer_max[index] = max(self.steer_max[index], self.unwrapped_angles[index])
                self.condition.notify_all()
        if self.recording is not None:
            self.recording.write("state", received=received, wall_ns=wall_ns,
                                 fresh=fresh, **state_data(state))

    def wait(self, timeout):
        with self.condition:
            if not self.condition.wait_for(lambda: self.last_received is not None, timeout):
                raise RuntimeError(
                    "No state/chassis feedback with advancing nonzero source timestamps; "
                    "check the Platform chassis plugin and motor feedback"
                )

    def reset_peak(self):
        with self.condition:
            self.wheel_peaks = [0.0] * 4
            self.previous_angles = [None] * 4
            self.unwrapped_angles = [0.0] * 4
            self.steer_min = [math.inf] * 4
            self.steer_max = [-math.inf] * 4
            self.sample_count = 0

    def summary(self):
        with self.condition:
            return dict(samples=self.sample_count, wheel_peak_rad_s=list(self.wheel_peaks),
                        steer_travel_deg=[math.degrees(hi - lo) if self.sample_count else 0.0
                                          for lo, hi in zip(self.steer_min, self.steer_max)])

    def check(self, max_age):
        with self.condition:
            if self.last_received is None or time.monotonic() - self.last_received > max_age:
                raise RuntimeError("state/chassis feedback became stale; stopping")
            return max(self.wheel_peaks)


class ChassisPublisher:
    def __init__(self, session, proto, recording=None):
        self.session = session
        self.proto = proto
        self.sequence = 0
        self.recording = recording

    def publish(self, vx=0.0, vy=0.0, vtheta=0.0):
        sent, wall_ns = time.monotonic(), time.time_ns()
        command = self.proto.ChassisCommand()
        self.sequence += 1
        seconds, nanos = divmod(wall_ns, 1_000_000_000)
        command.header.stamp.seconds = seconds
        command.header.stamp.nanos = nanos
        command.header.seq = self.sequence
        command.header.frame_id = "sharpa_mujoco_chassis_motion_driver"
        command.vx, command.vy, command.vtheta = vx, vy, vtheta
        command.v_l = 0.0
        self.session.put("action/chassis", command.SerializeToString())
        if self.recording is not None:
            self.recording.write("command", received=sent, wall_ns=wall_ns,
                                 **header_data(command),
                                 vx=command.vx, vy=command.vy, vtheta=command.vtheta, v_l=0.0)

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
    for name in motions:
        duration = motion_duration(name, args.segment_duration, args.angular_speed)
        print(f"[Plan] {name}: {duration:g} s x {args.repeat}")
    print(f"[Plan] translation <= {args.linear_speed:g} m/s, yaw <= {args.angular_speed:g} rad/s; "
          f"steering heading +/-{args.steer_angle_deg:g} deg, {args.steer_cycles} cycles")
    if args.dry_run:
        return 0
    zenoh, proto = load_dependencies(args.platform_root)
    recording = None if args.no_record else ChassisRecording(args, motions)
    session = publisher = None
    subscriber = None
    outcome, error = "failed", None
    try:
        session = open_session(zenoh, args.connect)
        publisher = ChassisPublisher(session, proto, recording)
        feedback = ChassisFeedback(proto.ChassisState, recording)
        subscriber = session.declare_subscriber("state/chassis", feedback.on_sample)
        print("[State] waiting for chassis feedback")
        feedback.wait(args.state_timeout)
        publisher.stop(args.stop_duration, args.rate)
        for repeat in range(1, args.repeat + 1):
            for segment, name in enumerate(motions, 1):
                duration = motion_duration(name, args.segment_duration, args.angular_speed)
                print(f"[Motion] repeat {repeat}/{args.repeat}, {name}: {duration:g} s")
                feedback.reset_peak()
                if recording is not None:
                    recording.phase("motion", name, repeat, segment)
                start = time.monotonic()
                while True:
                    feedback.check(args.stale_timeout)
                    if recording is not None:
                        recording.check()
                    elapsed = time.monotonic() - start
                    publisher.publish(*velocity_at(
                        name, elapsed, duration, args.linear_speed, args.angular_speed,
                        args.steer_angle_deg, args.steer_cycles,
                    ))
                    if elapsed >= duration:
                        break
                    # Absolute tick grid avoids accumulating serialization and
                    # scheduler delay; skip missed ticks instead of replaying them.
                    next_tick = min((math.floor(elapsed * args.rate) + 1) / args.rate, duration)
                    time.sleep(max(0.0, start + next_tick - time.monotonic()))
                # Exclude steering recenter/hold behavior during the stop from
                # the excitation's response check. It is still in the event log.
                summary = feedback.summary()
                if recording is not None:
                    recording.write("segment_summary", **summary)
                    recording.phase("settle", name, repeat, segment)
                publisher.stop(args.stop_duration, args.rate)
                feedback.check(args.stale_timeout)
                if not args.no_verify:
                    for corner, peak in zip(CORNERS, summary["wheel_peak_rad_s"]):
                        if peak < args.minimum_wheel_speed:
                            raise RuntimeError(f"{name}: {corner} no measured wheel response ({peak:.3f} rad/s)")
                    if name in STEERING_MOTIONS:
                        for corner, travel in zip(CORNERS, summary["steer_travel_deg"]):
                            if travel < args.minimum_steer_travel_deg:
                                raise RuntimeError(f"{name}: {corner} no measured steering response ({travel:.2f} deg)")
                print(f"[State] wheel peaks rad/s: {summary['wheel_peak_rad_s']}; "
                      f"steer travel deg: {[round(v, 2) for v in summary['steer_travel_deg']]}")
        if recording is not None:
            recording.check()
        outcome = "complete"
        return 0
    except BaseException as exc:
        outcome = "interrupted" if isinstance(exc, KeyboardInterrupt) else "failed"
        error = str(exc) or type(exc).__name__
        raise
    finally:
        try:
            try:
                if publisher is not None:
                    if recording is not None:
                        recording.phase("shutdown")
                    print("[Stop] publishing zero chassis velocity")
                    publisher.stop(args.stop_duration, args.rate)
            finally:
                try:
                    if subscriber is not None:
                        subscriber.undeclare()
                finally:
                    if session is not None:
                        session.close()
        except BaseException as exc:
            outcome = "interrupted" if isinstance(exc, KeyboardInterrupt) else "failed"
            error = str(exc) or type(exc).__name__
            raise
        finally:
            if recording is not None:
                recording.write("result", outcome=outcome, error=error)
                recording.close()


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
