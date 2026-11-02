#!/usr/bin/env python3
"""Shared trajectory and Zenoh helpers for kinematic motion tests.

This module intentionally imports Zenoh and the generated protobuf module only
when ``load_dependencies`` is called.  The trajectory functions therefore stay
small, deterministic, and testable without a running robot stack.
"""

from bisect import bisect_right
import importlib
import json
import math
import sys
import threading
import time
from dataclasses import dataclass
from pathlib import Path


ZENOH_CLIENT_CONFIG = Path(__file__).resolve().parents[2] / "resources/zenoh/client.json"
DEFAULT_ZENOH_ENDPOINT = json.loads(ZENOH_CLIENT_CONFIG.read_text())["connect"]["endpoints"][0]
PROFILE_NAMES = ("sweep", "sine", "trapezoid", "minjerk", "chirp")
# Reference peak speed for point-to-point profiles at --speed 1. Longer
# excursions take longer; coordinated axes share the slowest axis's timing.
REFERENCE_SPEED_DEG_PER_SECOND = 15.0
MAX_COMMAND_STEP_DEG = 5.0
# A complete chirp followed by a return lasting 20% of its sweep duration.
CHIRP_OSCILLATION_FRACTION = 5.0 / 6.0


@dataclass(frozen=True)
class ChannelSpec:
    """Zenoh and protobuf description for one independently addressed device."""

    label: str
    action_topic: str
    state_topic: str
    command_type: str
    state_type: str
    side: str = ""
    mirror_right: bool = False


@dataclass(frozen=True)
class ProfilePlan:
    """The original path with time added only where command travel needs it."""

    times: tuple
    targets: tuple
    nominal_duration: float
    limited_seconds: float
    return_start_time: float = 0.0

    @property
    def duration(self):
        return self.times[-1]

    def at(self, elapsed):
        if elapsed >= self.duration:
            return self.targets[-1]
        index = max(0, bisect_right(self.times, elapsed) - 1)
        blend = (elapsed - self.times[index]) / (
            self.times[index + 1] - self.times[index]
        )
        return {
            label: [
                start + (end - start) * blend
                for start, end in zip(positions, self.targets[index + 1][label])
            ]
            for label, positions in self.targets[index].items()
        }


def clamp(value, lower, upper):
    return max(lower, min(upper, value))


def minimum_jerk_progress(progress):
    """Return a quintic 0-to-1 move with zero endpoint velocity/acceleration."""

    value = clamp(progress, 0.0, 1.0)
    return value**3 * (10.0 - 15.0 * value + 6.0 * value**2)


def trapezoid_progress(progress, acceleration_fraction=0.25):
    """Return position for a unit move with trapezoidal velocity."""

    value = clamp(progress, 0.0, 1.0)
    if not 0.0 < acceleration_fraction < 0.5:
        raise ValueError("acceleration_fraction must be between 0 and 0.5")
    maximum_velocity = 1.0 / (1.0 - acceleration_fraction)
    acceleration = maximum_velocity / acceleration_fraction
    if value < acceleration_fraction:
        return 0.5 * acceleration * value**2
    if value <= 1.0 - acceleration_fraction:
        acceleration_distance = 0.5 * maximum_velocity * acceleration_fraction
        return acceleration_distance + maximum_velocity * (
            value - acceleration_fraction
        )
    remaining = 1.0 - value
    return 1.0 - 0.5 * acceleration * remaining**2


def round_trip(progress, outbound_profile):
    """Map a 0-to-1 profile to a baseline-target-baseline round trip."""

    value = clamp(progress, 0.0, 1.0)
    if value <= 0.5:
        return outbound_profile(2.0 * value)
    return outbound_profile(2.0 * (1.0 - value))


def motion_offset(
    profile,
    elapsed,
    duration,
    phase=0.0,
    frequency=0.25,
    chirp_end_frequency=1.0,
    smooth_chirp_return=False,
):
    """Return a normalized offset for one of the supported motion profiles."""

    if duration <= 0.0:
        raise ValueError("duration must be positive")
    progress = clamp(elapsed / duration, 0.0, 1.0)
    if profile in ("sweep", "minjerk"):
        return round_trip(progress, minimum_jerk_progress)
    if profile == "trapezoid":
        return round_trip(progress, trapezoid_progress)

    # A sin^2 window keeps continuous profiles at the captured baseline at
    # both ends, regardless of their phase or total number of cycles.
    window = math.sin(math.pi * progress) ** 2
    if profile == "sine":
        angle = 2.0 * math.pi * frequency * elapsed + phase
        return window * math.sin(angle)
    if profile == "chirp":
        if smooth_chirp_return:
            return sustained_chirp_offset(
                progress * duration, duration, phase, frequency, chirp_end_frequency
            )
        frequency_slope = (chirp_end_frequency - frequency) / duration
        angle = 2.0 * math.pi * (
            frequency * elapsed + 0.5 * frequency_slope * elapsed**2
        ) + phase
        return window * math.sin(angle)
    raise ValueError(f"unknown motion profile: {profile}")


def sustained_chirp_offset(elapsed, duration, phase, frequency, end_frequency):
    """Run the full frequency sweep, then brake and fade to baseline."""

    sweep_duration = duration * CHIRP_OSCILLATION_FRACTION
    frequency_slope = (end_frequency - frequency) / sweep_duration
    sweep_elapsed = min(elapsed, sweep_duration)
    angle = 2.0 * math.pi * (
        frequency * sweep_elapsed + 0.5 * frequency_slope * sweep_elapsed**2
    ) + phase
    if elapsed <= sweep_duration:
        # Smoothly enter from baseline, then keep the full amplitude even
        # through the final, fastest part of the frequency sweep.
        envelope = minimum_jerk_progress(elapsed / (0.2 * sweep_duration))
        return envelope * math.sin(angle)

    return_duration = duration - sweep_duration
    return_elapsed = elapsed - sweep_duration
    # Preserve phase velocity and acceleration at the join, then brake with
    # a quintic taper. Cap phase travel so the short return does not add many
    # oscillations when a high end frequency is requested.
    brake_duration = min(0.75 * return_duration, 0.5 / end_frequency)
    brake_progress = clamp(return_elapsed / brake_duration, 0.0, 1.0)
    # Integrals of (1 - minimum_jerk_progress(x)) and x times it.
    phase_travel = end_frequency * brake_duration * (
        brake_progress - 2.5 * brake_progress**4
        + 3.0 * brake_progress**5 - brake_progress**6
    ) + frequency_slope * brake_duration**2 * (
        0.5 * brake_progress**2 - 2.0 * brake_progress**5
        + 2.5 * brake_progress**6 - (6.0 / 7.0) * brake_progress**7
    )
    envelope = 1.0 - minimum_jerk_progress(return_elapsed / return_duration)
    return envelope * math.sin(angle + 2.0 * math.pi * phase_travel)


def profile_duration(profile, max_delta, frequency, chirp_end_frequency):
    """Choose nominal seconds from travel distance and profile complexity."""

    travel_time = abs(max_delta) / math.radians(REFERENCE_SPEED_DEG_PER_SECOND)
    if profile in ("sweep", "minjerk"):
        # Two minimum-jerk legs, each with peak normalized velocity 1.875.
        return max(1.0, 3.75 * travel_time)
    if profile == "trapezoid":
        return max(1.0, (8.0 / 3.0) * travel_time)
    if profile == "sine":
        # Keep at least two nominal cycles inside the amplitude window.
        return max(4.0, 2.0 / frequency, 3.75 * travel_time)
    if profile == "chirp":
        # Two cycles at the average nominal frequency, with time for settling.
        return max(4.0, 4.0 / (frequency + chirp_end_frequency), 3.75 * travel_time)
    raise ValueError(f"unknown motion profile: {profile}")


def parse_axes(value, axis_count):
    if value.strip().lower() == "all":
        return tuple(range(axis_count))
    try:
        axes = sorted(
            {int(item.strip()) - 1 for item in value.split(",") if item.strip()}
        )
    except ValueError as exc:
        raise ValueError("--axes must contain comma-separated integers") from exc
    if not axes or axes[0] < 0 or axes[-1] >= axis_count:
        raise ValueError(f"--axes values must be between 1 and {axis_count}")
    return tuple(axes)


def parse_profiles(value):
    if value.strip().lower() == "all":
        return PROFILE_NAMES
    profiles = tuple(
        dict.fromkeys(item.strip().lower() for item in value.split(",") if item.strip())
    )
    invalid = [profile for profile in profiles if profile not in PROFILE_NAMES]
    if not profiles or invalid:
        choices = ",".join(PROFILE_NAMES)
        suffix = f"; unknown: {','.join(invalid)}" if invalid else ""
        raise ValueError(f"--profiles must use {choices}, or all{suffix}")
    return profiles


def bounded_delta(baseline, lower, upper, amplitude, margin, preferred_sign):
    """Choose a signed motion delta that stays inside the requested margin."""

    if lower >= upper:
        raise ValueError("joint lower limit must be smaller than upper limit")
    if amplitude <= 0.0 or margin < 0.0:
        raise ValueError("amplitude must be positive and margin cannot be negative")
    positive_room = max(0.0, upper - margin - baseline)
    negative_room = max(0.0, baseline - (lower + margin))
    direction = 1.0 if preferred_sign >= 0.0 else -1.0
    preferred_room = positive_room if direction > 0.0 else negative_room
    other_room = negative_room if direction > 0.0 else positive_room
    # Preserve the preferred semantic direction when it can provide the full
    # requested excursion.  Otherwise use the roomier side so a pose near a
    # limit still produces a useful test response.
    if preferred_room < amplitude and other_room > preferred_room:
        direction *= -1.0
        preferred_room = other_room
    return direction * min(amplitude, preferred_room)


def add_common_arguments(parser, axis_count, default_platform_root):
    parser.add_argument(
        "--platform-root",
        type=Path,
        default=Path(default_platform_root),
        help="Root containing Python protobuf bindings (default: this repository)",
    )
    parser.add_argument("--connect", default=DEFAULT_ZENOH_ENDPOINT)
    parser.add_argument("--rate", type=float, default=100.0)
    parser.add_argument(
        "--axes",
        default="all",
        help=f"Comma-separated 1-based axes to move, or all (1-{axis_count})",
    )
    parser.add_argument(
        "--profiles",
        default="all",
        help="Comma-separated sweep,sine,trapezoid,minjerk,chirp, or all",
    )
    parser.add_argument(
        "--speed",
        type=float,
        default=1.0,
        help="Playback speed for all motions; 2 is twice as fast, 0.5 is half speed",
    )
    parser.add_argument(
        "--frequency",
        type=float,
        default=0.25,
        help="Starting sine/chirp frequency in Hz at --speed 1",
    )
    parser.add_argument(
        "--chirp-end-frequency",
        type=float,
        default=1.0,
        help="Final chirp frequency in Hz at --speed 1",
    )
    parser.add_argument(
        "--scale",
        dest="amplitude_scale",
        metavar="SCALE",
        type=float,
        default=1.0,
        help="Scale configured amplitudes; any positive value, bounded by joint limits",
    )
    parser.add_argument("--limit-margin-deg", type=float, default=1.0)
    parser.add_argument("--state-timeout", type=float, default=8.0)
    parser.add_argument("--stale-timeout", type=float, default=2.0)
    parser.add_argument("--minimum-response-deg", type=float, default=0.2)
    parser.add_argument(
        "--return-timeout",
        type=float,
        default=8.0,
        help="Maximum seconds to wait for measured joints to return to baseline",
    )
    parser.add_argument(
        "--tolerance",
        dest="return_tolerance_deg",
        metavar="TOLERANCE",
        type=float,
        default=0.5,
        help="Maximum measured joint error in degrees for arrival checks",
    )
    parser.add_argument("--no-verify", action="store_true")


def validate_args(args, axis_count):
    numeric_values = (
        args.rate,
        args.speed,
        args.frequency,
        args.chirp_end_frequency,
        args.amplitude_scale,
        args.limit_margin_deg,
        args.state_timeout,
        args.stale_timeout,
        args.minimum_response_deg,
        args.return_timeout,
        args.return_tolerance_deg,
    )
    if not all(math.isfinite(value) for value in numeric_values):
        raise ValueError("all numeric motion arguments must be finite")
    if args.rate <= 0.0 or args.speed <= 0.0:
        raise ValueError("--rate and --speed must be positive")
    if args.frequency <= 0.0 or args.chirp_end_frequency <= 0.0:
        raise ValueError("profile frequencies must be positive")
    if args.amplitude_scale <= 0.0:
        raise ValueError("--scale must be positive")
    if args.limit_margin_deg < 0.0:
        raise ValueError("--limit-margin-deg cannot be negative")
    if args.state_timeout <= 0.0 or args.stale_timeout <= 0.0:
        raise ValueError("state timeouts must be positive")
    if args.minimum_response_deg <= 0.0:
        raise ValueError("--minimum-response-deg must be positive")
    if args.return_timeout <= 0.0 or args.return_tolerance_deg <= 0.0:
        raise ValueError(
            "--return-timeout and --tolerance must be positive"
        )
    return parse_axes(args.axes, axis_count), parse_profiles(args.profiles)


def load_dependencies(platform_root):
    root = platform_root.expanduser().resolve()
    candidates = (
        (root / "proto" / "platform", "north_pb2"),
        (root / "proto", "north_pb2"),
        (root / "sharpa-proto", "north_pb2"),
    )
    generated = next(
        ((path, name) for path, name in candidates if (path / f"{name}.py").is_file()),
        None,
    )
    if generated is None:
        raise RuntimeError(
            f"north_pb2.py was not found under {root / 'proto/platform'}, "
            f"{root / 'proto'} or {root / 'sharpa-proto'}. The default bindings "
            "are distributed with sharpa_mujoco in proto/platform/north_pb2.py; "
            "restore that file or set "
            "--platform-root to a directory containing generated Platform bindings."
        )
    generated_path, module_name = generated
    sys.path.insert(0, str(generated_path))
    try:
        proto = importlib.import_module(module_name)
    except ImportError as exc:
        raise RuntimeError(
            f"cannot import {module_name}: {exc}; "
            "run: python -m pip install -r requirements.txt"
        ) from exc
    try:
        import zenoh
    except ImportError as exc:
        raise RuntimeError(
            "Python Zenoh is missing; run: python3 -m pip install -r requirements.txt"
        ) from exc
    return zenoh, proto


def open_session(zenoh, endpoint):
    config = zenoh.Config.from_file(str(ZENOH_CLIENT_CONFIG))
    config.insert_json5("connect/endpoints", json.dumps([endpoint]))
    print(f"[Zenoh] connecting to {endpoint}")
    return zenoh.open(config)


class JointStateListener:
    def __init__(self, message_type, axis_count):
        self._message_type = message_type
        self._axis_count = axis_count
        self._condition = threading.Condition()
        self._positions = [0.0] * axis_count
        self._count = 0
        self._last_received = 0.0
        self._velocities = []
        self._torques = []

    def on_sample(self, sample):
        try:
            state = self._message_type()
            state.ParseFromString(bytes(sample.payload))
            positions = list(state.joints.position)
            if len(positions) < self._axis_count:
                return
            if not all(math.isfinite(value) for value in positions[: self._axis_count]):
                return
            with self._condition:
                self._positions = positions[: self._axis_count]
                self._velocities = list(getattr(state.joints, "velocity", ()))[: self._axis_count]
                self._torques = list(getattr(state.joints, "torque", ()))[: self._axis_count]
                self._count += 1
                self._last_received = time.monotonic()
                self._condition.notify_all()
        except Exception:
            return

    def snapshot(self):
        with self._condition:
            return list(self._positions), self._count

    def diagnostics(self):
        """Copy optional feedback fields without changing the position API."""
        with self._condition:
            return {
                "velocity": list(self._velocities),
                "torque": list(self._torques),
                "age": max(0.0, time.monotonic() - self._last_received),
            }

    def wait_for_newer_than(self, count, timeout):
        deadline = time.monotonic() + timeout
        with self._condition:
            while self._count <= count:
                remaining = deadline - time.monotonic()
                if remaining <= 0.0:
                    return False
                self._condition.wait(remaining)
            return True

    def is_fresh(self, max_age):
        with self._condition:
            return (
                self._last_received > 0.0
                and time.monotonic() - self._last_received <= max_age
            )


class MotionRunner:
    """Publish bounded profiles and collect state response for several channels."""

    def __init__(
        self,
        session,
        proto,
        channels,
        joint_names,
        limits_deg,
        amplitudes_deg,
        preferred_directions,
        args,
        frame_id,
        smooth_chirp_return=False,
        enforce_bidirectional_limit_margin=False,
        unidirectional_continuous_profiles=False,
    ):
        axis_count = len(joint_names)
        if not (
            len(limits_deg)
            == len(amplitudes_deg)
            == len(preferred_directions)
            == axis_count
        ):
            raise ValueError("joint metadata lengths do not match")
        self._session = session
        self._proto = proto
        self._channels = tuple(channels)
        self._joint_names = tuple(joint_names)
        self._limits = tuple(
            (math.radians(lower), math.radians(upper))
            for lower, upper in limits_deg
        )
        self._amplitudes = tuple(
            math.radians(value * args.amplitude_scale) for value in amplitudes_deg
        )
        self._directions = tuple(preferred_directions)
        self._args = args
        self._frame_id = frame_id
        self._smooth_chirp_return = smooth_chirp_return
        self._enforce_bidirectional_limit_margin = (
            enforce_bidirectional_limit_margin
        )
        self._unidirectional_continuous_profiles = (
            unidirectional_continuous_profiles
        )
        self._sequence = 0
        self._baselines = None
        self._last_targets = None
        self._has_published = False
        self._hold_callback = None
        self._listeners = {
            channel.label: JointStateListener(
                getattr(proto, channel.state_type), axis_count
            )
            for channel in self._channels
        }
        self._subscribers = {
            channel.label: session.declare_subscriber(
                channel.state_topic, self._listeners[channel.label].on_sample
            )
            for channel in self._channels
        }

    def counts(self):
        return {
            label: listener.snapshot()[1]
            for label, listener in self._listeners.items()
        }

    def wait_for_baselines(self, counts_before=None):
        counts_before = counts_before or {
            channel.label: 0 for channel in self._channels
        }
        baselines = {}
        for channel in self._channels:
            print(
                f"[State] waiting up to {self._args.state_timeout:.1f}s for "
                f"fresh {channel.state_topic}"
            )
            if not self._listeners[channel.label].wait_for_newer_than(
                counts_before.get(channel.label, 0), self._args.state_timeout
            ):
                raise RuntimeError(
                    f"no fresh {channel.label} state received; check zenohd, "
                    "Platform, MuJoCo, device configuration, and endpoints"
                )
            positions, count = self._listeners[channel.label].snapshot()
            baselines[channel.label] = positions
            print(
                f"[State][{channel.label} #{count}] baseline deg: "
                f"{self._format_positions(positions)}"
            )
        self._baselines = baselines
        self._last_targets = {
            label: list(positions) for label, positions in baselines.items()
        }
        return baselines

    def run(self, axes, profiles, sweep_groups=None):
        """Run the requested profiles, optionally grouping axes for sweeps.

        ``sweep_groups`` maps a display label to axes that should move together.
        Selected axes not included in a group retain the default one-axis sweep.
        """

        if self._baselines is None:
            raise RuntimeError("state baselines have not been captured")
        grouped_sweeps = self._group_sweep_axes(axes, sweep_groups)
        segment_count = len(grouped_sweeps) if "sweep" in profiles else 0
        segment_count += sum(profile != "sweep" for profile in profiles)
        estimated = sum(
            self._segment_duration("sweep", group_axes)
            for _, group_axes in grouped_sweeps
        ) if "sweep" in profiles else 0.0
        estimated += sum(
            self._segment_duration(profile, axes)
            for profile in profiles if profile != "sweep"
        )
        print(
            f"[Test] axes={','.join(str(axis + 1) for axis in axes)} "
            f"profiles={','.join(profiles)} segments={segment_count} "
            f"speed={self._args.speed:g} estimated={estimated:.1f}s"
        )

        observed_min = {
            label: list(positions) for label, positions in self._baselines.items()
        }
        observed_max = {
            label: list(positions) for label, positions in self._baselines.items()
        }

        for profile in profiles:
            if profile == "sweep":
                for label, group_axes in grouped_sweeps:
                    axis_numbers = ",".join(str(axis + 1) for axis in group_axes)
                    joint_names = ", ".join(
                        self._joint_names[axis] for axis in group_axes
                    )
                    print(
                        f"[Profile] sweep {label}: axes {axis_numbers} "
                        f"({joint_names})"
                    )
                    self._drive_segment(
                        profile, group_axes, observed_min, observed_max
                    )
            else:
                print(
                    f"[Profile] {profile}: coordinated motion on "
                    f"{len(axes)} axes"
                )
                self._drive_segment(profile, axes, observed_min, observed_max)

        if not self._args.no_verify:
            threshold = math.radians(self._args.minimum_response_deg)
            failures = []
            for channel in self._channels:
                label = channel.label
                for axis in axes:
                    response = observed_max[label][axis] - observed_min[label][axis]
                    if response < threshold:
                        failures.append(
                            f"{label}:{axis + 1}({math.degrees(response):.2f}deg)"
                        )
            if failures:
                raise RuntimeError(
                    "insufficient state response on joints " + ",".join(failures)
                )
            print("[Verify] PASS: all selected joints reported motion")
        print("[Test] completed")

    def _group_sweep_axes(self, axes, sweep_groups):
        if sweep_groups is None:
            return tuple(
                (f"axis {axis + 1}", (axis,)) for axis in axes
            )

        selected = set(axes)
        assigned = set()
        grouped = []
        for label, candidates in sweep_groups.items():
            group_axes = tuple(axis for axis in candidates if axis in selected)
            duplicate = assigned.intersection(group_axes)
            if duplicate:
                duplicate_axis = min(duplicate)
                raise ValueError(
                    f"sweep axis {duplicate_axis + 1} belongs to multiple groups"
                )
            if group_axes:
                grouped.append((label, group_axes))
                assigned.update(group_axes)

        for axis in axes:
            if axis not in assigned:
                grouped.append((f"axis {axis + 1}", (axis,)))
        return tuple(grouped)

    def return_to_baseline(self, duration=1.0, tolerance_deg=None):
        """Smoothly command baseline and verify measured joints arrived."""

        if (
            self._baselines is None
            or self._last_targets is None
            or not self._has_published
        ):
            return True
        starts = {
            label: list(values) for label, values in self._last_targets.items()
        }
        duration = self._transition_duration(starts, self._baselines, duration)
        tolerance = math.radians(
            self._args.return_tolerance_deg if tolerance_deg is None else tolerance_deg
        )
        self._on_return_started(duration, tolerance)
        started = time.monotonic()
        tick = 0
        interval = 1.0 / self._args.rate
        while True:
            elapsed = time.monotonic() - started
            progress = clamp(elapsed / duration, 0.0, 1.0)
            blend = minimum_jerk_progress(progress)
            targets = {
                label: [
                    current + (baseline - current) * blend
                    for current, baseline in zip(starts[label], self._baselines[label])
                ]
                for label in starts
            }
            self._publish(targets)
            if progress >= 1.0:
                break
            tick += 1
            time.sleep(max(0.0, started + tick * interval - time.monotonic()))
        for _ in range(max(2, int(0.1 * self._args.rate))):
            self._publish(self._baselines)
            time.sleep(interval)

        wait_started = time.monotonic()
        deadline = wait_started + self._args.return_timeout
        while True:
            all_arrived = True
            max_error = 0.0
            feedback = {}
            for channel in self._channels:
                listener = self._listeners[channel.label]
                if not listener.is_fresh(self._args.stale_timeout):
                    print(
                        f"[Warning] {channel.label} state became stale while "
                        "returning to baseline",
                        file=sys.stderr,
                    )
                    return False
                positions, count = listener.snapshot()
                feedback[channel.label] = (positions, count)
                errors = [
                    abs(position - baseline)
                    for position, baseline in zip(
                        positions, self._baselines[channel.label]
                    )
                ]
                channel_error = max(errors, default=0.0)
                max_error = max(max_error, channel_error)
                if channel_error > tolerance:
                    all_arrived = False
            now = time.monotonic()
            status = "arrived" if all_arrived else "timeout" if now >= deadline else "waiting"
            self._on_return_feedback(now - wait_started, tolerance, feedback, status)
            if all_arrived:
                print(
                    "[Return] measured joints reached baseline "
                    f"(max error {math.degrees(max_error):.2f}deg)"
                )
                return True
            if now >= deadline:
                print(
                    "[Warning] baseline return timed out; "
                    f"max measured error {math.degrees(max_error):.2f}deg",
                    file=sys.stderr,
                )
                return False
            self._publish(self._baselines)
            time.sleep(interval)

    def _on_return_started(self, duration, tolerance):
        """Optional driver diagnostics; do not change return decisions."""

    def _on_return_feedback(self, elapsed, tolerance, feedback, status):
        """Optional driver diagnostics for the measured arrival check."""

    def return_to_zero(self, duration=4.0, tolerance_deg=None):
        """Return all channel joints to zero, regardless of the captured pose."""
        if self._baselines is None or not self._has_published:
            return True
        if any(not lower <= 0.0 <= upper for lower, upper in self._limits):
            raise ValueError("zero pose is outside the configured joint limits")
        self._baselines = {
            label: [0.0] * len(values) for label, values in self._baselines.items()
        }
        return self.return_to_baseline(duration, tolerance_deg=tolerance_deg)

    def hold_last_targets(self):
        """Keep publishing the most recent targets until interrupted."""

        if self._last_targets is None or not self._has_published:
            raise RuntimeError("no joint targets are available to hold")
        targets = {
            label: list(positions)
            for label, positions in self._last_targets.items()
        }
        interval = 1.0 / self._args.rate
        started = time.monotonic()
        tick = 0
        while True:
            for channel in self._channels:
                if not self._listeners[channel.label].is_fresh(
                    self._args.stale_timeout
                ):
                    raise RuntimeError(
                        f"{channel.label} state became stale while holding position"
                    )
            self._publish(targets)
            tick += 1
            time.sleep(max(0.0, started + tick * interval - time.monotonic()))

    def close(self):
        for subscriber in self._subscribers.values():
            subscriber.undeclare()

    def _transition_duration(self, starts, targets, nominal_duration):
        max_delta = max(
            abs(target - current)
            for label, positions in starts.items()
            for current, target in zip(positions, targets[label])
        )
        # Keep smooth transitions within Platform's inter-frame guard while
        # retaining their full travel even when a high speed is requested.
        return max(
            nominal_duration / self._args.speed,
            1.875 * max_delta
            / (math.radians(MAX_COMMAND_STEP_DEG) * self._args.rate),
        )

    def _profile_targets(self, profile, axes, deltas, duration, elapsed):
        targets = {}
        axis_order = {axis: offset for offset, axis in enumerate(axes)}
        for channel in self._channels:
            baseline = self._baselines[channel.label]
            target = list(baseline)
            for axis in axes:
                phase = 2.0 * math.pi * axis_order[axis] / max(1, len(axes))
                offset = motion_offset(
                    profile, elapsed, duration, phase=phase,
                    frequency=self._args.frequency,
                    chirp_end_frequency=self._args.chirp_end_frequency,
                    smooth_chirp_return=self._smooth_chirp_return,
                )
                if self._unidirectional_continuous_profiles and profile in ("sine", "chirp"):
                    progress = clamp(elapsed / duration, 0.0, 1.0)
                    offset = 0.5 * (offset + math.sin(math.pi * progress) ** 2)
                lower, upper = self._limits[axis]
                if self._enforce_bidirectional_limit_margin and profile in ("sine", "chirp"):
                    margin = math.radians(self._args.limit_margin_deg)
                    lower = min(lower + margin, baseline[axis])
                    upper = max(upper - margin, baseline[axis])
                target[axis] = clamp(
                    baseline[axis] + deltas[channel.label][axis] * offset,
                    lower, upper,
                )
            targets[channel.label] = target
        return targets

    def _profile_speed(self, profile, axes=()):
        return self._args.speed

    def _plan_segment(self, profile, axes, deltas=None):
        deltas = self._calculate_deltas(axes) if deltas is None else deltas
        max_delta = max(
            abs(values[axis]) for values in deltas.values() for axis in axes
        )
        nominal_duration = profile_duration(
            profile, max_delta, self._args.frequency, self._args.chirp_end_frequency
        )
        separate_chirp_return = profile == "chirp" and self._smooth_chirp_return
        if separate_chirp_return:
            nominal_duration /= CHIRP_OSCILLATION_FRACTION
        # Sample the actual path, including phase, limit clipping and the
        # smooth chirp return. Resolve oscillations and sub-degree travel well
        # below the command-frame interval; include the return join exactly.
        samples = max(256, math.ceil(32.0 * max_delta / math.radians(MAX_COMMAND_STEP_DEG)))
        if profile in ("sine", "chirp"):
            frequency = self._args.frequency
            if profile == "chirp":
                frequency = max(frequency, self._args.chirp_end_frequency)
            samples = max(samples, math.ceil(128.0 * nominal_duration * frequency))
        # Six subdivisions place both the midpoint and the 5/6 return join
        # on sample boundaries.
        subdivisions = 6 if separate_chirp_return else 2
        samples += -samples % subdivisions
        times = [0.0]
        points = [self._profile_targets(profile, axes, deltas, nominal_duration, 0.0)]
        requested_step_time = nominal_duration / samples / self._profile_speed(profile, axes)
        max_velocity = math.radians(MAX_COMMAND_STEP_DEG) * self._args.rate
        limited_seconds = 0.0
        for index in range(1, samples + 1):
            target = self._profile_targets(
                profile, axes, deltas, nominal_duration,
                nominal_duration * index / samples,
            )
            travel = max(
                abs(end - start)
                for label, positions in points[-1].items()
                for start, end in zip(positions, target[label])
            )
            step_time = max(requested_step_time, travel / max_velocity)
            times.append(times[-1] + step_time)
            points.append(target)
            limited_seconds += step_time - requested_step_time
        return_start_time = times[5 * samples // 6] if separate_chirp_return else 0.0
        return ProfilePlan(tuple(times), tuple(points), nominal_duration, limited_seconds,
                           return_start_time)

    def _segment_duration(self, profile, axes, deltas=None):
        return self._plan_segment(profile, axes, deltas).duration

    def _drive_segment(self, profile, axes, observed_min, observed_max):
        plan = self._plan_segment(profile, axes)
        duration = plan.duration
        detail = ""
        if plan.limited_seconds > 1e-6:
            detail = f" (+{plan.limited_seconds:.2f}s for local command step limits)"
        print(f"[Timing] {profile}: {duration:.2f}s at requested speed {self._profile_speed(profile, axes):g}{detail}")
        if plan.return_start_time:
            print(f"[Chirp] continuous oscillation: {plan.return_start_time:.2f}s; "
                  f"final smooth return: {duration - plan.return_start_time:.2f}s")
        return_reported = False
        interval = 1.0 / self._args.rate
        started = time.monotonic()
        next_report = started + 1.0
        previous_wall_time = started
        elapsed = 0.0

        while True:
            now = time.monotonic()
            # A delayed scheduler tick must not skip ahead across several
            # bounded command intervals and turn a safe path into a jump.
            elapsed = min(
                elapsed + min(max(0.0, now - previous_wall_time), interval),
                duration,
            )
            previous_wall_time = now
            if duration - elapsed < 1e-10:
                elapsed = duration
            if (
                plan.return_start_time
                and elapsed >= plan.return_start_time
                and not return_reported
            ):
                print("[Chirp] frequency sweep completed; starting final smooth return")
                return_reported = True
            targets = plan.at(elapsed)
            for channel in self._channels:
                listener = self._listeners[channel.label]
                if not listener.is_fresh(self._args.stale_timeout):
                    raise RuntimeError(
                        f"{channel.label} state became stale; stopping test commands"
                    )
                actual, _ = listener.snapshot()
                for axis in axes:
                    observed_min[channel.label][axis] = min(
                        observed_min[channel.label][axis], actual[axis]
                    )
                    observed_max[channel.label][axis] = max(
                        observed_max[channel.label][axis], actual[axis]
                    )

            self._publish(targets)
            if now >= next_report:
                for channel in self._channels:
                    actual, count = self._listeners[channel.label].snapshot()
                    selected = " ".join(
                        f"J{axis + 1}={math.degrees(actual[axis]):.1f}deg"
                        for axis in axes
                    )
                    print(f"[State][{channel.label} #{count}] {selected}")
                next_report = now + 1.0
            if elapsed >= duration:
                break
            time.sleep(max(0.0, now + interval - time.monotonic()))

    def _calculate_deltas(self, axes):
        margin = math.radians(self._args.limit_margin_deg)
        result = {}
        for channel in self._channels:
            channel_deltas = [0.0] * len(self._joint_names)
            side_sign = (
                -1.0
                if channel.side == "right" and channel.mirror_right
                else 1.0
            )
            for axis in axes:
                baseline = self._baselines[channel.label][axis]
                lower, upper = self._limits[axis]
                preferred_sign = self._directions[axis] * side_sign
                channel_deltas[axis] = bounded_delta(
                    baseline,
                    lower,
                    upper,
                    self._amplitudes[axis],
                    margin,
                    preferred_sign,
                )
                if channel_deltas[axis] == 0.0:
                    raise RuntimeError(
                        f"{channel.label} axis {axis + 1} baseline has no safe "
                        "motion room inside its configured limits"
                    )
            result[channel.label] = channel_deltas
        return result

    def _publish(self, targets):
        if self._hold_callback is not None:
            self._hold_callback(self)
        self._sequence += 1
        for channel in self._channels:
            positions = targets[channel.label]
            command = getattr(self._proto, channel.command_type)()
            now = time.time()
            command.header.stamp.seconds = int(now)
            command.header.stamp.nanos = int((now - int(now)) * 1e9)
            command.header.seq = self._sequence
            command.header.frame_id = self._frame_id
            if channel.side:
                command.side = (
                    self._proto.SIDE_LEFT
                    if channel.side == "left"
                    else self._proto.SIDE_RIGHT
                )
            command.mode = self._proto.SPACE_MODE_JOINT_SPACE
            command.joints.position.extend(positions)
            if hasattr(command.joints, "velocity"):
                command.joints.velocity.extend([0.0] * len(positions))
            if hasattr(command.joints, "torque"):
                command.joints.torque.extend([0.0] * len(positions))
            self._session.put(channel.action_topic, command.SerializeToString())
        self._last_targets = {
            label: list(positions) for label, positions in targets.items()
        }
        self._has_published = True

    @staticmethod
    def _format_positions(positions):
        return "[" + ", ".join(
            f"{math.degrees(value):6.1f}" for value in positions
        ) + "]"


def default_platform_root(script_file):
    script_path = Path(script_file).resolve()
    repo_root = next(
        (
            parent
            for parent in script_path.parents
            if (parent / "sharpa_mujoco.conf").is_file()
        ),
        None,
    )
    if repo_root is None:
        raise RuntimeError(
            f"cannot locate the sharpa_mujoco repository from {script_path}"
        )
    return repo_root


def run_main(function):
    try:
        return function()
    except KeyboardInterrupt:
        print("\n[Test] interrupted; returning to captured baseline")
        return 130
    except (RuntimeError, ValueError) as exc:
        print(f"[Error] {exc}", file=sys.stderr)
        return 1
