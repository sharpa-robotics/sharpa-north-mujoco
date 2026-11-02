#!/usr/bin/env python3
"""Perform a fixed squat, raised-arms, and thumbs-up sequence.

Start zenohd, sharpa_platform, and sharpa_mujoco_simulator first.  The example
captures the live pose, squats the torso while raising the selected arms, forms
a thumbs-up with each selected hand, and holds the final pose until Ctrl+C.  It
then returns all commanded joints to their captured starting pose and exits
with the torso and arms still enabled, letting Platform hold the final targets.
"""

import argparse
import math
import os
import sys
import time
from dataclasses import dataclass
from pathlib import Path


# Reuse the existing device limits and protocol helpers without changing the
# motion drivers themselves.
MOTION_DIR = Path(__file__).resolve().parent / "motion"
sys.path.insert(0, str(MOTION_DIR))

from arm_motion_driver import (  # noqa: E402
    JOINT_LIMITS_DEG as ARM_LIMITS_DEG,
    JOINT_NAMES as ARM_JOINT_NAMES,
    call_enable_service,
    call_reset_service,
    enable_with_recovery,
)
from hand_motion_driver import (  # noqa: E402
    JOINT_LIMITS_DEG as HAND_LIMITS_DEG,
    JOINT_NAMES as HAND_JOINT_NAMES,
)
from kinematic_motion import (  # noqa: E402
    DEFAULT_ZENOH_ENDPOINT,
    JointStateListener,
    clamp,
    default_platform_root,
    load_dependencies,
    minimum_jerk_progress,
    open_session,
    run_main,
)
from torso_motion_driver import (  # noqa: E402
    JOINT_LIMITS_DEG as TORSO_LIMITS_DEG,
    JOINT_NAMES as TORSO_JOINT_NAMES,
    make_enable,
)


COMMAND_MARGIN_DEG = 2.0
# Platform protects the torso against large inter-frame steps.  Transition
# duration is lengthened automatically when needed to stay below this value.
MAX_COMMAND_STEP_DEG = 1.25

# A moderate squat: the first three lower-body joints lower the torso while the
# waist roll/yaw and neck remain centered.
SQUAT_POSE_DEG = (
    45.0,
    -75.0,
    40.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
)

# The same logical joint pose is mirrored by the left/right arm models.  The
# elbow keeps a relaxed bend so the arm does not look fully extended.  J5-J7
# remain at zero so the wrist is not rolled or folded to manufacture the
# gesture.
RAISED_ARM_POSE_DEG = (
    45.0,
    -7.0,
    -5.0,
    60.0,
    0.0,
    0.0,
    0.0,
)

# Thumb joints remain mostly extended while the other four fingers form a
# relaxed fist.  The pose keeps at least COMMAND_MARGIN_DEG from every limit.
THUMBS_UP_POSE_DEG = (
    8.0, -15.0, -17.0, -4.0, 12.0,  # thumb
    65.0, 0.0, 75.0, 55.0,  # index
    70.0, 0.0, 80.0, 60.0,  # middle
    70.0, 0.0, 80.0, 60.0,  # ring
    5.0, 65.0, 0.0, 75.0, 55.0,  # pinky
)


@dataclass(frozen=True)
class DeviceSpec:
    label: str
    action_topic: str
    state_topic: str
    command_type: str
    state_type: str
    joint_names: tuple
    limits_deg: tuple
    side: str = ""


def selected_sides(side):
    return ("left", "right") if side == "both" else (side,)


def device_specs(side):
    sides = selected_sides(side)
    return (
        DeviceSpec(
            label="torso",
            action_topic="action/torso",
            state_topic="state/torso",
            command_type="TorsoCommand",
            state_type="TorsoState",
            joint_names=TORSO_JOINT_NAMES,
            limits_deg=TORSO_LIMITS_DEG,
        ),
        *(
            DeviceSpec(
                label=f"{arm_side}_arm",
                action_topic=f"action/{arm_side}_arm",
                state_topic=f"state/{arm_side}_arm",
                command_type="ArmCommand",
                state_type="ArmState",
                joint_names=ARM_JOINT_NAMES,
                limits_deg=ARM_LIMITS_DEG,
                side=arm_side,
            )
            for arm_side in sides
        ),
        *(
            DeviceSpec(
                label=f"{hand_side}_hand",
                action_topic=f"action/{hand_side}_hand",
                state_topic=f"state/{hand_side}_hand",
                command_type="HandCommand",
                state_type="HandState",
                joint_names=HAND_JOINT_NAMES,
                limits_deg=HAND_LIMITS_DEG,
                side=hand_side,
            )
            for hand_side in sides
        ),
    )


def parse_args():
    default_root = os.environ.get(
        "SHARPA_PLATFORM_ROOT", str(default_platform_root(__file__))
    )
    parser = argparse.ArgumentParser(
        description=(
            "Squat while raising both arms, then form a deterministic "
            "thumbs-up pose without crossing configured joint limits."
        )
    )
    parser.add_argument(
        "--platform-root",
        type=Path,
        default=Path(default_root),
        help="Root containing Python protobuf bindings (default: this repository)",
    )
    parser.add_argument("--connect", default=DEFAULT_ZENOH_ENDPOINT)
    parser.add_argument(
        "--side",
        choices=("left", "right", "both"),
        default="both",
        help="Arms and hands used for the gesture (default: both)",
    )
    parser.add_argument(
        "--rate", type=float, default=50.0, help="Command rate in Hz"
    )
    parser.add_argument(
        "--transition-duration",
        type=float,
        default=4.0,
        help="Minimum duration of each smooth pose transition in seconds",
    )
    parser.add_argument(
        "--hold-duration",
        type=float,
        default=0.0,
        help="Seconds to hold the thumbs-up; 0 means until Ctrl+C",
    )
    parser.add_argument("--state-timeout", type=float, default=8.0)
    parser.add_argument("--stale-timeout", type=float, default=2.0)
    parser.add_argument("--return-timeout", type=float, default=8.0)
    parser.add_argument(
        "--tolerance", dest="return_tolerance_deg", metavar="TOLERANCE",
        type=float, default=1.0, help="Maximum return error in degrees",
    )
    args = parser.parse_args()
    validate_args(args)
    return args


def validate_args(args):
    values = (
        args.rate,
        args.transition_duration,
        args.hold_duration,
        args.state_timeout,
        args.stale_timeout,
        args.return_timeout,
        args.return_tolerance_deg,
    )
    if not all(math.isfinite(value) for value in values):
        raise ValueError("all numeric arguments must be finite")
    if args.rate <= 0.0 or args.transition_duration <= 0.0:
        raise ValueError("--rate and --transition-duration must be positive")
    if args.hold_duration < 0.0:
        raise ValueError("--hold-duration cannot be negative")
    if args.state_timeout <= 0.0 or args.stale_timeout <= 0.0:
        raise ValueError("state timeouts must be positive")
    if args.return_timeout <= 0.0 or args.return_tolerance_deg <= 0.0:
        raise ValueError("return timeout and tolerance must be positive")


class WholeBodyController:
    def __init__(self, session, proto, specs, args):
        self._session = session
        self._proto = proto
        self._specs = tuple(specs)
        self._args = args
        self._sequence = 0
        self._baselines = None
        self._last_targets = None
        self._has_published = False
        self._listeners = {
            spec.label: JointStateListener(
                getattr(proto, spec.state_type), len(spec.joint_names)
            )
            for spec in self._specs
        }
        self._subscribers = {
            spec.label: session.declare_subscriber(
                spec.state_topic, self._listeners[spec.label].on_sample
            )
            for spec in self._specs
        }

    def counts(self):
        return {
            label: listener.snapshot()[1]
            for label, listener in self._listeners.items()
        }

    def capture_baselines(self, counts_before):
        baselines = {}
        for spec in self._specs:
            print(
                f"[State] waiting up to {self._args.state_timeout:.1f}s for "
                f"fresh {spec.state_topic}"
            )
            listener = self._listeners[spec.label]
            if not listener.wait_for_newer_than(
                counts_before.get(spec.label, 0), self._args.state_timeout
            ):
                raise RuntimeError(
                    f"no fresh {spec.label} state received; check zenohd, "
                    "Platform, MuJoCo, device configuration, and endpoints"
                )
            positions, count = listener.snapshot()
            baselines[spec.label] = self._validate_baseline(spec, positions)
            print(
                f"[State][{spec.label} #{count}] captured "
                f"{len(positions)} joint positions"
            )
        self._baselines = baselines
        self._last_targets = self._copy_targets(baselines)

    def run_sequence(self):
        if self._baselines is None:
            raise RuntimeError("joint baselines have not been captured")
        raised_arm, thumbs_up = self._stage_targets()
        self._move_to("squat and raise arms", raised_arm)
        self._move_to("thumbs up", thumbs_up)
        self._hold(thumbs_up)

    def return_to_baseline(self):
        if (
            self._baselines is None
            or self._last_targets is None
            or not self._has_published
        ):
            return True
        self._move_to("return to starting pose", self._baselines)
        for _ in range(max(2, int(0.2 * self._args.rate))):
            self._publish(self._baselines)
            time.sleep(1.0 / self._args.rate)
        return self._wait_for_return()

    def close(self):
        for subscriber in self._subscribers.values():
            subscriber.undeclare()

    def _stage_targets(self):
        raised_arm = self._copy_targets(self._baselines)
        raised_arm["torso"] = self._fixed_pose(
            self._spec_for("torso"), SQUAT_POSE_DEG
        )
        for spec in self._specs:
            if spec.label.endswith("_arm"):
                raised_arm[spec.label] = self._fixed_pose(
                    spec, RAISED_ARM_POSE_DEG
                )

        thumbs_up = self._copy_targets(raised_arm)
        for spec in self._specs:
            if spec.label.endswith("_hand"):
                thumbs_up[spec.label] = self._fixed_pose(
                    spec, THUMBS_UP_POSE_DEG
                )
        return raised_arm, thumbs_up

    def _move_to(self, stage, targets):
        starts = self._copy_targets(self._last_targets)
        duration = self._transition_duration(starts, targets)
        print(f"[Pose] {stage}: {duration:.2f}s")
        started = time.monotonic()
        tick = 0
        interval = 1.0 / self._args.rate
        while True:
            elapsed = time.monotonic() - started
            progress = clamp(elapsed / duration, 0.0, 1.0)
            blend = minimum_jerk_progress(progress)
            command_targets = {
                label: [
                    start + (target - start) * blend
                    for start, target in zip(starts[label], targets[label])
                ]
                for label in starts
            }
            self._ensure_fresh()
            self._publish(command_targets)
            if progress >= 1.0:
                return
            tick += 1
            time.sleep(max(0.0, started + tick * interval - time.monotonic()))

    def _hold(self, targets):
        duration = self._args.hold_duration
        duration_text = f"for {duration:.1f}s" if duration > 0.0 else "until Ctrl+C"
        print(f"[Pose] holding thumbs-up {duration_text}")
        started = time.monotonic()
        tick = 0
        interval = 1.0 / self._args.rate
        next_report = started + 5.0
        while duration == 0.0 or time.monotonic() - started < duration:
            self._ensure_fresh()
            self._publish(targets)
            now = time.monotonic()
            if now >= next_report:
                print(f"[Pose] thumbs-up held for {now - started:.1f}s")
                next_report = now + 5.0
            tick += 1
            time.sleep(max(0.0, started + tick * interval - time.monotonic()))
        print("[Pose] hold completed")

    def _transition_duration(self, starts, targets):
        max_distance_deg = max(
            (
                abs(math.degrees(target - start))
                for label in starts
                for start, target in zip(starts[label], targets[label])
            ),
            default=0.0,
        )
        # The maximum slope of the quintic minimum-jerk curve is 1.875.
        safe_duration = (
            1.05
            * 1.875
            * max_distance_deg
            / (MAX_COMMAND_STEP_DEG * self._args.rate)
        )
        return max(self._args.transition_duration, safe_duration)

    def _fixed_pose(self, spec, positions_deg):
        if len(positions_deg) != len(spec.joint_names):
            raise RuntimeError(f"fixed pose size does not match {spec.label}")
        result = []
        for axis, (value, (lower, upper)) in enumerate(
            zip(positions_deg, spec.limits_deg)
        ):
            safe_lower = lower + COMMAND_MARGIN_DEG
            safe_upper = upper - COMMAND_MARGIN_DEG
            if not safe_lower <= value <= safe_upper:
                raise RuntimeError(
                    f"{spec.label} axis {axis + 1} fixed target {value:.2f}deg "
                    f"is outside safe range [{safe_lower:.2f}, {safe_upper:.2f}]deg"
                )
            result.append(math.radians(value))
        return result

    def _ensure_fresh(self):
        for spec in self._specs:
            if not self._listeners[spec.label].is_fresh(
                self._args.stale_timeout
            ):
                raise RuntimeError(
                    f"{spec.label} state became stale; stopping pose commands"
                )

    def _publish(self, targets):
        self._sequence += 1
        for spec in self._specs:
            positions = targets[spec.label]
            command = getattr(self._proto, spec.command_type)()
            now = time.time()
            command.header.stamp.seconds = int(now)
            command.header.stamp.nanos = int((now - int(now)) * 1e9)
            command.header.seq = self._sequence
            command.header.frame_id = "sharpa_mujoco_whole_body_motion"
            if spec.side:
                command.side = (
                    self._proto.SIDE_LEFT
                    if spec.side == "left"
                    else self._proto.SIDE_RIGHT
                )
            command.mode = self._proto.SPACE_MODE_JOINT_SPACE
            command.joints.position.extend(positions)
            if hasattr(command.joints, "velocity"):
                command.joints.velocity.extend([0.0] * len(positions))
            if hasattr(command.joints, "torque"):
                command.joints.torque.extend([0.0] * len(positions))
            self._session.put(spec.action_topic, command.SerializeToString())
        self._last_targets = self._copy_targets(targets)
        self._has_published = True

    def _wait_for_return(self):
        tolerance = math.radians(self._args.return_tolerance_deg)
        deadline = time.monotonic() + self._args.return_timeout
        while time.monotonic() < deadline:
            max_error = 0.0
            all_arrived = True
            for spec in self._specs:
                listener = self._listeners[spec.label]
                if not listener.is_fresh(self._args.stale_timeout):
                    print(
                        f"[Warning] {spec.label} state became stale while "
                        "returning to the starting pose",
                        file=sys.stderr,
                    )
                    return False
                actual, _ = listener.snapshot()
                error = max(
                    (
                        abs(value - baseline)
                        for value, baseline in zip(
                            actual, self._baselines[spec.label]
                        )
                    ),
                    default=0.0,
                )
                max_error = max(max_error, error)
                all_arrived = all_arrived and error <= tolerance
            if all_arrived:
                print(
                    "[Return] measured joints reached the starting pose "
                    f"(max error {math.degrees(max_error):.2f}deg)"
                )
                return True
            self._publish(self._baselines)
            time.sleep(1.0 / self._args.rate)
        print(
            "[Warning] timed out waiting for all joints to return",
            file=sys.stderr,
        )
        return False

    def _spec_for(self, label):
        return next(spec for spec in self._specs if spec.label == label)

    @staticmethod
    def _copy_targets(targets):
        return {label: list(positions) for label, positions in targets.items()}

    @staticmethod
    def _validate_baseline(spec, positions):
        tolerance = math.radians(0.1)
        bounded_positions = []
        for axis, (position, (lower_deg, upper_deg)) in enumerate(
            zip(positions, spec.limits_deg)
        ):
            lower = math.radians(lower_deg)
            upper = math.radians(upper_deg)
            if position < lower - tolerance or position > upper + tolerance:
                raise RuntimeError(
                    f"{spec.label} axis {axis + 1} state "
                    f"{math.degrees(position):.2f}deg is outside configured "
                    f"limits [{lower_deg:.2f}, {upper_deg:.2f}]deg"
                )
            bounded_positions.append(clamp(position, lower, upper))
        return bounded_positions


def run(args):
    zenoh, proto = load_dependencies(args.platform_root)
    session = open_session(zenoh, args.connect)
    controller = WholeBodyController(
        session, proto, device_specs(args.side), args
    )
    sides = selected_sides(args.side)
    enabled_arms = []
    torso_enabled = False
    returned = True
    try:
        counts_before = controller.counts()
        print("[Enable] requesting torso enable")
        session.put("service/torso/enable", make_enable(proto, True))
        torso_enabled = True
        for side in sides:
            print(f"[Enable] requesting {side} arm enable/recovery")
            enable_with_recovery(session, proto, side, args.state_timeout)
            enabled_arms.append(side)
        controller.capture_baselines(counts_before)
        try:
            controller.run_sequence()
        except KeyboardInterrupt:
            print("\n[Pose] Ctrl+C received; returning to the starting pose")
        return 0
    finally:
        try:
            returned = controller.return_to_baseline()
        except Exception as exc:
            returned = False
            print(f"[Warning] return-to-baseline failed: {exc}", file=sys.stderr)

        if returned:
            print(
                "[Hold] return completed; torso and arms remain enabled "
                "for Platform to hold the starting pose"
            )
        else:
            if torso_enabled:
                print(
                    "[Warning] torso remains enabled because joints did not return "
                    "to the starting pose",
                    file=sys.stderr,
                )
            for side in reversed(enabled_arms):
                topic = f"service/{side}_arm/enable"
                try:
                    call_enable_service(
                        session, proto, topic, False, args.state_timeout
                    )
                except RuntimeError as exc:
                    print(
                        f"[Warning] {side} arm disable failed: {exc}",
                        file=sys.stderr,
                    )
                    try:
                        call_reset_service(session, proto, side, args.state_timeout)
                    except RuntimeError as reset_exc:
                        print(
                            f"[Warning] {side} arm reset failed: {reset_exc}",
                            file=sys.stderr,
                        )
        controller.close()
        session.close()


if __name__ == "__main__":
    sys.exit(run_main(lambda: run(parse_args())))
