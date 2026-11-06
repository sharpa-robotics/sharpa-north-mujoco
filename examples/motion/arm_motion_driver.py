#!/usr/bin/env python3
"""Exercise real robot arms with bounded, common kinematic motion profiles.

Connect to the robot at 192.168.100.100 with zenohd and sharpa_platform running,
with the arms in their neutral hanging pose and the torso stationary.
Open the shoulders before exercising the joints around a fixed spread pose.
On completion or interruption, return all joint groups to zero and leave them
enabled for Platform to keep holding their position targets.
"""

import argparse
from collections import deque
import math
import os
import sys

from kinematic_motion import (
    ChannelSpec,
    MotionRunner,
    REFERENCE_SPEED_DEG_PER_SECOND,
    add_common_arguments,
    default_platform_root,
    load_dependencies,
    open_session,
    run_main,
    validate_args,
)
from motion_session import MotionSession


JOINT_NAMES = tuple(f"arm_joint_{axis}" for axis in range(1, 8))
# Intersections of the Platform limits and current HA1 MuJoCo hard limits.
JOINT_LIMITS_DEG = (
    (-90.0, 170.0),
    (-120.0, 20.0),
    (-60.0, 240.0),
    (-10.0, 130.0),
    (-90.0, 120.0),
    (-60.0, 90.0),
    (-50.0, 50.0),
)
JOINT_AMPLITUDES_DEG = (15.0, 15.0, 15.0, 20.0, 15.0, 15.0, 15.0)
# HA1 uses the same logical angles on both arms; negative J2 opens both
# shoulders outward. Negating every right-arm joint is not a mirrored pose.
PREFERRED_DIRECTIONS = (1.0, -1.0, 1.0, 1.0, 1.0, 1.0, 1.0)
# Even the inward half of sine/chirp keeps J2 at or below -60 degrees.
# J4 stays between 5 and 45 degrees, with clearance for the wrist and hand.
EXERCISE_POSE_DEG = (0.0, -75.0, 0.0, 25.0, 0.0, 0.0, 0.0)
# Wrist feedback can retain about 1.6 degrees of steady-state holding error.
# Allow that error during exit without changing the test's arrival threshold.
EXIT_RETURN_TOLERANCE_DEG = 2.0
J1_SWEEP_DEG = (-45.0, 135.0, 0.0)
J1_SWEEP_SHOULDER_OPEN_DEG = -10.0


class ArmMotionRunner(MotionRunner):
    """Exercise a fixed spread pose, including smooth entry and return."""

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self._return_stage = "exercise return"

    def prepare(self):
        if self._baselines is None:
            raise RuntimeError("arm state baselines have not been captured")
        starts = {label: list(values) for label, values in self._baselines.items()}
        # Open J2 first: do not bend the elbow or rotate the wrist while the
        # arm is still alongside the torso.
        self._baselines = {label: list(values) for label, values in starts.items()}
        for positions in self._baselines.values():
            positions[1] = math.radians(EXERCISE_POSE_DEG[1])
        self._publish(starts)
        print("[Arms] opening shoulders")
        self._return_stage = "opening shoulders"
        if not self.return_to_baseline():
            raise RuntimeError("arms did not reach the open-shoulder pose")

        pose = [math.radians(value) for value in EXERCISE_POSE_DEG]
        self._baselines = {label: list(pose) for label in starts}
        print("[Arms] moving to exercise pose")
        self._return_stage = "moving to exercise pose"
        if not self.return_to_baseline():
            raise RuntimeError("arms did not reach the exercise pose")

    def return_to_baseline(self, duration=4.0, tolerance_deg=None):
        # During the test, the baseline is the spread exercise pose. Exit
        # returns to zero and reverses the shoulder-first preparation order.
        return super().return_to_baseline(duration, tolerance_deg=tolerance_deg)

    def _profile_speed(self, profile, axes=()):
        # Continuous swings can overspeed the coupled elbow/wrist motors.
        speed = super()._profile_speed(profile, axes)
        return speed * 0.5 if profile in ("sine", "chirp") else speed

    def sweep_j1(self):
        """Open the shoulders slightly, then move J1 through fixed endpoints."""
        print(
            "[Arms] J1 sweep: open shoulders to 10deg, then "
            "-45 -> 135 -> 0deg; J2 held open and J3-J7 held at zero"
        )
        if not self.return_to_zero():
            raise RuntimeError("arms did not reach zero before the J1 sweep")
        tolerance = max(self._args.return_tolerance_deg, EXIT_RETURN_TOLERANCE_DEG)

        shoulder_target = math.radians(J1_SWEEP_SHOULDER_OPEN_DEG)
        self._baselines = {
            label: [0.0, shoulder_target, 0.0, 0.0, 0.0, 0.0, 0.0]
            for label in self._baselines
        }
        self._return_stage = "opening shoulders for J1 sweep"
        if not self.return_to_baseline(tolerance_deg=tolerance):
            raise RuntimeError("arms did not open shoulders before the J1 sweep")

        for angle in J1_SWEEP_DEG:
            target = math.radians(angle)
            travel = max(abs(target - values[0]) for values in self._last_targets.values())
            self._baselines = {
                label: [target, shoulder_target, 0.0, 0.0, 0.0, 0.0, 0.0]
                for label in self._baselines
            }
            self._return_stage = f"J1 sweep to {angle:g}deg"
            # Minimum-jerk peak speed is 1.875 times its average speed.
            duration = max(1.0, 1.875 * travel / math.radians(REFERENCE_SPEED_DEG_PER_SECOND))
            if angle == 135.0:
                # Use eight times the reference speed for the -45 -> 135 segment.
                duration /= 8.0
            if not self.return_to_baseline(duration, tolerance_deg=tolerance):
                raise RuntimeError(f"arms did not reach J1={angle:g}deg")

    def return_to_zero(self, duration=4.0, tolerance_deg=None, close_shoulders=True):
        if self._baselines is None or not self._has_published:
            return True
        tolerance = max(
            self._args.return_tolerance_deg if tolerance_deg is None else tolerance_deg,
            EXIT_RETURN_TOLERANCE_DEG,
        )
        # Restore the elbow/wrist and other axes before closing the shoulders.
        # Keep J2 at the last commanded angle, including on partial preparation.
        self._baselines = {
            label: [0.0] * len(values) for label, values in self._last_targets.items()
        }
        for label, positions in self._baselines.items():
            positions[1] = self._last_targets[label][1]
        print("[Return] zeroing elbow/wrist joints before closing shoulders")
        self._return_stage = "zeroing elbow/wrist joints"
        if not self.return_to_baseline(duration, tolerance_deg=tolerance):
            print(
                "[Warning] zero elbow/wrist pose was not reached; "
                "keeping shoulders open",
                file=sys.stderr,
            )
            return False
        if not close_shoulders:
            return True

        self._baselines = {
            label: [0.0] * len(values) for label, values in self._baselines.items()
        }
        print(
            "[Return] zeroing shoulder positions "
            f"(exit tolerance {tolerance:.1f}deg)"
        )
        self._return_stage = "zeroing shoulder positions"
        return self.return_to_baseline(duration, tolerance_deg=tolerance)

    def _on_return_started(self, duration, tolerance):
        self._return_history = {label: deque() for label in self._listeners}
        self._next_return_report = 0.0
        print(
            f"[Arrival][{self._return_stage}] transition={duration:.2f}s "
            f"speed={self._args.speed:g} tolerance={math.degrees(tolerance):.2f}deg "
            f"wait_timeout={self._args.return_timeout:.2f}s"
        )
        for label, targets in self._baselines.items():
            print(f"[Arrival][{label}] target deg: {self._format_positions(targets)}")
        print(
            "[Arrival] upstream command velocity=0rad/s, torque=0Nm; "
            "Platform may add interpolation and dynamics feedforward before motor control"
        )

    def _on_return_feedback(self, elapsed, tolerance, feedback, status):
        # Keep distinct received frames from the last two seconds. Repeated
        # reads of one frame must not be interpreted as a stationary joint.
        for label, (positions, count) in feedback.items():
            history = self._return_history[label]
            if not history or count != history[-1][2]:
                history.append((elapsed, list(positions), count))
            while len(history) > 1 and history[0][0] < elapsed - 2.0:
                history.popleft()
        if status == "waiting" and elapsed < self._next_return_report:
            return
        self._next_return_report = elapsed + 1.0
        stream = sys.stderr if status == "timeout" else sys.stdout
        print(
            f"[Arrival][{self._return_stage}] {status} after {elapsed:.2f}s "
            "(error=actual-target)", file=stream,
        )
        for label, (positions, count) in feedback.items():
            targets = self._baselines[label]
            errors = [actual - target for actual, target in zip(positions, targets)]
            worst = max(range(len(errors)), key=lambda axis: abs(errors[axis]))
            diagnostics = self._listeners[label].diagnostics()
            # Optional velocity/torque fields may be absent in older publishers.
            diagnostics = diagnostics if isinstance(diagnostics, dict) else {}
            age = diagnostics.get("age")
            age_text = f"{age:.3f}s" if isinstance(age, (int, float)) else "n/a"
            print(
                f"[Arrival][{label}] frame={count} age={age_text} "
                f"worst=J{worst + 1}({self._joint_names[worst]}) "
                f"target={math.degrees(targets[worst]):.2f}deg "
                f"actual={math.degrees(positions[worst]):.2f}deg "
                f"error={math.degrees(errors[worst]):+.2f}deg", file=stream,
            )
            axes = range(len(errors)) if status == "timeout" else (
                axis for axis, error in enumerate(errors) if abs(error) > tolerance
            )
            history = self._return_history[label]
            span = history[-1][0] - history[0][0]
            for axis in axes:
                values = [sample[1][axis] for sample in history]
                movement = math.degrees(max(values) - min(values))
                velocities = diagnostics.get("velocity", ())
                torques = diagnostics.get("torque", ())
                velocity = (
                    f"{math.degrees(velocities[axis]):+.3f}deg/s"
                    if axis < len(velocities) and math.isfinite(velocities[axis]) else "n/a"
                )
                torque = (
                    f"{torques[axis]:+.3f}Nm"
                    if axis < len(torques) and math.isfinite(torques[axis]) else "n/a"
                )
                print(
                    f"[Arrival][{label} J{axis + 1} {self._joint_names[axis]}] "
                    f"target={math.degrees(targets[axis]):.2f}deg "
                    f"actual={math.degrees(positions[axis]):.2f}deg "
                    f"error={math.degrees(errors[axis]):+.2f}deg "
                    f"{'OVER' if abs(errors[axis]) > tolerance else 'OK'} "
                    f"velocity={velocity} torque={torque} "
                    f"range={movement:.3f}deg/{span:.2f}s frames={len(history)}", file=stream,
                )
                if status == "timeout" and abs(errors[axis]) > tolerance:
                    if span < 1.0 or len(history) < 2:
                        hint = "insufficient fresh history to assess settling"
                    elif movement <= 0.1:
                        hint = (
                            "persistent position offset: feedback barely changes; "
                            "check load/gravity/friction compensation, gains and constraints"
                        )
                    else:
                        hint = (
                            "position still varies: check convergence/oscillation, "
                            "damping and contacts"
                        )
                    print(f"[Arrival][{label} J{axis + 1}] {hint}", file=stream)

    def _publish(self, targets):
        for label, listener in self._listeners.items():
            if not listener.is_fresh(self._args.stale_timeout):
                raise RuntimeError(f"{label} state became stale; stopping arm commands")
        super()._publish(targets)


def call_enable_service(session, proto, topic, enabled, timeout):
    request = proto.EnableRequest()
    request.enable = enabled
    replies = session.get(
        topic,
        payload=request.SerializeToString(),
        timeout=timeout,
    )
    for reply in replies:
        try:
            response = proto.EnableResponse()
            response.ParseFromString(bytes(reply.ok.payload))
        except Exception as exc:
            raise RuntimeError(f"invalid response from {topic}: {exc}") from exc
        if not response.success:
            raise RuntimeError(
                f"{topic} rejected the request: "
                f"{response.message or 'unknown error'}"
            )
        return
    raise RuntimeError(f"{topic} returned no response")


def call_reset_service(session, proto, side, timeout):
    topic = f"service/{side}_arm/reset"
    request = proto.ResetRequest()
    replies = session.get(
        topic,
        payload=request.SerializeToString(),
        timeout=timeout,
    )
    for reply in replies:
        try:
            response = proto.ResetResponse()
            response.ParseFromString(bytes(reply.ok.payload))
        except Exception as exc:
            raise RuntimeError(f"invalid response from {topic}: {exc}") from exc
        if not response.success:
            raise RuntimeError(
                f"{topic} rejected the request: "
                f"{response.message or 'unknown error'}"
            )
        return
    raise RuntimeError(f"{topic} returned no response")


def enable_with_recovery(session, proto, side, timeout):
    topic = f"service/{side}_arm/enable"
    try:
        call_enable_service(session, proto, topic, True, timeout)
        return
    except RuntimeError as initial_error:
        print(f"[Recovery][{side}] enable failed: {initial_error}")

    try:
        call_enable_service(session, proto, topic, False, timeout)
        print(f"[Recovery][{side}] disabled stale Active state")
    except RuntimeError:
        call_reset_service(session, proto, side, timeout)
        print(f"[Recovery][{side}] reset stale EmergencyStop state")

    call_enable_service(session, proto, topic, True, timeout)


def parse_args():
    default_root = os.environ.get(
        "SHARPA_PLATFORM_ROOT", str(default_platform_root(__file__))
    )
    parser = argparse.ArgumentParser(
        description=(
            "Drive arms through joint sweeps, sine, trapezoidal acceleration, "
            "minimum-jerk, and chirp profiles."
        )
    )
    add_common_arguments(parser, len(JOINT_NAMES), default_root)
    # Retain margin at the hard limits and allow normal holding error when
    # verifying arrival at the fixed exercise pose.
    parser.set_defaults(limit_margin_deg=2.0, return_tolerance_deg=1.0)
    parser.add_argument(
        "--side",
        choices=("left", "right", "both"),
        default="both",
        help="Arm to exercise (default: both)",
    )
    parser.add_argument(
        "--j1-sweep-only",
        action="store_true",
        help=(
            "Run only the final J1 sweep (-45 -> 135 -> 0deg), after returning "
            "to zero; skip spread-pose preparation and joint profiles "
            "(--axes and --profiles do not select this sweep)"
        ),
    )
    return parser.parse_args()


def run(args):
    axes, profiles = validate_args(args, len(JOINT_NAMES))
    sides = ("left", "right") if args.side == "both" else (args.side,)
    zenoh, proto = load_dependencies(args.platform_root)
    session = open_session(zenoh, args.connect)
    runner = ArmMotionRunner(
        session=session,
        proto=proto,
        channels=tuple(
            ChannelSpec(
                label=f"{side}_arm",
                action_topic=f"action/{side}_arm",
                state_topic=f"state/{side}_arm",
                command_type="ArmCommand",
                state_type="ArmState",
                side=side,
            )
            for side in sides
        ),
        joint_names=JOINT_NAMES,
        limits_deg=JOINT_LIMITS_DEG,
        amplitudes_deg=JOINT_AMPLITUDES_DEG,
        preferred_directions=PREFERRED_DIRECTIONS,
        args=args,
        frame_id="sharpa_mujoco_arm_motion_driver",
        # Sine and chirp swing to both sides of the exercise pose. Keep their
        # targets inside the configured margin instead of touching hard limits.
        enforce_bidirectional_limit_margin=True,
    )
    motion = MotionSession(session, proto, args, (runner,))
    status = 0
    try:
        motion.start()
        if args.j1_sweep_only:
            print("[Arms] running only the final J1 sweep")
            runner.sweep_j1()
        else:
            original_speed = args.speed
            args.speed = original_speed * 2.0
            try:
                print(f"[Arms] preparation and joint exercises at speed={args.speed:g}")
                runner.prepare()
                runner.run(axes, profiles)
            finally:
                args.speed = original_speed
            if 0 in axes:
                runner.sweep_j1()
    except KeyboardInterrupt:
        print("\n[Test] interrupted; returning all joints to zero before exit")
        status = 130
    finally:
        try:
            if not motion.return_to_zero() and status == 0:
                status = 1
        finally:
            motion.close()
            session.close()
    return status


if __name__ == "__main__":
    sys.exit(run_main(lambda: run(parse_args())))
