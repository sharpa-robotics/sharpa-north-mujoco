#!/usr/bin/env python3
"""Exercise the simulated torso with common kinematic motion profiles.

The script covers the five lower-body/waist axes and three neck axes through
the production Zenoh topics used by ``torso_zenoh_test.py``. First spread both
arms and keep holding them throughout torso motion and its return to zero.
All other joint groups stay enabled with fixed targets; all groups return to
zero on completion or interruption, with the shoulders closing last.
"""

import argparse
import math
import os
import sys
from copy import copy

from arm_motion_driver import (
    JOINT_AMPLITUDES_DEG as ARM_AMPLITUDES_DEG,
    JOINT_LIMITS_DEG as ARM_LIMITS_DEG,
    JOINT_NAMES as ARM_JOINT_NAMES,
    PREFERRED_DIRECTIONS as ARM_DIRECTIONS,
    ArmMotionRunner,
)
from motion_session import MotionSession
from kinematic_motion import (
    ChannelSpec,
    MotionRunner,
    add_common_arguments,
    default_platform_root,
    load_dependencies,
    open_session,
    run_main,
    validate_args,
)


JOINT_NAMES = (
    "lower_body_joint_1",
    "lower_body_joint_2",
    "lower_body_joint_3",
    "lower_body_joint_4",
    "lower_body_joint_5",
    "neck_joint_1",
    "neck_joint_2",
    "neck_joint_3",
)
# Current HA1 MuJoCo joint limits; the configured test amplitudes stay well
# inside them and the common runner applies an additional CLI margin.
JOINT_LIMITS_DEG = (
    (0.0, 85.0),
    (-140.0, 0.0),
    (-30.0, 120.0),
    (-30.0, 30.0),
    (-180.0, 180.0),
    (-90.0, 90.0),
    (-30.0, 30.0),
    (-30.0, 60.0),
)
JOINT_AMPLITUDES_DEG = (5.0, 8.0, 15.0, 10.0, 20.0, 10.0, 12.0, 20.0)
# J2 commonly starts at its upper (zero-degree) limit, so exercise it in the
# negative direction.  Other axes prefer positive motion when headroom allows.
PREFERRED_DIRECTIONS = (1.0, -1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0)

# Both HA1 arms use the same logical joint angles: negative J2 opens each
# shoulder outward. A small elbow bend keeps the hands away from the torso.
ARM_CLEARANCE_POSE_DEG = (0.0, -37.5, 0.0, 15.0, 0.0, 0.0, 0.0)
ARM_LOWERED_POSE_DEG = (0.0,) * len(ARM_JOINT_NAMES)
ARM_SPREAD_DURATION = 4.0
# Arm holding dynamics have a small steady-state error; keep their arrival
# threshold separate from the stricter torso return-to-baseline tolerance.
ARM_CLEARANCE_TOLERANCE_DEG = 2.0
ARM_HOLD_TOLERANCE_DEG = 5.0


class ArmClearanceRunner(ArmMotionRunner):
    """Spread and lower both arms using the shared smooth return helper."""

    def spread(self, shoulder_deg=None, motion_name="torso"):
        if shoulder_deg is None:
            arrived = self._move_to_pose(ARM_CLEARANCE_POSE_DEG, "spreading both arms")
        else:
            # A hand test only needs shoulder clearance. Preserve each arm's
            # other joints, and never pull an already spread shoulder inward.
            starts = self._last_targets
            if starts is None:
                raise RuntimeError("arm state baselines have not been captured")
            targets = {label: list(values) for label, values in starts.items()}
            for positions in targets.values():
                positions[1] = min(positions[1], math.radians(shoulder_deg))
            arrived = self._move_to_targets(targets, "spreading shoulders for hand clearance")
        if not arrived:
            raise RuntimeError(f"arms did not reach clearance pose; {motion_name} motion aborted")
        print(f"[Arms] clearance pose reached; holding throughout {motion_name} motion")

    def lower(self):
        if not self._has_published:
            return True
        return self._move_to_pose(ARM_LOWERED_POSE_DEG, "lowering both arms")

    def _move_to_pose(self, pose_deg, stage):
        pose = [math.radians(value) for value in pose_deg]
        targets = {label: list(pose) for label in self._listeners}
        return self._move_to_targets(targets, stage)

    def _move_to_targets(self, targets, stage):
        starts = self._last_targets
        if starts is None:
            raise RuntimeError("arm state baselines have not been captured")
        for label, positions in starts.items():
            for axis, (position, limits) in enumerate(zip(positions, self._limits)):
                if not limits[0] <= position <= limits[1]:
                    raise RuntimeError(f"{label} J{axis + 1} is outside arm limits")
        for values in targets.values():
            for position, (lower, upper) in zip(values, self._limits):
                if not lower <= position <= upper:
                    raise ValueError("arm target pose is outside arm limits")
        # Make the requested pose this runner's baseline so the shared helper
        # interpolates from the last targets and verifies measured arrival.
        self._baselines = {label: list(values) for label, values in targets.items()}
        duration = self._transition_duration(starts, self._baselines, ARM_SPREAD_DURATION)
        self._publish(starts)
        print(f"[Arms] {stage} over {duration:.2f}s")
        self._return_stage = stage
        return self.return_to_baseline(ARM_SPREAD_DURATION)

    def hold_clearance(self):
        self.check_clearance()
        self._publish(self._baselines)

    def check_clearance(self):
        tolerance = math.radians(ARM_HOLD_TOLERANCE_DEG)
        for label, listener in self._listeners.items():
            positions, _ = listener.snapshot()
            if any(
                abs(position - target) > tolerance
                for position, target in zip(positions, self._baselines[label])
            ):
                raise RuntimeError(f"{label} left clearance pose; stopping motion commands")

    def _publish(self, targets):
        for label, listener in self._listeners.items():
            if not listener.is_fresh(self._args.stale_timeout):
                raise RuntimeError(f"{label} state became stale; stopping motion commands")
        super()._publish(targets)


class TorsoMotionRunner(MotionRunner):
    """Publish arm hold commands on every torso tick, including cleanup."""

    def __init__(self, arms, **kwargs):
        self._arms = arms
        super().__init__(**kwargs)

    def _profile_speed(self, profile, axes=()):
        # Keep coordinated profiles slow enough to see their velocity shapes.
        speed = super()._profile_speed(profile, axes)
        # The opening J1/J2 sweeps need gentler motion: omit their 2x boost.
        if profile == "sweep" and any(axis in (0, 1) for axis in axes):
            return speed
        return speed * (2.0 if profile == "sweep" else 0.5)

    def _publish(self, targets):
        if self._hold_callback is None:
            self._arms.hold_clearance()
        else:
            self._arms.check_clearance()
        super()._publish(targets)


def parse_args():
    default_root = os.environ.get(
        "SHARPA_PLATFORM_ROOT", str(default_platform_root(__file__))
    )
    parser = argparse.ArgumentParser(
        description=(
            "Drive torso and neck joints through sweeps, sine, trapezoidal "
            "acceleration, minimum-jerk, and chirp profiles while holding "
            "both arms spread away from the torso."
        )
    )
    add_common_arguments(parser, len(JOINT_NAMES), default_root)
    return parser.parse_args()


def make_enable(proto, enabled):
    request = proto.EnableRequest()
    request.enable = enabled
    return request.SerializeToString()


def run(args):
    axes, profiles = validate_args(args, len(JOINT_NAMES))
    zenoh, proto = load_dependencies(args.platform_root)
    session = open_session(zenoh, args.connect)
    arm_args = copy(args)
    arm_args.return_tolerance_deg = ARM_CLEARANCE_TOLERANCE_DEG
    arms = ArmClearanceRunner(
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
            for side in ("left", "right")
        ),
        joint_names=ARM_JOINT_NAMES,
        limits_deg=ARM_LIMITS_DEG,
        amplitudes_deg=ARM_AMPLITUDES_DEG,
        preferred_directions=ARM_DIRECTIONS,
        args=arm_args,
        frame_id="sharpa_mujoco_torso_arm_clearance",
    )
    runner = TorsoMotionRunner(
        arms=arms,
        session=session,
        proto=proto,
        channels=(
            ChannelSpec(
                label="torso",
                action_topic="action/torso",
                state_topic="state/torso",
                command_type="TorsoCommand",
                state_type="TorsoState",
            ),
        ),
        joint_names=JOINT_NAMES,
        limits_deg=JOINT_LIMITS_DEG,
        amplitudes_deg=JOINT_AMPLITUDES_DEG,
        preferred_directions=PREFERRED_DIRECTIONS,
        args=args,
        frame_id="sharpa_mujoco_torso_motion_driver",
        # Run the complete chirp before a separate smooth return to baseline.
        smooth_chirp_return=True,
    )
    motion = MotionSession(session, proto, args, (runner, arms))
    status = 0
    try:
        motion.start()
        arms.spread()
        runner.run(axes, profiles)
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
