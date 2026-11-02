#!/usr/bin/env python3
"""Exercise all simulated Wave hand joints with common motion profiles.

Start zenohd, sharpa_mujoco_simulator, and sharpa_platform with the Wave hand
plugin configured, then run this script.  Use ``--help`` to select hands,
joints, profiles, rates, and amplitudes. Other joint groups stay enabled with
fixed targets. Spread both shoulders slightly before exercising the fingers;
all groups return to zero on completion or interruption.
"""

import argparse
import os
import sys
from copy import copy
from itertools import groupby

from arm_motion_driver import (
    JOINT_AMPLITUDES_DEG as ARM_AMPLITUDES_DEG,
    JOINT_LIMITS_DEG as ARM_LIMITS_DEG,
    JOINT_NAMES as ARM_JOINT_NAMES,
    PREFERRED_DIRECTIONS as ARM_DIRECTIONS,
)

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
from motion_session import HAND_ZERO_TOLERANCE_DEG, MotionSession
from torso_motion_driver import ArmClearanceRunner, ARM_CLEARANCE_TOLERANCE_DEG


HAND_SHOULDER_CLEARANCE_DEG = -20.0


class HandMotionRunner(MotionRunner):
    """Check arm clearance before publishing hand exercise/return targets."""

    def __init__(self, arms, **kwargs):
        self._arms = arms
        super().__init__(**kwargs)

    def _publish(self, targets):
        self._arms.check_clearance()
        super()._publish(targets)

    def run(self, axes, profiles, sweep_groups=None):
        axes = flexion_axes(axes)
        if self._baselines is None:
            raise RuntimeError("hand state baselines have not been captured")
        self._baselines = {
            label: list(values) for label, values in self._baselines.items()
        }
        for values in self._baselines.values():
            for axis in SIDE_SWING_AXES:
                values[axis] = 0.0
        print("[Hands] zeroing and holding side-swing joints (axes 2,4,7,11,15,20)")
        if not self.return_to_baseline(
            duration=2.0,
            tolerance_deg=max(self._args.return_tolerance_deg, HAND_ZERO_TOLERANCE_DEG),
        ):
            raise RuntimeError("hands did not reach the zero side-swing pose")
        # Keep pinky CMC motion in individual-finger sweeps, but park it before
        # coordinated profiles so the pinky cannot swing into the ring finger.
        for is_sweep, stage_profiles in groupby(profiles, key=lambda profile: profile == "sweep"):
            stage_axes = axes
            if not is_sweep:
                stage_axes = tuple(axis for axis in axes if axis != PINKY_CMC_AXIS)
                for values in self._baselines.values():
                    values[PINKY_CMC_AXIS] = 0.0
                print("[Hands] zeroing and holding pinky CMC (axis 18) for coordinated motion")
                if not self.return_to_baseline(
                    duration=2.0,
                    tolerance_deg=max(self._args.return_tolerance_deg, HAND_ZERO_TOLERANCE_DEG),
                ):
                    raise RuntimeError("hands did not reach the zero pinky CMC pose")
            stage_profiles = tuple(stage_profiles)
            if stage_axes:
                super().run(stage_axes, stage_profiles, sweep_groups=sweep_groups)


JOINT_NAMES = (
    "thumb_CMC_FE",
    "thumb_CMC_AA",
    "thumb_MCP_FE",
    "thumb_MCP_AA",
    "thumb_IP",
    "index_MCP_FE",
    "index_MCP_AA",
    "index_PIP",
    "index_DIP",
    "middle_MCP_FE",
    "middle_MCP_AA",
    "middle_PIP",
    "middle_DIP",
    "ring_MCP_FE",
    "ring_MCP_AA",
    "ring_PIP",
    "ring_DIP",
    "pinky_CMC",
    "pinky_MCP_FE",
    "pinky_MCP_AA",
    "pinky_PIP",
    "pinky_DIP",
)
SIDE_SWING_AXES = tuple(
    axis for axis, name in enumerate(JOINT_NAMES) if name.endswith("_AA")
)
PINKY_CMC_AXIS = JOINT_NAMES.index("pinky_CMC")


def flexion_axes(axes):
    axes = tuple(axis for axis in axes if axis not in SIDE_SWING_AXES)
    if not axes:
        raise ValueError("hand side-swing motion is disabled; --axes must include a flexion joint")
    return axes


JOINT_LIMITS_DEG = (
    (-5.0, 105.0),
    (-18.0, 18.0),
    (-25.0, 75.0),
    (-18.0, 18.0),
    (0.0, 95.0),
    (-10.0, 84.0),
    (-18.0, 18.0),
    (0.0, 95.0),
    (0.0, 75.0),
    (-10.0, 84.0),
    (-18.0, 18.0),
    (0.0, 95.0),
    (0.0, 75.0),
    (-10.0, 84.0),
    (-18.0, 18.0),
    (0.0, 95.0),
    (0.0, 75.0),
    (0.0, 25.0),
    (-10.0, 84.0),
    (-18.0, 18.0),
    (0.0, 95.0),
    (0.0, 75.0),
)
# Side-swing joints stay at zero; flexion excursions are scaled by CLI input
# and clipped against live-pose headroom.
JOINT_AMPLITUDES_DEG = (
    25.0,
    0.0,
    20.0,
    0.0,
    20.0,
    20.0,
    0.0,
    20.0,
    15.0,
    20.0,
    0.0,
    20.0,
    15.0,
    20.0,
    0.0,
    20.0,
    15.0,
    8.0,
    20.0,
    0.0,
    20.0,
    15.0,
)
# Direction signs on disabled side-swing axes are retained for metadata only.
PREFERRED_DIRECTIONS = (
    1.0, -1.0, 1.0, 1.0, 1.0, 1.0, -1.0, 1.0, 1.0, 1.0, 1.0,
    1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0,
)

# The initial sweep moves selected flexion joints of one finger together.
FINGER_JOINT_GROUPS = {
    "thumb": tuple(range(0, 5)),
    "index": tuple(range(5, 9)),
    "middle": tuple(range(9, 13)),
    "ring": tuple(range(13, 17)),
    "pinky": tuple(range(17, 22)),
}


def parse_args():
    default_root = os.environ.get(
        "SHARPA_PLATFORM_ROOT", str(default_platform_root(__file__))
    )
    parser = argparse.ArgumentParser(
        description=(
            "Drive Wave hands through joint sweeps, sine, trapezoidal "
            "acceleration, minimum-jerk, and chirp profiles after spreading "
            "both shoulders for clearance. Side-swing joints stay at zero."
        )
    )
    add_common_arguments(parser, len(JOINT_NAMES), default_root)
    parser.set_defaults(limit_margin_deg=0.5)
    parser.add_argument(
        "--side",
        choices=("left", "right", "both"),
        default="both",
        help="Hand to exercise (default: both)",
    )
    return parser.parse_args()


def run(args):
    axes, profiles = validate_args(args, len(JOINT_NAMES))
    axes = flexion_axes(axes)
    sides = ("left", "right") if args.side == "both" else (args.side,)
    zenoh, proto = load_dependencies(args.platform_root)
    session = open_session(zenoh, args.connect)
    arm_args = copy(args)
    arm_args.return_tolerance_deg = ARM_CLEARANCE_TOLERANCE_DEG
    arms = ArmClearanceRunner(
        session=session, proto=proto,
        channels=tuple(ChannelSpec(
            label=f"{side}_arm", action_topic=f"action/{side}_arm",
            state_topic=f"state/{side}_arm", command_type="ArmCommand",
            state_type="ArmState", side=side,
        ) for side in ("left", "right")),
        joint_names=ARM_JOINT_NAMES, limits_deg=ARM_LIMITS_DEG,
        amplitudes_deg=ARM_AMPLITUDES_DEG, preferred_directions=ARM_DIRECTIONS,
        args=arm_args, frame_id="sharpa_mujoco_hand_arm_clearance",
    )
    runner = HandMotionRunner(
        arms=arms,
        session=session,
        proto=proto,
        channels=tuple(
            ChannelSpec(
                label=f"{side}_hand",
                action_topic=f"action/{side}_hand",
                state_topic=f"state/{side}_hand",
                command_type="HandCommand",
                state_type="HandState",
                side=side,
            )
            for side in sides
        ),
        joint_names=JOINT_NAMES,
        limits_deg=JOINT_LIMITS_DEG,
        amplitudes_deg=JOINT_AMPLITUDES_DEG,
        preferred_directions=PREFERRED_DIRECTIONS,
        args=args,
        frame_id="sharpa_mujoco_hand_motion_driver",
        # Most finger joints start at their lower limit.  A bipolar sine/chirp
        # would therefore be clipped, and can also cross the firmware's moving
        # differential limits.  Keep continuous profiles as smooth 0..1
        # excursions in each joint's selected safe direction.
        unidirectional_continuous_profiles=True,
    )
    motion = MotionSession(session, proto, args, (runner, arms))
    status = 0
    try:
        motion.start()
        arms.spread(shoulder_deg=HAND_SHOULDER_CLEARANCE_DEG, motion_name="hand")
        runner.run(axes, profiles, sweep_groups=FINGER_JOINT_GROUPS)
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
