"""Shared enabled position holding and zero-pose cleanup for motion drivers."""

import math
import sys
from copy import copy

from kinematic_motion import ChannelSpec, MotionRunner


# Wave feedback can settle about 0.51 degrees from a zero position target.
# This cleanup-only tolerance avoids leaving shoulders open for that small
# offset; commanded hand angles remain zero and exercise checks are unchanged.
HAND_ZERO_TOLERANCE_DEG = 1.0


class MotionSession:
    """Keep torso, arms and hands commanded during every motion/return tick."""

    def __init__(self, session, proto, args, runners):
        # Import metadata lazily so each driver can also import MotionSession.
        import arm_motion_driver as arm
        import hand_motion_driver as hand
        import torso_motion_driver as torso

        self.session = session
        self.proto = proto
        self.args = args
        self.runners = list(runners)
        self.ready = False
        self._publishing_holds = False
        covered = {
            channel.label for runner in self.runners for channel in runner._channels
        }
        for module, part, sides, runner_type in (
            (torso, "torso", (None,), MotionRunner),
            (arm, "arm", ("left", "right"), arm.ArmMotionRunner),
            (hand, "hand", ("left", "right"), hand.HandFeedbackRunner),
        ):
            channels = []
            for side in sides:
                label = f"{side}_{part}" if side else part
                if label not in covered:
                    channels.append(ChannelSpec(
                        label=label, action_topic=f"action/{label}",
                        state_topic=f"state/{label}",
                        command_type=f"{part.title()}Command",
                        state_type=f"{part.title()}State", side=side or "",
                    ))
            if channels:
                hold_args = copy(args)
                if part == "hand":
                    hold_args.limit_margin_deg = 0.5
                self.runners.append(runner_type(
                    session=session, proto=proto, channels=channels,
                    joint_names=module.JOINT_NAMES, limits_deg=module.JOINT_LIMITS_DEG,
                    amplitudes_deg=module.JOINT_AMPLITUDES_DEG,
                    preferred_directions=module.PREFERRED_DIRECTIONS,
                    args=hold_args, frame_id=f"sharpa_motion_hold_{part}",
                ))

    def start(self):
        from arm_motion_driver import call_enable_service, enable_with_recovery

        counts_before = {runner: runner.counts() for runner in self.runners}
        print("[Enable] enabling torso/neck and both arms for position holding")
        call_enable_service(
            self.session, self.proto, "service/torso/enable", True, self.args.state_timeout
        )
        for side in ("left", "right"):
            enable_with_recovery(self.session, self.proto, side, self.args.state_timeout)
        print("[Enable] both Wave hands must be active; verifying fresh feedback")
        for runner in self.runners:
            runner.wait_for_baselines(counts_before[runner])
        self.ready = True
        for runner in self.runners:
            runner._hold_callback = self.hold_others
        # Seed every group's fixed target before any preparation or exercise.
        for runner in self.runners:
            MotionRunner._publish(runner, runner._last_targets)
        print("[Hold] all joint groups enabled; fixed companion targets sent on every tick")

    def hold_others(self, moving_runner):
        if self._publishing_holds:
            return
        # Validate every group before sending the moving group's next target.
        for runner in self.runners:
            for label, listener in runner._listeners.items():
                if not listener.is_fresh(self.args.stale_timeout):
                    raise RuntimeError(f"{label} state became stale; stopping motion commands")
        self._publishing_holds = True
        try:
            for runner in self.runners:
                if runner is not moving_runner:
                    # Bypass exercise-specific publish hooks for a held group.
                    # These targets are its last commands, never live feedback:
                    # recapturing feedback would let a disturbed pose drift.
                    MotionRunner._publish(runner, runner._last_targets)
        finally:
            self._publishing_holds = False

    def return_to_zero(self):
        if not self.ready:
            return True  # No preparation/exercise commands have been issued.
        # Select by channel identity, not Python class identity: a driver run
        # directly defines its classes in __main__, separately from imports.
        arms = [runner for runner in self.runners
                if any(channel.label.endswith("_arm") for channel in runner._channels)]
        torso = [runner for runner in self.runners
                 if any(channel.label == "torso" for channel in runner._channels)]
        hands = [runner for runner in self.runners
                 if any(channel.label.endswith("_hand") for channel in runner._channels)]
        success = True

        def home(runner, **kwargs):
            nonlocal success
            labels = ",".join(channel.label for channel in runner._channels)
            if any(channel.label.endswith("_hand") for channel in runner._channels):
                kwargs["tolerance_deg"] = max(
                    runner._args.return_tolerance_deg, HAND_ZERO_TOLERANCE_DEG
                )
                print(f"[Return][{labels}] zero target, tolerance={kwargs['tolerance_deg']:.2f}deg")
            try:
                arrived = runner.return_to_zero(**kwargs)
            except Exception as exc:
                print(f"[Warning] {labels} zero return failed: {exc}", file=sys.stderr)
                arrived = False
            if not arrived:
                print(f"[Warning] {labels} did not reach zero within tolerance", file=sys.stderr)
                success = False
            return arrived

        print("[Return] returning all joint groups to zero; shoulders close last")
        # Open the fingers before bringing a fist back alongside the torso.
        hands_ready = all([home(runner) for runner in hands])
        # Keep shoulders spread until elbows/wrists and torso are at home.
        arms_ready = all([home(runner, close_shoulders=False) for runner in arms])
        torso_ready = all([home(runner) for runner in torso])
        if hands_ready and arms_ready and torso_ready:
            for runner in arms:
                home(runner)
        else:
            print("[Warning] keeping shoulder targets while zero return is incomplete", file=sys.stderr)
        for runner in self.runners:
            for label, listener in runner._listeners.items():
                actual, _ = listener.snapshot()
                zero_target = all(value == 0.0 for value in runner._last_targets[label])
                max_error = max((abs(math.degrees(value)) for value in actual), default=0.0)
                print(
                    f"[Exit][{label}] zero_target={'yes' if zero_target else 'NO'} "
                    f"max_measured_zero_error={max_error:.2f}deg"
                )
        print("[Exit] leaving all joint groups enabled; Platform holds the last position targets")
        return success

    def close(self):
        for runner in self.runners:
            runner._hold_callback = None
            runner.close()
