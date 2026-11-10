#!/usr/bin/env python3
# Copyright 2026 Sharpa
# SPDX-License-Identifier: Apache-2.0
"""Run a squat / raised-arms / thumbs-up sequence using the official North SDK.

By default, starts the bundled Platform 'inference'
task, sends complete position targets through its Master channel, returns to the
captured starting pose on completion or Ctrl+C, then stops that task. Holds until
Ctrl+C by default. --preview only reads state. Only the local simulation is used.
"""

import argparse
import json
import math
from pathlib import Path
import sys
import time


ENDPOINT = "udp/127.0.0.1:7449?rel=1;multistream=1;mixed_rel=0"
# Same device order and poses as examples/whole_body_motion.py; SDK positions are rad.
ARM_DEG = (45, -7, -5, 60, 0, 0, 0)
TORSO_DEG = (45, -75, 40, 0, 0, 0, 0, 0)
HAND_DEG = (8, -15, -17, -4, 12, 65, 0, 75, 55, 70, 0, 80, 60,
            70, 0, 80, 60, 5, 65, 0, 75, 55)
COUNTS = {"torso": 8, "left_arm": 7, "right_arm": 7,
          "left_hand": 22, "right_hand": 22}
MAX_STEP = math.radians(1.25)


def minimum_jerk(progress):
    x = min(1.0, max(0.0, progress))
    return x * x * x * (10.0 + x * (-15.0 + 6.0 * x))


def valid_positions(values, count):
    result = list(values)
    if len(result) != count or not all(math.isfinite(x) for x in result):
        raise RuntimeError("Expected complete finite joint feedback")
    return result


def make_command(targets):
    from north_sdk import (ArmCommand, HandCommand, JointCommand,
                           TorsoCommand, WholeBodyCommand)
    command = WholeBodyCommand()
    for name, values in targets.items():
        joints = JointCommand()
        joints.position = valid_positions(values, COUNTS[name])
        part_command = (TorsoCommand() if name == "torso" else
                        ArmCommand() if name.endswith("_arm") else HandCommand())
        part_command.joints = joints
        setattr(command, name, part_command)
    return command


def transition_duration(starts, targets, duration, rate):
    distance = max(abs(a - b) for name in starts
                   for a, b in zip(starts[name], targets[name]))
    # Same peak minimum-jerk speed and safety margin as the non-SDK example.
    return max(duration, 1.05 * 1.875 * distance / (MAX_STEP * rate))


def main(args):
    # Import lazily so --help and trajectory checks do not need a built SDK.
    from north_sdk import (CommandChannel, ErrorCode, MasterPhase, MasterResult,
                           NorthSDK, NorthSDKError, OperationMode, SdkConfig, Side)

    config = SdkConfig()
    config.connect_endpoints = [ENDPOINT]
    config.time_sync.enabled = False  # Host gPTP setup is unnecessary for local simulation.
    config.command_channel = CommandChannel.INFERENCE
    config.master_status_max_age_ms = 500
    config.master_timeout_ms = 15000
    report = {"endpoint": ENDPOINT, "executed": args.execute,
              "sent": 0, "interrupted": False, "sequence_completed": False,
              "return_verified": False, "motion_verified": False}

    def wait_for(reader, predicate, description, timeout=5.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            try:
                result = reader()
                if predicate(result):
                    return result
            except NorthSDKError as exc:
                if exc.code != ErrorCode.STALE_STATE:
                    raise
            time.sleep(0.02)
        raise RuntimeError("Timed out waiting for " + description)

    def accepted(response, operation):
        if response.result != MasterResult.OK:
            raise RuntimeError(operation + " rejected: " + response.message)

    with NorthSDK(config) as sdk:
        master = sdk.master()
        parts = {}
        if not args.arms_only:
            parts["torso"] = sdk.torso()
        for label, side in (("left", Side.LEFT), ("right", Side.RIGHT)):
            if args.side in (label, "both"):
                parts[label + "_arm"] = sdk.arm(side)
                if not args.arms_only:
                    parts[label + "_hand"] = sdk.hand(side)

        def status():
            return master.latest_status(500)

        def read_positions():
            return {name: valid_positions(client.latest_state(300).joints.position,
                                          COUNTS[name])
                    for name, client in parts.items()}

        subscriptions = [master.on_status(lambda _: None)]
        subscriptions.extend(part.on_state(lambda _: None) for part in parts.values())
        start_requested = False
        task_started = False
        try:
            current = wait_for(status, lambda _: True, "Master status")
            baseline = wait_for(read_positions, lambda _: True, "joint feedback")
            print(f"Master={current.phase.name} mode={current.mode.name} "
                  f"task={current.task_id!r}", flush=True)
            print("Local simulation. Parts: " + ", ".join(parts), flush=True)
            report["initial_positions_rad"] = baseline
            if not args.execute:
                print("Read-only preview. Run without --preview to start the inference task and move.")
                return report
            if current.phase != MasterPhase.STANDBY:
                raise RuntimeError("Fresh STANDBY required; finish the current task first")
            accepted(master.set_mode(OperationMode.INFERENCE), "set_mode")
            wait_for(status, lambda s: s.phase == MasterPhase.STANDBY and
                     s.mode == OperationMode.INFERENCE, "STANDBY / INFERENCE")
            start_requested = True
            accepted(master.start_task("inference"), "start_task(inference)")
            task_started = True
            wait_for(status, lambda s: s.phase == MasterPhase.RUNNING and
                     s.task_id == "inference" and s.mode == OperationMode.INFERENCE,
                     "RUNNING / inference", timeout=15.0)
            # Capture after task preparation; no zero fallback for missing positions.
            baseline = wait_for(read_positions, lambda _: True, "prepared feedback")
            report["initial_positions_rad"] = baseline
            last_targets = {name: q.copy() for name, q in baseline.items()}

            def send(targets):
                current = status()
                if (current.phase != MasterPhase.RUNNING or
                        current.task_id != "inference" or
                        current.mode != OperationMode.INFERENCE):
                    raise RuntimeError("Master task/mode changed; stopping commands")
                read_positions()  # Fail on stale or malformed feedback before every send.
                for name, values in targets.items():
                    q = valid_positions(values, COUNTS[name])
                    if max(abs(a - b) for a, b in zip(q, last_targets[name])) > MAX_STEP:
                        raise RuntimeError("Command step too large; simulation may be overloaded")
                command = make_command(targets)
                sdk.whole_body().send(CommandChannel.INFERENCE, command)
                for name, q in targets.items():
                    last_targets[name] = q.copy()
                report["sent"] += 1

            def move(targets, label):
                start_pose = {name: q.copy() for name, q in last_targets.items()}
                duration = transition_duration(start_pose, targets,
                                               args.transition_duration, args.rate)
                report["stage"] = label
                print(f"{label}: {duration:.1f}s", flush=True)
                started = next_tick = time.monotonic()
                while True:
                    elapsed = time.monotonic() - started
                    blend = minimum_jerk(elapsed / duration)
                    send({name: [a + (b - a) * blend
                                 for a, b in zip(start_pose[name], targets[name])]
                          for name in parts})
                    if elapsed >= duration:
                        break
                    next_tick = max(next_tick + 1.0 / args.rate, time.monotonic())
                    time.sleep(max(0.0, next_tick - time.monotonic()))

            def hold(targets, seconds):
                started = next_tick = time.monotonic()
                while seconds == 0.0 or time.monotonic() - started < seconds:
                    send(targets)
                    next_tick = max(next_tick + 1.0 / args.rate, time.monotonic())
                    time.sleep(max(0.0, next_tick - time.monotonic()))

            raised = {name: q.copy() for name, q in baseline.items()}
            for name in raised:
                if name.endswith("_arm"):
                    raised[name] = [math.radians(x) for x in ARM_DEG]
                elif name == "torso":
                    raised[name] = [math.radians(x) for x in TORSO_DEG]
            thumbs_up = {name: q.copy() for name, q in raised.items()}
            if not args.arms_only:
                for name in thumbs_up:
                    if name.endswith("_hand"):
                        thumbs_up[name] = [math.radians(x) for x in HAND_DEG]
            try:
                move(raised, "Raise arms" if args.arms_only else "Squat and raise arms")
                if not args.arms_only:
                    move(thumbs_up, "Thumbs up")
                report["stage"] = "Hold thumbs up"
                duration_text = (f"for {args.hold_duration:.1f}s"
                                 if args.hold_duration else "until Ctrl+C")
                print("Hold thumbs up " + duration_text, flush=True)
                hold(thumbs_up, args.hold_duration)
                report["sequence_completed"] = True
            except KeyboardInterrupt:
                report["interrupted"] = True
                report["interrupted_stage"] = report["stage"]
                print("\nCtrl+C received; returning to the starting pose", flush=True)

            if report["stage"] == "Hold thumbs up":
                observed = read_positions()
                report["peak_pose_error_rad"] = {
                    name: max(abs(a - b) for a, b in zip(observed[name], thumbs_up[name]))
                    for name in parts}
            try:
                move(baseline, "Return to captured starting pose")
                # Match the non-SDK example's short settling period before checking arrival.
                hold(baseline, 0.2)
                deadline = time.monotonic() + args.return_timeout
                while True:
                    observed = read_positions()
                    report["return_error_rad"] = {
                        name: max(abs(a - b) for a, b in zip(observed[name], baseline[name]))
                        for name in parts}
                    report["return_verified"] = all(
                        error <= math.radians(args.return_tolerance_deg)
                        for error in report["return_error_rad"].values())
                    if report["return_verified"]:
                        break
                    if time.monotonic() >= deadline:
                        raise RuntimeError("Timed out waiting for joints to return to the starting pose")
                    send(baseline)
                    time.sleep(1.0 / args.rate)
            except KeyboardInterrupt:
                report["return_interrupted"] = True
                raise
            report["motion_verified"] = (
                report["return_verified"] and bool(report.get("peak_pose_error_rad")) and
                all(error < 0.10 for error in report["peak_pose_error_rad"].values()))
            if not report["interrupted"] and not report["motion_verified"]:
                raise RuntimeError("Feedback did not meet demonstration bounds; inspect report")
            return report
        finally:
            execution_error = sys.exc_info()[1]
            execution_failed = execution_error is not None
            if execution_failed:
                report["error"] = f"{type(execution_error).__name__}: {execution_error}"
            cleanup_ok = not start_requested
            try:
                if start_requested:
                    # A service timeout can occur after the server starts a task.
                    # Never retry start_task; inspect the actual state before cleanup.
                    current = status()
                    if current.phase == MasterPhase.RUNNING:
                        if current.task_id != "inference":
                            raise RuntimeError("Another task is running; refusing to stop it")
                        accepted(master.stop_task(), "stop_task")
                    final = wait_for(status, lambda s: s.phase != MasterPhase.RUNNING,
                                     "task leaving RUNNING")
                    report["final_phase"] = final.phase.name
                    if final.phase != MasterPhase.STANDBY:
                        raise RuntimeError("Task did not finish in STANDBY")
                    cleanup_ok = True
                    if not task_started:
                        print("Task start was not acknowledged; observed STANDBY after "
                              "cleanup. Check Master again before another run.", file=sys.stderr)
            except Exception as exc:
                report["cleanup_error"] = str(exc)
                print("Could not confirm task cleanup. Inspect Master with sdk_read_state; "
                      "stop the inference task from Sharpa North App if it is still running. "
                      "Do not retry start_task blindly.", file=sys.stderr)
                raise
            finally:
                report["passed"] = (not execution_failed and cleanup_ok and
                                    (not args.execute or (report["return_verified"] and
                                     (report["interrupted"] or report["motion_verified"]))))
                report["outcome"] = (
                    "failed" if not report["passed"] else
                    "preview" if not args.execute else
                    "interrupted_returned" if report["interrupted"] else "completed")
                for subscription in subscriptions:
                    subscription.close()
                if args.output:
                    args.output.parent.mkdir(parents=True, exist_ok=True)
                    args.output.write_text(json.dumps(report, indent=2) + "\n")
                print(json.dumps(report), flush=True)


def arguments(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument("--preview", dest="execute", action="store_false",
                      help="Read state and exit without starting a task or sending commands")
    mode.add_argument("--execute", dest="execute", action="store_true", help=argparse.SUPPRESS)
    parser.set_defaults(execute=True)
    parser.add_argument("--arms-only", action="store_true")
    parser.add_argument("--side", choices=("left", "right", "both"), default="both")
    parser.add_argument("--transition-duration", "--move-duration", type=float, default=4.0,
                        help="Minimum smooth transition duration in seconds (default: 4)")
    parser.add_argument("--hold-duration", type=float, default=0.0,
                        help="Seconds to hold the thumbs-up; 0 means until Ctrl+C")
    parser.add_argument("--rate", "--hz", type=float, default=50.0,
                        help="Command rate in Hz (default: 50)")
    parser.add_argument("--return-timeout", type=float, default=8.0)
    parser.add_argument("--tolerance", dest="return_tolerance_deg", type=float, default=1.0,
                        help="Maximum return error in degrees (default: 1)")
    parser.add_argument("--output", type=Path)
    args = parser.parse_args(argv)
    for name in ("transition_duration", "hold_duration", "rate", "return_timeout",
                 "return_tolerance_deg"):
        value = getattr(args, name)
        if not math.isfinite(value) or value < 0 or (name != "hold_duration" and value == 0):
            parser.error(f"{name} must be finite and " +
                         ("nonnegative" if name == "hold_duration" else "positive"))
    return args


if __name__ == "__main__":
    try:
        main(arguments())
    except KeyboardInterrupt:
        print("Interrupted before motion or during return: commands stopped and "
              "owned task stop requested; return is unverified.", file=sys.stderr)
        sys.exit(130)
    except Exception as exc:
        print(f"SDK example failed: {exc}", file=sys.stderr)
        sys.exit(1)
