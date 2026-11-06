#!/usr/bin/env python3
"""Passively compare Wave hand commands and feedback; never send commands."""

import argparse
import math
import sys
import threading
import time
from pathlib import Path

from hand_motion_driver import JOINT_LIMITS_DEG, JOINT_NAMES
from kinematic_motion import (
    DEFAULT_ZENOH_ENDPOINT, default_platform_root, load_dependencies,
    open_session, run_main,
)


class StreamStats:
    """Accumulate every received sample, independently of the display rate."""

    def __init__(self, message_type):
        self.message_type = message_type
        self.lock = threading.Lock()
        self.count = 0
        self.invalid = 0
        self.last = None
        self.minimum = None
        self.maximum = None
        self.received_at = None
        self.names = ()

    def receive(self, sample):
        try:
            message = self.message_type()
            message.ParseFromString(bytes(sample.payload))
            values = list(message.joints.position)
            if len(values) != len(JOINT_NAMES) or not all(map(math.isfinite, values)):
                raise ValueError("expected 22 finite joint positions")
            values = [math.degrees(value) for value in values]
            with self.lock:
                self.minimum = values[:] if self.minimum is None else [
                    min(a, b) for a, b in zip(self.minimum, values)
                ]
                self.maximum = values[:] if self.maximum is None else [
                    max(a, b) for a, b in zip(self.maximum, values)
                ]
                self.last = values
                self.names = tuple(message.joints.name)
                self.received_at = time.monotonic()
                self.count += 1
        except Exception:
            with self.lock:
                self.invalid += 1

    def snapshot(self):
        with self.lock:
            return {
                "count": self.count, "invalid": self.invalid, "names": self.names,
                "last": None if self.last is None else self.last[:],
                "span": None if self.last is None else [
                    high - low for low, high in zip(self.minimum, self.maximum)
                ],
                "age": None if self.received_at is None else
                    max(0.0, time.monotonic() - self.received_at),
            }


def report(streams, summary=False):
    for side, pair in streams.items():
        command, state = (stream.snapshot() for stream in pair)
        print(f"[{side}] command_frames={command['count']} state_frames={state['count']} "
              f"invalid_command={command['invalid']} invalid_state={state['invalid']}")
        for label, data in (("target", command), ("actual", state)):
            if data["last"] is None:
                print(f"  {label}: NO DATA")
                continue
            worst = max(range(len(JOINT_NAMES)), key=lambda axis: abs(data["last"][axis]))
            print(f"  {label}: age={data['age']:.3f}s J5={data['last'][4]:.4f}deg "
                  f"J5_span={data['span'][4]:.4f}deg "
                  f"largest_abs=J{worst + 1}({JOINT_NAMES[worst]}) "
                  f"{data['last'][worst]:+.4f}deg")
            if data["names"]:
                print(f"  {label} reported names: {','.join(data['names'])}")
        if summary and state["last"] is not None:
            print("  axis name                 target_span  actual_span  actual_last (deg)")
            for axis, name in enumerate(JOINT_NAMES):
                target_span = "NO DATA" if command["span"] is None else f"{command['span'][axis]:.4f}"
                lower, upper = JOINT_LIMITS_DEG[axis]
                outside = " OUTSIDE_SCRIPT_LIMITS" if not lower <= state["last"][axis] <= upper else ""
                print(f"  J{axis + 1:<2} {name:<20} {target_span:>11} "
                      f"{state['span'][axis]:>12.4f} {state['last'][axis]:>12.4f}{outside}")
    sys.stdout.flush()


def observe(args):
    zenoh, proto = load_dependencies(args.platform_root)
    session = open_session(zenoh, args.connect)
    sides = ("left", "right") if args.side == "both" else (args.side,)
    streams = {side: (StreamStats(proto.HandCommand), StreamStats(proto.HandState))
               for side in sides}
    subscribers = []
    try:
        for side, (command, state) in streams.items():
            subscribers.append(session.declare_subscriber(f"action/{side}_hand", command.receive))
            subscribers.append(session.declare_subscriber(f"state/{side}_hand", state.receive))
        print("[Listen] read-only; all displayed angles are degrees; Ctrl+C prints summary")
        print("[Listen] spans cover this listener's entire observation window, including cleanup")
        deadline = time.monotonic() + args.duration
        while time.monotonic() < deadline:
            time.sleep(min(args.interval, max(0.0, deadline - time.monotonic())))
            report(streams)
    except KeyboardInterrupt:
        pass
    finally:
        for subscriber in subscribers:
            subscriber.undeclare()
        session.close()
    print("[Summary] span = maximum - minimum over every valid received frame")
    report(streams, summary=True)
    return 0 if all(pair[1].snapshot()["count"] for pair in streams.values()) else 1


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--connect", default=DEFAULT_ZENOH_ENDPOINT)
    parser.add_argument("--platform-root", type=Path, default=default_platform_root(__file__))
    parser.add_argument("--side", choices=("left", "right", "both"), default="both")
    parser.add_argument("--duration", type=float, default=60.0, help="Observation seconds (default: 60)")
    parser.add_argument("--interval", type=float, default=1.0, help="Display interval in seconds")
    args = parser.parse_args()
    if any(not math.isfinite(value) or value <= 0 for value in (args.duration, args.interval)):
        parser.error("--duration and --interval must be finite and positive")
    return args


if __name__ == "__main__":
    sys.exit(run_main(lambda: observe(parse_args())))
