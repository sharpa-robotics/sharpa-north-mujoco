#!/usr/bin/env python3
"""Unified passive monitor for torso, hand, arm, glove, and chassis paths."""

from __future__ import annotations

import argparse
import os
import sys
import tempfile
from enum import IntEnum
from pathlib import Path


TOOL_DIR = Path(__file__).resolve().parent
REPO_ROOT = TOOL_DIR.parents[1]
MUJOCO_CONFIG = REPO_ROOT / "sharpa_mujoco.conf"
PLATFORM_PROTO_DIR = REPO_ROOT.parent / "sharpa_platform" / "proto"
DEFAULT_HISTORY_SECONDS = 30.0
GUI_REFRESH_HZ = 50
RETAINED_SAMPLE_HZ = float("inf")

# Keep runtime imports independent from stale or corrupt system/user .pyc files.
# This matters for GUI backends such as TkAgg, which are imported only when the
# first figure is created and would otherwise fail after capture has started.
_RUNTIME_PYCACHE = None
if sys.pycache_prefix is None:
    _RUNTIME_PYCACHE = tempfile.TemporaryDirectory(
        prefix=f"sharpa-monitor-pycache-{os.getuid()}-"
    )
    sys.pycache_prefix = _RUNTIME_PYCACHE.name

sys.path.insert(0, str(TOOL_DIR))

from arm_config import load_arm_config  # noqa: E402
from arm_history import MonitorHistory as ArmMonitorHistory  # noqa: E402
from arm_plotter import show_monitor as show_arm_monitor  # noqa: E402
from arm_sources import WireCapture as ArmWireCapture  # noqa: E402
from arm_sources import ZenohCapture as ArmZenohCapture  # noqa: E402
from chassis_config import load_chassis_config  # noqa: E402
from chassis_history import MonitorHistory as ChassisMonitorHistory  # noqa: E402
from chassis_plotter import show_monitor as show_chassis_monitor  # noqa: E402
from chassis_sources import WireCapture as ChassisWireCapture  # noqa: E402
from chassis_sources import ZenohCapture as ChassisZenohCapture  # noqa: E402
from hand_config import load_hand_config  # noqa: E402
from hand_history import MonitorHistory as HandMonitorHistory  # noqa: E402
from hand_plotter import show_monitor as show_hand_monitor  # noqa: E402
from hand_sources import WireCapture as HandWireCapture  # noqa: E402
from hand_sources import ZenohCapture as HandZenohCapture  # noqa: E402
from glove_plotter import show_monitor as show_glove_monitor  # noqa: E402
from glove_sources import ZenohCapture as GloveZenohCapture  # noqa: E402
from torso_config import load_torso_config  # noqa: E402
from torso_history import MonitorHistory as TorsoMonitorHistory  # noqa: E402
from torso_plotter import show_monitor as show_torso_monitor  # noqa: E402
from torso_sources import WireCapture as TorsoWireCapture  # noqa: E402
from torso_sources import ZenohCapture as TorsoZenohCapture  # noqa: E402
from zenoh_connection import ZENOH_ENDPOINT  # noqa: E402


class MonitorPart(IntEnum):
    """Numeric values accepted by the monitor command line."""

    TORSO = 1
    LEFT_HAND = 2
    RIGHT_HAND = 3
    LEFT_ARM = 4
    RIGHT_ARM = 5
    LEFT_GLOVE = 6
    RIGHT_GLOVE = 7
    CHASSIS = 8

    @property
    def hand_side(self):
        if self is MonitorPart.LEFT_HAND:
            return "left"
        if self is MonitorPart.RIGHT_HAND:
            return "right"
        return None

    @property
    def arm_side(self):
        if self is MonitorPart.LEFT_ARM:
            return "left"
        if self is MonitorPart.RIGHT_ARM:
            return "right"
        return None

    @property
    def glove_side(self):
        if self is MonitorPart.LEFT_GLOVE:
            return "left"
        if self is MonitorPart.RIGHT_GLOVE:
            return "right"
        return None


def _load_dependencies():
    generated_proto = PLATFORM_PROTO_DIR / "north_pb2.py"
    if not generated_proto.is_file():
        raise RuntimeError(
            f"missing {generated_proto}; build sharpa_platform protobuf output first"
        )
    sys.path.insert(0, str(PLATFORM_PROTO_DIR))
    try:
        import north_pb2 as proto
    except ImportError as exc:
        raise RuntimeError(f"cannot import north_pb2: {exc}") from exc
    try:
        import zenoh
    except ImportError as exc:
        raise RuntimeError(
            "missing eclipse-zenoh; install tools/monitor/requirements.txt"
        ) from exc
    try:
        os.environ.setdefault(
            "MPLCONFIGDIR",
            str(Path(tempfile.gettempdir()) / f"sharpa-monitor-mpl-{os.getuid()}"),
        )
        import matplotlib.pyplot as plt
        from matplotlib.animation import FuncAnimation
    except ImportError as exc:
        raise RuntimeError(
            "missing matplotlib; install tools/monitor/requirements.txt"
        ) from exc
    return zenoh, proto, plt, FuncAnimation


def _parse_part(value: str) -> MonitorPart:
    try:
        return MonitorPart(int(value))
    except (TypeError, ValueError) as exc:
        raise argparse.ArgumentTypeError(
            "part must be 1 (torso), 2 (left hand), 3 (right hand), "
            "4 (left arm), 5 (right arm), 6 (left glove), 7 (right glove), or 8 (chassis)"
        ) from exc


def parse_args(argv=None):
    parser = argparse.ArgumentParser(
        description="Passively compare torso, hand, arm, glove, or chassis communication layers."
    )
    parser.add_argument(
        "part",
        type=_parse_part,
        metavar="{1,2,3,4,5,6,7,8}",
        help=(
            "1=torso, 2=left hand, 3=right hand, 4=left arm, 5=right arm, "
            "6=left glove, 7=right glove, 8=chassis"
        ),
    )
    parser.add_argument(
        "--time",
        type=float,
        default=DEFAULT_HISTORY_SECONDS,
        metavar="SECONDS",
        help=f"visible history in seconds (default: {DEFAULT_HISTORY_SECONDS:g})",
    )
    args = parser.parse_args(argv)
    if args.time <= 0.0:
        parser.error("--time must be greater than zero")
    return args


def _run_torso(zenoh, proto, plt, animation_class, history_seconds: float) -> int:
    config = load_torso_config(MUJOCO_CONFIG)
    history = TorsoMonitorHistory(
        config.motor_ids,
        config.motor_signs,
        history_seconds=history_seconds,
        plot_rate_hz=RETAINED_SAMPLE_HZ,
    )
    wire_capture = TorsoWireCapture(config, history)
    zenoh_capture = TorsoZenohCapture(zenoh, proto, history)
    try:
        wire_capture.start()
        zenoh_capture.start()
        print(f"[Config] {config.source_path}")
        print(
            f"[Wire] {config.transport_label}: "
            f"{', '.join(wire_capture.interfaces)}"
        )
        print(f"[Zenoh] action/torso + state/torso via {ZENOH_ENDPOINT}")
        print(
            "[View] auto per motor: MIT -> raw wire motor position (rad); "
            "Position/CSP -> reconstructed joint position (deg)"
        )
        print(
            f"[Plot] {history_seconds:g} s history, "
            f"all samples retained, {GUI_REFRESH_HZ} Hz GUI refresh"
        )
        show_torso_monitor(
            plt,
            animation_class,
            history,
            config.transport_label,
            wire_capture.interfaces,
            history_seconds,
        )
        return 0
    finally:
        zenoh_capture.stop()
        wire_capture.stop()


def _run_hand(
    side: str, zenoh, proto, plt, animation_class, history_seconds: float
) -> int:
    config = load_hand_config(MUJOCO_CONFIG, (side,))
    histories = {
        side: HandMonitorHistory(
            history_seconds=history_seconds,
            plot_rate_hz=RETAINED_SAMPLE_HZ,
        )
    }
    wire_capture = HandWireCapture(config, histories)
    zenoh_capture = HandZenohCapture(zenoh, proto, histories)
    try:
        wire_capture.start()
        zenoh_capture.start()
        print(f"[Config] {config.source_path}")
        print(f"[Wave UDP] {', '.join(wire_capture.interfaces)}: {side}")
        print(
            f"[Zenoh] action/{side}_hand + state/{side}_hand "
            f"via {ZENOH_ENDPOINT}"
        )
        print(
            f"[Plot] {history_seconds:g} s history, "
            f"all samples retained, {GUI_REFRESH_HZ} Hz GUI refresh"
        )
        show_hand_monitor(
            plt,
            animation_class,
            histories,
            wire_capture.interfaces,
            history_seconds,
        )
        return 0
    finally:
        zenoh_capture.stop()
        wire_capture.stop()


def _run_arm(
    side: str, zenoh, proto, plt, animation_class, history_seconds: float
) -> int:
    config = load_arm_config(MUJOCO_CONFIG, side)
    history = ArmMonitorHistory(
        side,
        history_seconds=history_seconds,
        plot_rate_hz=RETAINED_SAMPLE_HZ,
    )
    wire_capture = ArmWireCapture(config, history)
    zenoh_capture = ArmZenohCapture(zenoh, proto, side, history)
    try:
        wire_capture.start()
        zenoh_capture.start()
        print(f"[Config] {config.source_path}")
        print(f"[Arm UDP] {', '.join(wire_capture.interfaces)}: {side}")
        print(
            f"[Zenoh] action/{side}_arm + state/{side}_arm + "
            f"system/heartbeat/{side}_arm via {ZENOH_ENDPOINT}"
        )
        print(
            "[View] auto: MotorPd -> raw UDP motor position (rad); "
            "other modes -> reconstructed joint position (deg)"
        )
        print(
            f"[Plot] {history_seconds:g} s history, "
            f"all samples retained, {GUI_REFRESH_HZ} Hz GUI refresh"
        )
        show_arm_monitor(
            plt,
            animation_class,
            history,
            side,
            wire_capture.interfaces,
            history_seconds,
        )
        return 0
    finally:
        zenoh_capture.stop()
        wire_capture.stop()


def _run_glove(
    side: str, zenoh, proto, plt, animation_class, history_seconds: float
) -> int:
    history = HandMonitorHistory(
        history_seconds=history_seconds,
        plot_rate_hz=RETAINED_SAMPLE_HZ,
    )
    zenoh_capture = GloveZenohCapture(zenoh, proto, side, history)
    try:
        zenoh_capture.start()
        print(f"[Zenoh] action/{side}_glove/raw via {ZENOH_ENDPOINT}")
        print(
            f"[Plot] {history_seconds:g} s history, "
            f"all samples retained, {GUI_REFRESH_HZ} Hz GUI refresh"
        )
        show_glove_monitor(
            plt,
            animation_class,
            history,
            side,
            history_seconds,
        )
        return 0
    finally:
        zenoh_capture.stop()


def _run_chassis(zenoh, proto, plt, animation_class, history_seconds: float) -> int:
    config = load_chassis_config(MUJOCO_CONFIG)
    history = ChassisMonitorHistory(config, history_seconds=history_seconds)
    wire_capture = ChassisWireCapture(config, history)
    zenoh_capture = ChassisZenohCapture(zenoh, proto, history)
    try:
        wire_capture.start()
        zenoh_capture.start()
        print(f"[Config] {config.source_path}")
        print(f"[Kinco UDP] {', '.join(wire_capture.interfaces)}: "
              f"{config.peer_ip}:{config.peer_port} <-> {config.bind_ip}:{config.bind_port}")
        print(f"[Zenoh] action/chassis + state/chassis via {ZENOH_ENDPOINT}")
        print("[View] left: Zenoh vx/vy (m/s), vtheta (rad/s); right: Kinco motor command + feedback")
        print("[Wire] M1-M4 motor position (rad), M5-M8 motor speed (rad/s); stretch ignored")
        print("[Units] protocol conversion only; no gearing, calibration, body mapping or chassis FK")
        print(f"[Wire] independent motor samples, {config.sample_timeout:g} s expiry; "
              "current control -> that motor's position/speed command unavailable")
        print(f"[Plot] {history_seconds:g} s history, {GUI_REFRESH_HZ} Hz GUI refresh")
        show_chassis_monitor(plt, animation_class, history, wire_capture.interfaces, history_seconds)
        return 0
    finally:
        zenoh_capture.stop()
        wire_capture.stop()


def run(part: MonitorPart, history_seconds: float = DEFAULT_HISTORY_SECONDS) -> int:
    part = MonitorPart(part)
    zenoh, proto, plt, animation_class = _load_dependencies()
    if part is MonitorPart.CHASSIS:
        return _run_chassis(zenoh, proto, plt, animation_class, history_seconds)
    if part is MonitorPart.TORSO:
        return _run_torso(zenoh, proto, plt, animation_class, history_seconds)
    if part.hand_side is not None:
        return _run_hand(
            part.hand_side,
            zenoh,
            proto,
            plt,
            animation_class,
            history_seconds,
        )
    if part.glove_side is not None:
        return _run_glove(
            part.glove_side,
            zenoh,
            proto,
            plt,
            animation_class,
            history_seconds,
        )
    return _run_arm(
        part.arm_side,
        zenoh,
        proto,
        plt,
        animation_class,
        history_seconds,
    )


def main() -> int:
    try:
        args = parse_args()
        return run(args.part, args.time)
    except Exception as exc:
        print(f"[Error] {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
