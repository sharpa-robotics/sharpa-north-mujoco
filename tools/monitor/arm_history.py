"""Thread-safe, rate-limited histories for one monitored seven-axis arm."""

from __future__ import annotations

import math
import threading
import time
from collections import deque
from dataclasses import dataclass
from typing import Dict, Iterable, List, Mapping, Optional, Tuple

from arm_transmission import AXIS_COUNT, ArmTransmission


CHANNELS = ("zenoh_action", "zenoh_state", "wire_action", "wire_state")
WIRE_CHANNELS = ("wire_action", "wire_state")
CONTROL_MODE_NAMES = {
    0: "JointPosition",
    1: "JointPd",
    2: "JointTorque",
    3: "Speed",
    4: "Floating",
    5: "MotorPd",
    6: "CartesianImpedance",
}
MOTOR_VIEW_CONTROL_MODES = frozenset((5,))
JOINT_MOTOR_DEPENDENCIES = (
    (0,),
    (1,),
    (2,),
    (3, 4),
    (3, 4),
    (5, 6),
    (5, 6),
)


@dataclass(frozen=True)
class MonitorSnapshot:
    samples: Mapping[str, List[List[Tuple[float, float]]]]
    counts: Mapping[str, int]
    errors: Mapping[str, int]
    last_times: Mapping[str, Optional[float]]
    last_error: str
    now: float
    control_mode: Optional[int]
    control_mode_name: str
    control_mode_source: str
    display_space: str


class MonitorHistory:
    def __init__(
        self,
        side: str,
        history_seconds: float = 30.0,
        plot_rate_hz: float = 20.0,
        clock=time.monotonic,
    ):
        if history_seconds <= 0.0 or plot_rate_hz <= 0.0:
            raise ValueError("history_seconds and plot_rate_hz must be positive")
        self._transmission = ArmTransmission(side)
        self._clock = clock
        self._started = clock()
        self._history_seconds = history_seconds
        self._interval = 1.0 / plot_rate_hz
        self._lock = threading.Lock()
        self._samples = {
            channel: [deque() for _ in range(AXIS_COUNT)] for channel in CHANNELS
        }
        self._wire_motor_samples = {
            channel: [deque() for _ in range(AXIS_COUNT)]
            for channel in WIRE_CHANNELS
        }
        self._last_plot = {
            channel: [-math.inf] * AXIS_COUNT for channel in CHANNELS
        }
        self._last_wire_motor = {
            "wire_action": [math.nan] * AXIS_COUNT,
            "wire_state": [math.nan] * AXIS_COUNT,
        }
        self._last_wire_motor_plot = {
            channel: [-math.inf] * AXIS_COUNT for channel in WIRE_CHANNELS
        }
        self._counts = {channel: 0 for channel in CHANNELS}
        self._errors: Dict[str, int] = {
            "zenoh_decode": 0,
            "wire_decode": 0,
            "plot": 0,
            "source": 0,
        }
        self._last_times: Dict[str, Optional[float]] = {
            channel: None for channel in CHANNELS
        }
        self._last_error = ""
        self._control_mode: Optional[int] = None
        self._control_mode_source = "unknown"

    def _relative_now(self) -> float:
        return self._clock() - self._started

    def set_control_mode(self, mode: int, source: str = "heartbeat") -> bool:
        """Record the platform arm mode; an exact heartbeat beats UDP inference."""
        try:
            value = int(mode)
        except (TypeError, ValueError):
            self.record_error("zenoh_decode", f"invalid arm control mode: {mode!r}")
            return False
        now = self._relative_now()
        with self._lock:
            if self._control_mode_source == "heartbeat" and source != "heartbeat":
                return False
            if self._control_mode != value:
                self._clear_samples_locked()
            self._control_mode = value
            self._control_mode_source = source
            self._last_times["control_mode"] = now
        return True

    def observe_udp_mode(self, driver_mode: int, has_mit_gains: bool) -> bool:
        """Infer only UDP modes that map unambiguously to a platform mode."""
        inferred = {8: 0, 9: 3, 10: 2}.get(int(driver_mode))
        if int(driver_mode) == 11 and has_mit_gains:
            inferred = 5
        if inferred is None:
            return False
        return self.set_control_mode(inferred, source="UDP inference")

    def _clear_samples_locked(self) -> None:
        for channel in CHANNELS:
            for samples in self._samples[channel]:
                samples.clear()
            self._last_plot[channel] = [-math.inf] * AXIS_COUNT
        for channel in WIRE_CHANNELS:
            for samples in self._wire_motor_samples[channel]:
                samples.clear()
            self._last_wire_motor[channel] = [math.nan] * AXIS_COUNT
            self._last_wire_motor_plot[channel] = [-math.inf] * AXIS_COUNT

    def add_zenoh(self, channel: str, joint_radians: Iterable[float]) -> bool:
        if channel not in ("zenoh_action", "zenoh_state"):
            raise ValueError(f"invalid arm Zenoh channel: {channel}")
        values = tuple(joint_radians)
        if len(values) < AXIS_COUNT or not all(
            math.isfinite(value) for value in values[:AXIS_COUNT]
        ):
            self.record_error(
                "zenoh_decode", f"{channel} has invalid arm joint positions"
            )
            return False
        now = self._relative_now()
        with self._lock:
            self._counts[channel] += 1
            self._last_times[channel] = now
            if now - self._last_plot[channel][0] < self._interval:
                return True
            for axis, radians in enumerate(values[:AXIS_COUNT]):
                self._append_locked(channel, axis, radians, now)
            self._trim_locked(now)
        return True

    def add_wire(self, channel: str, motor_radians: Mapping[int, float]) -> bool:
        if channel not in ("wire_action", "wire_state"):
            raise ValueError(f"invalid arm wire channel: {channel}")
        values = tuple(motor_radians.items())
        if not values or any(
            motor < 0 or motor >= AXIS_COUNT or not math.isfinite(radians)
            for motor, radians in values
        ):
            self.record_error(
                "wire_decode", f"{channel} has invalid arm motor positions"
            )
            return False
        now = self._relative_now()
        with self._lock:
            self._counts[channel] += 1
            self._last_times[channel] = now
            cache = self._last_wire_motor[channel]
            fresh_motors = set()
            for motor, radians in values:
                cache[motor] = radians
                fresh_motors.add(motor)
                if now - self._last_wire_motor_plot[channel][motor] >= self._interval:
                    self._wire_motor_samples[channel][motor].append((now, radians))
                    self._last_wire_motor_plot[channel][motor] = now

            # Each control tick arrives as five endpoint datagrams:
            # M0, M1, M2, M3+M4 and M5+M6.  Only append joints whose complete
            # motor dependency is fresh in this datagram.  Appending all seven
            # cached joints for every datagram creates four stale duplicates per
            # joint and makes a smooth command look stepped/jagged.
            joints = self._transmission.motor_to_joint(cache)
            for axis, dependencies in enumerate(JOINT_MOTOR_DEPENDENCIES):
                if not fresh_motors.issuperset(dependencies):
                    continue
                if now - self._last_plot[channel][axis] < self._interval:
                    continue
                self._append_locked(channel, axis, joints[axis], now)
            self._trim_locked(now)
        return True

    def _append_locked(
        self, channel: str, axis: int, joint_radians: float, now: float
    ) -> None:
        self._samples[channel][axis].append((now, math.degrees(joint_radians)))
        self._last_plot[channel][axis] = now

    def _trim_locked(self, now: float) -> None:
        cutoff = now - self._history_seconds
        for channel in CHANNELS:
            for samples in self._samples[channel]:
                while samples and samples[0][0] < cutoff:
                    samples.popleft()
        for channel in WIRE_CHANNELS:
            for samples in self._wire_motor_samples[channel]:
                while samples and samples[0][0] < cutoff:
                    samples.popleft()

    def record_error(self, kind: str, message: str) -> None:
        with self._lock:
            self._errors[kind] = self._errors.get(kind, 0) + 1
            self._last_error = message

    def snapshot(self) -> MonitorSnapshot:
        now = self._relative_now()
        with self._lock:
            self._trim_locked(now)
            display_space = (
                "motor" if self._control_mode in MOTOR_VIEW_CONTROL_MODES else "joint"
            )
            selected_samples = {}
            for channel in CHANNELS:
                source = (
                    self._wire_motor_samples[channel]
                    if display_space == "motor" and channel in WIRE_CHANNELS
                    else self._samples[channel]
                )
                selected_samples[channel] = [list(axis) for axis in source]
            return MonitorSnapshot(
                samples=selected_samples,
                counts=dict(self._counts),
                errors=dict(self._errors),
                last_times=dict(self._last_times),
                last_error=self._last_error,
                now=now,
                control_mode=self._control_mode,
                control_mode_name=CONTROL_MODE_NAMES.get(
                    self._control_mode, "Unknown"
                ),
                control_mode_source=self._control_mode_source,
                display_space=display_space,
            )
