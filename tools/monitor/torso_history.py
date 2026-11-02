"""Thread-safe, rate-limited histories for the torso monitor."""

from __future__ import annotations

import math
import threading
import time
from collections import deque
from dataclasses import dataclass
from typing import Dict, Iterable, List, Mapping, Optional, Sequence, Tuple

from torso_kinematics import ParallelWaistKinematics
from torso_protocol import CONTROL_MODE_MIT, CONTROL_MODE_POSITION


AXIS_COUNT = 8
CHANNELS = ("zenoh_action", "zenoh_state", "wire_action", "wire_state")
WIRE_CHANNELS = ("wire_action", "wire_state")


@dataclass(frozen=True)
class MonitorSnapshot:
    samples: Mapping[str, List[List[Tuple[float, float]]]]
    counts: Mapping[str, int]
    errors: Mapping[str, int]
    last_times: Mapping[str, Optional[float]]
    last_error: str
    now: float
    motor_ids: Tuple[int, ...]
    control_modes: Tuple[Optional[str], ...]
    mode_last_times: Tuple[Optional[float], ...]
    display_spaces: Tuple[str, ...]


class MonitorHistory:
    def __init__(
        self,
        motor_ids: Sequence[int],
        motor_signs: Sequence[float],
        history_seconds: float = 20.0,
        plot_rate_hz: float = 20.0,
        clock=time.monotonic,
    ):
        if len(motor_ids) != AXIS_COUNT or len(motor_signs) != AXIS_COUNT:
            raise ValueError("torso history requires eight motors and eight signs")
        if history_seconds <= 0.0 or plot_rate_hz <= 0.0:
            raise ValueError("history_seconds and plot_rate_hz must be positive")
        self._clock = clock
        self._started = clock()
        self._history_seconds = history_seconds
        self._interval = 1.0 / plot_rate_hz
        self._motor_ids = tuple(motor_ids)
        self._motor_to_axis = {motor_id: axis for axis, motor_id in enumerate(motor_ids)}
        self._motor_signs = tuple(motor_signs)
        self._kinematics = ParallelWaistKinematics()
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
        self._control_modes: List[Optional[str]] = [None] * AXIS_COUNT
        self._mode_last_times: List[Optional[float]] = [None] * AXIS_COUNT
        self._counts = {channel: 0 for channel in CHANNELS}
        self._errors: Dict[str, int] = {
            "zenoh_decode": 0,
            "wire_decode": 0,
            "waist_fk": 0,
            "plot": 0,
            "source": 0,
        }
        self._last_times: Dict[str, Optional[float]] = {
            channel: None for channel in CHANNELS
        }
        self._last_error = ""

    @property
    def started(self) -> float:
        return self._started

    def _relative_now(self) -> float:
        return self._clock() - self._started

    def add_zenoh(self, channel: str, joint_radians: Iterable[float]) -> bool:
        if channel not in ("zenoh_action", "zenoh_state"):
            raise ValueError(f"invalid Zenoh channel: {channel}")
        values = tuple(joint_radians)
        if len(values) < AXIS_COUNT or not all(math.isfinite(value) for value in values[:AXIS_COUNT]):
            self.record_error("zenoh_decode", "Zenoh torso message has invalid joint positions")
            return False
        now = self._relative_now()
        with self._lock:
            self._counts[channel] += 1
            self._last_times[channel] = now
            if now - self._last_plot[channel][0] < self._interval:
                return True
            for axis, radians in enumerate(values[:AXIS_COUNT]):
                self._append_locked(channel, axis, now, math.degrees(radians))
            self._trim_locked(now)
        return True

    def add_wire(
        self,
        channel: str,
        motor_radians: Mapping[int, float],
        control_mode: Optional[str] = None,
    ) -> bool:
        if channel not in ("wire_action", "wire_state"):
            raise ValueError(f"invalid wire channel: {channel}")
        if control_mode not in (None, CONTROL_MODE_POSITION, CONTROL_MODE_MIT):
            raise ValueError(f"invalid torso control mode: {control_mode}")
        now = self._relative_now()
        with self._lock:
            self._counts[channel] += 1
            self._last_times[channel] = now
            valid_values = []
            for motor_id, radians in motor_radians.items():
                axis = self._motor_to_axis.get(motor_id)
                if axis is None or not math.isfinite(radians):
                    continue
                valid_values.append((axis, radians))

            if channel == "wire_action" and control_mode is not None:
                mode_changed_axes = set()
                for axis, _radians in valid_values:
                    if self._control_modes[axis] == control_mode:
                        continue
                    mode_changed_axes.add(axis)
                self._clear_wire_axes_locked(mode_changed_axes)
                for axis, _radians in valid_values:
                    self._control_modes[axis] = control_mode
                    self._mode_last_times[axis] = now

            changed_axes = []
            cache = self._last_wire_motor[channel]
            for axis, radians in valid_values:
                cache[axis] = radians
                changed_axes.append(axis)
                if now - self._last_wire_motor_plot[channel][axis] >= self._interval:
                    self._wire_motor_samples[channel][axis].append((now, radians))
                    self._last_wire_motor_plot[channel][axis] = now

            for axis in changed_axes:
                if axis in (2, 3) or now - self._last_plot[channel][axis] < self._interval:
                    continue
                joint_radians = self._motor_signs[axis] * cache[axis]
                self._append_locked(channel, axis, now, math.degrees(joint_radians))

            if (
                any(axis in (2, 3) for axis in changed_axes)
                and math.isfinite(cache[2])
                and math.isfinite(cache[3])
                and now - min(self._last_plot[channel][2], self._last_plot[channel][3])
                >= self._interval
            ):
                waist = self._kinematics.motor_to_joint(cache[2], cache[3])
                if waist is None:
                    self._errors["waist_fk"] += 1
                    self._last_error = f"{channel}: parallel waist FK failed"
                else:
                    self._append_locked(channel, 2, now, math.degrees(waist[0]))
                    self._append_locked(channel, 3, now, math.degrees(waist[1]))
            self._trim_locked(now)
        return bool(changed_axes)

    def _clear_wire_axes_locked(self, axes: Iterable[int]) -> None:
        mode_changed_axes = set(axes)
        joint_axes = set(mode_changed_axes)
        if mode_changed_axes.intersection((2, 3)):
            joint_axes.update((2, 3))
        for channel in WIRE_CHANNELS:
            for axis in joint_axes:
                self._samples[channel][axis].clear()
                self._last_plot[channel][axis] = -math.inf
                self._last_wire_motor[channel][axis] = math.nan
            for axis in mode_changed_axes:
                self._wire_motor_samples[channel][axis].clear()
                self._last_wire_motor_plot[channel][axis] = -math.inf

    def _append_locked(self, channel: str, axis: int, now: float, degrees: float) -> None:
        self._samples[channel][axis].append((now, degrees))
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
            display_spaces = tuple(
                "motor" if mode == CONTROL_MODE_MIT else "joint"
                for mode in self._control_modes
            )
            selected_samples = {}
            for channel in CHANNELS:
                selected_samples[channel] = [
                    list(self._wire_motor_samples[channel][axis])
                    if channel in WIRE_CHANNELS and display_spaces[axis] == "motor"
                    else list(self._samples[channel][axis])
                    for axis in range(AXIS_COUNT)
                ]
            return MonitorSnapshot(
                samples=selected_samples,
                counts=dict(self._counts),
                errors=dict(self._errors),
                last_times=dict(self._last_times),
                last_error=self._last_error,
                now=now,
                motor_ids=self._motor_ids,
                control_modes=tuple(self._control_modes),
                mode_last_times=tuple(self._mode_last_times),
                display_spaces=display_spaces,
            )
