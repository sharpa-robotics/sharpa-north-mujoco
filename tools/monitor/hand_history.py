"""Thread-safe, rate-limited histories for one monitored 22-axis hand."""

from __future__ import annotations

import math
import threading
import time
from collections import deque
from dataclasses import dataclass
from typing import Dict, Iterable, List, Mapping, Optional, Tuple


AXIS_COUNT = 22
CHANNELS = ("zenoh_action", "zenoh_state", "wire_action", "wire_state")


@dataclass(frozen=True)
class MonitorSnapshot:
    samples: Mapping[str, List[List[Tuple[float, float]]]]
    counts: Mapping[str, int]
    errors: Mapping[str, int]
    last_times: Mapping[str, Optional[float]]
    last_error: str
    now: float


class MonitorHistory:
    def __init__(
        self,
        history_seconds: float = 30.0,
        plot_rate_hz: float = 20.0,
        clock=time.monotonic,
    ):
        if history_seconds <= 0.0 or plot_rate_hz <= 0.0:
            raise ValueError("history_seconds and plot_rate_hz must be positive")
        self._clock = clock
        self._started = clock()
        self._history_seconds = history_seconds
        self._interval = 1.0 / plot_rate_hz
        self._lock = threading.Lock()
        self._samples = {
            channel: [deque() for _ in range(AXIS_COUNT)] for channel in CHANNELS
        }
        self._last_plot = {channel: -math.inf for channel in CHANNELS}
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

    def _relative_now(self) -> float:
        return self._clock() - self._started

    def add(self, channel: str, joint_radians: Iterable[float]) -> bool:
        if channel not in CHANNELS:
            raise ValueError(f"invalid hand monitor channel: {channel}")
        values = tuple(joint_radians)
        if len(values) < AXIS_COUNT or not all(
            math.isfinite(value) for value in values[:AXIS_COUNT]
        ):
            kind = "zenoh_decode" if channel.startswith("zenoh") else "wire_decode"
            self.record_error(kind, f"{channel} has invalid joint positions")
            return False
        now = self._relative_now()
        with self._lock:
            self._counts[channel] += 1
            self._last_times[channel] = now
            if now - self._last_plot[channel] < self._interval:
                return True
            for axis, radians in enumerate(values[:AXIS_COUNT]):
                self._samples[channel][axis].append((now, math.degrees(radians)))
            self._last_plot[channel] = now
            self._trim_locked(now)
        return True

    def _trim_locked(self, now: float) -> None:
        cutoff = now - self._history_seconds
        for channel in CHANNELS:
            for samples in self._samples[channel]:
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
            return MonitorSnapshot(
                samples={
                    channel: [list(axis) for axis in self._samples[channel]]
                    for channel in CHANNELS
                },
                counts=dict(self._counts),
                errors=dict(self._errors),
                last_times=dict(self._last_times),
                last_error=self._last_error,
                now=now,
            )
