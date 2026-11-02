"""Thread-safe histories for chassis Twist and independent motor-shaft data."""

from collections import deque
from dataclasses import dataclass
import math
import threading
import time

from chassis_config import MOTOR_IDS, ChassisMonitorConfig
from chassis_protocol import position_radians, velocity_radians_per_second


CHANNELS = ("zenoh_action", "zenoh_state", "wire_action", "wire_state")
AXIS_NAMES = ("vx", "vy", "vtheta")
UNITS = ("m/s", "m/s", "rad/s")
MOTOR_NAMES = ("M1 steer A/LF", "M2 steer B/RF", "M3 steer C/RR", "M4 steer D/LR",
               "M5 drive A/LF", "M6 drive B/RF", "M7 drive C/RR", "M8 drive D/LR")
MOTOR_UNITS = ("rad",) * 4 + ("rad/s",) * 4
STALE_SECONDS = 1.0


@dataclass(frozen=True)
class MonitorSnapshot:
    samples: dict
    counts: dict
    errors: dict
    last_times: dict
    unavailable: dict
    motor_modes: dict
    last_error: str
    now: float


class MonitorHistory:
    def __init__(self, config: ChassisMonitorConfig, history_seconds=30.0,
                 clock=time.monotonic):
        if not math.isfinite(history_seconds) or history_seconds <= 0:
            raise ValueError("history_seconds must be finite and positive")
        self._config = config
        self._clock = clock
        self._started = clock()
        self._history_seconds = history_seconds
        self._lock = threading.Lock()
        self._samples = {
            channel: [deque() for _ in (AXIS_NAMES if channel.startswith("zenoh") else MOTOR_IDS)]
            for channel in CHANNELS
        }
        self._counts = dict.fromkeys(CHANNELS, 0)
        self._last_times = {channel: [None] * len(axes) for channel, axes in self._samples.items()}
        self._unavailable = {channel: ["waiting for data"] * len(axes)
                             for channel, axes in self._samples.items()}
        self._errors = {"zenoh_decode": 0, "wire_decode": 0, "source": 0, "plot": 0}
        self._last_error = ""
        self._modes = dict.fromkeys(MOTOR_IDS)
        self._enabled = dict.fromkeys(MOTOR_IDS)
        self._relative_steer = set()

    def _now(self):
        return self._clock() - self._started

    def _gap_locked(self, channel, axis, reason, now):
        if not self._unavailable[channel][axis]:
            self._samples[channel][axis].append((now, math.nan))
        self._unavailable[channel][axis] = reason

    def _invalidate_locked(self, channel, axis, reason, now):
        self._gap_locked(channel, axis, reason, now)
        self._last_times[channel][axis] = None

    def _append_locked(self, channel, axis, value, now):
        self._samples[channel][axis].append((now, value))
        self._last_times[channel][axis] = now
        self._unavailable[channel][axis] = ""

    def add(self, channel, values):
        if channel not in ("zenoh_action", "zenoh_state"):
            raise ValueError(f"invalid chassis Zenoh channel: {channel}")
        values = tuple(values)
        if len(values) != 3 or not all(math.isfinite(value) for value in values):
            self.record_error("zenoh_decode", f"{channel} has invalid velocity")
            return False
        with self._lock:
            now = self._now()
            self._expire_locked(now)
            for axis, value in enumerate(values):
                self._append_locked(channel, axis, value, now)
            self._counts[channel] += 1
            self._trim_locked(now)
        return True

    def mark_unavailable(self, channel, reason):
        with self._lock:
            now = self._now()
            for axis in range(len(self._samples[channel])):
                self._invalidate_locked(channel, axis, reason, now)

    def _target_reason_locked(self, node):
        mode = self._modes[node]
        expected = 1 if node <= 4 else -3
        if mode not in (None, expected):
            return "current control" if mode == 4 else f"unsupported mode {mode}"
        if self._enabled[node] is False:
            return "drive disabled"
        if node in self._relative_steer:
            return "relative position target"
        return ""

    def _expire_locked(self, now):
        for channel in CHANNELS:
            timeout = STALE_SECONDS if channel.startswith("zenoh") else self._config.sample_timeout
            for axis, last in enumerate(self._last_times[channel]):
                reason = self._target_reason_locked(axis + 1) if channel == "wire_action" else ""
                if not reason and last is not None and now - last > timeout:
                    reason = "stale"
                if reason:
                    self._gap_locked(channel, axis, reason, now)

    def add_wire(self, event):
        """Append only this motor's reported/commanded value, never reconstruct Twist."""
        node, kind = event.motor_id, event.kind
        if node not in MOTOR_IDS:
            return
        axis = node - 1
        with self._lock:
            now = self._now()
            self._expire_locked(now)
            expected_mode = 1 if node <= 4 else -3
            if kind == "reset":
                self._modes[node] = self._enabled[node] = None
                self._relative_steer.discard(node)
                for channel in ("wire_action", "wire_state"):
                    self._invalidate_locked(channel, axis, "drive reset; waiting for data", now)
            elif kind in ("mode", "current"):
                mode = 4 if kind == "current" else event.value
                if self._modes[node] != mode:
                    self._invalidate_locked("wire_action", axis, "mode changed; waiting for target", now)
                    self._relative_steer.discard(node)
                self._modes[node] = mode
            elif kind in ("target", "control"):
                control = event.value if kind == "control" else event.control_word
                if control is not None:
                    self._enabled[node] = control & 0xF == 0xF
                    if node <= 4:
                        if control & 0x40:
                            self._relative_steer.add(node)
                        else:
                            self._relative_steer.discard(node)
                reason = self._target_reason_locked(node)
                if reason:
                    self._invalidate_locked("wire_action", axis, reason, now)
                elif kind == "target":
                    # A monitor may join after the initial mode writes. Infer
                    # from the fixed RPDO1/SDO target only while mode is unknown;
                    # never override an observed current/unsupported mode.
                    if self._modes[node] is None:
                        self._modes[node] = expected_mode
                    self._add_motor_locked("wire_action", node, event.value, now)
            elif kind == "feedback":
                self._add_motor_locked("wire_state", node, event.value, now)
            self._expire_locked(now)
            self._trim_locked(now)

    def _add_motor_locked(self, channel, node, raw, now):
        convert = position_radians if node <= 4 else velocity_radians_per_second
        value = convert(raw, self._config.encoder_resolution)
        self._append_locked(channel, node - 1, value, now)
        self._counts[channel] += 1

    def _trim_locked(self, now):
        cutoff = now - self._history_seconds
        for axes in self._samples.values():
            for samples in axes:
                while samples and samples[0][0] < cutoff:
                    samples.popleft()

    def record_error(self, kind, message):
        with self._lock:
            self._errors[kind] = self._errors.get(kind, 0) + 1
            self._last_error = message

    def snapshot(self):
        with self._lock:
            now = self._now()
            self._expire_locked(now)
            self._trim_locked(now)
            return MonitorSnapshot(
                samples={channel: [list(axis) for axis in axes]
                         for channel, axes in self._samples.items()},
                counts=dict(self._counts), errors=dict(self._errors),
                last_times={channel: list(times) for channel, times in self._last_times.items()},
                unavailable={channel: list(reasons) for channel, reasons in self._unavailable.items()},
                motor_modes=dict(self._modes), last_error=self._last_error, now=now,
            )
