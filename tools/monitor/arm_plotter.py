"""Low-latency Matplotlib view for one arm's link comparison."""

from __future__ import annotations

from typing import Dict, List

from arm_history import CHANNELS, MonitorHistory
from plot_zoom import AxesMaximizeController


DEFAULT_HISTORY_SECONDS = 30.0
REFRESH_MS = 20
JOINT_NAMES = tuple(f"arm_joint_{index}" for index in range(1, 8))
MOTOR_NAMES = tuple(f"arm_motor_{index}" for index in range(7))
CHANNEL_LABELS = {
    "zenoh_action": "Zenoh action",
    "zenoh_state": "Zenoh state",
    "wire_action": "UDP action",
    "wire_state": "UDP state",
}
CHANNEL_STYLES = {
    "zenoh_action": {"color": "tab:blue", "linestyle": "-"},
    "zenoh_state": {"color": "tab:green", "linestyle": "-"},
    "wire_action": {"color": "tab:purple", "linestyle": "--"},
    "wire_state": {"color": "tab:red", "linestyle": "--"},
}


class MonitorPlot:
    def __init__(
        self,
        plt,
        history: MonitorHistory,
        side: str,
        interfaces: List[str],
        history_seconds: float = DEFAULT_HISTORY_SECONDS,
    ):
        self._history = history
        self._side = side
        self._interfaces = tuple(interfaces)
        self._history_seconds = history_seconds
        self.paused = False
        self.figure, self.axes_grid = plt.subplots(
            7, 2, sharex=True, figsize=(16, 17)
        )
        self.figure.canvas.manager.set_window_title(
            f"Arm link monitor - {side}"
        )
        self.axes = list(self.axes_grid.flat)
        self.lines: Dict[str, list] = {channel: [] for channel in CHANNELS}
        column_channels = (
            ("zenoh_action", "zenoh_state"),
            ("wire_action", "wire_state"),
        )
        for joint_index, joint_name in enumerate(JOINT_NAMES):
            for column, channels in enumerate(column_channels):
                axis = self.axes_grid[joint_index, column]
                for channel in channels:
                    (line,) = axis.plot(
                        [],
                        [],
                        linewidth=1.25,
                        label="action" if channel.endswith("action") else "state",
                        **CHANNEL_STYLES[channel],
                    )
                    self.lines[channel].append(line)
                axis.set_title(
                    f"J{joint_index + 1} {joint_name} | "
                    + (
                        "Zenoh joint view (deg)"
                        if column == 0
                        else "UDP reconstructed joint view (deg)"
                    ),
                    fontsize=9,
                )
                axis.set_ylim(-30.0, 30.0)
                axis.grid(True, alpha=0.3)
                axis.set_ylabel("joint angle (deg)")
                if joint_index == 0:
                    axis.legend(loc="upper right", ncol=2, fontsize=8)
        self.axes_grid[-1, 0].set_xlabel("time (s)")
        self.axes_grid[-1, 1].set_xlabel("time (s)")
        self.status_text = self.figure.text(
            0.5, 0.99, "", ha="center", va="top", fontsize=9
        )
        from matplotlib.widgets import Button

        button_axis = self.figure.add_axes((0.925, 0.968, 0.065, 0.022))
        self.pause_button = Button(
            button_axis, "Pause", color="#eeeeee", hovercolor="#d8e7ff"
        )
        self._pause_callback_id = self.pause_button.on_clicked(self.toggle_pause)
        self.figure.subplots_adjust(top=0.955, hspace=0.36, wspace=0.12)
        self.maximize_controller = AxesMaximizeController(
            self.figure,
            self.axes,
            auxiliary_axes=(button_axis,),
            auxiliary_artists=(self.status_text,),
        )

    def toggle_pause(self, _event=None):
        self.paused = not self.paused
        self.pause_button.label.set_text("Resume" if self.paused else "Pause")
        self.figure.canvas.draw_idle()

    def update(self, _frame=None):
        if self.paused:
            return [line for lines in self.lines.values() for line in lines]
        snapshot = self._history.snapshot()
        raw_motor_view = snapshot.display_space == "motor"
        mode_code = "?" if snapshot.control_mode is None else str(snapshot.control_mode)
        mode_label = f"{mode_code} {snapshot.control_mode_name}"
        wire_view_label = (
            "UDP raw motor view (rad)"
            if raw_motor_view
            else "UDP reconstructed joint view (deg)"
        )
        self.figure.canvas.manager.set_window_title(
            f"Arm link monitor - {self._side} | mode {mode_label} | "
            f"{wire_view_label}"
        )
        display_end = max(self._history_seconds, snapshot.now)
        display_start = max(0.0, snapshot.now - self._history_seconds)
        self.axes_grid[-1, 0].set_xlim(display_start, display_end)
        artists = []
        column_channels = (
            ("zenoh_action", "zenoh_state"),
            ("wire_action", "wire_state"),
        )
        for joint_index, joint_name in enumerate(JOINT_NAMES):
            for column, channels in enumerate(column_channels):
                visible_values = []
                latest_values = []
                unit_suffix = " rad" if column == 1 and raw_motor_view else "°"
                for channel in channels:
                    samples = snapshot.samples[channel][joint_index]
                    line = self.lines[channel][joint_index]
                    if samples:
                        x_values, y_values = zip(*samples)
                        line.set_data(x_values, y_values)
                        visible_values.extend(y_values)
                        short_label = "A" if channel.endswith("action") else "S"
                        latest_values.append(
                            f"{short_label} {y_values[-1]:+.3f}{unit_suffix}"
                        )
                    else:
                        line.set_data([], [])
                    artists.append(line)
                axis = self.axes_grid[joint_index, column]
                if visible_values:
                    low, high = min(visible_values), max(visible_values)
                    minimum_span = 0.2 if column == 1 and raw_motor_view else 10.0
                    if high - low < minimum_span:
                        center = 0.5 * (high + low)
                        low, high = center - minimum_span / 2.0, center + minimum_span / 2.0
                    else:
                        minimum_padding = 0.02 if column == 1 and raw_motor_view else 1.0
                        padding = max(minimum_padding, (high - low) * 0.08)
                        low, high = low - padding, high + padding
                    axis.set_ylim(low, high)
                if column == 0:
                    item_label = f"J{joint_index + 1} {joint_name}"
                    view_label = "Zenoh joint view (deg)"
                    axis.set_ylabel("joint angle (deg)")
                elif raw_motor_view:
                    item_label = f"M{joint_index} {MOTOR_NAMES[joint_index]}"
                    view_label = "UDP raw motor view (rad)"
                    axis.set_ylabel("motor position (rad)")
                else:
                    item_label = f"J{joint_index + 1} {joint_name}"
                    view_label = "UDP reconstructed joint view (deg)"
                    axis.set_ylabel("joint angle (deg)")
                axis.set_title(
                    f"{item_label} | {view_label}"
                    + (f" | {'  '.join(latest_values)}" if latest_values else ""),
                    fontsize=8,
                )

        stale_codes = {
            "zenoh_action": "ZA",
            "zenoh_state": "ZS",
            "wire_action": "WA",
            "wire_state": "WS",
        }
        stale = [
            stale_codes[channel]
            for channel in CHANNELS
            if snapshot.last_times[channel] is None
            or snapshot.now - snapshot.last_times[channel] > 1.0
        ]
        self.status_text.set_text(
            f"{self._side.upper()} ARM | MODE {mode_label} "
            f"[{snapshot.control_mode_source}] | VIEW {wire_view_label} | "
            f"UDP {','.join(self._interfaces)} | "
            f"ZA/ZS {snapshot.counts['zenoh_action']}/{snapshot.counts['zenoh_state']} | "
            f"WA/WS {snapshot.counts['wire_action']}/{snapshot.counts['wire_state']} | "
            f"E {sum(snapshot.errors.values())} | stale {','.join(stale) or '-'}"
        )
        return artists


def show_monitor(
    plt,
    animation_class,
    history: MonitorHistory,
    side: str,
    interfaces,
    history_seconds: float,
) -> None:
    monitor_plot = MonitorPlot(
        plt, history, side, list(interfaces), history_seconds
    )

    def safe_update(frame):
        try:
            return monitor_plot.update(frame)
        except Exception as exc:
            history.record_error(
                "plot", f"plot refresh failed: {type(exc).__name__}: {exc}"
            )
            return [line for lines in monitor_plot.lines.values() for line in lines]

    animation = animation_class(
        monitor_plot.figure,
        safe_update,
        interval=REFRESH_MS,
        blit=False,
        cache_frame_data=False,
    )
    monitor_plot.figure._arm_monitor_animation = animation
    plt.show()
