"""Compact Matplotlib view for Wave hand link comparison."""

from __future__ import annotations

from typing import Dict, List, Mapping

from hand_history import CHANNELS, MonitorHistory
from plot_zoom import AxesMaximizeController


DEFAULT_HISTORY_SECONDS = 30.0
REFRESH_MS = 20
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
CHANNEL_LABELS = {
    "zenoh_action": "Zenoh action",
    "zenoh_state": "Zenoh state",
    "wire_action": "Wave action",
    "wire_state": "Wave state",
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
            11, 4, sharex=True, figsize=(24, 19)
        )
        self.figure.canvas.manager.set_window_title(
            f"Wave hand link monitor - {side}"
        )
        self.axes = list(self.axes_grid.flat)
        self.joint_axes = []
        self.lines: Dict[str, list] = {channel: [] for channel in CHANNELS}
        for joint_index, joint_name in enumerate(JOINT_NAMES):
            row = joint_index // 2
            column = (joint_index % 2) * 2
            zenoh_axis = self.axes_grid[row, column]
            wire_axis = self.axes_grid[row, column + 1]
            wire_axis.sharey(zenoh_axis)
            self.joint_axes.append((zenoh_axis, wire_axis))
            for axis, channels, layer in (
                (zenoh_axis, ("zenoh_action", "zenoh_state"), "Zenoh"),
                (wire_axis, ("wire_action", "wire_state"), "Wave UDP"),
            ):
                for channel in channels:
                    (line,) = axis.plot(
                        [],
                        [],
                        linewidth=1.15,
                        label="action" if channel.endswith("action") else "state",
                        **CHANNEL_STYLES[channel],
                    )
                    self.lines[channel].append(line)
                axis.set_title(f"J{joint_index + 1} {joint_name} | {layer}", fontsize=8)
                axis.set_ylim(-30.0, 30.0)
                axis.grid(True, alpha=0.3)
                if column in (0, 2) and axis is zenoh_axis:
                    axis.set_ylabel("angle (deg)", fontsize=8)
                if row == 0:
                    axis.legend(loc="upper right", ncol=2, fontsize=7)
        for axis in self.axes_grid[-1, :]:
            axis.set_xlabel("time (s)")

        self.status_text = self.figure.text(
            0.5, 0.992, "", ha="center", va="top", fontsize=9
        )
        from matplotlib.widgets import Button

        button_axis = self.figure.add_axes((0.935, 0.972, 0.055, 0.018))
        self.pause_button = Button(
            button_axis, "Pause", color="#eeeeee", hovercolor="#d8e7ff"
        )
        self._pause_callback_id = self.pause_button.on_clicked(self.toggle_pause)
        self.figure.subplots_adjust(top=0.958, hspace=0.45, wspace=0.16)
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
        display_end = max(self._history_seconds, snapshot.now)
        display_start = max(0.0, snapshot.now - self._history_seconds)
        self.axes_grid[-1, 0].set_xlim(display_start, display_end)
        artists = []
        for joint_index, joint_name in enumerate(JOINT_NAMES):
            visible_values = []
            latest_by_column = [[], []]
            for column, channels in enumerate(
                (("zenoh_action", "zenoh_state"), ("wire_action", "wire_state"))
            ):
                for channel in channels:
                    samples = snapshot.samples[channel][joint_index]
                    line = self.lines[channel][joint_index]
                    if samples:
                        x_values, y_values = zip(*samples)
                        line.set_data(x_values, y_values)
                        visible_values.extend(y_values)
                        short_label = "A" if channel.endswith("action") else "S"
                        latest_by_column[column].append(
                            f"{short_label} {y_values[-1]:+.2f}°"
                        )
                    else:
                        line.set_data([], [])
                    artists.append(line)
            if visible_values:
                low, high = min(visible_values), max(visible_values)
                if high - low < 10.0:
                    center = 0.5 * (high + low)
                    low, high = center - 5.0, center + 5.0
                else:
                    padding = max(1.0, (high - low) * 0.08)
                    low, high = low - padding, high + padding
                self.joint_axes[joint_index][0].set_ylim(low, high)
            for column, layer in enumerate(("Zenoh", "Wave UDP")):
                suffix = (
                    " | " + "  ".join(latest_by_column[column])
                    if latest_by_column[column]
                    else f" | {layer}"
                )
                self.joint_axes[joint_index][column].set_title(
                    f"J{joint_index + 1} {joint_name}{suffix}", fontsize=8
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
            f"{self._side.upper()} | Wave UDP {','.join(self._interfaces)} | "
            f"ZA/ZS {snapshot.counts['zenoh_action']}/{snapshot.counts['zenoh_state']} | "
            f"WA/WS {snapshot.counts['wire_action']}/{snapshot.counts['wire_state']} | "
            f"E {sum(snapshot.errors.values())} | stale {','.join(stale) or '-'}"
        )
        return artists


def show_monitor(
    plt,
    animation_class,
    histories: Mapping[str, MonitorHistory],
    interfaces,
    history_seconds: float,
) -> None:
    plots = []
    for side, history in histories.items():
        monitor_plot = MonitorPlot(
            plt, history, side, list(interfaces), history_seconds
        )

        def safe_update(frame, plot=monitor_plot, plot_history=history):
            try:
                return plot.update(frame)
            except Exception as exc:
                plot_history.record_error(
                    "plot", f"plot refresh failed: {type(exc).__name__}: {exc}"
                )
                return [line for lines in plot.lines.values() for line in lines]

        animation = animation_class(
            monitor_plot.figure,
            safe_update,
            interval=REFRESH_MS,
            blit=False,
            cache_frame_data=False,
        )
        monitor_plot.figure._hand_monitor_animation = animation
        plots.append(monitor_plot)
    plt.show()
