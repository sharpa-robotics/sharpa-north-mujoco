"""Low-latency Matplotlib view for torso link comparison."""

from __future__ import annotations

from typing import Dict, List

from plot_zoom import AxesMaximizeController
from torso_history import CHANNELS, MonitorHistory
from torso_protocol import CONTROL_MODE_MIT, CONTROL_MODE_POSITION


DEFAULT_HISTORY_SECONDS = 30.0
REFRESH_MS = 20
JOINT_NAMES = (
    "lower_body_joint_1",
    "lower_body_joint_2",
    "waist pitch",
    "waist roll",
    "lower_body_joint_5",
    "neck_joint_1",
    "neck_joint_2",
    "neck_joint_3",
)
CHANNEL_LABELS = {
    "zenoh_action": "Zenoh action",
    "zenoh_state": "Zenoh state",
    "wire_action": "CAN/UDP action",
    "wire_state": "CAN/UDP state",
}
CHANNEL_STYLES = {
    "zenoh_action": {"color": "tab:blue", "linestyle": "-"},
    "zenoh_state": {"color": "tab:green", "linestyle": "-"},
    "wire_action": {"color": "tab:purple", "linestyle": "--"},
    "wire_state": {"color": "tab:red", "linestyle": "--"},
}


def _mode_summary(modes) -> str:
    groups = (
        (CONTROL_MODE_POSITION, "CSP"),
        (CONTROL_MODE_MIT, "MIT"),
        (None, "?"),
    )
    parts = []
    for mode, label in groups:
        joints = ",".join(
            f"J{index + 1}" for index, actual in enumerate(modes) if actual == mode
        )
        if joints:
            parts.append(f"{label} {joints}")
    return " | ".join(parts)


class MonitorPlot:
    def __init__(
        self,
        plt,
        history: MonitorHistory,
        transport: str,
        interfaces: List[str],
        history_seconds: float = DEFAULT_HISTORY_SECONDS,
    ):
        self._plt = plt
        self._history = history
        self._transport = transport
        self._interfaces = tuple(interfaces)
        self._history_seconds = history_seconds
        self.paused = False
        self.figure, self.axes_grid = plt.subplots(
            8, 2, sharex=True, figsize=(16, 18)
        )
        self.figure.canvas.manager.set_window_title(
            "Torso link monitor | modes unknown"
        )
        self.axes = list(self.axes_grid.flat)
        self.lines: Dict[str, list] = {channel: [] for channel in CHANNELS}
        column_channels = (
            ("zenoh_action", "zenoh_state"),
            ("wire_action", "wire_state"),
        )
        for joint_index in range(8):
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
                    f"J{joint_index + 1} {JOINT_NAMES[joint_index]} | "
                    + (
                        "Zenoh joint view (deg)"
                        if column == 0
                        else f"{self._transport} reconstructed joint view (deg)"
                    )
                    + " | mode Unknown",
                    fontsize=9,
                )
                axis.set_ylim(-30.0, 30.0)
                axis.grid(True, alpha=0.3)
                axis.set_ylabel("joint angle (deg)")
                if joint_index == 0:
                    axis.legend(loc="upper right", ncol=2, fontsize=8)
        self.axes_grid[-1, 0].set_xlabel("time (s)")
        self.axes_grid[-1, 1].set_xlabel("time (s)")
        # Reserve independent rows for status and legend. Using suptitle and an
        # upper-center legend puts both artists in the same area on Qt/Tk.
        self.status_text = self.figure.text(
            0.5,
            0.99,
            "",
            ha="center",
            va="top",
            fontsize=9,
        )
        # Keep the control in the status strip so it does not consume plot
        # height. Pausing freezes the view only; capture threads keep receiving
        # raw messages so resuming can jump directly to the latest window.
        from matplotlib.widgets import Button

        button_axis = self.figure.add_axes((0.925, 0.968, 0.065, 0.022))
        self.pause_button = Button(
            button_axis,
            "Pause",
            color="#eeeeee",
            hovercolor="#d8e7ff",
        )
        self._pause_callback_id = self.pause_button.on_clicked(self.toggle_pause)
        self.figure.subplots_adjust(top=0.955, hspace=0.32, wspace=0.12)
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
        mode_summary = _mode_summary(snapshot.control_modes)
        self.figure.canvas.manager.set_window_title(
            f"Torso link monitor | {mode_summary}"
        )
        display_end = max(self._history_seconds, snapshot.now)
        display_start = max(0.0, snapshot.now - self._history_seconds)
        # All 16 axes share X. Setting every axis separately makes Matplotlib
        # rebroadcast the same limit to all peers (16x16 callbacks per frame),
        # which can swamp the GUI event loop once the time window starts moving.
        self.axes_grid[-1, 0].set_xlim(display_start, display_end)
        artists = []
        column_channels = (
            ("zenoh_action", "zenoh_state"),
            ("wire_action", "wire_state"),
        )
        for joint_index in range(8):
            control_mode = snapshot.control_modes[joint_index] or "Unknown"
            raw_motor_view = snapshot.display_spaces[joint_index] == "motor"
            for column, channels in enumerate(column_channels):
                visible_values = []
                latest_values = []
                axis = self.axes_grid[joint_index, column]
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
                if visible_values:
                    low, high = min(visible_values), max(visible_values)
                    minimum_span = 0.2 if column == 1 and raw_motor_view else 10.0
                    if high - low < minimum_span:
                        center = 0.5 * (high + low)
                        low = center - minimum_span / 2.0
                        high = center + minimum_span / 2.0
                    else:
                        minimum_padding = (
                            0.02 if column == 1 and raw_motor_view else 1.0
                        )
                        padding = max(minimum_padding, (high - low) * 0.08)
                        low, high = low - padding, high + padding
                    axis.set_ylim(low, high)

                if column == 0:
                    item_label = f"J{joint_index + 1} {JOINT_NAMES[joint_index]}"
                    view_label = "Zenoh joint view (deg)"
                    axis.set_ylabel("joint angle (deg)")
                elif raw_motor_view:
                    item_label = (
                        f"motor_id {snapshot.motor_ids[joint_index]} "
                        f"(J{joint_index + 1} backing motor)"
                    )
                    view_label = f"{self._transport} raw motor view (rad)"
                    axis.set_ylabel("motor position (rad)")
                else:
                    item_label = f"J{joint_index + 1} {JOINT_NAMES[joint_index]}"
                    view_label = f"{self._transport} reconstructed joint view (deg)"
                    axis.set_ylabel("joint angle (deg)")
                axis.set_title(
                    f"{item_label} | {view_label} | mode {control_mode}"
                    + (f" | {'  '.join(latest_values)}" if latest_values else ""),
                    fontsize=8,
                )

        stale = []
        for channel in CHANNELS:
            last_time = snapshot.last_times[channel]
            if last_time is None or snapshot.now - last_time > 1.0:
                stale.append(CHANNEL_LABELS[channel])
        error_count = sum(snapshot.errors.values())
        stale_codes = {
            "zenoh_action": "ZA",
            "zenoh_state": "ZS",
            "wire_action": "WA",
            "wire_state": "WS",
        }
        self.status_text.set_text(
            f"MODES {mode_summary} | {self._transport} {','.join(self._interfaces)} | "
            f"ZA/ZS {snapshot.counts['zenoh_action']}/{snapshot.counts['zenoh_state']} | "
            f"WA/WS {snapshot.counts['wire_action']}/{snapshot.counts['wire_state']} | "
            f"E {error_count} | stale "
            f"{','.join(stale_codes[channel] for channel in CHANNELS if CHANNEL_LABELS[channel] in stale) or '-'}"
        )
        return artists


def show_monitor(
    plt, animation_class, history, transport, interfaces, history_seconds
) -> None:
    monitor_plot = MonitorPlot(
        plt, history, transport, list(interfaces), history_seconds
    )
    def safe_update(frame):
        try:
            return monitor_plot.update(frame)
        except Exception as exc:
            # Matplotlib timer callbacks otherwise propagate through the GUI
            # backend and may close the window without a useful terminal error.
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
    monitor_plot.figure._torso_monitor_animation = animation
    plt.show()
