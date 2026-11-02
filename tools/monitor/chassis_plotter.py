"""Platform chassis velocity on the left, eight motor-shaft plots on the right."""

import math

from chassis_history import AXIS_NAMES, CHANNELS, MOTOR_NAMES, MOTOR_UNITS, UNITS
from plot_zoom import AxesMaximizeController


REFRESH_MS = 20
COLORS = dict(zip(CHANNELS, ("tab:blue", "tab:green", "tab:purple", "tab:red")))
CODES = dict(zip(CHANNELS, ("ZA", "ZS", "WA", "WS")))
MODE_NAMES = {None: "Unknown", 1: "Position", -3: "Velocity", 4: "Current"}


class MonitorPlot:
    def __init__(self, plt, history, interfaces=(), history_seconds=30.0):
        self._history = history
        self._interfaces = tuple(interfaces)
        self._history_seconds = history_seconds
        self.paused = False
        self.figure = plt.figure(figsize=(16, 12))
        self.figure.canvas.manager.set_window_title("Chassis / motor link monitor")
        columns = self.figure.add_gridspec(1, 2, width_ratios=(1.0, 1.2),
                                           left=0.07, right=0.98, bottom=0.065,
                                           top=0.94, wspace=0.22)
        left_grid = columns[0].subgridspec(3, 1, hspace=0.3)
        right_grid = columns[1].subgridspec(8, 1, hspace=0.75)
        self.zenoh_axes = []
        self.motor_axes = []
        self.axes = []
        self.lines = {channel: [] for channel in CHANNELS}
        self.axis_status = []
        self._views = []
        for grid, axes, names, units, channels, layer in (
            (left_grid, self.zenoh_axes, AXIS_NAMES, UNITS,
             ("zenoh_action", "zenoh_state"), "Zenoh"),
            (right_grid, self.motor_axes, MOTOR_NAMES, MOTOR_UNITS,
             ("wire_action", "wire_state"), "Kinco UDP"),
        ):
            for index, (name, unit) in enumerate(zip(names, units)):
                axis = self.figure.add_subplot(grid[index], sharex=self.axes[0] if self.axes else None)
                axes.append(axis)
                self.axes.append(axis)
                for channel in channels:
                    (line,) = axis.plot([], [], color=COLORS[channel], linewidth=1.1,
                                        label="command" if channel.endswith("action") else "feedback")
                    self.lines[channel].append(line)
                axis.set_title(f"{layer} | {name}", fontsize=9)
                axis.set_ylabel(unit)
                axis.set_ylim(-0.1, 0.1)
                axis.grid(True, alpha=0.3)
                if index == 0:
                    axis.legend(loc="upper right", ncol=2, fontsize=7)
                if index == len(names) - 1:
                    axis.set_xlabel("time (s)")
                else:
                    axis.tick_params(axis="x", labelbottom=False)
                status = axis.text(0.01, 0.04, "", transform=axis.transAxes,
                                   ha="left", va="bottom", fontsize=7,
                                   bbox={"facecolor": "white", "alpha": 0.8, "edgecolor": "none", "pad": 1})
                self.axis_status.append(status)
                self._views.append((axis, status, index, name, unit, channels, layer))
        self.status_text = self.figure.text(0.5, 0.995, "", ha="center", va="top", fontsize=9)
        self.profile_text = self.figure.text(
            0.5, 0.015, "Left: chassis body velocity | Right: motor-shaft position / speed (protocol units only)",
            ha="center", va="bottom", fontsize=9,
        )
        from matplotlib.widgets import Button

        button_axis = self.figure.add_axes((0.93, 0.964, 0.055, 0.022))
        self.pause_button = Button(button_axis, "Pause", color="#eeeeee", hovercolor="#d8e7ff")
        self.pause_button.on_clicked(self.toggle_pause)
        self.maximize_controller = AxesMaximizeController(
            self.figure, self.axes, auxiliary_axes=(button_axis,),
            auxiliary_artists=(self.status_text, self.profile_text),
        )

    def toggle_pause(self, _event=None):
        self.paused = not self.paused
        self.pause_button.label.set_text("Resume" if self.paused else "Pause")
        self.figure.canvas.draw_idle()

    def update(self, _frame=None):
        artists = [line for lines in self.lines.values() for line in lines]
        if self.paused:
            return artists
        snapshot = self._history.snapshot()
        self.zenoh_axes[-1].set_xlim(max(0.0, snapshot.now - self._history_seconds),
                                     max(self._history_seconds, snapshot.now))
        for axis, status, index, name, unit, channels, layer in self._views:
            visible, latest, availability = [], [], []
            for channel in channels:
                samples = snapshot.samples[channel][index]
                reason = snapshot.unavailable[channel][index]
                line = self.lines[channel][index]
                if samples:
                    times, values = zip(*samples)
                    line.set_data(times, values)
                    visible.extend(value for value in values if math.isfinite(value))
                else:
                    line.set_data([], [])
                label = "A" if channel.endswith("action") else "S"
                value = f"{samples[-1][1]:+.3f}" if samples and not reason else "N/A"
                latest.append(f"{label} {value}")
                availability.append(f"{label}: {reason or 'OK'}")
            mode_label = ""
            if channels[0] == "wire_action":
                mode = snapshot.motor_modes[index + 1]
                mode_label = " | " + MODE_NAMES.get(mode, f"Mode {mode}")
            axis.set_title(f"{layer} | {name}{mode_label} | {'  '.join(latest)} {unit}", fontsize=9)
            status.set_text(" | ".join(availability))
            if visible:
                low, high = min(visible), max(visible)
                padding = max(0.05, (high - low) * 0.08)
                axis.set_ylim(low - padding, high + padding)
        self.status_text.set_text(
            f"CHASSIS | Kinco UDP {','.join(self._interfaces)} | "
            + "  ".join(f"{CODES[c]} {snapshot.counts[c]}" for c in CHANNELS)
            + f" | errors {sum(snapshot.errors.values())}"
        )
        return artists


def show_monitor(plt, animation_class, history, interfaces, history_seconds):
    plot = MonitorPlot(plt, history, interfaces, history_seconds)

    def safe_update(frame):
        try:
            return plot.update(frame)
        except Exception as exc:
            history.record_error("plot", f"plot refresh failed: {type(exc).__name__}: {exc}")
            return [line for lines in plot.lines.values() for line in lines]

    animation = animation_class(plot.figure, safe_update, interval=REFRESH_MS,
                                blit=False, cache_frame_data=False)
    plot.figure._chassis_monitor_animation = animation
    plt.show()
