"""Low-latency Matplotlib view for one raw glove command topic."""

from __future__ import annotations

from hand_history import MonitorHistory
from hand_plotter import JOINT_NAMES
from plot_zoom import AxesMaximizeController


REFRESH_MS = 20


class MonitorPlot:
    def __init__(
        self,
        plt,
        history: MonitorHistory,
        side: str,
        history_seconds: float,
    ):
        self._history = history
        self._side = side
        self._history_seconds = history_seconds
        self.paused = False
        self.figure, self.axes_grid = plt.subplots(
            11, 2, sharex=True, figsize=(14, 19)
        )
        self.figure.canvas.manager.set_window_title(
            f"Raw glove monitor - {side}"
        )
        self.axes = list(self.axes_grid.flat)
        self.lines = []
        for joint_index, (axis, joint_name) in enumerate(
            zip(self.axes, JOINT_NAMES)
        ):
            (line,) = axis.plot([], [], color="tab:blue", linewidth=1.15)
            self.lines.append(line)
            axis.set_title(f"J{joint_index + 1} {joint_name}", fontsize=8)
            axis.set_ylim(-30.0, 30.0)
            axis.grid(True, alpha=0.3)
            if joint_index % 2 == 0:
                axis.set_ylabel("angle (deg)", fontsize=8)
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
        self.pause_button.on_clicked(self.toggle_pause)
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
            return self.lines
        snapshot = self._history.snapshot()
        display_end = max(self._history_seconds, snapshot.now)
        display_start = max(0.0, snapshot.now - self._history_seconds)
        self.axes_grid[-1, 0].set_xlim(display_start, display_end)
        for joint_index, axis in enumerate(self.axes):
            samples = snapshot.samples["zenoh_action"][joint_index]
            line = self.lines[joint_index]
            if samples:
                x_values, y_values = zip(*samples)
                line.set_data(x_values, y_values)
                low, high = min(y_values), max(y_values)
                if high - low < 10.0:
                    center = 0.5 * (high + low)
                    low, high = center - 5.0, center + 5.0
                else:
                    padding = max(1.0, (high - low) * 0.08)
                    low, high = low - padding, high + padding
                axis.set_ylim(low, high)
                axis.set_title(
                    f"J{joint_index + 1} {JOINT_NAMES[joint_index]} | "
                    f"{y_values[-1]:+.2f}°",
                    fontsize=8,
                )
            else:
                line.set_data([], [])
        last_time = snapshot.last_times["zenoh_action"]
        stale = last_time is None or snapshot.now - last_time > 1.0
        self.status_text.set_text(
            f"{self._side.upper()} | action/{self._side}_glove/raw | "
            f"received {snapshot.counts['zenoh_action']} | "
            f"errors {sum(snapshot.errors.values())} | "
            f"stale {'yes' if stale else 'no'}"
        )
        return self.lines


def show_monitor(
    plt,
    animation_class,
    history: MonitorHistory,
    side: str,
    history_seconds: float,
) -> None:
    monitor_plot = MonitorPlot(plt, history, side, history_seconds)

    def safe_update(frame):
        try:
            return monitor_plot.update(frame)
        except Exception as exc:
            history.record_error(
                "plot", f"plot refresh failed: {type(exc).__name__}: {exc}"
            )
            return monitor_plot.lines

    animation = animation_class(
        monitor_plot.figure,
        safe_update,
        interval=REFRESH_MS,
        blit=False,
        cache_frame_data=False,
    )
    monitor_plot.figure._glove_monitor_animation = animation
    plt.show()
