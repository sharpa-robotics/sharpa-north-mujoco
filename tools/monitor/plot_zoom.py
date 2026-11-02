"""Lightweight per-axis maximize controls for monitor figures."""

from __future__ import annotations

from typing import Iterable, Optional


class AxesMaximizeController:
    """Maximize one subplot inside its figure and restore the original grid."""

    def __init__(
        self,
        figure,
        axes: Iterable,
        *,
        auxiliary_axes: Iterable = (),
        auxiliary_artists: Iterable = (),
    ):
        self.figure = figure
        self.axes = tuple(axes)
        self.auxiliary_axes = tuple(auxiliary_axes)
        self.auxiliary_artists = tuple(auxiliary_artists)
        self.maximized_axis: Optional[object] = None
        self._original_positions = {
            axis: axis.get_position().frozen() for axis in self.axes
        }
        self._original_visibility = {
            axis: axis.get_visible() for axis in self.axes
        }
        self._original_auxiliary_axis_visibility = {
            axis: axis.get_visible() for axis in self.auxiliary_axes
        }
        self._original_auxiliary_artist_visibility = {
            artist: artist.get_visible() for artist in self.auxiliary_artists
        }
        self._original_xlabels = {
            axis: axis.get_xlabel() for axis in self.axes
        }
        self._original_labelbottom = {
            axis: any(label.get_visible() for label in axis.get_xticklabels())
            for axis in self.axes
        }
        self.buttons = {}
        self._button_to_axis = {}

        # Boxed Text artists are substantially lighter than creating a nested
        # matplotlib.widgets.Button axis for every subplot (up to 44 here).
        for axis in self.axes:
            button = axis.text(
                0.008,
                0.975,
                "MAX",
                transform=axis.transAxes,
                ha="left",
                va="top",
                fontsize=6,
                color="#202020",
                zorder=20,
                picker=True,
                bbox={
                    "boxstyle": "round,pad=0.18",
                    "facecolor": "#eeeeee",
                    "edgecolor": "#777777",
                    "linewidth": 0.6,
                    "alpha": 0.9,
                },
            )
            self.buttons[axis] = button
            self._button_to_axis[button] = axis

        self._pick_callback_id = self.figure.canvas.mpl_connect(
            "pick_event", self._on_pick
        )
        self._key_callback_id = self.figure.canvas.mpl_connect(
            "key_press_event", self._on_key_press
        )

    def _on_pick(self, event) -> None:
        axis = self._button_to_axis.get(event.artist)
        if axis is not None:
            self.toggle(axis)

    def _on_key_press(self, event) -> None:
        if event.key in ("escape", "esc") and self.maximized_axis is not None:
            self.restore()

    def toggle(self, axis) -> None:
        """Maximize ``axis``, or restore it when it is already maximized."""
        if axis not in self.buttons:
            raise ValueError("axis is not managed by this maximize controller")
        if self.maximized_axis is axis:
            self.restore()
            return
        if self.maximized_axis is not None:
            self.restore(draw=False)
        self.maximize(axis)

    def maximize(self, axis) -> None:
        if axis not in self.buttons:
            raise ValueError("axis is not managed by this maximize controller")
        self.maximized_axis = axis
        for candidate in self.axes:
            candidate.set_visible(candidate is axis)
        for auxiliary_axis in self.auxiliary_axes:
            auxiliary_axis.set_visible(False)
        for artist in self.auxiliary_artists:
            artist.set_visible(False)

        axis.set_position((0.065, 0.075, 0.91, 0.88))
        axis.set_xlabel(self._original_xlabels[axis] or "time (s)")
        axis.tick_params(axis="x", labelbottom=True)
        self.buttons[axis].set_text("BACK")
        self.figure.canvas.draw_idle()

    def restore(self, *, draw: bool = True) -> None:
        if self.maximized_axis is None:
            return
        maximized_axis = self.maximized_axis
        self.maximized_axis = None

        for axis in self.axes:
            axis.set_position(self._original_positions[axis])
            axis.set_visible(self._original_visibility[axis])
            axis.set_xlabel(self._original_xlabels[axis])
            axis.tick_params(
                axis="x", labelbottom=self._original_labelbottom[axis]
            )
            self.buttons[axis].set_text("MAX")
        for auxiliary_axis in self.auxiliary_axes:
            auxiliary_axis.set_visible(
                self._original_auxiliary_axis_visibility[auxiliary_axis]
            )
        for artist in self.auxiliary_artists:
            artist.set_visible(
                self._original_auxiliary_artist_visibility[artist]
            )

        # Keep the selected axis alive until all of its properties have been
        # restored; this also makes the order explicit for GUI backends.
        maximized_axis.set_visible(self._original_visibility[maximized_axis])
        if draw:
            self.figure.canvas.draw_idle()
