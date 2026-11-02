import os
import sys
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace


os.environ.setdefault("MPLBACKEND", "Agg")
os.environ.setdefault("MPLCONFIGDIR", tempfile.mkdtemp(prefix="plot-zoom-mpl-"))
TOOL_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(TOOL_DIR))

from plot_zoom import AxesMaximizeController


class AxesMaximizeControllerTest(unittest.TestCase):
    def test_button_maximizes_subplot_and_restores_grid(self):
        import matplotlib.pyplot as plt
        from matplotlib.backend_bases import MouseEvent

        figure, axes_grid = plt.subplots(2, 2, sharex=True)
        axes = list(axes_grid.flat)
        status = figure.text(0.5, 0.99, "status")
        control_axis = figure.add_axes((0.9, 0.96, 0.08, 0.025))
        original_positions = [axis.get_position().frozen() for axis in axes]
        controller = AxesMaximizeController(
            figure,
            axes,
            auxiliary_axes=(control_axis,),
            auxiliary_artists=(status,),
        )

        selected = axes[1]
        figure.canvas.draw()
        bounds = controller.buttons[selected].get_window_extent(
            figure.canvas.get_renderer()
        )
        click = MouseEvent(
            "button_press_event",
            figure.canvas,
            (bounds.x0 + bounds.x1) / 2.0,
            (bounds.y0 + bounds.y1) / 2.0,
            button=1,
        )
        figure.canvas.callbacks.process("button_press_event", click)
        self.assertIs(controller.maximized_axis, selected)
        self.assertEqual(controller.buttons[selected].get_text(), "BACK")
        self.assertTrue(selected.get_visible())
        self.assertTrue(
            all(not axis.get_visible() for axis in axes if axis is not selected)
        )
        self.assertFalse(status.get_visible())
        self.assertFalse(control_axis.get_visible())
        self.assertGreater(selected.get_position().width, original_positions[1].width)

        controller._on_key_press(SimpleNamespace(key="escape"))
        self.assertIsNone(controller.maximized_axis)
        self.assertTrue(all(axis.get_visible() for axis in axes))
        self.assertTrue(status.get_visible())
        self.assertTrue(control_axis.get_visible())
        self.assertEqual(
            [tuple(axis.get_position().bounds) for axis in axes],
            [tuple(position.bounds) for position in original_positions],
        )
        self.assertTrue(
            all(button.get_text() == "MAX" for button in controller.buttons.values())
        )
        plt.close(figure)

    def test_clicking_a_second_button_switches_the_maximized_subplot(self):
        import matplotlib.pyplot as plt

        figure, axes_grid = plt.subplots(1, 2)
        axes = list(axes_grid.flat)
        controller = AxesMaximizeController(figure, axes)

        controller.toggle(axes[0])
        controller.toggle(axes[1])
        self.assertIs(controller.maximized_axis, axes[1])
        self.assertFalse(axes[0].get_visible())
        self.assertTrue(axes[1].get_visible())
        self.assertEqual(controller.buttons[axes[0]].get_text(), "MAX")
        self.assertEqual(controller.buttons[axes[1]].get_text(), "BACK")
        controller.toggle(axes[1])
        self.assertIsNone(controller.maximized_axis)
        plt.close(figure)


if __name__ == "__main__":
    unittest.main()
