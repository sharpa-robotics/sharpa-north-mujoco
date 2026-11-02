import ast
import contextlib
import io
import os
import sys
import tempfile
import unittest
from pathlib import Path


os.environ.setdefault("MPLBACKEND", "Agg")
os.environ.setdefault("MPLCONFIGDIR", tempfile.mkdtemp(prefix="torso-monitor-mpl-"))
TOOL_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(TOOL_DIR))

from monitor import MonitorPart, parse_args
from torso_history import MonitorHistory
from torso_plotter import CHANNEL_STYLES, MonitorPlot
from torso_protocol import CONTROL_MODE_MIT, CONTROL_MODE_POSITION


class PlotAndPassiveTest(unittest.TestCase):
    def test_time_argument(self):
        self.assertEqual(parse_args(["1"]).part, MonitorPart.TORSO)
        self.assertEqual(parse_args(["1"]).time, 30.0)
        self.assertEqual(parse_args(["1", "--time", "45"]).time, 45.0)
        with contextlib.redirect_stderr(io.StringIO()), self.assertRaises(SystemExit):
            parse_args(["1", "--time", "0"])

    def test_plot_reuses_exactly_32_lines(self):
        import matplotlib.pyplot as plt

        history = MonitorHistory(
            (1, 2, 4, 3, 5, 6, 7, 8),
            (-1, -1, 1, 1, -1, 1, 1, 1),
        )
        plot = MonitorPlot(plt, history, "CAN", ["vcan0", "vcan1"])
        status_id = id(plot.status_text)
        line_ids = [id(line) for lines in plot.lines.values() for line in lines]
        self.assertEqual(len(plot.axes), 16)
        self.assertEqual(len(plot.maximize_controller.buttons), 16)
        self.assertEqual(len(line_ids), 32)
        plot.update()
        plot.update()
        self.assertEqual(
            line_ids, [id(line) for lines in plot.lines.values() for line in lines]
        )
        self.assertEqual(status_id, id(plot.status_text))
        self.assertLessEqual(max(axis.get_position().y1 for axis in plot.axes), 0.955)
        self.assertEqual(plot.status_text.get_position(), (0.5, 0.99))
        self.assertNotIn("\n", plot.status_text.get_text())
        self.assertTrue(all("\n" not in axis.get_title() for axis in plot.axes))
        self.assertTrue(all(len(axis.lines) == 2 for axis in plot.axes))
        self.assertTrue(
            all(axis.get_xlim() == plot.axes_grid[-1, 0].get_xlim() for axis in plot.axes)
        )
        for joint_index in range(8):
            self.assertEqual(
                plot.axes_grid[joint_index, 0].get_shared_y_axes().joined(
                    plot.axes_grid[joint_index, 0], plot.axes_grid[joint_index, 1]
                ),
                False,
            )
        plt.close(plot.figure)

    def test_each_row_uses_its_detected_control_mode(self):
        import matplotlib.pyplot as plt

        history = MonitorHistory(
            (1, 2, 4, 3, 5, 6, 7, 8),
            (-1, -1, 1, 1, -1, 1, 1, 1),
        )
        history.add_wire(
            "wire_action", {1: 0.25}, control_mode=CONTROL_MODE_MIT
        )
        history.add_wire("wire_state", {1: 0.20})
        history.add_wire(
            "wire_action", {2: 0.40}, control_mode=CONTROL_MODE_POSITION
        )
        history.add_wire("wire_state", {2: 0.30})

        plot = MonitorPlot(plt, history, "CAN", ["vcan0", "vcan1"])
        plot.update()
        self.assertIn("motor_id 1", plot.axes_grid[0, 1].get_title())
        self.assertIn("CAN raw motor view (rad)", plot.axes_grid[0, 1].get_title())
        self.assertIn("mode MIT", plot.axes_grid[0, 1].get_title())
        self.assertEqual(plot.axes_grid[0, 1].get_ylabel(), "motor position (rad)")

        self.assertIn("J2", plot.axes_grid[1, 1].get_title())
        self.assertIn(
            "CAN reconstructed joint view (deg)",
            plot.axes_grid[1, 1].get_title(),
        )
        self.assertIn("mode Position/CSP", plot.axes_grid[1, 1].get_title())
        self.assertEqual(plot.axes_grid[1, 1].get_ylabel(), "joint angle (deg)")
        self.assertIn("CSP J2", plot.status_text.get_text())
        self.assertIn("MIT J1", plot.status_text.get_text())
        plt.close(plot.figure)

    def test_pause_freezes_lines_until_resumed(self):
        import matplotlib.pyplot as plt

        now = [0.0]
        history = MonitorHistory(
            (1, 2, 4, 3, 5, 6, 7, 8),
            (-1, -1, 1, 1, -1, 1, 1, 1),
            clock=lambda: now[0],
        )
        plot = MonitorPlot(plt, history, "CAN", ["vcan0", "vcan1"])
        now[0] = 0.1
        history.add_zenoh("zenoh_action", [0.1] * 8)
        plot.update()
        before = tuple(plot.lines["zenoh_action"][0].get_xdata())

        plot.toggle_pause()
        self.assertTrue(plot.paused)
        self.assertEqual(plot.pause_button.label.get_text(), "Resume")
        now[0] = 0.2
        history.add_zenoh("zenoh_action", [0.2] * 8)
        plot.update()
        self.assertEqual(
            before, tuple(plot.lines["zenoh_action"][0].get_xdata())
        )

        plot.toggle_pause()
        plot.update()
        self.assertFalse(plot.paused)
        self.assertEqual(plot.pause_button.label.get_text(), "Pause")
        self.assertGreater(
            len(plot.lines["zenoh_action"][0].get_xdata()), len(before)
        )
        plt.close(plot.figure)

    def test_wire_action_and_state_colors_are_distinct(self):
        self.assertEqual(CHANNEL_STYLES["wire_action"]["color"], "tab:purple")
        self.assertEqual(CHANNEL_STYLES["wire_state"]["color"], "tab:red")

    def test_runtime_sources_have_no_write_calls_or_mujoco_access(self):
        forbidden_calls = {"send", "sendto", "sendmsg", "sendall", "put", "publish"}
        runtime_files = [
            path
            for path in TOOL_DIR.glob("*.py")
            if path.name != "__init__.py"
        ]
        for path in runtime_files:
            tree = ast.parse(path.read_text(encoding="utf-8"), filename=str(path))
            for node in ast.walk(tree):
                if isinstance(node, ast.Call) and isinstance(node.func, ast.Attribute):
                    self.assertNotIn(node.func.attr, forbidden_calls, f"{path}:{node.lineno}")
                if isinstance(node, (ast.Import, ast.ImportFrom)):
                    names = [alias.name for alias in node.names]
                    if isinstance(node, ast.ImportFrom) and node.module:
                        names.append(node.module)
                    self.assertFalse(
                        any(name.startswith("sharpa_mujoco") for name in names),
                        f"{path}:{node.lineno}",
                    )


if __name__ == "__main__":
    unittest.main()
