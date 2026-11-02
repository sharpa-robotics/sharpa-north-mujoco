import ast
import contextlib
import io
import os
import sys
import tempfile
import unittest
from pathlib import Path


os.environ.setdefault("MPLBACKEND", "Agg")
os.environ.setdefault("MPLCONFIGDIR", tempfile.mkdtemp(prefix="hand-monitor-mpl-"))
TOOL_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(TOOL_DIR))

from hand_history import MonitorHistory
from hand_plotter import MonitorPlot
from glove_plotter import MonitorPlot as GloveMonitorPlot
from monitor import MonitorPart, parse_args


class PlotAndPassiveTest(unittest.TestCase):
    def test_arguments(self):
        args = parse_args(["2"])
        self.assertEqual(args.part, MonitorPart.LEFT_HAND)
        self.assertEqual(args.time, 30.0)
        args = parse_args(["3", "--time", "45"])
        self.assertEqual((args.part, args.time), (MonitorPart.RIGHT_HAND, 45.0))
        self.assertEqual(parse_args(["6"]).part, MonitorPart.LEFT_GLOVE)
        self.assertEqual(parse_args(["7"]).part, MonitorPart.RIGHT_GLOVE)
        with contextlib.redirect_stderr(io.StringIO()), self.assertRaises(SystemExit):
            parse_args(["9"])

    def test_glove_plot_has_22_raw_action_lines(self):
        import matplotlib.pyplot as plt

        now = [0.0]
        history = MonitorHistory(clock=lambda: now[0])
        plot = GloveMonitorPlot(plt, history, "left", 30.0)
        self.assertEqual(len(plot.axes), 22)
        self.assertEqual(len(plot.maximize_controller.buttons), 22)
        self.assertEqual(len(plot.lines), 22)
        now[0] = 0.1
        history.add("zenoh_action", [0.1] * 22)
        plot.update()
        self.assertEqual(len(plot.lines[0].get_xdata()), 1)
        plt.close(plot.figure)

    def test_compact_plot_reuses_lines_and_pause_works(self):
        import matplotlib.pyplot as plt

        now = [0.0]
        history = MonitorHistory(clock=lambda: now[0])
        plot = MonitorPlot(plt, history, "left", ["lo"])
        line_ids = [id(line) for lines in plot.lines.values() for line in lines]
        self.assertEqual(len(plot.axes), 44)
        self.assertEqual(len(plot.maximize_controller.buttons), 44)
        self.assertEqual(len(line_ids), 88)
        now[0] = 0.1
        history.add("zenoh_action", [0.1] * 22)
        plot.update()
        before = tuple(plot.lines["zenoh_action"][0].get_xdata())
        plot.toggle_pause()
        now[0] = 0.2
        history.add("zenoh_action", [0.2] * 22)
        plot.update()
        self.assertEqual(before, tuple(plot.lines["zenoh_action"][0].get_xdata()))
        plot.toggle_pause()
        plot.update()
        self.assertGreater(len(plot.lines["zenoh_action"][0].get_xdata()), len(before))
        self.assertEqual(
            line_ids, [id(line) for lines in plot.lines.values() for line in lines]
        )
        plt.close(plot.figure)

    def test_runtime_is_passive_and_has_no_mujoco_access(self):
        forbidden_calls = {"send", "sendto", "sendmsg", "sendall", "put", "publish"}
        for path in TOOL_DIR.glob("*.py"):
            if path.name == "__init__.py":
                continue
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
