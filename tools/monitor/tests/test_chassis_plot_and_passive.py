import os
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import Mock, patch


os.environ.setdefault("MPLBACKEND", "Agg")
os.environ.setdefault("MPLCONFIGDIR", tempfile.mkdtemp(prefix="chassis-monitor-mpl-"))
TOOL_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(TOOL_DIR))

from chassis_config import ChassisMonitorConfig
from chassis_history import MonitorHistory
from chassis_plotter import MonitorPlot
from chassis_protocol import MotorEvent
from chassis_sources import WireCapture
from monitor import MonitorPart, parse_args, _run_chassis


class PlotAndPassiveTest(unittest.TestCase):
    def test_cli_and_three_chassis_eight_motor_plots_with_pause(self):
        import matplotlib.pyplot as plt

        args = parse_args(["8", "--time", "45"])
        self.assertEqual((args.part, args.time), (MonitorPart.CHASSIS, 45.0))
        now = [0.0]
        history = MonitorHistory(ChassisMonitorConfig(Path("test")), clock=lambda: now[0])
        plot = MonitorPlot(plt, history, ("lo",))
        self.addCleanup(plt.close, plot.figure)
        self.assertEqual((len(plot.zenoh_axes), len(plot.motor_axes)), (3, 8))
        self.assertEqual(len(plot.maximize_controller.buttons), 11)
        original_ids = [id(line) for lines in plot.lines.values() for line in lines]
        self.assertEqual(len(original_ids), 22)
        self.assertEqual(plot.zenoh_axes[0].get_ylabel(), "m/s")
        self.assertEqual(plot.zenoh_axes[2].get_ylabel(), "rad/s")
        self.assertEqual([axis.get_ylabel() for axis in plot.motor_axes], ["rad"] * 4 + ["rad/s"] * 4)
        self.assertFalse(plot.zenoh_axes[0].get_shared_y_axes().joined(plot.zenoh_axes[0], plot.motor_axes[0]))
        self.assertTrue(plot.zenoh_axes[0].get_shared_x_axes().joined(plot.zenoh_axes[0], plot.motor_axes[7]))
        history.add("zenoh_action", (0.2, -0.3, 0.4))
        history.add_wire(MotorEvent(5, "current"))
        history.add_wire(MotorEvent(5, "feedback", -16777216))
        plot.update()
        self.assertEqual(list(plot.lines["zenoh_action"][2].get_ydata()), [0.4])
        self.assertIn("A N/A", plot.motor_axes[4].get_title())
        self.assertIn("S -98.175", plot.motor_axes[4].get_title())
        self.assertIn("Current", plot.motor_axes[4].get_title())
        self.assertIn("Unknown", plot.motor_axes[0].get_title())
        self.assertIn("current control", plot.axis_status[7].get_text())
        plot.toggle_pause()
        now[0] = 0.02
        history.add("zenoh_action", (0.4, 0.5, 0.6))
        plot.update()
        self.assertEqual(len(plot.lines["zenoh_action"][0].get_xdata()), 1)
        plot.toggle_pause()
        plot.update()
        self.assertEqual(len(plot.lines["zenoh_action"][0].get_xdata()), 2)
        self.assertEqual(original_ids, [id(line) for lines in plot.lines.values() for line in lines])
        positions = [axis.get_position().bounds for axis in plot.axes]
        plot.maximize_controller.maximize(plot.motor_axes[7])
        self.assertFalse(plot.zenoh_axes[0].get_visible())
        plot.maximize_controller.restore()
        for axis, position in zip(plot.axes, positions):
            self.assertTrue(axis.get_visible())
            for actual, expected in zip(axis.get_position().bounds, position):
                self.assertAlmostEqual(actual, expected)
        plot.figure.canvas.draw()

    def test_wire_capture_binds_only_packet_interface_and_closes_on_failure(self):
        config = ChassisMonitorConfig(Path("test"))
        capture = WireCapture(config, MonitorHistory(config))
        capture_socket = Mock()
        with patch("chassis_sources.socket.socket", return_value=capture_socket) as factory, \
                patch("chassis_sources.threading.Thread"):
            capture.start()
            import socket
            self.assertEqual(factory.call_args.args[:2], (socket.AF_PACKET, socket.SOCK_DGRAM))
            capture_socket.bind.assert_called_once_with(("lo", 0))
            capture.stop()
            capture_socket.close.assert_called_once()

        capture = WireCapture(config, MonitorHistory(config))
        capture_socket = Mock()
        capture_socket.bind.side_effect = PermissionError("denied")
        with patch("chassis_sources.socket.socket", return_value=capture_socket):
            with self.assertRaisesRegex(RuntimeError, "CAP_NET_RAW"):
                capture.start()
        capture_socket.close.assert_called_once()

    def test_runner_releases_both_sources_if_zenoh_start_fails(self):
        config = ChassisMonitorConfig(Path("test"))
        with patch("monitor.load_chassis_config", return_value=config), \
                patch("monitor.ChassisWireCapture") as wire, \
                patch("monitor.ChassisZenohCapture") as zenoh:
            zenoh.return_value.start.side_effect = RuntimeError("connect failed")
            with self.assertRaisesRegex(RuntimeError, "connect failed"):
                _run_chassis(None, None, None, None, 30)
            wire.return_value.stop.assert_called_once()
            zenoh.return_value.stop.assert_called_once()


if __name__ == "__main__":
    unittest.main()
