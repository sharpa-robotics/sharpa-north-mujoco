import ast
import contextlib
import io
import math
import os
import socket
import struct
import sys
import tempfile
import unittest
from pathlib import Path


os.environ.setdefault("MPLBACKEND", "Agg")
os.environ.setdefault("MPLCONFIGDIR", tempfile.mkdtemp(prefix="arm-monitor-mpl-"))
TOOL_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(TOOL_DIR))

from arm_config import load_arm_config
from arm_history import MonitorHistory
from arm_plotter import MonitorPlot
from arm_sources import WireCapture, ZenohCapture
from arm_transmission import ArmTransmission
from monitor import MonitorPart, parse_args
from test_arm_protocol import make_control, make_feedback


class PlotAndPassiveTest(unittest.TestCase):
    def test_arguments(self):
        self.assertEqual(parse_args(["4"]).part, MonitorPart.LEFT_ARM)
        args = parse_args(["5", "--time", "45"])
        self.assertEqual((args.part, args.time), (MonitorPart.RIGHT_ARM, 45.0))
        with contextlib.redirect_stderr(io.StringIO()), self.assertRaises(SystemExit):
            parse_args(["9"])

    def test_plot_reuses_lines_and_pause_works(self):
        import matplotlib.pyplot as plt

        now = [0.0]
        history = MonitorHistory("left", clock=lambda: now[0])
        plot = MonitorPlot(plt, history, "left", ["lo"])
        line_ids = [id(line) for lines in plot.lines.values() for line in lines]
        self.assertEqual(len(plot.axes), 14)
        self.assertEqual(len(plot.maximize_controller.buttons), 14)
        self.assertEqual(len(line_ids), 28)
        now[0] = 0.1
        history.add_zenoh("zenoh_action", [0.1] * 7)
        plot.update()
        before = tuple(plot.lines["zenoh_action"][0].get_xdata())
        plot.toggle_pause()
        now[0] = 0.2
        history.add_zenoh("zenoh_action", [0.2] * 7)
        plot.update()
        self.assertEqual(before, tuple(plot.lines["zenoh_action"][0].get_xdata()))
        plot.toggle_pause()
        plot.update()
        self.assertGreater(len(plot.lines["zenoh_action"][0].get_xdata()), len(before))
        self.assertEqual(
            line_ids, [id(line) for lines in plot.lines.values() for line in lines]
        )
        plt.close(plot.figure)

    def test_runtime_remains_passive(self):
        forbidden_calls = {"send", "sendto", "sendmsg", "sendall", "put", "publish"}
        for path in TOOL_DIR.glob("arm_*.py"):
            tree = ast.parse(path.read_text(encoding="utf-8"), filename=str(path))
            for node in ast.walk(tree):
                if isinstance(node, ast.Call) and isinstance(node.func, ast.Attribute):
                    self.assertNotIn(node.func.attr, forbidden_calls, f"{path}:{node.lineno}")

    def test_extended_udp_feedback_reaches_all_state_lines(self):
        import matplotlib.pyplot as plt

        for side in ("left", "right"):
            for mode in (0, 5):
                with self.subTest(side=side, mode=mode):
                    config = load_arm_config(TOOL_DIR.parents[1] / "sharpa_mujoco.conf", side)
                    now = [0.0]
                    history = MonitorHistory(side, clock=lambda: now[0])
                    history.set_control_mode(mode)
                    capture = WireCapture(config, history)
                    joints = (0.1, -0.2, 0.3, -0.4, 0.5, -0.6, 0.7)
                    motors = ArmTransmission(side).joint_to_motor(joints)
                    packets = []
                    for tick in (0.1, 0.2):
                        for endpoint in config.endpoints:
                            axes = [
                                (slot, motors[motor], 0.0, 0.0)
                                for slot, motor in enumerate(endpoint.motor_indices)
                            ]
                            for feedback in (False, True):
                                payload = (
                                    make_feedback(axes, extended=True)
                                    if feedback else make_control([axis + (0.0, 0.0) for axis in axes])
                                )
                                source_ip, destination_ip = endpoint.peer_ip, endpoint.bind_ip
                                source_port, destination_port = endpoint.peer_port, endpoint.bind_port
                                if feedback:
                                    source_ip, destination_ip = destination_ip, source_ip
                                    source_port, destination_port = destination_port, source_port
                                ip = bytearray(20)
                                ip[0], ip[9] = 0x45, 17
                                ip[12:16] = socket.inet_aton(source_ip)
                                ip[16:20] = socket.inet_aton(destination_ip)
                                udp = struct.pack(
                                    ">HHHH", source_port, destination_port, 8 + len(payload), 0
                                )
                                packets.append((tick, bytes(ip) + udp + payload))

                    class FakeSocket:
                        def recvfrom(self, _size):
                            if not packets:
                                capture._stop.set()
                                raise OSError("capture finished")
                            now[0], packet = packets.pop(0)
                            return packet, ("lo", 0, 0)

                    capture._udp_loop(FakeSocket(), "lo", config.endpoints)
                    snapshot = history.snapshot()
                    self.assertEqual(snapshot.counts["wire_action"], 10)
                    self.assertEqual(snapshot.counts["wire_state"], 10)
                    self.assertEqual(sum(snapshot.errors.values()), 0)
                    plot = MonitorPlot(plt, history, side, ["lo"])
                    self.addCleanup(plt.close, plot.figure)
                    plot.update()
                    for axis in range(7):
                        expected = motors[axis] if mode == 5 else math.degrees(joints[axis])
                        for channel in ("wire_action", "wire_state"):
                            values = plot.lines[channel][axis].get_ydata()
                            self.assertEqual(len(values), 2)
                            self.assertAlmostEqual(values[-1], expected, places=3)
                        self.assertIn("S ", plot.axes_grid[axis, 1].get_title())
                    plt.close(plot.figure)

    def test_heartbeat_selects_motor_view_and_labels_plot(self):
        import matplotlib.pyplot as plt

        class FakeHeartbeat:
            def ParseFromString(self, payload):
                self.control_mode = int(payload.decode("ascii"))

        class FakeProto:
            Heartbeat = FakeHeartbeat

        class FakeSample:
            payload = b"5"

        history = MonitorHistory("left")
        capture = ZenohCapture(None, FakeProto, "left", history)
        capture._on_heartbeat(FakeSample())
        history.add_wire("wire_action", {0: 0.25})

        plot = MonitorPlot(plt, history, "left", ["lo"])
        plot.update()
        self.assertIn("MODE 5 MotorPd [heartbeat]", plot.status_text.get_text())
        self.assertIn(
            "M0 arm_motor_0 | UDP raw motor view (rad)",
            plot.axes_grid[0, 1].get_title(),
        )
        self.assertEqual(plot.axes_grid[0, 1].get_ylabel(), "motor position (rad)")
        plt.close(plot.figure)


if __name__ == "__main__":
    unittest.main()
