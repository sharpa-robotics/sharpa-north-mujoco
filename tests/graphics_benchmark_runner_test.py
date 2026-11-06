#!/usr/bin/env python3
# Copyright 2026 Sharpa
# SPDX-License-Identifier: Apache-2.0
"""Check acceptance runner ownership, cleanup and failure reporting without a GPU."""
import contextlib
import importlib.util
import io
import json
from pathlib import Path
import signal
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import patch

spec = importlib.util.spec_from_file_location("graphics_benchmark",
    Path(__file__).resolve().parents[1] / "tools/graphics/benchmark.py")
runner = importlib.util.module_from_spec(spec)
spec.loader.exec_module(runner)


class RunnerTest(unittest.TestCase):
    def exercise(self, last_exit=0, preview_exit=0):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            config = root / "simulation.conf"
            config.write_text("test config")
            binary = root / "simulator"
            binary.write_text("fake executable")
            args = SimpleNamespace(output_dir=root / "results", sdk_root=root,
                config=config, simulator=binary, driver="580.178.04", workload="static")
            children = []

            class Child:
                def __init__(self, command, **kwargs):
                    self.returncode = None
                    self.signals = []
                    children.append(self)
                    report = Path(command[command.index("--benchmark-report") + 1])
                    report.write_text(json.dumps({"gl_version": "4.6 NVIDIA 580.178.04",
                        "passed": True, "average_fps": 60, "frame_time_p95_ms": 17,
                        "realtime_ratio": 1}))

                def poll(self):
                    return self.returncode

                def send_signal(self, value):
                    self.signals.append(value)

                def wait(self, timeout):
                    self.returncode = last_exit if len(children) == 3 else 0
                    return self.returncode

            def output(*command):
                if command[0] == "nvidia-smi": return "NVIDIA GeForce RTX 4070 Ti SUPER, 580.178.04"
                if command[0] == "uname": return "6.8.0-138-generic"
                if command[0] == "bash": return "{}"
                return "test-commit"

            read_text = Path.read_text

            def read(path, *positional, **named):
                if str(path) == "/proc/driver/nvidia/version": return "NVIDIA 580.178.04"
                return read_text(path, *positional, **named)

            with patch.object(runner, "output", side_effect=output), \
                 patch.object(Path, "read_text", read), \
                 patch.object(runner.subprocess, "check_output", return_value=b""), \
                 patch.object(runner.subprocess, "Popen", Child), \
                 patch.object(runner.subprocess, "run", return_value=SimpleNamespace(
                     returncode=preview_exit, stdout="Master=STANDBY mode=INFERENCE", stderr="")) as preview_run, \
                 patch.object(runner.time, "sleep"), contextlib.redirect_stdout(io.StringIO()):
                error = None
                try:
                    result = runner.run(args)
                except RuntimeError as exception:
                    result, error = None, str(exception)
            for call in preview_run.call_args_list:
                self.assertEqual(call.args[0], [runner.sys.executable,
                    str(runner.ROOT / "examples/sdk/whole_body_motion.py"), "--preview"])
            summary = json.loads((args.output_dir / "summary.json").read_text())
            self.assertTrue(all(c.signals == [signal.SIGINT] for c in children))
            return result, error, summary

    def test_success_stops_owned_simulators(self):
        result, error, summary = self.exercise()
        self.assertEqual(result, 0)
        self.assertIsNone(error)
        self.assertTrue(summary["passed"])
        self.assertEqual(len(summary["runs"]), 3)

    def test_failed_cleanup_overrides_passing_frame_reports(self):
        _, error, summary = self.exercise(last_exit=1)
        self.assertIn("did not exit cleanly", error)
        self.assertEqual(len(summary["runs"]), 3)
        self.assertFalse(summary["passed"])

    def test_preview_failure_still_stops_owned_simulator(self):
        _, error, summary = self.exercise(preview_exit=1)
        self.assertIn("SDK preview failed", error)
        self.assertFalse(summary["passed"])


if __name__ == "__main__":
    unittest.main()
