#!/usr/bin/env python3
"""Launcher-boundary tests: no real network changes or privilege escalation."""
import os
from pathlib import Path
import subprocess
import tempfile
import unittest

SCRIPT = Path(__file__).resolve().parents[1] / "scripts" / "prepare_environment.sh"


class EnvironmentCheckTest(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory(prefix="mujoco-preflight-")
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.config = self.root / "simulation profile.conf"
        self.config.write_text("[devices.torso]\nenabled=true\ndriver=can\n")
        self.checker = self.executable("checker", """#!/bin/bash
case "$TEST_CHECK_STATE" in
  ready) exit 0 ;;
  setup) printf 'vcan0\\nvcan1\\n'; exit 10 ;;
  unsafe) printf 'vcan0\\nvcan/bad\\n'; exit 10 ;;
  invalid) echo 'invalid configuration' >&2; exit 2 ;;
  *) echo "$TEST_CHECK_STATE interface is not usable" >&2; exit 1 ;;
esac
""")
        self.env = dict(
            os.environ,
            SHARPA_MUJOCO_ENVIRONMENT_CHECKER=str(self.checker),
            TEST_CHECK_STATE="ready",
        )

    def executable(self, name, text):
        path = self.root / name
        path.write_text(text)
        path.chmod(0o755)
        return path

    def run_check(self, *args):
        return subprocess.run(
            ["bash", str(SCRIPT), *args, "--config", str(self.config)],
            env=self.env,
            stdin=subprocess.DEVNULL,
            capture_output=True,
            text=True,
            timeout=5,
        )

    def test_ready(self):
        result = self.run_check()
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(result.stderr, "")

    def test_setup_required_shows_command_without_sudo(self):
        self.env["TEST_CHECK_STATE"] = "setup"
        for mode in ("--check", "--interactive"):
            with self.subTest(mode=mode):
                result = self.run_check(mode)
                self.assertEqual(result.returncode, 1)
                self.assertIn("sudo bash", result.stderr)
                self.assertIn("setup_vcan.sh vcan0 vcan1", result.stderr)
                self.assertIn("restart simulation", result.stderr)

    def test_unpreparable_issue_is_propagated(self):
        self.env["TEST_CHECK_STATE"] = "physical CAN"
        result = self.run_check("--interactive")
        self.assertEqual(result.returncode, 1)
        self.assertIn("physical CAN interface is not usable", result.stderr)
        self.assertNotIn("sudo bash", result.stderr)

    def test_invalid_configuration_is_propagated(self):
        self.env["TEST_CHECK_STATE"] = "invalid"
        result = self.run_check()
        self.assertEqual(result.returncode, 2)
        self.assertIn("invalid configuration", result.stderr)

    def test_checker_output_is_treated_as_untrusted(self):
        self.env["TEST_CHECK_STATE"] = "unsafe"
        result = self.run_check()
        self.assertEqual(result.returncode, 2)
        self.assertIn("unsafe vcan name", result.stderr)

    def test_missing_checker_is_configuration_error(self):
        self.env["SHARPA_MUJOCO_ENVIRONMENT_CHECKER"] = str(self.root / "missing")
        result = self.run_check()
        self.assertEqual(result.returncode, 2)
        self.assertIn("checker is missing", result.stderr)


if __name__ == "__main__":
    unittest.main()
