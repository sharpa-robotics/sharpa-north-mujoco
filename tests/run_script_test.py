#!/usr/bin/env python3
"""Check launcher paths and preflight consistency without starting devices."""

import json
import os
from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest


SCRIPTS = Path(__file__).resolve().parents[1] / "scripts"
STUB = """#!/usr/bin/env python3
import json, os, pathlib, sys
with open(os.environ['LAUNCH_TEST_LOG'], 'a') as log:
    log.write(json.dumps({'exe': str(pathlib.Path(sys.argv[0]).resolve()),
                          'args': sys.argv[1:], 'cwd': os.getcwd()}) + '\\n')
if pathlib.Path(sys.argv[0]).name == 'sharpa_mujoco_environment':
    sys.exit(int(os.environ.get('LAUNCH_TEST_PREFLIGHT_EXIT', '0')))
"""


class RunScriptTest(unittest.TestCase):
    def setUp(self):
        temporary = tempfile.TemporaryDirectory(prefix="sharpa-launch-")
        self.addCleanup(temporary.cleanup)
        self.root = Path(temporary.name).resolve()
        self.project = self.root / "project with spaces"
        self.package = self.root / "runtime package"
        self.caller = self.root / "outside working directory"
        self.caller.mkdir()
        self.log = self.root / "launch.jsonl"
        self.env = dict(os.environ, LAUNCH_TEST_LOG=str(self.log))
        for key in ("SHARPA_MUJOCO_CONFIG", "SHARPA_MUJOCO_SIMULATOR",
                    "SHARPA_MUJOCO_ENVIRONMENT_CHECKER"):
            self.env.pop(key, None)
        for root in (self.project, self.package):
            (root / "scripts").mkdir(parents=True)
            (root / "sharpa_mujoco.conf").write_text("# test configuration\n")
            shutil.copy2(SCRIPTS / "prepare_environment.sh", root / "scripts")
        shutil.copy2(SCRIPTS / "run.sh", self.project / "scripts/run.sh")
        shutil.copy2(SCRIPTS / "runtime_run.sh", self.package / "run.sh")
        self.make_build(self.project / "build-linux")
        self.make_build(self.project / "build-linux-no-platform")
        for name in ("sharpa_mujoco_simulator", "sharpa_mujoco_environment"):
            self.executable(self.package / "bin" / name)
        # The simulator double is a script, so there are no runtime libraries.
        self.executable(self.root / "bin/ldd", "#!/bin/sh\nexit 0\n")
        self.env["PATH"] = str(self.root / "bin") + os.pathsep + self.env["PATH"]
        self.relative_config = self.caller / "profiles/my simulation.conf"
        self.relative_config.parent.mkdir()
        self.relative_config.write_text("# caller configuration\n")

    def executable(self, path, text=STUB):
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(text)
        path.chmod(0o755)
        return path

    def make_build(self, directory):
        self.executable(directory / "apps/simulator/sharpa_mujoco_simulator")
        self.executable(directory / "sharpa_mujoco_environment")

    def launch(self, kind, *args, **env):
        self.log.unlink(missing_ok=True)
        script = self.project / "scripts/run.sh" if kind == "source" else self.package / "run.sh"
        return subprocess.run(
            ["bash", str(script), *map(str, args)], cwd=self.caller,
            env=dict(self.env, **env), text=True, capture_output=True,
            stdin=subprocess.DEVNULL, timeout=5,
        )

    def records(self):
        return [json.loads(line) for line in self.log.read_text().splitlines()] if self.log.exists() else []

    def assert_config(self, kind, result, expected):
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        records = self.records()
        self.assertEqual(len(records), 2)
        for record in records:
            paths = [record["args"][i + 1] for i, arg in enumerate(record["args"]) if arg == "--config"]
            self.assertEqual(paths[-1], str(expected))
        self.assertEqual(records[1]["cwd"], str(self.project if kind == "source" else self.package))

    def test_default_configuration(self):
        for kind, root in (("source", self.project), ("package", self.package)):
            with self.subTest(kind=kind):
                self.assert_config(kind, self.launch(kind), root / "sharpa_mujoco.conf")

    def test_relative_config_is_identical_at_preflight_and_launch(self):
        for kind in ("source", "package"):
            with self.subTest(kind=kind):
                result = self.launch(kind, "--config", "profiles/my simulation.conf", "--no-viewer")
                self.assert_config(kind, result, self.relative_config)
                self.assertIn("--no-viewer", self.records()[1]["args"])

    def test_relative_environment_config(self):
        for kind in ("source", "package"):
            with self.subTest(kind=kind):
                result = self.launch(kind, SHARPA_MUJOCO_CONFIG="profiles/my simulation.conf")
                self.assert_config(kind, result, self.relative_config)

    def test_cli_config_overrides_environment(self):
        for kind in ("source", "package"):
            with self.subTest(kind=kind):
                result = self.launch(kind, "--config", self.relative_config,
                                     SHARPA_MUJOCO_CONFIG="missing.conf")
                self.assert_config(kind, result, self.relative_config)

    def test_repeated_config_uses_last_value_for_both_processes(self):
        for kind in ("source", "package"):
            with self.subTest(kind=kind):
                result = self.launch(kind, "--config", "missing.conf", "--config", "profiles/my simulation.conf")
                self.assert_config(kind, result, self.relative_config)

    def test_invalid_or_missing_config_does_not_launch(self):
        for kind in ("source", "package"):
            for args in (("--config",), ("--config", ""), ("--config", "--no-viewer"),
                         ("--config", "missing.conf")):
                with self.subTest(kind=kind, args=args):
                    self.assertNotEqual(self.launch(kind, *args).returncode, 0)
                    self.assertEqual(self.records(), [])

    def test_failed_preflight_does_not_launch(self):
        for kind in ("source", "package"):
            with self.subTest(kind=kind):
                self.assertEqual(self.launch(kind, LAUNCH_TEST_PREFLIGHT_EXIT="1").returncode, 1)
                self.assertEqual(len(self.records()), 1)

    def test_without_platform_selects_matching_build_and_checker(self):
        result = self.launch("source", "--without-platform")
        self.assert_config("source", result, self.project / "sharpa_mujoco.conf")
        for record in self.records():
            self.assertIn("/build-linux-no-platform/", record["exe"])
        self.assertIn("--without-platform", self.records()[1]["args"])

    def test_explicit_build_overrides_default_variant(self):
        directory = self.project / "custom build"
        self.make_build(directory)
        for option, value in (("-B", "custom build"), ("--build-dir", str(directory))):
            with self.subTest(option=option):
                result = self.launch("source", option, value, "--without-platform")
                self.assert_config("source", result, self.project / "sharpa_mujoco.conf")
                for record in self.records():
                    self.assertTrue(record["exe"].startswith(str(directory) + "/"))
                    self.assertNotIn(option, record["args"])

    def test_missing_selected_variant_never_falls_back_to_old_build(self):
        (self.project / "build-linux-no-platform/apps/simulator/sharpa_mujoco_simulator").unlink()
        result = self.launch("source", "--without-platform")
        self.assertNotEqual(result.returncode, 0)
        self.assertIn("build-linux-no-platform", result.stderr)
        self.assertEqual(self.records(), [])

    def test_executable_override_uses_its_matching_checker(self):
        directory = self.root / "override build"
        self.make_build(directory)
        relative = os.path.relpath(directory / "apps/simulator/sharpa_mujoco_simulator", self.caller)
        result = self.launch("source", SHARPA_MUJOCO_SIMULATOR=relative)
        self.assert_config("source", result, self.project / "sharpa_mujoco.conf")
        self.assertEqual(self.records()[0]["exe"], str(directory / "sharpa_mujoco_environment"))

    def test_invalid_build_selection_fails_before_preflight(self):
        for args in (("-B",), ("-B", ""), ("--with-platform", "--without-platform")):
            with self.subTest(args=args):
                self.assertEqual(self.launch("source", *args).returncode, 2)
                self.assertEqual(self.records(), [])


if __name__ == "__main__":
    unittest.main()
