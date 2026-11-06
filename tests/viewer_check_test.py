#!/usr/bin/env python3
# Copyright 2026 Sharpa
# SPDX-License-Identifier: Apache-2.0
"""Exercise the native preflight only: never start devices or Platform."""
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest

SIMULATOR = Path(sys.argv.pop(1)).resolve()


class ViewerCheckTest(unittest.TestCase):
    def check(self, overrides=None, extra=()):
        environment = dict(os.environ)
        for key in ("LIBGL_ALWAYS_SOFTWARE", "__GLX_VENDOR_LIBRARY_NAME",
                    "MESA_EXTENSION_OVERRIDE", "MESA_GL_VERSION_OVERRIDE"):
            environment.pop(key, None)
        environment.update(overrides or {})
        with tempfile.TemporaryDirectory() as directory:
            config = Path(directory) / "disabled.conf"
            config.write_text("[simulation]\nmodel=unused.xml\n[viewer]\nenabled=false\n")
            return subprocess.run([str(SIMULATOR), "--config", str(config),
                "--check-viewer", *extra], env=environment,
                capture_output=True, text=True, timeout=15)

    def assert_failure(self, result, reason):
        self.assertNotEqual(result.returncode, 0)
        self.assertIn(reason, result.stderr)
        self.assertIn("Required:", result.stderr)
        self.assertIn("GL_ARB_vertex_buffer_object", result.stderr)
        self.assertIn("CONFIGURATION", result.stderr)
        self.assertNotIn("Platform process started", result.stdout)
        if "is unavailable" in reason or "compatibility functions" in reason:
            for label in ("vendor", "renderer", "version"):
                self.assertIn("OpenGL " + label + ":", result.stderr)

    def test_disabled_config_still_checks_display(self):
        self.assert_failure(self.check({"DISPLAY": ":9876"}), "initialize GLFW")

    def test_no_viewer_argument_still_checks_display(self):
        self.assert_failure(self.check({"DISPLAY": ":9876"}, ("--no-viewer",)), "initialize GLFW")

    def test_mesa_injected_failures(self):
        # These variables inject test faults; the application never sets them.
        mesa = {"__GLX_VENDOR_LIBRARY_NAME": "mesa", "LIBGL_ALWAYS_SOFTWARE": "1"}
        if self.check(mesa).returncode:
            self.skipTest("No working desktop Mesa test context")
        for extension in ("GL_ARB_framebuffer_object", "GL_ARB_vertex_buffer_object"):
            with self.subTest(extension=extension):
                self.assert_failure(self.check(dict(mesa, MESA_EXTENSION_OVERRIDE="-" + extension)),
                                    extension + " is unavailable")
        self.assert_failure(self.check(dict(mesa, MESA_GL_VERSION_OVERRIDE="3.3FC")),
                            "compatibility functions")
        self.assert_failure(self.check(dict(mesa, MESA_GL_VERSION_OVERRIDE="3.1FC",
                            MESA_EXTENSION_OVERRIDE="-GL_ARB_compatibility")),
                            "compatibility functions")
        self.assert_failure(self.check(dict(mesa, MESA_GL_VERSION_OVERRIDE="1.4")),
                            "create an OpenGL window")


if __name__ == "__main__":
    unittest.main()
