#!/usr/bin/env python3
# Copyright 2026 Sharpa
# SPDX-License-Identifier: Apache-2.0
import importlib.util
from pathlib import Path
import tempfile
import unittest

spec = importlib.util.spec_from_file_location("robot_assets",
    Path(__file__).resolve().parents[1] / "tools/robot_assets.py")
assets = importlib.util.module_from_spec(spec)
spec.loader.exec_module(assets)


class AssetsTest(unittest.TestCase):
    def setUp(self):
        directory = tempfile.TemporaryDirectory()
        self.addCleanup(directory.cleanup)
        self.root = Path(directory.name) / "assets with spaces"
        (self.root / "model").mkdir(parents=True)
        (self.root / "common").mkdir()
        for name in ("LICENSE.txt", "NOTICE.txt"):
            (self.root / name).write_text("notice")
        (self.root / "common/shape.stl").write_bytes(b"actual mesh content")
        (self.root / "model/robot.xml").write_text(
            '<mujoco><asset><mesh file="../common/shape.stl"/></asset></mujoco>')
        (self.root / "scene.xml").write_text(
            '<mujoco><include file="model/robot.xml"/></mujoco>')

    def test_cross_directory_dependency_closure(self):
        result = {str(p.relative_to(self.root)) for p in assets.dependencies(self.root, Path("scene.xml"))}
        self.assertEqual(result, {"scene.xml", "model/robot.xml", "common/shape.stl", "LICENSE.txt", "NOTICE.txt"})

    def test_lfs_placeholder_is_not_a_ready_asset(self):
        (self.root / "common/shape.stl").write_text(
            "version https://git-lfs.github.com/spec/v1\noid sha256:0123\nsize 42\n")
        with self.assertRaisesRegex(ValueError, "Git LFS content not downloaded"):
            assets.dependencies(self.root, Path("scene.xml"))

    def test_missing_dependency_is_reported(self):
        (self.root / "common/shape.stl").unlink()
        with self.assertRaisesRegex(ValueError, "Missing asset"):
            assets.dependencies(self.root, Path("scene.xml"))

    def test_external_reference_is_not_packaged(self):
        (self.root / "model/robot.xml").write_text(
            '<mujoco><asset><mesh file="../../outside.stl"/></asset></mujoco>')
        with self.assertRaisesRegex(ValueError, "escapes repository"):
            assets.dependencies(self.root, Path("scene.xml"))


if __name__ == "__main__":
    unittest.main()
