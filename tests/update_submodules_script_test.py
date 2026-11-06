#!/usr/bin/env python3
# Copyright 2026 Sharpa
# SPDX-License-Identifier: Apache-2.0
"""Exercise the updater against local Git remotes, without network or robot assets."""
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]


class UpdateSubmodulesTest(unittest.TestCase):
    def git(self, directory, *args):
        return subprocess.check_output(["git", "-C", str(directory), *args],
                                       env=self.env, text=True, stderr=subprocess.PIPE).strip()

    def commit(self, directory):
        self.git(directory, "add", ".")
        self.git(directory, "commit", "-qm", "fixture")
        return self.git(directory, "rev-parse", "HEAD")

    def sdk_checksums(self):
        records = []
        for path in sorted(self.sdk.rglob("*")):
            if path.is_file() and ".git" not in path.parts and path.name != "SHA256SUMS":
                records.append(f"{hashlib.sha256(path.read_bytes()).hexdigest()}  {path.relative_to(self.sdk)}")
        (self.sdk / "SHA256SUMS").write_text("\n".join(records) + "\n")

    def setUp(self):
        temporary = tempfile.TemporaryDirectory()
        self.addCleanup(temporary.cleanup)
        self.root = Path(temporary.name) / "workspace with spaces"
        self.root.mkdir()
        self.env = dict(os.environ, GIT_ALLOW_PROTOCOL="file", GIT_CONFIG_NOSYSTEM="1", LC_ALL="C",
                        GIT_CONFIG_GLOBAL=os.devnull, GIT_AUTHOR_NAME="Test",
                        GIT_AUTHOR_EMAIL="test@example.invalid", GIT_COMMITTER_NAME="Test",
                        GIT_COMMITTER_EMAIL="test@example.invalid")
        # Stub only LFS: real Git still initializes, fetches and checks out submodules.
        bin_dir = self.root / "bin"
        bin_dir.mkdir()
        (bin_dir / "git-lfs").write_text('#!/bin/sh\necho "$*" >> "$LFS_TEST_LOG"\n')
        (bin_dir / "git-lfs").chmod(0o755)
        self.env["PATH"] = str(bin_dir) + os.pathsep + self.env["PATH"]
        self.env["LFS_TEST_LOG"] = str(self.root / "lfs.log")
        self.assets = self.root / "asset-remote"
        self.sdk = self.root / "sdk-remote"
        self.parent = self.root / "simulation"
        for directory in (self.assets, self.sdk, self.parent):
            directory.mkdir()
            self.git(directory, "init", "-qb", "main")
        (self.assets / "mesh").write_text("original asset")
        self.asset_pinned = self.commit(self.assets)
        files = {"Version": "VERSION=1.2.0\n",
                 "BUILD-MANIFEST.json": json.dumps({"version": "1.2.0"}),
                 "setup.bash": "# SDK setup\n", "check.sh": "#!/bin/sh\nexit 0\n",
                 "include/north_sdk/NorthSDK.h": "header",
                 "lib/libnorth_sdk.so": "library",
                 "lib/cmake/NorthSDK/NorthSDKConfig.cmake": "# config",
                 "lib/python/north_sdk/__init__.py": "# binding",
                 "lib/python/north_sdk/_north_sdk.cpython-310.so": "binding"}
        for name, content in files.items():
            path = self.sdk / name
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_text(content)
        self.sdk_checksums()
        self.sdk_pinned = self.commit(self.sdk)
        for source, destination in ((self.assets, "mjcf/sharpa-assets"),
                                    (self.sdk, "third_party/north_sdk")):
            self.git(self.parent, "submodule", "add", "-b", "main", str(source), destination)
        (self.parent / "scripts").mkdir()
        shutil.copy2(ROOT / "scripts/update_submodules.sh", self.parent / "scripts/update_submodules.sh")
        (self.parent / "tools").mkdir()
        (self.parent / "tools/robot_assets.py").write_text("print('asset layout checked')\n")
        self.commit(self.parent)
        (self.assets / "mesh").write_text("updated asset")
        self.asset_latest = self.commit(self.assets)
        (self.sdk / "README.md").write_text("updated SDK")
        self.sdk_checksums()
        self.sdk_latest = self.commit(self.sdk)
        self.sdk_checkout = self.parent / "third_party/north_sdk"
        self.asset_checkout = self.parent / "mjcf/sharpa-assets"

    def run_update(self, *args):
        return subprocess.run([str(self.parent / "scripts/update_submodules.sh"), *args],
                              cwd=self.root, env=self.env, text=True, capture_output=True)

    def test_default_and_pinned_initialize_both_at_recorded_commits(self):
        self.git(self.parent, "submodule", "deinit", "--force", "--all")
        for args in ((), ("--pinned",)):
            result = self.run_update(*args)
            self.assertEqual(result.returncode, 0, result.stderr)
            self.assertEqual(self.git(self.sdk_checkout, "rev-parse", "HEAD"), self.sdk_pinned)
            self.assertEqual(self.git(self.asset_checkout, "rev-parse", "HEAD"), self.asset_pinned)

    def test_remote_updates_both_but_pinned_restores_recorded_commits(self):
        result = self.run_update("--remote")
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(self.git(self.sdk_checkout, "rev-parse", "HEAD"), self.sdk_latest)
        self.assertEqual(self.git(self.asset_checkout, "rev-parse", "HEAD"), self.asset_latest)
        result = self.run_update()
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(self.git(self.sdk_checkout, "rev-parse", "HEAD"), self.sdk_pinned)
        self.assertEqual(self.git(self.asset_checkout, "rev-parse", "HEAD"), self.asset_pinned)

    def test_sdk_only_skips_assets_and_lfs_and_ignores_runtime_bytecode(self):
        (self.asset_checkout / "local-edit").write_text("keep")
        cache = self.sdk_checkout / "lib/python/north_sdk/__pycache__"
        cache.mkdir()
        (cache / "__init__.pyc").write_bytes(b"runtime bytecode")
        result = self.run_update("--only", "sdk", "--remote")
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(self.git(self.sdk_checkout, "rev-parse", "HEAD"), self.sdk_latest)
        self.assertEqual(self.git(self.asset_checkout, "rev-parse", "HEAD"), self.asset_pinned)
        self.assertFalse((self.root / "lfs.log").exists())
        self.assertEqual(self.git(self.sdk_checkout, "status", "--porcelain"), "")

    def test_assets_only_leaves_sdk_alone(self):
        (self.sdk_checkout / "local-edit").write_text("keep")
        result = self.run_update("--only", "assets", "--remote")
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(self.git(self.asset_checkout, "rev-parse", "HEAD"), self.asset_latest)
        self.assertEqual(self.git(self.sdk_checkout, "rev-parse", "HEAD"), self.sdk_pinned)
        self.assertIn("--include=d01/common/visual/**", (self.root / "lfs.log").read_text())

    def test_dirty_selected_submodule_blocks_all_updates(self):
        (self.sdk_checkout / "local-edit").write_text("keep")
        result = self.run_update("--remote")
        self.assertNotEqual(result.returncode, 0)
        self.assertIn("local changes", result.stderr)
        self.assertEqual(self.git(self.asset_checkout, "rev-parse", "HEAD"), self.asset_pinned)
        self.assertEqual((self.sdk_checkout / "local-edit").read_text(), "keep")

    def test_bad_upstream_checksum_is_rejected(self):
        (self.sdk / "lib/libnorth_sdk.so").write_text("corrupt download")
        self.commit(self.sdk)
        result = self.run_update("--remote", "--only", "sdk")
        self.assertNotEqual(result.returncode, 0)
        self.assertIn("FAILED", result.stdout + result.stderr)

    def test_manifest_version_mismatch_is_rejected(self):
        (self.sdk / "Version").write_text("VERSION=2.0.0\n")
        self.sdk_checksums()
        self.commit(self.sdk)
        result = self.run_update("--only", "sdk", "--remote")
        self.assertNotEqual(result.returncode, 0)
        self.assertIn("disagree", result.stderr)

    def test_invalid_options_do_not_update(self):
        for args in (("--only",), ("--only", "both"), ("--pinned", "--remote"), ("--bad",)):
            result = self.run_update(*args)
            self.assertEqual(result.returncode, 2, result.stderr)
        self.assertEqual(self.git(self.sdk_checkout, "rev-parse", "HEAD"), self.sdk_pinned)


if __name__ == "__main__":
    unittest.main()
