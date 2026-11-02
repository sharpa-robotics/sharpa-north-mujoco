#!/usr/bin/env python3
"""Exercise the generic Platform artifact boundary without plugin knowledge."""

import importlib.util
import io
import json
from pathlib import Path
import shutil
import subprocess
import sys
import tarfile
import tempfile
import unittest
from unittest.mock import patch

spec = importlib.util.spec_from_file_location(
    "platform_runtime", Path(__file__).parents[1] / "scripts/platform_runtime.py"
)
runtime = importlib.util.module_from_spec(spec)
spec.loader.exec_module(runtime)
LFS_POINTER = (
    "version https://git-lfs.github.com/spec/v1\n"
    f"oid sha256:{'a' * 64}\nsize 173390510\n"
)


class RuntimeArchiveTest(unittest.TestCase):
    BASE_PROFILES = {
        "fixture": {
            "config": "config/sim/fixture.json",
            "files": ["config/sim/fixture.M1.B1.json"],
        },
    }

    def production(self, base, target_platform, profiles=None):
        source = base / f"production-{target_platform.lower()}"
        platform_spec = next(
            item
            for item in runtime.PLATFORM_SPECS.values()
            if item["target_platform"] == target_platform
        )
        source.mkdir(parents=True)
        elf = bytearray(20)
        elf[:6] = b"\x7fELF\x02\x01"
        elf[18:20] = platform_spec["elf_machine"].to_bytes(2, "little")
        for name in ("loader", "avatar_sdk/bin/avatar-backend"):
            path = source / name
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_bytes(elf)
            path.chmod(0o755)
        release = {
            "build_id": f"test.{target_platform}",
            "target_platform": target_platform,
        }
        (source / "release.json").write_text(json.dumps(release))
        for profile in (profiles or self.BASE_PROFILES).values():
            for name in profile["files"]:
                path = source / name
                path.parent.mkdir(parents=True, exist_ok=True)
                path.write_text("{}")
        return source

    def package(self, source, bundle, profiles=None):
        profiles = profiles or self.BASE_PROFILES
        bundle.mkdir(parents=True, exist_ok=True)
        archive = bundle / runtime.ARCHIVE
        with tarfile.open(archive, "w:gz") as tar:
            for path in sorted(source.rglob("*")):
                tar.add(path, arcname=path.relative_to(source), recursive=False)
        release = json.loads((source / "release.json").read_text())
        manifest = {
            "schema_version": 2,
            "name": "sharpa_platform",
            "archive": runtime.ARCHIVE,
            "archive_format": "tar.gz",
            "archive_sha256": runtime.digest(archive),
            "archive_size": archive.stat().st_size,
            "entrypoint": "loader",
            "default_profile": "fixture",
            "profiles": profiles,
            "release": release,
        }
        manifest_path = bundle / runtime.MANIFEST
        manifest_path.write_text(json.dumps(manifest))
        return archive, manifest_path

    def direct_archive(self, source, bundle):
        bundle.mkdir(parents=True, exist_ok=True)
        archive = bundle / runtime.ARCHIVE
        with tarfile.open(archive, "w:gz", dereference=False) as tar:
            for path in sorted(source.iterdir()):
                tar.add(path, arcname=path.name, recursive=True)
        return archive

    def extract(self, members):
        temporary = tempfile.TemporaryDirectory(prefix="sharpa-archive-test-")
        self.addCleanup(temporary.cleanup)
        root = Path(temporary.name)
        archive = root / runtime.ARCHIVE
        with tarfile.open(archive, "w:gz") as tar:
            for name, link in members:
                info = tarfile.TarInfo(name)
                if link:
                    info.type = tarfile.SYMTYPE
                    info.linkname = link
                    tar.addfile(info)
                else:
                    info.size = 4
                    info.mode = 0o755
                    tar.addfile(info, io.BytesIO(b"test"))
        destination = root / "expanded"
        destination.mkdir()
        runtime.extract_archive(archive, destination)
        return destination

    def test_safe_files_and_symlinks(self):
        root = self.extract([("lib/test.so.1", None), ("lib/test.so", "test.so.1")])
        self.assertEqual((root / "lib/test.so").read_bytes(), b"test")
        self.assertTrue((root / "lib/test.so.1").stat().st_mode & 0o111)

    def test_unsafe_and_duplicate_members(self):
        for name, target in [
            ("../escape", None),
            ("/tmp/escape", None),
            ("lib/escape", "../../escape"),
        ]:
            with self.subTest(name=name), self.assertRaises(ValueError):
                self.extract([(name, target)])
        with self.assertRaises(ValueError):
            self.extract([("config/test", None), ("config/test", None)])

    def test_invalid_contract_paths(self):
        for name in ["../x", "/x", "a\nb", "a\0b", ""]:
            with self.assertRaises(ValueError):
                runtime.safe_relative(name)

    def test_x86_bundle_stages_manifest_profiles_without_plugin_knowledge(self):
        with tempfile.TemporaryDirectory(prefix="sharpa-runtime-test-") as temporary:
            base = Path(temporary)
            source = self.production(base, "GEN_X86", self.BASE_PROFILES)
            bundle = base / "linux-x86_64"
            self.package(source, bundle, self.BASE_PROFILES)
            destination = base / "stage/sharpa_platform"
            runtime.stage_runtime(bundle, destination)
            descriptor = (destination / "companion.conf").read_text()
            self.assertNotIn("environment.SHARPA_LOG_DIR=", descriptor)
            self.assertIn("default_profile=fixture", descriptor)
            self.assertIn(
                "profile.fixture=config/sim/fixture.json",
                descriptor,
            )
            before = (destination / "loader").stat().st_ino
            runtime.stage_runtime(bundle, destination)
            self.assertEqual(before, (destination / "loader").stat().st_ino)
            (bundle / runtime.ARCHIVE).write_bytes(b"corrupt archive")
            with self.assertRaisesRegex(ValueError, "SHA-256/size mismatch"):
                runtime.stage_runtime(bundle, destination)
            self.assertEqual(before, (destination / "loader").stat().st_ino)
            (bundle / runtime.ARCHIVE).write_text(LFS_POINTER)
            with self.assertRaisesRegex(ValueError, "Git LFS pointer"):
                runtime.stage_runtime(bundle, destination)
            self.assertEqual(before, (destination / "loader").stat().st_ino)

    def test_thor_uses_manifest_without_x86_dependency_knowledge(self):
        with tempfile.TemporaryDirectory(prefix="sharpa-thor-runtime-test-") as temporary:
            base = Path(temporary)
            source = self.production(base, "GEN_THOR", self.BASE_PROFILES)
            bundle = base / "linux-aarch64"
            self.package(source, bundle, self.BASE_PROFILES)
            destination = base / "stage/sharpa_platform"
            runtime.stage_runtime(bundle, destination)
            manifest = json.loads((bundle / runtime.MANIFEST).read_text())
            self.assertEqual(manifest["release"]["target_platform"], "GEN_THOR")
            self.assertIn(
                "profile.fixture=config/sim/fixture.json",
                (destination / "companion.conf").read_text(),
            )
            wrong_bundle = base / "linux-x86_64"
            shutil.copytree(bundle, wrong_bundle)
            with self.assertRaisesRegex(ValueError, "must target GEN_X86"):
                runtime.check_package_available(wrong_bundle)

    def test_runtime_overlay_survives_restaging_and_invalidates_cache(self):
        with tempfile.TemporaryDirectory(prefix="sharpa-overlay-test-") as temporary:
            base = Path(temporary)
            source = self.production(base, "GEN_X86")
            bundle = base / "linux-x86_64"
            self.package(source, bundle)
            destination = base / "stage/sharpa_platform"
            overlay = base / "overlay"
            config = overlay / "config/zenoh_config.json"
            config.parent.mkdir(parents=True)
            config.write_text('{"zenoh":{"transport_mode":"udp"}}')
            with patch.object(runtime, "RUNTIME_OVERLAY", overlay):
                runtime.stage_runtime(bundle, destination)
                staged_config = destination / "config/zenoh_config.json"
                self.assertEqual(staged_config.read_bytes(), config.read_bytes())
                before = (destination / "loader").stat().st_ino
                runtime.stage_runtime(bundle, destination)
                self.assertEqual(before, (destination / "loader").stat().st_ino)
                config.write_text('{"zenoh":{"transport_mode":"udp","qos":true}}')
                runtime.stage_runtime(bundle, destination)
                self.assertEqual(staged_config.read_bytes(), config.read_bytes())
                self.assertNotEqual(before, (destination / "loader").stat().st_ino)

    def test_variant_catalog_stages_without_base_file_on_both_targets(self):
        for target, bundle_name in [("GEN_X86", "linux-x86_64"), ("GEN_THOR", "linux-aarch64")]:
            with self.subTest(target=target), tempfile.TemporaryDirectory() as temporary:
                base = Path(temporary)
                source = self.production(base, target)
                second = "config/sim/fixture.M1.B2.json"
                (source / second).write_text("{}")
                other = "config/sim/alternate.json"
                (source / other).write_text("{}")
                profiles = {
                    "fixture": {"config": "config/sim/fixture.json",
                                "files": [*self.BASE_PROFILES["fixture"]["files"], second]},
                    "alternate": {"config": other, "files": [other]},
                }
                bundle = base / bundle_name
                self.package(source, bundle, profiles)
                destination = base / "stage/sharpa_platform"
                runtime.stage_runtime(bundle, destination)
                descriptor = (destination / "companion.conf").read_text()
                self.assertIn("schema_version=2\n", descriptor)
                self.assertIn("profile.fixture=config/sim/fixture.json\n", descriptor)
                self.assertIn(f"profile_file.fixture.1={second}\n", descriptor)
                self.assertIn(f"profile.alternate={other}\n", descriptor)
                self.assertFalse((destination / "config/sim/fixture.json").exists())
                # Cache hits must still reject a removed actual variant.
                (destination / second).unlink()
                with self.assertRaisesRegex(ValueError, "profile is missing"):
                    runtime.stage_runtime(bundle, destination)

    def test_variant_catalog_rejects_missing_unsafe_and_empty_files(self):
        with tempfile.TemporaryDirectory() as temporary:
            base = Path(temporary)
            source = self.production(base, "GEN_X86")
            bundle = base / "linux-x86_64"
            for files, message in [([], "has no files"), (["../escape"], "Unsafe"),
                                   (["config/sim/missing.M1.B1.json"], "profile is missing")]:
                with self.subTest(files=files):
                    self.package(source, bundle, {
                        "fixture": {"config": "config/sim/fixture.json", "files": files}
                    })
                    with self.assertRaisesRegex(ValueError, message):
                        runtime.stage_runtime(bundle, base / "stage/sharpa_platform")

    def test_manifestless_archive_is_rejected(self):
        with tempfile.TemporaryDirectory(prefix="sharpa-direct-runtime-test-") as temporary:
            base = Path(temporary)
            source = self.production(base, "GEN_THOR", self.BASE_PROFILES)
            bundle = base / "linux-aarch64"
            destination = base / "stage/sharpa_platform"
            self.direct_archive(source, bundle)
            with self.assertRaisesRegex(ValueError, "manifest is missing"):
                runtime.stage_runtime(bundle, destination)

    def test_stage_rejects_missing_manifest_profile(self):
        with tempfile.TemporaryDirectory(prefix="sharpa-profile-contract-test-") as temporary:
            base = Path(temporary)
            source = self.production(base, "GEN_X86", self.BASE_PROFILES)
            bundle = base / "linux-x86_64"
            profiles = {
                **self.BASE_PROFILES,
                "missing": {"config": "config/sim/missing.json", "files": ["config/sim/missing.json"]},
            }
            self.package(source, bundle, profiles)
            with self.assertRaisesRegex(ValueError, "profile is missing"):
                runtime.stage_runtime(bundle, base / "stage/sharpa_platform")

    def test_check_reports_missing_archive_and_lfs_pointers(self):
        with tempfile.TemporaryDirectory(prefix="sharpa-preflight-test-") as temporary:
            bundle = Path(temporary)
            (bundle / runtime.MANIFEST).write_text("{}")
            archive = bundle / runtime.ARCHIVE
            for content in (None, LFS_POINTER, LFS_POINTER.replace("\n", "\r\n")):
                with self.subTest(content=content):
                    if content is not None:
                        archive.write_bytes(content.encode())
                    result = subprocess.run(
                        [
                            sys.executable,
                            str(Path(runtime.__file__)),
                            "check",
                            "--bundle",
                            str(bundle),
                        ],
                        capture_output=True,
                        text=True,
                    )
                    self.assertNotEqual(result.returncode, 0)
                    self.assertIn(
                        "is missing" if content is None else "Git LFS pointer",
                        result.stderr,
                    )
                    self.assertIn("--without-platform", result.stderr)
                    self.assertNotIn("Traceback", result.stderr)

    def test_check_rejects_missing_manifest(self):
        with tempfile.TemporaryDirectory(prefix="sharpa-manifest-test-") as temporary:
            bundle = Path(temporary) / "linux-x86_64"
            bundle.mkdir()
            (bundle / runtime.ARCHIVE).write_bytes(b"downloaded archive fixture")
            with self.assertRaisesRegex(ValueError, "manifest is missing"):
                runtime.check_package_available(bundle)


@unittest.skipUnless(shutil.which("cmake"), "CMake is required for configuration tests")
class RuntimeConfigureTest(unittest.TestCase):
    def setUp(self):
        temporary = tempfile.TemporaryDirectory(prefix="sharpa-configure-test-")
        self.addCleanup(temporary.cleanup)
        self.source = Path(temporary.name) / "source with spaces"
        self.build = Path(temporary.name) / "build"
        self.bundle = self.source / "third_party/sharpa_platform/linux-x86_64"
        self.bundle.mkdir(parents=True)
        self.archive = self.bundle / runtime.ARCHIVE
        self.archive.write_bytes(b"downloaded archive fixture")
        self.write_manifest(self.bundle, "GEN_X86", self.archive)
        for name in ("scripts/platform_runtime.py", "cmake/PlatformRuntime.cmake"):
            destination = self.source / name
            destination.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(runtime.PROJECT_ROOT / name, destination)
        (self.source / "CMakeLists.txt").write_text(
            "cmake_minimum_required(VERSION 3.22)\n"
            "project(PlatformCheck LANGUAGES NONE)\n"
            "set(CMAKE_SYSTEM_NAME Linux)\n"
            "set(CMAKE_SYSTEM_PROCESSOR x86_64)\n"
            "set(CMAKE_INSTALL_LIBDIR lib)\n"
            "include(cmake/PlatformRuntime.cmake)\n"
            "add_custom_target(compile_probe ALL\n"
            '  COMMAND "${CMAKE_COMMAND}" -E touch "${CMAKE_BINARY_DIR}/compiled")\n'
        )

    def cmake(self, *args):
        return subprocess.run(
            ["cmake", *map(str, args)],
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
        )

    def configure(self, *args):
        return self.cmake("-S", self.source, "-B", self.build, *args)

    def write_manifest(self, bundle, target_platform, archive):
        manifest = {
            "schema_version": 2,
            "name": "sharpa_platform",
            "archive": runtime.ARCHIVE,
            "archive_format": "tar.gz",
            "archive_sha256": "a" * 64,
            "archive_size": archive.stat().st_size,
            "entrypoint": "loader",
            "default_profile": "fixture",
            "profiles": RuntimeArchiveTest.BASE_PROFILES,
            "release": {"target_platform": target_platform},
        }
        (bundle / runtime.MANIFEST).write_text(json.dumps(manifest))

    def test_configuration_rejects_pointer_before_generation(self):
        self.archive.write_text(LFS_POINTER)
        result = self.configure()
        self.assertNotEqual(result.returncode, 0, result.stdout)
        self.assertIn("Git LFS pointer", result.stdout)
        self.assertIn("git lfs pull", result.stdout)
        self.assertFalse((self.build / "compiled").exists())
        self.assertNotIn("Generating done", result.stdout)

    def test_build_rechecks_archive_after_checkout(self):
        result = self.configure()
        self.assertEqual(result.returncode, 0, result.stdout)
        self.archive.write_text(LFS_POINTER)
        result = self.cmake("--build", self.build, "--parallel", "4")
        self.assertNotEqual(result.returncode, 0, result.stdout)
        self.assertIn("Git LFS pointer", result.stdout)
        self.assertFalse((self.build / "compiled").exists())

    def test_platform_disabled_allows_pointer(self):
        self.archive.write_text(LFS_POINTER)
        result = self.configure("-DSHARPA_MUJOCO_STAGE_PLATFORM=OFF")
        self.assertEqual(result.returncode, 0, result.stdout)
        result = self.cmake("--build", self.build, "--parallel", "4")
        self.assertEqual(result.returncode, 0, result.stdout)
        self.assertTrue((self.build / "compiled").is_file())

    def test_thor_configuration_selects_aarch64_bundle(self):
        thor_bundle = self.source / "third_party/sharpa_platform/linux-aarch64"
        thor_bundle.mkdir(parents=True)
        thor_archive = thor_bundle / runtime.ARCHIVE
        thor_archive.write_bytes(b"downloaded Thor archive fixture")
        self.write_manifest(thor_bundle, "GEN_THOR", thor_archive)
        self.archive.write_text(LFS_POINTER)
        cmake_lists = self.source / "CMakeLists.txt"
        cmake_lists.write_text(
            cmake_lists.read_text().replace(
                "set(CMAKE_SYSTEM_PROCESSOR x86_64)",
                "set(CMAKE_SYSTEM_PROCESSOR aarch64)",
            )
        )
        result = self.configure()
        self.assertEqual(result.returncode, 0, result.stdout)
        self.assertNotIn("Git LFS pointer", result.stdout)
        shutil.rmtree(self.build)
        thor_archive.write_text(LFS_POINTER)
        result = self.configure()
        self.assertNotEqual(result.returncode, 0, result.stdout)
        self.assertIn("linux-aarch64", result.stdout)
        self.assertIn("Git LFS pointer", result.stdout)


if __name__ == "__main__":
    unittest.main()
