#!/usr/bin/env python3
"""Check build/clean behavior in isolation; never compile or delete real builds."""

import json
import os
from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest


SCRIPT = Path(__file__).resolve().parents[1] / "scripts" / "build.sh"


class BuildScriptTest(unittest.TestCase):
    def setUp(self):
        temporary = tempfile.TemporaryDirectory(prefix="sharpa-build-test-")
        self.addCleanup(temporary.cleanup)
        self.root = Path(temporary.name).resolve()
        self.project = self.root / "project with spaces"
        (self.project / "scripts").mkdir(parents=True)
        (self.project / "CMakeLists.txt").write_text("project(SharpaMujoco)\n")
        self.script = self.project / "scripts" / "build.sh"
        shutil.copy2(SCRIPT, self.script)
        self.build = self.project / "build-linux"
        self.log = self.root / "commands.jsonl"
        self.bin = self.root / "bin"
        self.bin.mkdir()
        # The rm stub is a second safety boundary: even a broken guard must
        # never remove anything outside this test's temporary directory.
        stub = """#!/usr/bin/env python3
import json, os, pathlib, shutil, sys
tool = pathlib.Path(sys.argv[0]).name
args = sys.argv[1:]
with open(os.environ['TEST_COMMAND_LOG'], 'a') as log:
    log.write(json.dumps([tool, args]) + '\\n')
if tool == 'cmake' and '-S' in args:
    source = pathlib.Path(args[args.index('-S') + 1]).resolve()
    build = pathlib.Path(args[args.index('-B') + 1]).resolve()
    build.mkdir(parents=True, exist_ok=True)
    (build / 'CMakeCache.txt').write_text(
        f'CMAKE_HOME_DIRECTORY:INTERNAL={source}\\n'
        f'CMAKE_CACHEFILE_DIR:INTERNAL={build}\\n')
elif tool == 'rm':
    target = pathlib.Path(args[-1]).resolve()
    root = pathlib.Path(os.environ['TEST_TEMP_ROOT'])
    if root not in target.parents:
        sys.exit('test safety boundary refused deletion: ' + str(target))
    shutil.rmtree(target)
"""
        for name in ("cmake", "ctest", "rm"):
            executable = self.bin / name
            executable.write_text(stub)
            executable.chmod(0o755)
        self.env = dict(
            os.environ,
            PATH=str(self.bin) + os.pathsep + os.environ["PATH"],
            TEST_COMMAND_LOG=str(self.log),
            TEST_TEMP_ROOT=str(self.root),
        )

    def configure_directory(self, directory=None, source=None, cached_build=None):
        directory = self.build if directory is None else directory
        source = self.project if source is None else source
        cached_build = directory if cached_build is None else cached_build
        directory.mkdir(parents=True, exist_ok=True)
        (directory / "CMakeCache.txt").write_text(
            f"CMAKE_HOME_DIRECTORY:INTERNAL={source}\n"
            f"CMAKE_CACHEFILE_DIR:INTERNAL={cached_build}\n"
        )
        marker = directory / "previous-output.o"
        marker.write_text("keep until explicitly cleaned")
        return marker

    def run_build(self, *arguments):
        self.log.unlink(missing_ok=True)
        return subprocess.run(
            ["bash", str(self.script), *map(str, arguments)],
            cwd=self.root,
            env=self.env,
            capture_output=True,
            text=True,
            timeout=10,
        )

    def commands(self):
        return [json.loads(line) for line in self.log.read_text().splitlines()] if self.log.exists() else []

    def assert_refused(self, directory):
        result = self.run_build("--clean", "-B", directory)
        self.assertEqual(result.returncode, 2, result.stdout + result.stderr)
        self.assertIn("refusing to clean", result.stderr)
        self.assertEqual(self.commands(), [])

    def test_test_selection_preserves_build_outputs(self):
        marker = self.configure_directory()
        for arguments, enabled in (
            ((), False),
            (("--tests",), True),
            (("--no-tests",), False),
            (("--tests", "--no-tests"), False),
            (("--no-tests", "--tests"), True),
        ):
            with self.subTest(arguments=arguments):
                result = self.run_build(*arguments)
                self.assertEqual(result.returncode, 0, result.stderr)
                self.assertTrue(marker.is_file())
                commands = self.commands()
                expected_tools = ["cmake", "cmake"] + (["ctest"] if enabled else [])
                self.assertEqual([tool for tool, _ in commands], expected_tools)
                self.assertIn(str(self.build), commands[0][1])
                self.assertIn(
                    "-DSHARPA_MUJOCO_BUILD_TESTS=" + ("ON" if enabled else "OFF"),
                    commands[0][1],
                )

    def test_without_platform_reuses_its_separate_build_directory(self):
        directory = self.project / "build-linux-no-platform"
        marker = self.configure_directory(directory)
        result = self.run_build("--without-platform")
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertTrue(marker.is_file())
        self.assertIn(str(directory), self.commands()[0][1])
        self.assertIn("-DSHARPA_MUJOCO_STAGE_PLATFORM=OFF", self.commands()[0][1])

    def test_clean_removes_verified_outputs_before_rebuilding(self):
        marker = self.configure_directory()
        result = self.run_build("--clean")
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertFalse(marker.exists())
        self.assertTrue((self.build / "CMakeCache.txt").is_file())
        self.assertEqual([tool for tool, _ in self.commands()], ["rm", "cmake", "cmake"])

    def test_clean_supports_verified_external_build_directory(self):
        directory = self.root / "custom build"
        marker = self.configure_directory(directory)
        result = self.run_build("--clean", "-B", directory, "--tests")
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertFalse(marker.exists())
        self.assertEqual([tool for tool, _ in self.commands()], ["rm", "cmake", "cmake", "ctest"])

    def test_clean_supports_new_relative_build_directory(self):
        result = self.run_build("--clean", "-B", "new-build")
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertTrue((self.project / "new-build" / "CMakeCache.txt").is_file())
        self.assertNotIn("rm", [tool for tool, _ in self.commands()])

    def test_clean_preserves_unrecognized_directory_and_files(self):
        directory = self.root / "Downloads"
        directory.mkdir()
        document = directory / "important.txt"
        document.write_text("user data")
        self.assert_refused(directory)
        self.assertEqual(document.read_text(), "user data")

    def test_clean_preserves_other_project_build(self):
        marker = self.configure_directory(source=self.root / "other-project")
        self.assert_refused(self.build)
        self.assertTrue(marker.is_file())

    def test_clean_rejects_copied_cache(self):
        marker = self.configure_directory(cached_build=self.root / "original-build")
        self.assert_refused(self.build)
        self.assertTrue(marker.is_file())

    def test_clean_rejects_missing_cache_identity(self):
        marker = self.configure_directory()
        cache = self.build / "CMakeCache.txt"
        for text in ("", f"CMAKE_HOME_DIRECTORY:INTERNAL={self.project}\n",
                     f"CMAKE_CACHEFILE_DIR:INTERNAL={self.build}\n"):
            with self.subTest(cache=text):
                cache.write_text(text)
                self.assert_refused(self.build)
                self.assertTrue(marker.is_file())

    def test_clean_rejects_source_ancestors_and_system_directories(self):
        for directory in (self.project, self.project.parent, "/", "/lib", "/usr/lib",
                          str(self.project / "nested" / "..")):
            with self.subTest(directory=directory):
                self.assert_refused(directory)
        self.assertTrue(self.script.is_file())

    def test_clean_rejects_source_directory_even_with_matching_cache(self):
        marker = self.configure_directory()
        (self.build / "CMakeLists.txt").write_text("project(Other)\n")
        self.assert_refused(self.build)
        self.assertTrue(marker.is_file())

    def test_clean_rejects_symbolic_links_and_linked_parents(self):
        marker = self.configure_directory()
        link = self.root / "build-link"
        link.symlink_to(self.build, target_is_directory=True)
        parent_link = self.root / "parent-link"
        parent_link.symlink_to(self.project, target_is_directory=True)
        for directory in (link, str(link) + "/", parent_link / self.build.name):
            with self.subTest(directory=directory):
                self.assert_refused(directory)
                self.assertTrue(marker.is_file())

    def test_clean_rejects_dangling_symbolic_link(self):
        self.build.symlink_to(self.root / "missing", target_is_directory=True)
        self.assert_refused(self.build)
        self.assertTrue(self.build.is_symlink())

    def test_clean_rejects_symbolic_link_cache(self):
        marker = self.configure_directory()
        cache = self.build / "CMakeCache.txt"
        external_cache = self.root / "cache.txt"
        cache.rename(external_cache)
        cache.symlink_to(external_cache)
        self.assert_refused(self.build)
        self.assertTrue(marker.is_file())

    def test_clean_rejects_regular_file_target(self):
        self.build.write_text("user data")
        self.assert_refused(self.build)
        self.assertEqual(self.build.read_text(), "user data")

    def test_clean_preserves_build_if_platform_preflight_fails(self):
        marker = self.configure_directory()
        for arch in ("x86_64", "aarch64"):
            bundle = self.project / "third_party" / "sharpa_platform" / f"linux-{arch}"
            bundle.mkdir(parents=True)
            (bundle / "sharpa-platform.tar.gz").write_text("version https://git-lfs.github.com/spec/v1\n")
        result = self.run_build("--clean")
        self.assertEqual(result.returncode, 1, result.stderr)
        self.assertTrue(marker.is_file())
        self.assertEqual(self.commands(), [])

    def test_empty_build_directory_argument_is_rejected(self):
        result = self.run_build("-B", "")
        self.assertEqual(result.returncode, 2, result.stderr)
        self.assertEqual(self.commands(), [])

    def test_removed_incremental_option_is_rejected(self):
        result = self.run_build("--incremental")
        self.assertEqual(result.returncode, 2, result.stderr)
        self.assertIn("unknown option", result.stderr)
        self.assertEqual(self.commands(), [])

    def test_help_describes_new_defaults(self):
        result = self.run_build("--help")
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn("incrementally by default", result.stdout)
        self.assertIn("--clean", result.stdout)
        self.assertIn("--tests", result.stdout)
        self.assertIn("--no-tests", result.stdout)
        self.assertNotIn("--incremental", result.stdout)
        self.assertEqual(self.commands(), [])


class MuJoCoCacheTest(unittest.TestCase):
    """Exercise real CMake discovery without compiling the simulator."""

    def setUp(self):
        temporary = tempfile.TemporaryDirectory(prefix="sharpa-mujoco-cache-")
        self.addCleanup(temporary.cleanup)
        self.root = Path(temporary.name).resolve()
        self.project = self.root / "project with spaces"
        (self.project / "scripts").mkdir(parents=True)
        (self.project / "cmake").mkdir()
        shutil.copy2(SCRIPT, self.project / "scripts/build.sh")
        shutil.copy2(SCRIPT.parent.parent / "cmake/FindMuJoCo.cmake", self.project / "cmake")
        (self.project / "CMakeLists.txt").write_text('''
cmake_minimum_required(VERSION 3.22)
project(MuJoCoDiscovery NONE)
list(PREPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")
find_package(MuJoCo REQUIRED MODULE)
file(WRITE "${CMAKE_BINARY_DIR}/mujoco-paths.txt"
    "${MUJOCO_ROOT}\\n${MuJoCo_INCLUDE_DIR}\\n${MuJoCo_LIBRARY}\\n${MuJoCo_RUNTIME_LIBRARY}\\n")
''')
        self.env = dict(os.environ, CMAKE_GENERATOR="Unix Makefiles")
        self.env.pop("MUJOCO_ROOT", None)
        self.first_sdk = self.make_sdk("first SDK")
        self.second_sdk = self.make_sdk("second SDK")

    def make_sdk(self, name):
        sdk = self.root / name
        (sdk / "include/mujoco").mkdir(parents=True)
        (sdk / "include/mujoco/mujoco.h").touch()
        (sdk / "lib").mkdir()
        (sdk / "lib/libmujoco.so").touch()
        return sdk

    def configure(self, sdk):
        (self.project / "sharpa_mujoco.conf").write_text(f'mujoco_root="{sdk}"\n')

    def build(self, **env):
        return subprocess.run(
            ["bash", str(self.project / "scripts/build.sh")], cwd=self.root,
            env=dict(self.env, **env), text=True, capture_output=True, timeout=15,
        )

    def assert_sdk(self, result, sdk):
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        paths = (self.project / "build-linux/mujoco-paths.txt").read_text().splitlines()
        self.assertEqual(paths, [str(sdk), str(sdk / "include"),
                                 str(sdk / "lib/libmujoco.so"), str(sdk / "lib/libmujoco.so")])

    def test_corrected_config_recovers_from_failed_first_build(self):
        self.configure(self.root / "missing SDK")
        self.assertNotEqual(self.build().returncode, 0)
        self.configure(self.first_sdk)
        self.assert_sdk(self.build(), self.first_sdk)

    def test_changed_config_refreshes_all_sdk_paths(self):
        self.configure(self.first_sdk)
        self.assert_sdk(self.build(), self.first_sdk)
        marker = self.project / "build-linux/previous-output.o"
        marker.write_text("existing build output")
        self.configure(self.second_sdk)
        self.assert_sdk(self.build(), self.second_sdk)
        self.assertTrue(marker.is_file())

    def test_environment_override_is_refreshed_and_can_be_removed(self):
        self.configure(self.first_sdk)
        self.assert_sdk(self.build(), self.first_sdk)
        self.assert_sdk(self.build(MUJOCO_ROOT=str(self.second_sdk)), self.second_sdk)
        self.assert_sdk(self.build(), self.first_sdk)

    def test_missing_new_sdk_does_not_reuse_previous_library(self):
        self.configure(self.first_sdk)
        self.assert_sdk(self.build(), self.first_sdk)
        self.configure(self.root / "missing SDK")
        self.assertNotEqual(self.build().returncode, 0)


if __name__ == "__main__":
    unittest.main()
