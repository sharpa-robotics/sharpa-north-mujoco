#!/usr/bin/env python3
"""Validate and stage a prebuilt Platform runtime bundle."""

import argparse
import contextlib
import fcntl
import hashlib
import json
import os
from pathlib import Path, PurePosixPath
import platform
import re
import shutil
import sys
import tarfile
import tempfile

PROJECT_ROOT = Path(__file__).resolve().parent.parent
PLATFORM_ROOT = PROJECT_ROOT / "third_party/sharpa_platform"
ARCHIVE = "sharpa-platform.tar.gz"
MANIFEST = "sharpa-platform.manifest.json"
PLATFORM_SPECS = {
    "linux-x86_64": {
        "target_platform": "GEN_X86",
        "elf_machine": 0x3E,
        "description": "Linux x86_64",
    },
    "linux-aarch64": {
        "target_platform": "GEN_THOR",
        "elf_machine": 0xB7,
        "description": "Linux aarch64 (Thor)",
    },
}
MACHINE_BUNDLES = {
    "x86_64": "linux-x86_64",
    "amd64": "linux-x86_64",
    "aarch64": "linux-aarch64",
    "arm64": "linux-aarch64",
}
HOST_BUNDLE_NAME = MACHINE_BUNDLES.get(
    platform.machine().lower(), f"linux-{platform.machine().lower()}"
)
BUNDLE = PLATFORM_ROOT / HOST_BUNDLE_NAME
RUNTIME_OVERLAY = PROJECT_ROOT / "resources/platform_runtime"


def require(condition, message):
    if not condition:
        raise ValueError(message)


def read_json(path):
    with path.open() as stream:
        return json.load(stream)


def digest(path):
    sha256 = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            sha256.update(chunk)
    return sha256.hexdigest()


def safe_relative(value):
    require(
        isinstance(value, str)
        and value
        and "\n" not in value
        and "\r" not in value
        and "\0" not in value,
        f"Invalid runtime path: {value!r}",
    )
    path = PurePosixPath(value)
    require(not path.is_absolute() and ".." not in path.parts, f"Unsafe runtime path: {value}")
    return path


def bundle_spec(bundle):
    name = Path(bundle).name
    require(
        name in PLATFORM_SPECS,
        f"Unsupported Platform bundle directory '{name}'; expected linux-x86_64 or linux-aarch64",
    )
    return PLATFORM_SPECS[name]


def check_archive_available(archive):
    try:
        repository_archive = archive.relative_to(PROJECT_ROOT)
    except ValueError:
        repository_archive = archive
    recovery = (
        "Install Git LFS if needed (Ubuntu/Debian):\n"
        "  sudo apt-get install git-lfs\n"
        "Then run from the sharpa_mujoco repository root:\n"
        "  git lfs install --local\n"
        f'  git lfs pull --include="{repository_archive}" --exclude=""\n'
        "Retry the build after downloading the archive.\n"
        "For a MuJoCo-only build, pass --without-platform to scripts/build.sh or\n"
        "scripts/package_runtime.sh, or configure CMake with -DSHARPA_MUJOCO_STAGE_PLATFORM=OFF."
    )
    require(archive.is_file(), f"Platform archive is missing: {archive}\n{recovery}")
    with archive.open("rb") as stream:
        header = stream.read(128).partition(b"\n")[0].rstrip(b"\r")
    require(
        header != b"version https://git-lfs.github.com/spec/v1",
        f"Platform archive is a Git LFS pointer; the runtime has not been downloaded:\n"
        f"  {archive}\n{recovery}",
    )


def manifest_contract(manifest, bundle):
    spec = bundle_spec(bundle)
    require(
        manifest.get("schema_version") == 2 and manifest.get("name") == "sharpa_platform",
        "Expected Platform manifest v2; regenerate the Platform bundle",
    )
    require(
        manifest.get("archive") == ARCHIVE and manifest.get("archive_format") == "tar.gz",
        "Unsupported Platform archive format",
    )
    sha256 = manifest.get("archive_sha256")
    size = manifest.get("archive_size")
    require(
        isinstance(sha256, str) and re.fullmatch(r"[0-9a-f]{64}", sha256),
        "Invalid Platform archive checksum",
    )
    require(isinstance(size, int) and size > 0, "Invalid Platform archive size")

    release = manifest.get("release")
    require(isinstance(release, dict), "Platform manifest is missing release identity")
    require(
        release.get("target_platform") == spec["target_platform"],
        f"Platform archive in {Path(bundle).name} must target {spec['target_platform']}",
    )

    entrypoint = safe_relative(manifest.get("entrypoint"))
    raw_profiles = manifest.get("profiles")
    require(isinstance(raw_profiles, dict) and raw_profiles, "Platform manifest has no launch profiles")
    profiles, profile_files = {}, {}
    for name, profile in raw_profiles.items():
        require(
            isinstance(name, str) and re.fullmatch(r"[A-Za-z0-9_-]{1,128}", name),
            f"Invalid Platform profile name: {name!r}",
        )
        require(isinstance(profile, dict), f"Invalid Platform profile: {name}")
        path, files = profile.get("config"), profile.get("files")
        require(isinstance(files, list) and files, f"Platform profile has no files: {name}")
        profiles[name] = safe_relative(path).as_posix()
        profile_files[name] = [safe_relative(item).as_posix() for item in files]
        require(len(set(profile_files[name])) == len(files), f"Duplicate Platform profile files: {name}")
    default_profile = manifest.get("default_profile")
    require(default_profile in profiles, "Platform default profile is not declared")
    return release, entrypoint.as_posix(), default_profile, profiles, profile_files, sha256, size


def package_files(bundle):
    manifest = bundle / MANIFEST
    archive = bundle / ARCHIVE
    return manifest, archive


@contextlib.contextmanager
def directory_lock(parent, name):
    parent.mkdir(parents=True, exist_ok=True)
    with (parent / name).open("a") as lock:
        fcntl.flock(lock, fcntl.LOCK_EX)
        yield


def publish(staged, destination):
    backup = staged.parent / "previous"
    existed = destination.exists()
    if existed:
        destination.rename(backup)
    try:
        staged.rename(destination)
    except BaseException:
        if existed:
            backup.rename(destination)
        raise


def load_package(bundle):
    manifest_path, archive = package_files(bundle)
    check_archive_available(archive)
    require(
        manifest_path.is_file(),
        f"Platform manifest is missing: {manifest_path}",
    )
    manifest = read_json(manifest_path)
    _, _, _, _, _, sha256, size = manifest_contract(manifest, bundle)
    require(
        archive.stat().st_size == size and digest(archive) == sha256,
        "Platform SHA-256/size mismatch. Fetch the complete Git LFS bundle again.",
    )
    return manifest, manifest_path, archive


def check_package_available(bundle):
    manifest_path, archive = package_files(bundle)
    check_archive_available(archive)
    require(
        manifest_path.is_file(),
        f"Platform manifest is missing: {manifest_path}",
    )
    manifest = read_json(manifest_path)
    _, _, _, _, _, _, size = manifest_contract(manifest, bundle)
    require(
        archive.stat().st_size == size,
        "Platform archive size does not match its manifest. Fetch the complete Git LFS object.",
    )


def extract_archive(archive, destination):
    require(archive.name == ARCHIVE, f"Unsupported Platform archive: {archive}")
    seen = set()
    with tarfile.open(archive, mode="r|gz") as tar:
        for member in tar:
            relative = safe_relative(member.name)
            path = destination / relative
            require(member.name not in seen, f"Duplicate archive member: {member.name}")
            seen.add(member.name)
            require(path.resolve().is_relative_to(destination), f"Archive path escapes runtime: {member.name}")
            require(
                member.isdir() or member.isfile() or member.issym(),
                f"Unsupported archive entry: {member.name}",
            )
            path.parent.mkdir(parents=True, exist_ok=True)
            if member.isdir():
                path.mkdir(exist_ok=True)
            elif member.issym():
                require(
                    not Path(member.linkname).is_absolute()
                    and (path.parent / member.linkname).resolve().is_relative_to(destination),
                    f"Archive symlink escapes runtime: {member.name}",
                )
                path.symlink_to(member.linkname)
            else:
                with tar.extractfile(member) as source, path.open("xb") as output:
                    shutil.copyfileobj(source, output)
                path.chmod(0o755 if member.mode & 0o111 else 0o644)


def validate_runtime(root, bundle, manifest):
    spec = bundle_spec(bundle)
    release_path = root / "release.json"
    require(release_path.is_file(), "Platform runtime is missing release.json")
    release = read_json(release_path)
    require(
        release.get("target_platform") == spec["target_platform"],
        f"Expected {spec['target_platform']} runtime, found {release.get('target_platform')}",
    )

    identity, entrypoint, default_profile, profiles, profile_files, _, _ = manifest_contract(manifest, bundle)
    require(release == identity, "Platform manifest/release identity mismatch")

    executable = root / entrypoint
    require(
        executable.is_file() and os.access(executable, os.X_OK),
        f"Platform entrypoint is not executable: {executable}",
    )
    with executable.open("rb") as stream:
        header = stream.read(20)
    elf_machine = int.from_bytes(header[18:20], "little") if len(header) >= 20 else -1
    require(
        header[:6] == b"\x7fELF\x02\x01" and elf_machine == spec["elf_machine"],
        f"Expected a {spec['description']} ELF executable: {executable}",
    )
    for path in profiles.values():
        logical = root / path
        require(logical.resolve().is_relative_to(root), f"Platform profile escapes runtime: {path}")
        require(logical.parent.is_dir(), f"Platform profile directory is missing: {path}")
        require(not logical.exists() or logical.is_file(), f"Invalid Platform profile path: {path}")
    for files in profile_files.values():
        for path in files:
            actual = root / path
            require(actual.resolve().is_relative_to(root), f"Platform profile escapes runtime: {path}")
            require(actual.is_file(), f"Platform profile is missing: {path}")
    return entrypoint, default_profile, profiles, profile_files


def configure_simulation_calibration(root, profile_files):
    """Use MuJoCo's zero offsets in the bundled D01-W HA1.2 profile only."""
    for relative in {path for files in profile_files.values() for path in files}:
        if relative != "config/sim/sim_north.D01W.HA12.json":
            continue
        path = root / relative
        config = read_json(path)
        plugins = config.get("plugins", [])
        calibration = "config/sim/joint_zero_offsets.json"
        require((root / calibration).is_file(), "Simulation joint zero offsets file is missing")
        for plugin in plugins:
            if plugin.get("common", {}).get("name") in (
                "sharpa_arm_plugin", "sharpa_body_plugin"):
                plugin["joint_zero_offsets_file"] = f"./{calibration}"
        path.write_text(json.dumps(config, ensure_ascii=False, indent=4) + "\n")


def stage_runtime(bundle, destination):
    bundle = bundle.resolve()
    bundle_spec(bundle)
    destination = destination.absolute()
    require(
        destination.name == "sharpa_platform" and not destination.is_symlink(),
        "Staging destination must be a non-symlink sharpa_platform directory",
    )
    require(
        not bundle.is_relative_to(destination) and not destination.is_relative_to(bundle),
        "Bundle and staging paths must be separate",
    )

    with directory_lock(bundle.parent, ".import.lock"), directory_lock(destination.parent, ".stage.lock"):
        manifest, manifest_path, archive = load_package(bundle)
        stamp_input = manifest_path.read_bytes()
        overlay_files = sorted(path for path in RUNTIME_OVERLAY.rglob("*") if path.is_file())
        stamp_hash = hashlib.sha256(stamp_input + Path(__file__).read_bytes())
        for path in overlay_files:
            require(not path.is_symlink(), f"Runtime overlay must not contain symlinks: {path}")
            stamp_hash.update(path.relative_to(RUNTIME_OVERLAY).as_posix().encode())
            stamp_hash.update(b"\0")
            stamp_hash.update(path.read_bytes())
        stamp = stamp_hash.hexdigest()
        marker = destination / ".bundle.sha256"
        if marker.is_file() and marker.read_text().strip() == stamp:
            validate_runtime(destination, bundle, manifest)
            print(f"Platform staging is current: {destination}")
            return
        if destination.exists():
            require(marker.is_file(), f"Refusing to replace an unmanaged runtime directory: {destination}")

        with tempfile.TemporaryDirectory(prefix=".stage-", dir=destination.parent) as temporary:
            staged = Path(temporary) / "runtime"
            staged.mkdir()
            extract_archive(archive, staged)
            entrypoint, default_profile, profiles, profile_files = validate_runtime(staged, bundle, manifest)
            for path in overlay_files:
                target = staged / path.relative_to(RUNTIME_OVERLAY)
                require(target.resolve().is_relative_to(staged), f"Runtime overlay escapes staging: {target}")
                target.parent.mkdir(parents=True, exist_ok=True)
                shutil.copy2(path, target)
            configure_simulation_calibration(staged, profile_files)
            shutil.copy2(manifest_path, staged / MANIFEST)
            descriptor = [
                f"schema_version={manifest['schema_version']}",
                "name=sharpa_platform",
                f"executable={entrypoint}",
                "argument=-c",
                "environment.SHARPA_PLATFORM_ROOT={root}",
                f"default_profile={default_profile}",
            ]
            descriptor += [f"profile.{name}={path}" for name, path in profiles.items()]
            descriptor += [
                f"profile_file.{name}.{index}={path}"
                for name, files in profile_files.items()
                for index, path in enumerate(files)
            ]
            (staged / "companion.conf").write_text("\n".join(descriptor) + "\n")
            (staged / ".bundle.sha256").write_text(stamp + "\n")
            publish(staged, destination)
    print(f"Staged Platform: {destination}")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    commands = parser.add_subparsers(dest="command", required=True)
    check = commands.add_parser("check", help="Check that the package archive is available")
    check.add_argument("--bundle", type=Path, default=BUNDLE)
    stage = commands.add_parser("stage")
    stage.add_argument("--bundle", type=Path, default=BUNDLE)
    stage.add_argument("--destination", type=Path, required=True)
    args = parser.parse_args()
    if args.command == "check":
        check_package_available(args.bundle)
    else:
        stage_runtime(args.bundle, args.destination)


if __name__ == "__main__":
    try:
        main()
    except (ValueError, OSError, KeyError, tarfile.TarError) as error:
        sys.exit(f"Platform runtime error: {error}")
