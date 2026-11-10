# Sharpa Platform prebuilt runtime

`sharpa_platform` is a separately licensed, closed-source robot runtime. This
open-source repository contains only its release artifacts, never its source
tree or source-build paths.

Each supported target directory has exactly two managed inputs:

```text
linux-x86_64/   # GEN_X86
  sharpa-platform.tar.gz
  sharpa-platform.manifest.json
linux-aarch64/  # GEN_THOR
  sharpa-platform.tar.gz
  sharpa-platform.manifest.json
```

Both archives are stored with Git LFS. The manifest declares the archive
checksum and size, target architecture, executable entrypoint, release identity
and supported launch profiles. Expanded runtime files are generated only in a
build directory and must not be committed here.

Manifest v2 records each profile's logical `config` argument and actual `files`.
Staging checks those files; startup passes the logical name to Loader, which
selects the device variant using `device.conf`. The base file need not exist.
Regenerate bundles with the current packaging tools; v1 manifests are unsupported.

## Build and package

Fetch the release artifacts, then use the public entrypoints:

```sh
git lfs install
git lfs pull
./scripts/build.sh
./scripts/package_runtime.sh
```

The standard build and runtime package require the bundle matching the target
architecture. Use `--without-platform` only when intentionally building or
packaging MuJoCo by itself.

```sh
./scripts/build.sh --without-platform
./scripts/package_runtime.sh --without-platform
```

CMake rejects missing Git LFS content before compilation. Staging also verifies
the manifest, SHA-256, size, architecture, entrypoint, release identity and
profile paths. Archive extraction rejects unsafe paths. The validated runtime
is expanded to:

```text
<build>/sharpa_mujoco/companions/sharpa_platform
```

An installation or self-contained runtime package carries it under:

```text
lib/sharpa_mujoco/companions/sharpa_platform
```

Runtime startup does not require Python, compression tools, Git LFS or Platform
source code.

## Run

When a validated Platform runtime is bundled, the simulator launches it with
the manifest's default profile. Select another bundled entry with
`--platform-config <id>`. Disable it explicitly when an external Platform
process is already running or only MuJoCo is required.

```sh
./scripts/run.sh
./scripts/run.sh --without-platform
```

Platform is supervised as a separate process. Startup failure rolls back MuJoCo
startup; later exit is reported; shutdown signals and reaps only the owned
process group. A per-user lock rejects duplicate owners. Processes started
outside sharpa-north-mujoco are neither detected nor terminated.

The C ABI exposes
`sharpa_mujoco_companion_create/start/status/stop/destroy` for embedding hosts.
Process startup does not imply device health readiness. Zenoh, virtual CAN and
physical input prerequisites remain the caller's responsibility.

The bundled Platform runtime is not covered by this repository's Apache-2.0
license. Distributors must obtain the corresponding binary redistribution
rights.
