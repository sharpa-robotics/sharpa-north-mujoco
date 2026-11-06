# Control the simulation with North SDK

[English](sdk.md) | [简体中文](sdk.zh-CN.md) · [Back to README](../README.md)

Use the official [North SDK](https://github.com/sharpa-robotics/sharpa-north-sdk)
C++ / Python APIs to read simulation state, run whole-body motion, and execute Home.
Run all commands below from this simulation repository's root.

## Scope and versions

SDK and simulation run on the same computer, using the bundled `sim_north` profile
and `D01W / HA12` device configuration. The prebuilt SDK targets Linux x86_64
(built on Ubuntu 22.04) and includes CPython 3.10 and 3.12 bindings.

| Component | Version |
| --- | --- |
| MuJoCo | 3.8.1 |
| Bundled Platform, Linux x86_64 | 1.7.10, `868484dbc138` |
| North SDK | 1.2.0, submodule commit `33306ab4fda789b8c3fa8a48662fc4c2a3a73180` |
| Python bindings | CPython 3.10 / 3.12, Linux x86_64 |
| Zenoh router | 1.10.0 |

See the [bundle manifest](../third_party/sharpa_platform/linux-x86_64/sharpa-platform.manifest.json)
for the Platform version.

## 1. Get the SDK submodule

```bash
./scripts/update_submodules.sh
```

By default, the script initializes both robot assets and SDK at the commits recorded
by the parent repository; `--pinned` has the same behavior.
Use `./scripts/update_submodules.sh --only sdk` to fetch just the SDK.
The SDK in `third_party/north_sdk` is ready to use without extraction.
The script checks its layout, version manifests, and SHA-256 checksums, and refuses
to overwrite local changes in selected submodules.

For an explicit upgrade, use `./scripts/update_submodules.sh --remote`, optionally with
`--only sdk`. Revalidate the examples and commit the updated submodule pointer in
the parent repository; the table above describes the currently pinned version.
Private repositories require GitHub access; HTTPS or SSH authentication follows
your local Git configuration.

## 2. Load the SDK and check installation

Choose an installed Python matching a bundled binding (`python3.12` also works):

```bash
export SDK_PYTHON=python3.10
source third_party/north_sdk/setup.bash
PYTHON="$SDK_PYTHON" ./third_party/north_sdk/check.sh
```

`check.sh` checks local library loading and Python imports without connecting to a
robot. Source `setup.bash` in each new example terminal to set `NORTHSDK_PREFIX`,
`PYTHONPATH`, and runtime library paths.
For C++ libraries only, run `./third_party/north_sdk/check.sh --cpp-only`.

## 3. Start the simulation and read state

Follow the [README quick start](../README.md#quick-start). Keep the router and
simulation running; Platform starts automatically with the simulator.
The examples use this local endpoint and disable host time synchronization:

```text
udp/127.0.0.1:7449?rel=1;multistream=1;mixed_rel=0
```

Build this repository's examples with a C++17 compiler and CMake 3.16+:

```bash
cmake -S examples/sdk -B build-sdk -DCMAKE_PREFIX_PATH="$NORTHSDK_PREFIX"
cmake --build build-sdk -j2
./build-sdk/sdk_read_state
```

The read-only example prints Master phase, mode, task ID, and left-arm joint
positions in radians. Source: [read_state.cpp](../examples/sdk/read_state.cpp).

## 4. Run Python whole-body motion

Finish other controllers' motion tasks and confirm Master is in `STANDBY`.
If the ordinary Python motion example is running, press Ctrl+C and wait for it to
return and exit. Run the motion directly:

```bash
"$SDK_PYTHON" examples/sdk/whole_body_motion.py
```

The sequence matches the ordinary Python example: squat while raising the arms,
then form a thumbs-up and hold until Ctrl+C. The first Ctrl+C smoothly returns to
the captured starting pose, verifies arrival, and stops the inference task started
by this example. A timed hold also returns before stopping. Another Ctrl+C aborts
the return and cleans up the task. Connection, stale-feedback, or task-state errors
stop commands without attempting a return.

| Option | Default and meaning |
| --- | --- |
| `--rate` | 50 Hz; legacy alias `--hz` remains available |
| `--transition-duration` | At least 4 seconds per transition; legacy alias `--move-duration` remains available; extended when required by the per-frame step limit |
| `--hold-duration` | 0, hold until Ctrl+C; a positive value holds for that many seconds |
| `--side` | `both`; also accepts `left` / `right` |
| `--arms-only` | Control only the selected arms |
| `--return-timeout` | Wait up to 8 seconds for feedback arrival after the return trajectory |
| `--tolerance` | Return tolerance, 1 degree |
| `--output` | Optional JSON report of stages, interruption, return, task cleanup, and verification |

For a finite run with a report:

```bash
"$SDK_PYTHON" examples/sdk/whole_body_motion.py --hold-duration 2 --output /tmp/sdk-motion.json
```

SDK joint positions use radians. Running without arguments starts the task and
moves, matching the ordinary Python example. Add `--preview` explicitly to read
state without motion. Source: [whole_body_motion.py](../examples/sdk/whole_body_motion.py).

## 5. Optional: invoke Home from C++

```bash
./build-sdk/sdk_home
```

By default, this executes the task, moves both arms and torso to Platform's
configured Home pose, and leaves them there. For a read-only check, use
`./build-sdk/sdk_home --preview`. Check feedback and the Viewer for arrival; wait for Master to return
to `STANDBY` before another operation. Source: [home.cpp](../examples/sdk/home.cpp).

## Troubleshooting

| Symptom | Action |
| --- | --- |
| Missing SDK or checksum failure | Check local submodule changes, run `./scripts/update_submodules.sh --only sdk`, and confirm GitHub access. |
| CMake cannot find NorthSDK | Source `setup.bash`; check that `NORTHSDK_PREFIX` points to `third_party/north_sdk`. |
| Cannot import `north_sdk` | Use Python 3.10 or 3.12 and source `setup.bash` in this terminal. |
| A `.so` reports a missing library | Source `setup.bash` and run `check.sh` to identify missing dependencies. |
| Connection fails or no state arrives | Check the router, Platform, port 7449, and reliable UDP parameters. |
| Motion rejected or task error | Inspect Master state and Platform logs; finish existing tasks in their controller. |

See the [SDK English manual](../third_party/north_sdk/manual/en/SDK/Application%20Development.md)
for the full API.
