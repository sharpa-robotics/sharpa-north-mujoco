# sharpa-north-mujoco

[English](README.md) | [简体中文](README.zh-CN.md)

sharpa-north-mujoco simulates the **Sharpa D01** torso, dual arms, and dual dexterous
hands in MuJoCo, with hardware-compatible UDP and CAN/CAN-FD device interfaces.

Simulation in mujoco, Simulation with unreal5 and Sharpa North App demo:

https://github.com/user-attachments/assets/a71ac2f2-62c3-483b-a34a-ec6c08877c1b

## Key features

- **Hardware-compatible interfaces**: D01 UDP and CAN/CAN-FD communication,
  device state, and joint commands.
- **Whole-body simulation**: torso, dual arms, and dual dexterous hands in one
  MuJoCo model.
- **Two control options**: programmatic control with North SDK or graphical
  control with Sharpa North App.

## System architecture

```text
C++ / Python application     Sharpa North App
          |                        |
      North SDK                    |
          +-----------+------------+
                    Zenoh
                      |
               Sharpa Platform
                      |
               UDP / CAN / CAN-FD
                      |
            D01 simulated device plugins
                      |
             MuJoCo runtime and model
```

SDK and Sharpa North App communicate with Platform over Zenoh. Platform controls
the simulation through device protocols. Device plugins map protocol commands
to the MuJoCo model and return state feedback.

## Quick start

Supports Linux x86_64 and aarch64. Clone the
[sharpa-north-mujoco](https://github.com/sharpa-robotics/sharpa-north-mujoco)
repository, then run the commands below from its root:

```bash
git clone https://github.com/sharpa-robotics/sharpa-north-mujoco.git
cd sharpa-north-mujoco
```

### 1. Prepare the environment

Install these base tools:

- Linux x86_64 or aarch64, a C++17 compiler, CMake 3.22+, Git, and Python 3 with `venv`.
- A graphical desktop for the Viewer.
- A graphics driver providing desktop OpenGL 1.5+ compatibility functions,
  `GL_ARB_framebuffer_object`, and `GL_ARB_vertex_buffer_object`. See [graphics requirements and validated environments](CONFIGURATION.md#graphics-requirements).
- The complete [MuJoCo 3.8.1 Linux release](https://github.com/google-deepmind/mujoco/releases/tag/3.8.1),
  extracted with `include/mujoco`, `lib`, and `simulate` present.

On Ubuntu/Debian, install the base tools and project dependencies:

```bash
sudo apt-get update
sudo apt-get install build-essential cmake curl python3 python3-venv
./scripts/install_dependencies.sh
```

The dependency script installs GLFW and Git LFS, and downloads the Platform runtime.

Robot model assets are available in the official
[sharpa-urdf-usd-xml](https://github.com/sharpa-robotics/sharpa-urdf-usd-xml) repository.
This project currently uses the D01-W HA1.2 precise model.

After installing dependencies, fetch the pinned robot assets and North SDK before building:

```bash
./scripts/update_submodules.sh --pinned
```

Download the **standalone** archive for your host architecture from the
[Zenoh 1.10.0 release](https://github.com/eclipse-zenoh/zenoh/releases/tag/1.10.0):

- x86_64: `zenoh-1.10.0-x86_64-unknown-linux-gnu-standalone.zip`
- aarch64 / ARM64: `zenoh-1.10.0-aarch64-unknown-linux-gnu-standalone.zip`

Extract and install `zenohd`, replacing `/path/to/zenohd` with its actual path:

```bash
install -Dm755 /path/to/zenohd "$HOME/.local/bin/zenohd"
export PATH="$HOME/.local/bin:$PATH"
zenohd --version
```

Do not use the Snap package; its sandbox may block access to the project config file.

### 2. Configure MuJoCo and the simulated device

Set the extracted MuJoCo path in `sharpa_mujoco.conf`:

```ini
mujoco_root=/absolute/path/to/mujoco-3.8.1
```

See the [configuration reference](CONFIGURATION.md) for all parameters.

Create `/userapp/sharpa/device.conf` before the first run:

```bash
sudo mkdir -p /userapp/sharpa
sudo tee /userapp/sharpa/device.conf > /dev/null <<'EOF'
[device]
sn = SIM001
model = D01W
batch = HA12
EOF
```

### 3. Build

```bash
./scripts/build.sh
```

The default build directory is `build-linux`. Repeating the command reuses it and updates build outputs incrementally.

### 4. Start the router and simulation

In terminal A, start the router on UDP port 7449:

```bash
export PATH="$HOME/.local/bin:$PATH"
zenohd -c resources/zenoh/router.json
```

Keep terminal A running. In terminal B, start the simulator:

```bash
./scripts/run.sh
```

The launcher checks the environment, offers virtual CAN setup when needed,
and starts Platform and the Viewer automatically.
If graphics are unavailable, startup reports the error and exits.

### 5. Optional: verify the startup

Keep the router and simulator running. In terminal C, install the example
dependencies from the repository root:

```bash
python3 -m venv .venv
source .venv/bin/activate
python -m pip install -r requirements.txt
```

Run the whole-body motion example:

```bash
python examples/whole_body_motion.py
```

The robot squats, raises its arms, and forms a thumbs-up. It holds the pose
until Ctrl+C, then returns to its starting pose and exits.

The simulation environment is now ready. Choose North SDK or Sharpa North App below
to control the simulation.

## Control the simulation

### North SDK

Use the official **C++ or Python API** to read state and control motion for
algorithm development and application integration. This repository includes
state-reading, whole-body motion, and Home examples.

Official project: [sharpa-north-sdk](https://github.com/sharpa-robotics/sharpa-north-sdk).

→ [Connect to the simulation with SDK](docs/sdk.md)

### Sharpa North App

Use the desktop graphical interface to connect to the simulation, view device
and joint state, and execute Home.
Download and install the `production` build to get started.

Official project: [sharpa-north-app](https://github.com/sharpa-robotics/sharpa-north-app).

→ [Connect to the simulation with Sharpa North App](docs/sharpanorth-app.md)

Use one source of motion commands at a time. Finish the current task before
switching controllers or closing the Viewer. Closing the Viewer stops Platform;
press Ctrl+C in the router terminal to stop the router.

## Simulation setup and startup troubleshooting

| Symptom | Action |
| --- | --- |
| Missing Platform bundle | Check Git LFS access and rerun `./scripts/install_dependencies.sh`. |
| Missing robot assets, `Git LFS content not downloaded`, or `Skipping object checkout, Git LFS is not installed` | Run `./scripts/update_submodules.sh --pinned`. Git LFS must be configured in the asset submodule as well as installed on the system; the script handles this automatically. |
| Startup reports `sharpa_mujoco.conf:line: expected key=value` | Check the reported line: settings use `key=value` and comments start with `#`. Remove accidental characters, for example change `1#` to `#`. Correct the file and run again; no rebuild is needed. |
| MuJoCo not found | Set `mujoco_root` to the extracted complete release; if `MUJOCO_ROOT` is set in the environment, check that path. Then rerun `./scripts/build.sh`, which automatically refreshes cached MuJoCo paths. |
| Startup reports `Simulator not found or not executable` | Run `./scripts/build.sh` and confirm it succeeds before running `./scripts/run.sh`. Both use the same build directory by default. |
| Window creation fails or reports a GLX error | Check your desktop session and graphics driver. NVIDIA users can run `nvidia-smi`; if it reports `Driver/library version mismatch`, save your work and reboot. Devices and Platform are not started when graphics checks fail. |
| `wave_tactile` cannot bind port 59998 | Check that no other simulator instance is running. Before devices bind ports, startup reclaims a Platform process left by a previous crashed simulator. |
| Missing virtual CAN interfaces | Run `sudo bash scripts/setup_vcan.sh vcan0 vcan1`. |
| Platform cannot select its device profile | Check that `device.conf` uses model `D01W` and batch `HA12`. |
| Port 7449 is in use | Check the existing router; use this repository's `resources/zenoh/router.json`. |
| Viewer opens but the controller has no state | Check Platform startup output and the router configuration. |

## UE visualization preview

An **experimental preview** with Unreal Engine (UE) rendering is also available, powered by `sharpa-north-mujoco`. You can also integrate your own renderer with this repository.

[Download the Linux preview (AvatarSimWorld)](https://packages.sharpa.com/repository/raw-releases/apps/sim/AvatarSimWorld-Linux.tar.zst)

Run the following commands in the extracted directory:

```bash
# Initialize virtual CAN interfaces
sudo bash AvatarSimWorld/Binaries/Linux/sharpa_mujoco/scripts/setup_vcan.sh

# Start the application
bash AvatarSimWorld.sh
```

## More documentation

- [Simulator configuration](CONFIGURATION.md)
- [Platform runtime](third_party/sharpa_platform/README.md)

## License

[Apache-2.0](LICENSE). For the bundled Platform runtime's license, see its
[distribution notes](third_party/sharpa_platform/README.md).
