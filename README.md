# Sharpa MuJoCo

[English](README.md) | [简体中文](README.zh-CN.md)

Sharpa MuJoCo is a high-fidelity simulation environment for the **Sharpa D01**
robot. Built on MuJoCo, it reproduces the D01 hardware interfaces, device
lifecycle, joint mappings, and motion-control behavior used by the physical
robot.

Instead of introducing a simulator-specific control API, the project uses the
same production UDP and CAN/CAN-FD paths as the physical devices. Applications
can therefore share one control stack across simulation and real hardware,
moving from development to the physical robot with only endpoint or
configuration changes.

**[Sharpa Platform](third_party/sharpa_platform/README.md)** is the robot runtime
shared by simulation and the physical D01, allowing applications to reuse the
same control interfaces and workflows. This project bundles Platform as prebuilt
binaries and starts it automatically with the simulator.

## Highlights

- **Hardware-compatible communication** — emulates production UDP protocols
  and Linux SocketCAN/CAN-FD interfaces rather than a simulator-only API.
- **Control-stack reuse** — works with the same device lifecycle, state
  feedback, and command paths used on the D01.
- **Broad motion-control coverage** — supports the primary control modes used
  by the torso, arms, and Wave hands, including CSP, MIT, zero-force, position,
  and admittance control where applicable.
- **Consistent kinematics and dynamics** — preserves joint directions, coupled
  transmissions, limits, feedback units, gravity, contact, and calibrated
  actuator behavior to reduce the sim-to-real gap.
- **Whole-body simulation** — integrates the torso, dual arms, dual dexterous
  hands, and fingertip force/torque sensing in one shared MuJoCo runtime.
- **Deployment flexibility** — runs as a standalone simulator with Viewer or
  as an embeddable C runtime for applications that own their render loop.

## System architecture

```text
Application / Algorithm / Teleoperation
                  |
       Production UDP or CAN/CAN-FD
                  |
      D01 simulated device plugins
                  |
        Shared MuJoCo runtime/model
```

The protocol boundary is aligned with the real robot. Switching targets
typically requires only an endpoint or configuration change; upper-level
applications and motion logic remain unchanged.

## Prerequisites

Install these base tools:

- Linux with a C++17 compiler, CMake 3.22+, Git, and Python 3 with `venv`
- [MuJoCo 3.8.1](https://github.com/google-deepmind/mujoco/releases/tag/3.8.1)

Download and extract the complete MuJoCo Linux release. The archive must
contain `include/mujoco`, `lib`, and `simulate`.

## Quick start

### 1. Install dependencies

From the `sharpa_mujoco` project root:

```bash
./scripts/install_dependencies.sh
```

Download the **standalone** binary archive for your host architecture from the
[official Zenoh 1.10.0 release](https://github.com/eclipse-zenoh/zenoh/releases/tag/1.10.0).

- x86_64: `zenoh-1.10.0-x86_64-unknown-linux-gnu-standalone.zip`
- aarch64 / ARM64: `zenoh-1.10.0-aarch64-unknown-linux-gnu-standalone.zip`

After extracting the archive, run the following commands to install `zenohd`
into `~/.local/bin` and add it to the current terminal's PATH. Replace
`/path/to/zenohd` with the actual extracted executable path:

```bash
install -Dm755 /path/to/zenohd "$HOME/.local/bin/zenohd"
export PATH="$HOME/.local/bin:$PATH"
```

Do not use the Snap package; its sandbox may block access to the project config file.

### 2. Configure MuJoCo

Set the extracted MuJoCo path in `sharpa_mujoco.conf`:

```ini
mujoco_root=/absolute/path/to/mujoco-3.8.1
```

For all `sharpa_mujoco.conf` parameters, see the
[configuration reference](CONFIGURATION.md).

### 3. Create the device configuration

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

### 4. Build

```bash
./scripts/build.sh
```

### 5. Run

Start the router on UDP port 7449:

```bash
zenohd -c resources/zenoh/router.json
```

Then start the simulator:

```bash
./scripts/run.sh
```

The launcher starts the required components. Closing the Viewer stops the
simulation environment.

### 6. Optional: verify the startup

With the simulator running, install the motion-example dependencies in another
terminal:

```bash
python3 -m venv .venv
source .venv/bin/activate
python -m pip install -r requirements.txt
```

Run the whole-body motion example:

```bash
python examples/whole_body_motion.py
```

## License

Apache-2.0. See [LICENSE](LICENSE).
