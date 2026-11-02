# LiDAR simulation plugin

[English](README.md) | [简体中文](README.zh-CN.md)

The plugin samples the MuJoCo model from three configured sensor sites and
sends FTX140 or JT16-compatible UDP datagrams. FTX140 uses raw UDP; JT16 is
wrapped in the POC2 sensor uplink envelope.

The supplied calibration files are synchronized from `sharpa_platform` with
`scripts/sync_lidar_calibration.py`. JT16 is copied directly; FTX140 is sampled
from the even rows and columns of the platform 384x512 correction grid to match
the simulator's 192x256 packet grid. Configuration in `sharpa_mujoco.conf`
owns all mount poses, network endpoints, and range limits.

JT16 scan directions use the same azimuth-to-XYZ convention as the
`sharpa_platform` JT16 decoder so simulated and decoded clouds are not mirrored.

## UDP point cloud visualization

`sharpa_mujoco_lidar_udp_visualization_test` is a separate UDP receiver that
runs alongside the simulator. Its four panels show the front chest FTX140 cloud
(red, top left), rear chest FTX140 cloud (green, top right), chassis JT16 cloud
(blue, bottom left), and a top-down reference view of the test scene's static
obstacles (bottom right). The interface displays received frame rates and valid
point counts. Press `Esc` or close the window to exit.

Run all commands below from the repository root. They use the default
`build-linux` build directory.

### 1. Build and configure

A Linux graphical desktop, GLFW, and OpenGL are required. Follow the
[project README](../../README.md) for initial setup, then build with:

```bash
./scripts/build.sh
```

This executable requires `SHARPA_MUJOCO_BUILD_TESTS=ON` and
`SHARPA_MUJOCO_BUILD_VIEWER=ON`; do not pass `--no-tests` or `--no-viewer` to the
build script. It is interactive and is not launched by CTest.

Set `enabled=true` in all three existing sections of the
[main configuration](../../sharpa_mujoco.conf), retaining their other driver,
mount, calibration, and network settings:

```ini
[devices.chest_lidar_front]
enabled=true

[devices.chest_lidar_rear]
enabled=true

[devices.chassis_lidar_front]
enabled=true
```

Both the simulator and receiver read this configuration. The receiver accepts
only `--config` and optional `--model`; it does not support `--enable-device`.
All three LiDAR devices must be enabled in the configuration file.

### 2. Start the simulator and visualization

Ensure the Zenoh router is running. If it is not, start it in one terminal:

```bash
zenohd -c resources/zenoh/router.json
```

In another terminal, start the simulator with the LiDAR
test scene, which includes walls, steps, and cylinders:

```bash
./scripts/run.sh --config ./sharpa_mujoco.conf \
  --model ./tests/lidar/lidar_test_scene.xml
```

Keep the simulator running and open the live point cloud interface in another
terminal:

```bash
./build-linux/tests/lidar/sharpa_mujoco_lidar_udp_visualization_test \
  --config ./sharpa_mujoco.conf \
  --model ./tests/lidar/lidar_test_scene.xml
```

Use the same configuration and scene for both programs. To use the scene from
the configuration file, omit `--model` from both commands. The bottom-right
reference view shows static obstacles only; it does not track the robot's live
pose.

### 3. Integration and troubleshooting

- The receiver binds each LiDAR's `peer_ip:peer_port`. The current defaults are
  `127.0.0.1:9180`, `127.0.0.1:9181`, and `127.0.0.1:9280`. Run only one point
  cloud receiver on these ports at a time to avoid bind conflicts or packets
  being consumed by another receiver.
- If clouds are empty or frame rates remain zero, check that the simulator is
  running, all three LiDAR devices are enabled, and both programs use matching
  configuration, calibration files, and UDP endpoints.
- To also run `examples/motion/chassis_motion_driver.py`, retain the complete
  `[devices.chassis]` section in the main configuration with `enabled=true`, and
  keep Platform and the Zenoh router running. `tests/data/lidar_enabled.conf`
  is a LiDAR test fixture without chassis driver settings; `--config` does not
  merge it with the main configuration. Using that fixture for the simulator
  leaves chassis motion without motor feedback and results in
  `No state/chassis feedback`.

## Integration TODOs

- [ ] Replace the provisional FTX140 optical-center extrinsics and the JT16 mock mount with verified CAD or calibration values.
- [ ] Confirm the official FTX140 instrumented range; use the configurable 0.1–60.0 m default until then.
- [ ] Validate the FTX140 packet layout against a real capture if the target hardware uses custom firmware; otherwise retain the F04 standard layout.
