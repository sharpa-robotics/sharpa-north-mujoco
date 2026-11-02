# Sharpa MuJoCo Configuration Reference

[English](CONFIGURATION.md) | [简体中文](CONFIGURATION.zh-CN.md)

This document describes every parameter in [sharpa_mujoco.conf](sharpa_mujoco.conf).
For installation, building, and first startup, see the [README](README.md).

## Configuration conventions

- Configuration uses `key=value` entries and `[section]` headers. Text after `#` is a comment.
- “Current value” means the value explicitly set in the repository configuration file,
  not the code default used when a parameter is omitted. The loaded configuration
  determines runtime behavior; restart after making changes.
- Use `true` / `false` or `1` / `0` for booleans. Separate array entries with commas.
- `simulation.model`, device `sensor_model`, and `calibration_file` accept absolute paths.
  Relative paths are resolved from the configuration file's directory. Use an absolute
  path for `mujoco_root`.
- For network settings, `bind` is the simulated device's local listening endpoint and
  `peer` is the remote endpoint. Ports normally range from `1–65535`; the arm setting
  `sdo_port=0` is a special case that disables SDO. Keep endpoint and driver changes
  consistent with the Platform / SDK configuration.
- Units are Hz for frequency, seconds for time, rad for angles, rad/s for angular
  velocity, and N·m for torque, unless the parameter name specifies another unit.

To start with a separate configuration file:

```bash
./scripts/run.sh --config /absolute/path/to/sharpa_mujoco.conf
```

This option selects the runtime configuration. The MuJoCo SDK is selected at build time.

## Root parameters

| Parameter | Description |
| --- | --- |
| `mujoco_root` | Extracted MuJoCo 3.8.1 SDK directory, containing `include/mujoco`, `lib`, and `simulate`. Set the actual local path, such as `/absolute/path/to/mujoco-3.8.1`. Build and packaging tools read this setting; the simulator does not use it to select a scene. |

An existing CMake `MUJOCO_ROOT` cache entry, or the environment variable of the same
name at initial configuration, takes precedence over the file value. If a previous
SDK path persists after switching SDKs, use a new build directory or reconfigure
with the cleanup option described by `scripts/build.sh --help`.

## Simulation and Viewer

### `[simulation]`

| Parameter | Current value | Description |
| --- | --- | --- |
| `model` | `mjcf/NTH-HA1-MJCF/NTH-HA1-scene.xml` | Main scene MJCF file. |
| `snapshot_rate_hz` | `500` | Runtime state snapshot frequency in simulation time. Must be positive; the model still determines the physics timestep. |

### `[viewer]`

| Parameter | Current value | Description |
| --- | --- | --- |
| `enabled` | `true` | Whether to start the Viewer. Disable for headless operation. |
| `sync_rate_hz` | `60` | Viewer synchronization frequency. Must be positive. |
| `vsync` | `false` | Whether to enable graphics vertical synchronization. The current configuration disables VSync and uses software pacing at `sync_rate_hz` to avoid background GLX VSync stalls. |

## Device enablement and drivers

Every `[devices.*]` section has these two parameters:

| Parameter | Description |
| --- | --- |
| `enabled` | Whether to load the device plugin. Disabled devices do not start communication or feedback. |
| `driver` | Device protocol or transport implementation. Available values depend on the device type, as listed below. |

| Section | Current `enabled` | Current `driver` | Available drivers |
| --- | --- | --- | --- |
| `[devices.torso]` | `true` | `can` | `can`, `udp` |
| `[devices.arm_left]`, `[devices.arm_right]` | `true` | `udp_arm` | `udp_arm` |
| `[devices.hand_left]`, `[devices.hand_right]` | `true` | `wave_sdk_loopback` | `wave_sdk_loopback` |
| `[devices.tactile]` | `true` | `wave_tactile` | `wave_tactile` |
| `[devices.chassis]` | `true` | `udp_mcu_kinco` | `udp_mcu_kinco`, `udp_mcu_jiechang` |
| `[devices.chest_lidar_front]`, `[devices.chest_lidar_rear]` | `false` | `hesai_ftx140` | `hesai_ftx140`, `hesai_jt16`; must match the model and calibration file |
| `[devices.chassis_lidar_front]` | `false` | `hesai_jt16` | `hesai_ftx140`, `hesai_jt16`; must match the model and calibration file |

## Torso `[devices.torso]`

### Communication and feedback

`driver=can` uses Linux SocketCAN, currently through `vcan0` / `vcan1`.
`driver=udp` uses MCU UDP encapsulation. `lower_*` refers to the lower torso and
`upper_*` to the upper torso. Endpoint motor lists use physical CAN IDs.

| Parameter | Current value | Description |
| --- | --- | --- |
| `lower_can_interface` | `vcan0` | Lower torso SocketCAN interface name, used in CAN mode. |
| `upper_can_interface` | `vcan1` | Upper torso SocketCAN interface name, used in CAN mode. |
| `lower_bind_ip` | `127.0.0.1` | Lower torso local UDP listening IP. |
| `lower_bind_port` | `45520` | Lower torso local UDP listening port. |
| `lower_peer_ip` | `127.0.0.1` | Lower torso UDP peer IP. |
| `lower_peer_port` | `45120` | Lower torso UDP peer port. |
| `lower_can_if` | `0` | CAN channel number in lower torso MCU UDP packets, in `0–255`. This is not a Linux interface name. |
| `lower_motor_ids` | `1,2,3,4,5` | Physical motor CAN IDs handled by the lower torso endpoint. |
| `upper_bind_ip` | `127.0.0.1` | Upper torso local UDP listening IP. |
| `upper_bind_port` | `45521` | Upper torso local UDP listening port. |
| `upper_peer_ip` | `127.0.0.1` | Upper torso UDP peer IP. |
| `upper_peer_port` | `45121` | Upper torso UDP peer port. |
| `upper_can_if` | `0` | CAN channel number in upper torso MCU UDP packets, in `0–255`. |
| `upper_motor_ids` | `6,7,8` | Physical motor CAN IDs handled by the upper torso endpoint. |
| `feedback_rate_hz` | `200` | Motor state feedback frequency. Must be positive. |
| `heartbeat_rate_hz` | `10` | Heartbeat frequency. Must be positive. |

### Motor control and calibration

Arrays below have 8 entries in logical motor order J1–J8, corresponding to physical
CAN IDs `1,2,4,3,5,6,7,8`. J3 / J4 are the coupled waist motors; J6–J8 are the neck
motors. Do not reorder these arrays by ascending CAN ID.

| Parameter | Current value | Description |
| --- | --- | --- |
| `command_delay_s` | `0,0,0,0,0,0,0,0` | Motion command interface delay; see “Motor interface delays” below for constraints and behavior. |
| `feedback_delay_s` | `0.070,0.034,0.044,0.044,0.039,0.046,0.044,0.051` | Per-motor state feedback interface delays, calibrated after the mixed CSP/MIT startup-brake fix. Coupled waist motors J3/J4 use the same delay. |
| `motor_signs` | `-1,-1,1,1,-1,1,1,1` | Direction signs between motor protocol coordinates and model coordinates. Use `-1` or `1`. |
| `csp_kp` | `15910.01,9429.39,1882.00,1321.60,854.16,100,100,75` | CSP position proportional gains, in N·m/rad. |
| `csp_kd` | `1178.62,708.16,98.54,76.82,46.94,10,10,5` | CSP velocity error gains, in N·m·s/rad. |
| `command_velocity_filter_s` | `0.03,0.03,0.03,0.03,0.03,0.03,0.03,0.03` | Filter time constants for estimating velocity from position command arrival cadence. `0` disables the estimator for that axis. |
| `command_velocity_scale` | `1,0,1,1,0.6,1,1,1` | Feedforward multipliers for estimated command velocity. `0` disables estimated velocity feedforward for that axis. |
| `mit_kp_scale` | `6.36400,3.77176,3,3,1,1,1,1` | Simulation multipliers for position gains received in MIT packets. |
| `mit_kd_scale` | `11.7862,3,2.8521,1.5000,1,1,1,1` | Simulation multipliers for velocity gains received in MIT packets. |
| `mit_torque_ff_scale` | `1,1,1,1,1,1,1,1` | MIT feedforward torque multipliers. |
| `mit_torque_limit_nm` | `500,400,250,250,167,300,300,300` | MIT control torque limits. Must be positive; model actuator limits still apply. |
| `mit_torque_time_constant_s` | `0,0.03,0,0,0,0,0,0` | First-order time constants for MIT torque response. `0` gives an instantaneous response. |
| `mit_reference_velocity_limit_rad_s` | `0,0,0,0,0,0,0,0` | Velocity limits for MIT position reference resampling. |
| `mit_reference_acceleration_limit_rad_s2` | `0,0,0,0,0,0,0,0` | Acceleration limits for MIT position reference resampling, used together with the velocity limits. |
| `motor_reduction_ratios` | `161,161,161,161,161,101,101,101` | Gear ratios for converting load-side velocity to rotor-side velocity feedback. Must be positive. |
| `feedback_velocity_scale` | `1,1,1,1,1,1,1,1` | Correction multipliers applied only to velocity feedback. Must be positive; encoder geometry is unchanged. |
| `dynamic_waist_armature` | `true` | Whether to update inertia reflected onto the model's degrees of freedom according to the coupled waist pose. |
| `waist_motor_1_inertia` | `1.647253629` | Inertia of the first coupled waist motor used for reflected inertia calculations, in kg·m². Must be nonnegative. |
| `waist_motor_2_inertia` | `1.647253629` | Inertia of the second coupled waist motor used for reflected inertia calculations, in kg·m². Must be nonnegative. |

Gains, feedforward multipliers, and time constants must be finite and nonnegative.
For each axis, the MIT reference velocity and acceleration limits must both be zero
to disable resampling, or both be positive to enable it. Resampling smooths
asynchronous commands in model time.

The current calibration retains these settings:

- Based on unloaded recordings from 2026-10-19, the J1 feedback delay is constrained
  by motion correlation regression; J6 corrects the remaining state publication lead.
- Command velocity feedforward reduces phase lag on J1 / J3 / J4 / J5 and the neck
  axes. It is disabled on J2.
- Calibrate J3 / J4 gains and torque headroom together. J5 uses the model's full
  167 N·m actuator range during fast reversals.
- Protocol encoding already handles each motor's velocity units. No additional
  empirical velocity correction is currently applied.

## Arms `[devices.arm_left]` / `[devices.arm_right]`

Each arm has 7 motors. Motor arrays use indices `0–6`; `sim_joint_*` arrays use
joint indices `0–6`. Both arms currently share the same settings except for their
UDP endpoints.

### Control and feedback

| Parameter | Current value | Description |
| --- | --- | --- |
| `command_delay_s` | `0,0,0,0,0,0,0` | Motion command interface delays; see “Motor interface delays” below. |
| `feedback_delay_s` | `0,0,0,0,0,0,0` | State feedback interface delays; see “Motor interface delays” below. |
| `extended_feedback` | `true` | Use feedback packets with extended state and sample timestamps. `false` selects the basic feedback format. |
| `sdo_port` | `65530` | SDO service port on each board IP. `0` disables SDO and keeps only PDO. When enabled, each board requires a distinct listening IP. |
| `bus_voltage_v` | `48` | Reported synthetic bus voltage, in V. |
| `driver_temperature_c` | `25` | Reported synthetic driver temperature, in °C. |
| `motor_temperature_c` | `25` | Reported synthetic motor temperature, in °C. |
| `feedback_rate_hz` | `1000` | State feedback frequency. Must be positive. |
| `command_timeout_s` | `0.2` | Motion command lifetime, measured in wall-clock time from the original packet arrival. Must be positive; expiry triggers stopping. |
| `quick_stop_deceleration_rad_s2` | `10` | Target velocity deceleration during quick stop. Must be positive. |
| `quick_stop_velocity_threshold_rad_s` | `0.02` | Velocity threshold for completing quick stop. Must be positive. |
| `quick_stop_timeout_s` | `1` | Maximum quick-stop duration. Must be positive. |
| `quick_stop_kp` | `20` | Quick-stop velocity error gain, in N·m·s/rad. Must be positive. |
| `speed_kp` | `5` | Speed mode proportional gain, in N·m·s/rad. Must be nonnegative. |
| `speed_ki` | `20` | Speed mode integral gain, in N·m/rad. Must be nonnegative. |
| `speed_integral_limit_nm` | `20` | Torque limit for the speed loop integral term. Must be nonnegative. |
| `diagnostic_logging` | `true` | Whether to emit arm diagnostic logs. |
| `csp_control_space` | `joint` | CSP control space: `joint` selects joint-space control; `motor` selects motor-space control. |
| `sim_joint_kp` | `500,500,300,200,50,50,50` | Joint-space CSP position gains, in N·m/rad. |
| `sim_joint_kd` | `7.5,7.5,2,7.5,1,1,1` | Joint-space CSP velocity gains, in N·m·s/rad. |
| `csp_kp` | `100,100,100,40,40,30,30` | Motor-space CSP position gains, in N·m/rad. Also used to hold the physical pose during homing. |
| `csp_kd` | `10,10,10,5,5,5,5` | Motor-space CSP velocity gains, in N·m·s/rad. Also used to hold the physical pose during homing. |
| `torque_limits_nm` | `40,40,30,6.9,6.9,6.9,6.9` | Per-motor torque limits. Must be positive. |

Voltage and temperature are synthetic telemetry, not measured hardware values.
They must be finite and within `[-128,127]` to fit the protocol's signed 8-bit
fields. Control gains must be finite and nonnegative. With
`csp_control_space=joint`, normal CSP uses `sim_joint_kp` / `sim_joint_kd`;
switching to `motor` selects the legacy motor-space CSP gains.

### Board UDP endpoints

Each arm has five endpoint groups. Replace `N` below with `0`, `1`, `2`, `3`, or `4`,
as in `endpoint0_bind_ip` and `endpoint4_motor_indices`.

| Parameter | Description |
| --- | --- |
| `endpointN_bind_ip` | Local listening IP of board N. |
| `endpointN_bind_port` | Local PDO UDP port of board N; currently `21000` for every board. |
| `endpointN_peer_ip` | Peer IP of board N; currently `127.0.0.1` for every board. |
| `endpointN_peer_port` | Feedback peer port of board N; currently `8080` for all left-arm boards and `8081` for all right-arm boards. |
| `endpointN_motor_indices` | One or two motor indices handled by board N, separated by commas. Indices range from `0–6`. |

| N | Left arm `endpointN_bind_ip` | Right arm `endpointN_bind_ip` | Both arms `endpointN_motor_indices` |
| --- | --- | --- | --- |
| `0` | `127.0.40.101` | `127.0.40.106` | `0` |
| `1` | `127.0.40.102` | `127.0.40.107` | `1` |
| `2` | `127.0.40.103` | `127.0.40.108` | `2` |
| `3` | `127.0.40.104` | `127.0.40.109` | `3,4` |
| `4` | `127.0.40.105` | `127.0.40.110` | `5,6` |

The five groups must cover all 7 motors without duplicates. Motors on the same
board must use equal command delays and equal feedback delays.

## Motor interface delays

This section applies to `command_delay_s` / `feedback_delay_s` for the torso and
both arms. Both are fixed wall-clock delays in seconds. Omitted values default to
zero, which bypasses buffering. Every entry must be finite and within `[0,0.1]`.

- The torso takes 8 entries in J1–J8 order. Coupled waist motors J3 / J4 must have
  equal command delays and equal feedback delays.
- Each arm takes 7 entries in motor order `0–6`. Motors on the same board must
  have equal command delays and equal feedback delays. Each command delay must
  also be less than that arm's `command_timeout_s`.
- Command delay queues normal motion targets until a physics step reaches their
  delivery deadline. Initial enable, mode changes, brake, stop, and homing are
  handled immediately and discard queued motion targets. Arm timeouts still use
  original packet arrival times.
- Feedback delay postpones position, velocity, torque, and encoder feedback from
  the same sample together. Extended arm packets retain the original sample
  timestamp while reporting current drive status. After startup, reset, or
  feedback queue invalidation, affected axes wait one delay period before
  publishing feedback. SDO replies and heartbeats are not queued.
- Both paths use bounded timestamp queues. They do not sleep the physics thread,
  change MuJoCo time or H5 timestamps, or vary the delay with motion speed. Reset
  and plugin shutdown clear the queues. Overflow beyond 1024 entries discards
  the oldest entry.

Calibrate the command and feedback paths separately, using real state / sim state
errors on the original timeline and motion regression tests. Do not add the full
action→state lag already accounted for by the controller and dynamics to the
interface again.

## Hands `[devices.hand_left]` / `[devices.hand_right]`

| Parameter | Current left-hand value | Current right-hand value | Description |
| --- | --- | --- | --- |
| `device_ip` | `127.0.50.10` | `127.0.50.20` | Simulated hand's local listening IP. |
| `sdk_ip` | `127.0.0.1` | `127.0.0.1` | SDK host IP receiving feedback and heartbeats. |
| `serial_number` | `SIM-WAVE-L-0001` | `SIM-WAVE-R-0001` | Simulated device serial number. Must differ between hands. |
| `product_number` | `Wave01-MOCK` | `Wave01-MOCK` | Simulated device product model. |
| `tcp_port` | `59999` | `59999` | Device TCP configuration service port. |
| `command_udp_port` | `50020` | `50030` | UDP port receiving joint control commands. |
| `sdk_joint_port` | `50000` | `50010` | SDK UDP port receiving joint feedback. |
| `sdk_heartbeat_port` | `54321` | `54321` | SDK UDP port receiving device heartbeats. |
| `feedback_source_port` | `10000` | `10000` | Local UDP source port used by the device to send feedback. |
| `feedback_rate_hz` | `500` | `500` | Joint feedback frequency. Must be positive. |
| `heartbeat_rate_hz` | `1` | `1` | Heartbeat frequency. Must be positive. |
| `strict_command_state` | `false` | `false` | When `true`, a command packet's control source and mode must match the device's current state. When `false`, running-state, fault, and lifecycle checks still apply. |

## Fingertip tactile sensing `[devices.tactile]`

| Parameter | Current value | Description |
| --- | --- | --- |
| `sdk_ip` | `127.0.0.1` | SDK host IP receiving tactile data. |
| `right_device_ip` | `127.0.50.20` | Right-hand tactile device IP, corresponding to the right-hand configuration service. |
| `left_device_ip` | `127.0.50.10` | Left-hand tactile device IP, corresponding to the left-hand configuration service. |
| `config_port` | `59998` | Tactile TCP configuration port listening on both device IPs. |
| `right_sdk_port` | `50001` | SDK destination UDP port for right-hand tactile data. |
| `left_sdk_port` | `50011` | SDK destination UDP port for left-hand tactile data. |
| `rate_hz` | `30` | Tactile data publication frequency. Must be positive. |
| `sensor_model` | `mjcf/sensors/tactile/wave_fingertip_frame.xml` | Fingertip sensor MJCF model, attached to all ten fingertips across both hands when loaded. |

`right_sdk_port` / `left_sdk_port` follow the tactile plugin's naming convention.
Match them to the tactile SDK receivers when making changes; do not infer handedness
from the numerical order of the joint feedback ports.

## Chassis `[devices.chassis]`

### Drivers, endpoints, and general control

`driver` selects the steering and wheel motor protocol. `stretch_driver`
independently selects the extension motor protocol. The current combination uses
Kinco steering / wheel motors and Jiechang extension motors, matching Platform's
`config/sharpa_chassis_plugin_config.json`. Jiechang extension uses classic CAN,
with IDs `9–12`, AB on channel 0, and CD on channel 1. It shares the MCU UDP endpoint
selected for the steering / wheel motors.

| Parameter | Current value | Description |
| --- | --- | --- |
| `stretch_driver` | `jiechang` | Extension driver: `jiechang` or `m42p`. |
| `motor_signs` | `1,1,1,1,-1,-1,-1,-1` | 8 direction signs, ordered as steering axes 1–4 followed by wheel axes 1–4. Only `-1` / `1` are allowed; extension axes are excluded. |
| `jiechang_bind_ip` | `127.0.0.1` | Local UDP IP when `driver=udp_mcu_jiechang`. |
| `jiechang_bind_port` | `45620` | Local UDP port of the Jiechang steering / wheel MCU. |
| `jiechang_peer_ip` | `127.0.0.1` | Peer IP of the Jiechang steering / wheel MCU. |
| `jiechang_peer_port` | `45220` | Peer port of the Jiechang steering / wheel MCU. |
| `kinco_bind_ip` | `127.0.0.1` | Local UDP IP when `driver=udp_mcu_kinco`. |
| `kinco_bind_port` | `45620` | Local UDP port of the Kinco steering / wheel MCU. |
| `kinco_peer_ip` | `127.0.0.1` | Peer IP of the Kinco steering / wheel MCU. |
| `kinco_peer_port` | `45220` | Peer port of the Kinco steering / wheel MCU. |
| `m42p_bind_ip` | `127.0.0.1` | Local UDP IP of the extension MCU, used only with `stretch_driver=m42p`. |
| `m42p_bind_port` | `45621` | Local UDP port of the M42P extension MCU. |
| `m42p_peer_ip` | `127.0.0.1` | Peer IP of the M42P extension MCU. |
| `m42p_peer_port` | `45221` | Peer port of the M42P extension MCU. |
| `feedback_rate_hz` | `200` | Chassis feedback frequency. Must be positive; the current value corresponds to a 5 ms period. |
| `command_timeout_s` | `0.1` | Motion command timeout. Must be positive; currently matches Platform's 100 ms command timeout. |
| `jiechang_stretch_counts_per_meter` | `262144000` | Jiechang extension position counts per meter, based on 262144 counts per revolution, 1 mm screw lead, and a gear ratio of 1. |
| `slider_steps_per_meter` | `2133333.333333` | M42P extension steps per meter, used only with `stretch_driver=m42p`. |
| `slider_minimum_m` | `0` | Minimum extension position, in m. |
| `slider_maximum_m` | `0.06` | Maximum extension position, in m. |

Only the endpoint group for the selected main motor driver is activated, so the
two groups may retain the same ports. The current Jiechang extension configuration
does not start the M42P endpoint.

### Jiechang steering / wheel parameters

These settings apply to `driver=udp_mcu_jiechang`. The current configuration
retains them for switching drivers.

| Parameter | Current value | Description |
| --- | --- | --- |
| `steer_gear_ratio` | `47.25` | Steering motor gear ratio. |
| `wheel_gear_ratio` | `6.28` | Wheel motor gear ratio. |
| `steer_rated_current_a` | `1.2` | Steering motor rated current, in A. |
| `wheel_rated_current_a` | `3.2` | Wheel motor rated current, in A. |
| `steer_rated_motor_torque_nm` | `0.26` | Steering motor shaft rated torque, in N·m. Output torque conversion includes the gear ratio. |
| `wheel_rated_motor_torque_nm` | `0.50` | Wheel motor shaft rated torque, in N·m. |
| `steer_position_kp` | `120` | Steering position loop proportional gain, in N·m/rad. |
| `steer_position_kd` | `12` | Steering position loop velocity gain, in N·m·s/rad. |
| `wheel_velocity_kv` | `15` | Wheel output-shaft velocity loop proportional gain, in N·m·s/rad. |
| `wheel_velocity_ki` | `100` | Wheel output-shaft velocity loop integral gain, in N·m/rad. `0` disables integration. |
| `wheel_velocity_integral_limit_nm` | `3.14` | Torque limit for the wheel velocity integral term. |
| `jiechang_steer_torque_limit_nm` | `12.285` | Steering output torque limit. |
| `jiechang_wheel_torque_limit_nm` | `3.14` | Wheel output torque limit. |
| `jiechang_mit_kp_scale` | `1` | Jiechang MIT mode position gain multiplier. |
| `jiechang_mit_kd_scale` | `1` | Jiechang MIT mode velocity gain multiplier. |

### Kinco steering / wheel parameters

These settings apply to `driver=udp_mcu_kinco`. Current values match Platform's
Kinco protocol drivers.

Platform's `steer_calib_mode=protocol` homing sequence is supported, including
`0x6098=-17`, `0x607C`, `0x6099`, `0x609A`, mode `6` readback and homing status.
Simulation uses the MJCF steering joint zero as its known calibrated reference:
the joint moves to zero at the configured homing search speed and acceleration.
Homing control always runs at each physics step and reports completion only after
both position and velocity settle within tolerance.
The physical end-stop search and origin offset are not simulated; `0x607C` and
related parameters retain read/write values without shifting model coordinates.
SDO-started homing runs autonomously until stopped, disabled, faulted or switched
out of homing mode. Ordinary RPDO motion retains its command watchdog.

| Parameter | Current value | Description |
| --- | --- | --- |
| `kinco_encoder_resolution` | `65536` | Encoder counts per motor shaft revolution. |
| `kinco_steer_gear_ratio` | `9` | Steering motor gear ratio. |
| `kinco_wheel_gear_ratio` | `10` | Wheel motor gear ratio. |
| `kinco_steer_rated_current_a` | `6.5` | Steering motor rated current, in A. The current implementation retains this value but does not use it for control or feedback conversion. |
| `kinco_wheel_rated_current_a` | `8.5` | Wheel motor rated current, in A. The current implementation retains this value but does not use it for control or feedback conversion. |
| `kinco_steer_max_current_arms` | `14.3` | Steering current command / feedback conversion reference, using RMS current according to the driver protocol. |
| `kinco_wheel_max_current_arms` | `25` | Wheel current conversion setting. The current protocol uses `25 / sqrt(2)` A RMS when converting torque commands. |
| `kinco_steer_output_kt_nm_per_a` | `0.9230769231` | Steering output-shaft torque constant, in N·m/A. |
| `kinco_wheel_output_kt_nm_per_a` | `1.0588235294` | Wheel output-shaft torque constant, in N·m/A. |
| `kinco_steer_position_kp` | `120` | Steering position loop proportional gain, in N·m/rad. |
| `kinco_steer_profile_acceleration_scale` | `0` | Optional output-shaft reference profile using programmed `0x6081`, `0x6083`, and `0x6084` limits. Positive values scale acceleration/deceleration; `0` retains direct position PD. |
| `kinco_steer_command_delay_s` | `0` | Reference-command delay when the optional steering profile is enabled, in seconds (`0` to `1`). |
| `kinco_steer_velocity_feedforward` | `0` | Multiplier on the profile's reference velocity in the steering damping term. |
| `kinco_control_in_step` | `false` | Evaluate Kinco motor torque from current joint state on every physics step. Protocol traffic and feedback remain on their existing threads; `false` retains snapshot-based updates. |
| `kinco_steer_position_kd` | `12` | Steering position loop velocity gain, in N·m·s/rad. |
| `kinco_wheel_velocity_kp` | `15` | Wheel output-shaft velocity loop proportional gain, in N·m·s/rad. |
| `kinco_wheel_velocity_ki` | `100` | Wheel output-shaft velocity loop integral gain, in N·m/rad. `0` restores proportional-only control. |
| `kinco_wheel_velocity_integral_limit_nm` | `9` | Torque limit for the wheel velocity integral term. |
| `kinco_steer_continuous_torque_nm` | `6` | Steering output-shaft continuous torque, used in the torque–speed envelope. |
| `kinco_steer_peak_torque_nm` | `13.5` | Steering output-shaft peak torque. |
| `kinco_steer_peak_speed_rpm` | `245` | Highest steering output-shaft speed at which peak torque is maintained, in rpm. |
| `kinco_steer_rated_speed_rpm` | `277` | Steering output-shaft speed at which the torque envelope reaches continuous torque, in rpm. |
| `kinco_wheel_continuous_torque_nm` | `9` | Wheel output-shaft continuous torque, used in the torque–speed envelope. |
| `kinco_wheel_peak_torque_nm` | `27` | Wheel output-shaft peak torque. |
| `kinco_wheel_peak_speed_rpm` | `200` | Highest wheel output-shaft speed at which peak torque is maintained, in rpm. |
| `kinco_wheel_rated_speed_rpm` | `280` | Wheel output-shaft speed at which the torque envelope reaches continuous torque, in rpm. |

Wheel PI control uses the output-shaft velocity error:
`torque = Kp × velocity_error + Ki × integral(velocity_error)`.
The corresponding `*_velocity_integral_limit_nm` caps the integral term and must
be positive when integration is enabled. Between peak and rated speed, the Kinco
torque limit decreases linearly to continuous torque. Model actuator limits also apply.

## LiDAR

These settings apply to `[devices.chest_lidar_front]`, `[devices.chest_lidar_rear]`,
and `[devices.chassis_lidar_front]`. All three are currently disabled. Verify the
mount models and calibration files before enabling them. For protocol and mount
development details, see the [LiDAR plugin documentation](plugins/lidar/README.md).

### Common parameters

| Parameter | Description |
| --- | --- |
| `parent_body` | Name of the body in the main model to which the sensor is attached. |
| `site_name` | Ray-origin site name within the sensor model, currently `ray_origin` for all instances. The device name is automatically added as a prefix during assembly. |
| `sensor_model` | Sensor MJCF file to attach. Must exist. |
| `mount_pos` | Position relative to the parent body, as three values `x,y,z` in m. |
| `mount_quat` | Orientation relative to the parent body, as a quaternion in `w,x,y,z` order. Must be finite and nonzero; normalized when loaded. |
| `calibration_file` | Scan angle calibration CSV file matching the LiDAR model. Must exist. |
| `raycast_exclude_meshes` | Model mesh names excluded from raycasting, separated by commas. |
| `scan_rate_hz` | Scan frequency, currently `10` for all instances. Must be finite and positive. |
| `min_range_m` | Minimum valid range. Must be finite and positive. |
| `max_range_m` | Maximum valid range. Must be finite and greater than the minimum range. |
| `raycast_threads` | Raycasting thread count, in `0–256`. `0` selects automatic allocation; currently `1` for all instances. |
| `bind_ip` | Local UDP IP, currently `127.0.0.1` for all instances. |
| `bind_port` | Local UDP port. |
| `peer_ip` | Peer IP receiving LiDAR data, currently `127.0.0.1` for all instances. |
| `peer_port` | Peer UDP port receiving LiDAR data. |

### Current values by instance

| Parameter | Chest front `chest_lidar_front` | Chest rear `chest_lidar_rear` | Chassis front `chassis_lidar_front` |
| --- | --- | --- | --- |
| `parent_body` | `lower_body_link_5` | `lower_body_link_5` | `north` |
| `sensor_model` | `mjcf/sensors/lidar/hesai_ftx140_frame.xml` | `mjcf/sensors/lidar/hesai_ftx140_frame.xml` | `mjcf/sensors/lidar/hesai_jt16_mock.xml` |
| `mount_pos` | `-2.94958e-07,-0.0803,0.11755` | `-0.0126496,0.1171,0.22518` | `0.2685,0,0.12463` |
| `mount_quat` | `-0.707106781,0,0,0.707106781` | `0.459227,-0.537688,-0.459231,0.537688` | `1,0,0,0` |
| `calibration_file` | `resources/lidar/calibration/FTX140_angle_calib_file.csv` | `resources/lidar/calibration/FTX140_angle_calib_file.csv` | `resources/lidar/calibration/JT16_sample_angle.csv` |
| `raycast_exclude_meshes` | `mesh_meshes_torso__torso_FTX_link,mesh_meshes_torso__torso_base_link,mesh_meshes_torso__torso_base_link002,mesh_meshes_torso__torso_base_link003` | `mesh_meshes_torso__torso_FTX_link` | `mesh_root_link` |
| `min_range_m` | `0.1` | `0.1` | `0.15` |
| `max_range_m` | `60` | `60` | `100` |
| `bind_port` | `45980` | `45981` | `45982` |
| `peer_port` | `9180` | `9181` | `9280` |
