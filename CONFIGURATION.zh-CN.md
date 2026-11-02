# Sharpa MuJoCo 配置参考

[English](CONFIGURATION.md) | [简体中文](CONFIGURATION.zh-CN.md)

本文集中说明 [sharpa_mujoco.conf](sharpa_mujoco.conf) 中的全部参数。
安装、构建和首次启动请查看 [README](README.zh-CN.md)。

## 配置约定

- 配置采用 `key=value` 和 `[section]` 格式，`#` 后的内容为注释。
- 表中的“当前值”指仓库配置文件中显式填写的值，不代表省略参数时的代码默认值。
  实际运行以所加载的配置文件为准；修改后重新启动。
- 布尔值统一使用 `true` / `false` 或 `1` / `0`。数组使用逗号分隔。
- `simulation.model`、各设备的 `sensor_model` 和 `calibration_file` 支持绝对路径，
  相对路径以配置文件所在目录为基准。`mujoco_root` 建议填写绝对路径。
- 网络参数中，`bind` 表示模拟设备本地监听端点，`peer` 表示对端通信端点。
  常规端口范围为 `1–65535`；手臂的 `sdo_port=0` 是关闭 SDO 的特例。
  修改端点或通信驱动时，需要与 Platform / SDK 侧配置保持一致。
- 频率单位为 Hz，时间为秒，角度为 rad，角速度为 rad/s，扭矩为 N·m；
  参数名另有单位后缀时以该后缀为准。

可用独立配置文件启动：

```bash
./scripts/run.sh --config /absolute/path/to/sharpa_mujoco.conf
```

该选项选择运行配置；MuJoCo SDK 的选择发生在构建阶段。

## 根级参数

| 参数 | 说明 |
| --- | --- |
| `mujoco_root` | MuJoCo 3.8.1 SDK 解压目录，需包含 `include/mujoco`、`lib` 和 `simulate`。填写本机实际路径，例如 `/absolute/path/to/mujoco-3.8.1`。由构建、打包工具读取，模拟器不使用此项选择场景。 |

CMake 中已有的 `MUJOCO_ROOT` 缓存或首次配置时设置的同名环境变量会优先于文件值。
更换 SDK 后如仍引用旧路径，可使用新的构建目录，或按 `scripts/build.sh --help`
中的清理选项重新配置。

## 仿真与 Viewer

### `[simulation]`

| 参数 | 当前值 | 说明 |
| --- | --- | --- |
| `model` | `mjcf/NTH-HA1-MJCF/NTH-HA1-scene.xml` | 主场景 MJCF 文件。 |
| `snapshot_rate_hz` | `500` | 按仿真时间生成运行时状态快照的频率，必须大于零；物理步长仍由模型设置。 |

### `[viewer]`

| 参数 | 当前值 | 说明 |
| --- | --- | --- |
| `enabled` | `true` | 是否启动 Viewer。关闭后可无界面运行。 |
| `sync_rate_hz` | `60` | Viewer 同步频率，必须大于零。 |
| `vsync` | `false` | 是否启用图形垂直同步。当前关闭 VSync，使用 `sync_rate_hz` 软件限速，避免后台 GLX VSync 阻塞。 |

## 设备启用与驱动

所有 `[devices.*]` 节都有以下两个参数：

| 参数 | 说明 |
| --- | --- |
| `enabled` | 是否加载该设备插件。禁用后不启动该设备的通信和反馈。 |
| `driver` | 设备协议或传输实现；可选值与设备类型有关，见下表。 |

| 配置节 | 当前 `enabled` | 当前 `driver` | 可选驱动 |
| --- | --- | --- | --- |
| `[devices.torso]` | `true` | `can` | `can`、`udp` |
| `[devices.arm_left]`、`[devices.arm_right]` | `true` | `udp_arm` | `udp_arm` |
| `[devices.hand_left]`、`[devices.hand_right]` | `true` | `wave_sdk_loopback` | `wave_sdk_loopback` |
| `[devices.tactile]` | `true` | `wave_tactile` | `wave_tactile` |
| `[devices.chassis]` | `true` | `udp_mcu_kinco` | `udp_mcu_kinco`、`udp_mcu_jiechang` |
| `[devices.chest_lidar_front]`、`[devices.chest_lidar_rear]` | `false` | `hesai_ftx140` | `hesai_ftx140`、`hesai_jt16`，需匹配模型和标定文件 |
| `[devices.chassis_lidar_front]` | `false` | `hesai_jt16` | `hesai_ftx140`、`hesai_jt16`，需匹配模型和标定文件 |

## 躯干 `[devices.torso]`

### 通信与反馈

`driver=can` 使用 Linux SocketCAN，当前接口为 `vcan0` / `vcan1`；
`driver=udp` 使用 MCU UDP 封装。`lower_*` 对应下躯干，`upper_*` 对应上躯干。
端点的电机列表按物理 CAN ID 填写。

| 参数 | 当前值 | 说明 |
| --- | --- | --- |
| `lower_can_interface` | `vcan0` | 下躯干 SocketCAN 接口名，CAN 模式使用。 |
| `upper_can_interface` | `vcan1` | 上躯干 SocketCAN 接口名，CAN 模式使用。 |
| `lower_bind_ip` | `127.0.0.1` | 下躯干 UDP 本地监听 IP。 |
| `lower_bind_port` | `45520` | 下躯干 UDP 本地监听端口。 |
| `lower_peer_ip` | `127.0.0.1` | 下躯干 UDP 对端 IP。 |
| `lower_peer_port` | `45120` | 下躯干 UDP 对端端口。 |
| `lower_can_if` | `0` | 下躯干 MCU UDP 报文中的 CAN 通道编号，范围 `0–255`，不是 Linux 接口名。 |
| `lower_motor_ids` | `1,2,3,4,5` | 下躯干端点负责的物理电机 CAN ID。 |
| `upper_bind_ip` | `127.0.0.1` | 上躯干 UDP 本地监听 IP。 |
| `upper_bind_port` | `45521` | 上躯干 UDP 本地监听端口。 |
| `upper_peer_ip` | `127.0.0.1` | 上躯干 UDP 对端 IP。 |
| `upper_peer_port` | `45121` | 上躯干 UDP 对端端口。 |
| `upper_can_if` | `0` | 上躯干 MCU UDP 报文中的 CAN 通道编号，范围 `0–255`。 |
| `upper_motor_ids` | `6,7,8` | 上躯干端点负责的物理电机 CAN ID。 |
| `feedback_rate_hz` | `200` | 电机状态反馈频率，必须大于零。 |
| `heartbeat_rate_hz` | `10` | 心跳频率，必须大于零。 |

### 电机控制与标定

下表数组均为 8 项，按逻辑电机 J1–J8 排列，对应物理 CAN ID
`1,2,4,3,5,6,7,8`。J3 / J4 是并联腰电机，J6–J8 是颈部电机；
不要按 CAN ID 大小重新排序数组。

| 参数 | 当前值 | 说明 |
| --- | --- | --- |
| `command_delay_s` | `0,0,0,0,0,0,0,0` | 运动命令接口延迟；约束和处理规则见下文“电机接口延迟”。 |
| `feedback_delay_s` | `0.070,0.034,0.044,0.044,0.039,0.046,0.044,0.051` | 逐电机状态反馈接口延迟，已在 CSP/MIT 混合模式启动抱闸修复后重新标定；并联腰 J3/J4 使用相同延迟。 |
| `motor_signs` | `-1,-1,1,1,-1,1,1,1` | 电机协议坐标与模型坐标之间的方向符号，使用 `-1` 或 `1`。 |
| `csp_kp` | `15910.01,9429.39,1882.00,1321.60,854.16,100,100,75` | CSP 位置环比例增益，N·m/rad。 |
| `csp_kd` | `1178.62,708.16,98.54,76.82,46.94,10,10,5` | CSP 速度误差增益，N·m·s/rad。 |
| `command_velocity_filter_s` | `0.03,0.03,0.03,0.03,0.03,0.03,0.03,0.03` | 按位置命令到达节奏估计速度的滤波时间常数；`0` 关闭该轴速度估计。 |
| `command_velocity_scale` | `1,0,1,1,0.6,1,1,1` | 估计命令速度的前馈倍率；`0` 不使用该轴的估计速度前馈。 |
| `mit_kp_scale` | `6.36400,3.77176,3,3,1,1,1,1` | MIT 报文位置增益的仿真倍率。 |
| `mit_kd_scale` | `11.7862,3,2.8521,1.5000,1,1,1,1` | MIT 报文速度增益的仿真倍率。 |
| `mit_torque_ff_scale` | `1,1,1,1,1,1,1,1` | MIT 前馈扭矩倍率。 |
| `mit_torque_limit_nm` | `500,400,250,250,167,300,300,300` | MIT 控制扭矩限幅，必须为正；模型执行器仍有自身限幅。 |
| `mit_torque_time_constant_s` | `0,0.03,0,0,0,0,0,0` | MIT 扭矩响应的一阶时间常数；`0` 表示瞬时响应。 |
| `mit_reference_velocity_limit_rad_s` | `0,0,0,0,0,0,0,0` | MIT 位置参考重采样的速度上限。 |
| `mit_reference_acceleration_limit_rad_s2` | `0,0,0,0,0,0,0,0` | MIT 位置参考重采样的加速度上限，与速度上限配套使用。 |
| `motor_reduction_ratios` | `161,161,161,161,161,101,101,101` | 负载侧速度换算为转子侧速度反馈的减速比，必须为正。 |
| `feedback_velocity_scale` | `1,1,1,1,1,1,1,1` | 仅作用于速度反馈的修正倍率，必须为正，不改变编码器几何映射。 |
| `dynamic_waist_armature` | `true` | 根据并联腰姿态更新折算到模型自由度上的惯量。 |
| `waist_motor_1_inertia` | `1.647253629` | 并联腰第一个电机用于惯量折算的值，kg·m²，非负。 |
| `waist_motor_2_inertia` | `1.647253629` | 并联腰第二个电机用于惯量折算的值，kg·m²，非负。 |

增益、前馈倍率和时间常数使用有限非负数。每轴的 MIT 参考速度与加速度上限必须
同时为零（关闭重采样），或同时为正（启用）；该功能按模型时间平滑异步命令。

当前标定保留了以下设置：

- 依据 2026-10-19 空载录制，J1 反馈延迟受动作相关性回归约束，J6 用于修正剩余的状态发布超前。
- 命令速度前馈用于减小 J1 / J3 / J4 / J5 及颈部轴的相位滞后，J2 关闭。
- J3 / J4 的增益和扭矩余量应一起标定；J5 在快速反向时使用模型的完整 167 N·m 执行器范围。
- 速度反馈的协议编码已处理各电机单位，当前不额外施加经验修正。

## 双臂 `[devices.arm_left]` / `[devices.arm_right]`

左右臂各有 7 个电机，电机数组按索引 `0–6` 排列。`sim_joint_*` 数组按
关节 `0–6` 排列。除 UDP 端点外，两侧当前参数相同。

### 控制与反馈

| 参数 | 当前值 | 说明 |
| --- | --- | --- |
| `command_delay_s` | `0,0,0,0,0,0,0` | 运动命令接口延迟，见下文“电机接口延迟”。 |
| `feedback_delay_s` | `0,0,0,0,0,0,0` | 状态反馈接口延迟，见下文“电机接口延迟”。 |
| `extended_feedback` | `true` | 使用含扩展状态及采样时间戳的反馈报文；`false` 使用基础反馈格式。 |
| `sdo_port` | `65530` | 各板卡 IP 上的 SDO 服务端口；`0` 关闭 SDO，仅保留 PDO。启用时每块板卡需使用不同监听 IP。 |
| `bus_voltage_v` | `48` | 上报的模拟母线电压，V。 |
| `driver_temperature_c` | `25` | 上报的模拟驱动器温度，°C。 |
| `motor_temperature_c` | `25` | 上报的模拟电机温度，°C。 |
| `feedback_rate_hz` | `1000` | 状态反馈频率，必须大于零。 |
| `command_timeout_s` | `0.2` | 运动命令有效期，按原始收包墙钟时间判断，必须大于零；超时触发停止处理。 |
| `quick_stop_deceleration_rad_s2` | `10` | 快速停止时目标速度的减速度，必须为正。 |
| `quick_stop_velocity_threshold_rad_s` | `0.02` | 判断快速停止完成的速度阈值，必须为正。 |
| `quick_stop_timeout_s` | `1` | 快速停止最长持续时间，必须为正。 |
| `quick_stop_kp` | `20` | 快速停止的速度误差增益，N·m·s/rad，必须为正。 |
| `speed_kp` | `5` | 速度模式的比例增益，N·m·s/rad，非负。 |
| `speed_ki` | `20` | 速度模式的积分增益，N·m/rad，非负。 |
| `speed_integral_limit_nm` | `20` | 速度环积分项的扭矩限幅，非负。 |
| `diagnostic_logging` | `true` | 是否输出手臂诊断日志。 |
| `csp_control_space` | `joint` | CSP 控制空间：`joint` 在关节空间控制，`motor` 使用电机空间控制。 |
| `sim_joint_kp` | `500,500,300,200,50,50,50` | 关节空间 CSP 位置增益，N·m/rad。 |
| `sim_joint_kd` | `7.5,7.5,2,7.5,1,1,1` | 关节空间 CSP 速度增益，N·m·s/rad。 |
| `csp_kp` | `100,100,100,40,40,30,30` | 电机空间 CSP 位置增益，N·m/rad；也用于 homing 时保持物理姿态。 |
| `csp_kd` | `10,10,10,5,5,5,5` | 电机空间 CSP 速度增益，N·m·s/rad；也用于 homing 时保持物理姿态。 |
| `torque_limits_nm` | `40,40,30,6.9,6.9,6.9,6.9` | 每个电机的扭矩限幅，必须为正。 |

电压、温度是合成遥测值，不代表实测值，需为 `[-128,127]` 范围内的有限数，
以适配协议的有符号 8 位字段。控制增益使用有限非负数。
`csp_control_space=joint` 时，常规 CSP 使用 `sim_joint_kp` / `sim_joint_kd`；
切换到 `motor` 才使用旧的电机空间 CSP 增益。

### 板卡 UDP 端点

每臂有五组端点，以下 `N` 分别替换为 `0`、`1`、`2`、`3`、`4`，
例如 `endpoint0_bind_ip` 和 `endpoint4_motor_indices`。

| 参数 | 说明 |
| --- | --- |
| `endpointN_bind_ip` | 第 N 块板卡的本地监听 IP。 |
| `endpointN_bind_port` | 第 N 块板卡的本地 PDO UDP 端口，当前全部为 `21000`。 |
| `endpointN_peer_ip` | 第 N 块板卡的对端 IP，当前全部为 `127.0.0.1`。 |
| `endpointN_peer_port` | 第 N 块板卡的反馈对端端口，左臂全部为 `8080`，右臂全部为 `8081`。 |
| `endpointN_motor_indices` | 第 N 块板卡负责的 1 或 2 个电机索引，使用逗号分隔，索引范围 `0–6`。 |

| N | 左臂 `endpointN_bind_ip` | 右臂 `endpointN_bind_ip` | 两侧 `endpointN_motor_indices` |
| --- | --- | --- | --- |
| `0` | `127.0.40.101` | `127.0.40.106` | `0` |
| `1` | `127.0.40.102` | `127.0.40.107` | `1` |
| `2` | `127.0.40.103` | `127.0.40.108` | `2` |
| `3` | `127.0.40.104` | `127.0.40.109` | `3,4` |
| `4` | `127.0.40.105` | `127.0.40.110` | `5,6` |

五组端点应完整且不重复地覆盖 7 个电机。同一板卡上的电机必须使用相同的
命令延迟和反馈延迟。

## 电机接口延迟

本节适用于躯干和双臂的 `command_delay_s` / `feedback_delay_s`。
二者都是固定墙钟时间延迟，单位为秒，省略时各项为零；零值直接绕过缓冲。
每项必须有限且位于 `[0,0.1]`。

- 躯干为 8 项，按 J1–J8 顺序；并联腰 J3 / J4 的命令延迟必须相等，
  反馈延迟也必须相等。
- 每臂为 7 项，按电机 `0–6` 顺序；同一板卡内各电机的命令延迟必须相等，
  反馈延迟也必须相等。每项命令延迟还必须小于该臂的 `command_timeout_s`。
- 命令延迟将正常运动目标排队，到期后在物理步中执行。初次使能、模式切换、
  刹车、停止和 homing 立即处理，并清除旧运动目标。手臂超时仍按原始收包时间判断。
- 反馈延迟将同次采样的位置、速度、扭矩及编码器反馈一起延后发布。手臂扩展报文
  保留原采样时间，驱动状态使用当前状态。启动、复位或清空反馈队列后，需要等待
  一个延迟周期才能发布相应轴的反馈。SDO 响应和心跳不排队。
- 两者均采用有界时间戳队列，不休眠物理线程，不修改 MuJoCo 时间或 H5 时间戳，
  不随运动速度改变延迟。复位或插件停止时清空队列；超出 1024 项时丢弃最旧项。

命令和反馈路径应分别标定，使用原时间轴上的 real state / sim state 误差及动作
回归验证。已有控制器和动力学产生的 action→state 总滞后不应再次完整叠加到接口上。

## 灵巧手 `[devices.hand_left]` / `[devices.hand_right]`

| 参数 | 左手当前值 | 右手当前值 | 说明 |
| --- | --- | --- | --- |
| `device_ip` | `127.0.50.10` | `127.0.50.20` | 模拟手设备的本地监听 IP。 |
| `sdk_ip` | `127.0.0.1` | `127.0.0.1` | 接收反馈和心跳的 SDK 主机 IP。 |
| `serial_number` | `SIM-WAVE-L-0001` | `SIM-WAVE-R-0001` | 模拟设备序列号，左右手必须不同。 |
| `product_number` | `Wave01-MOCK` | `Wave01-MOCK` | 模拟设备产品型号。 |
| `tcp_port` | `59999` | `59999` | 设备 TCP 配置服务端口。 |
| `command_udp_port` | `50020` | `50030` | 接收关节控制命令的 UDP 端口。 |
| `sdk_joint_port` | `50000` | `50010` | SDK 接收关节反馈的 UDP 端口。 |
| `sdk_heartbeat_port` | `54321` | `54321` | SDK 接收设备心跳的 UDP 端口。 |
| `feedback_source_port` | `10000` | `10000` | 设备发送反馈所使用的本地 UDP 源端口。 |
| `feedback_rate_hz` | `500` | `500` | 关节反馈频率，必须大于零。 |
| `heartbeat_rate_hz` | `1` | `1` | 心跳频率，必须大于零。 |
| `strict_command_state` | `false` | `false` | 为 `true` 时，命令包的控制源和控制模式必须匹配设备当前状态；为 `false` 时仍保留运行状态、故障和生命周期检查。 |

## 指尖触觉 `[devices.tactile]`

| 参数 | 当前值 | 说明 |
| --- | --- | --- |
| `sdk_ip` | `127.0.0.1` | 接收触觉数据的 SDK 主机 IP。 |
| `right_device_ip` | `127.0.50.20` | 右手触觉设备 IP，与右手配置服务对应。 |
| `left_device_ip` | `127.0.50.10` | 左手触觉设备 IP，与左手配置服务对应。 |
| `config_port` | `59998` | 两个设备 IP 上监听的触觉 TCP 配置端口。 |
| `right_sdk_port` | `50001` | 右手触觉数据的 SDK UDP 目标端口。 |
| `left_sdk_port` | `50011` | 左手触觉数据的 SDK UDP 目标端口。 |
| `rate_hz` | `30` | 触觉数据发布频率，必须大于零。 |
| `sensor_model` | `mjcf/sensors/tactile/wave_fingertip_frame.xml` | 指尖传感器 MJCF 模型，加载后分别挂载到双手共十个指尖。 |

`right_sdk_port` / `left_sdk_port` 的命名按触觉插件配置定义，修改时需与触觉
SDK 接收端对应，不能根据关节反馈端口的大小推断左右手。

## 底盘 `[devices.chassis]`

### 驱动、端点与通用控制

`driver` 选择转向和轮毂电机协议，`stretch_driver` 单独选择伸缩电机协议。
当前组合为 Kinco 转向 / 轮毂和 Jiechang 伸缩，对应 Platform 的
`config/sharpa_chassis_plugin_config.json`。Jiechang 伸缩使用经典 CAN，
ID 为 `9–12`，AB 对应通道 0，CD 对应通道 1，并复用所选转向 / 轮毂的 MCU UDP 端点。

| 参数 | 当前值 | 说明 |
| --- | --- | --- |
| `stretch_driver` | `jiechang` | 伸缩驱动，可选 `jiechang` 或 `m42p`。 |
| `motor_signs` | `1,1,1,1,-1,-1,-1,-1` | 8 项方向符号，依次为转向轴 1–4、轮轴 1–4，仅允许 `-1` / `1`；不含伸缩轴。 |
| `jiechang_bind_ip` | `127.0.0.1` | `driver=udp_mcu_jiechang` 时的本地 UDP IP。 |
| `jiechang_bind_port` | `45620` | Jiechang 转向 / 轮毂 MCU 本地 UDP 端口。 |
| `jiechang_peer_ip` | `127.0.0.1` | Jiechang 转向 / 轮毂 MCU 对端 IP。 |
| `jiechang_peer_port` | `45220` | Jiechang 转向 / 轮毂 MCU 对端端口。 |
| `kinco_bind_ip` | `127.0.0.1` | `driver=udp_mcu_kinco` 时的本地 UDP IP。 |
| `kinco_bind_port` | `45620` | Kinco 转向 / 轮毂 MCU 本地 UDP 端口。 |
| `kinco_peer_ip` | `127.0.0.1` | Kinco 转向 / 轮毂 MCU 对端 IP。 |
| `kinco_peer_port` | `45220` | Kinco 转向 / 轮毂 MCU 对端端口。 |
| `m42p_bind_ip` | `127.0.0.1` | 仅 `stretch_driver=m42p` 时使用的伸缩 MCU 本地 UDP IP。 |
| `m42p_bind_port` | `45621` | M42P 伸缩 MCU 本地 UDP 端口。 |
| `m42p_peer_ip` | `127.0.0.1` | M42P 伸缩 MCU 对端 IP。 |
| `m42p_peer_port` | `45221` | M42P 伸缩 MCU 对端端口。 |
| `feedback_rate_hz` | `200` | 底盘反馈频率，必须大于零；当前对应 5 ms 周期。 |
| `command_timeout_s` | `0.1` | 运动命令超时阈值，必须大于零；当前与 Platform 的 100 ms 命令超时对应。 |
| `jiechang_stretch_counts_per_meter` | `262144000` | Jiechang 伸缩位置计数 / 米；按每转 262144 计数、1 mm 导程及减速比 1 换算。 |
| `slider_steps_per_meter` | `2133333.333333` | M42P 伸缩步数 / 米，仅 `stretch_driver=m42p` 使用。 |
| `slider_minimum_m` | `0` | 伸缩位置下限，m。 |
| `slider_maximum_m` | `0.06` | 伸缩位置上限，m。 |

两种主电机驱动只启用所选的一组 UDP 端点，因此它们的配置可保留相同端口。
当前使用 Jiechang 伸缩时不启动 M42P 端点。

### Jiechang 转向 / 轮毂参数

本组用于 `driver=udp_mcu_jiechang`，当前配置保留这些值供切换驱动使用。

| 参数 | 当前值 | 说明 |
| --- | --- | --- |
| `steer_gear_ratio` | `47.25` | 转向电机减速比。 |
| `wheel_gear_ratio` | `6.28` | 轮毂电机减速比。 |
| `steer_rated_current_a` | `1.2` | 转向电机额定电流，A。 |
| `wheel_rated_current_a` | `3.2` | 轮毂电机额定电流，A。 |
| `steer_rated_motor_torque_nm` | `0.26` | 转向电机轴额定扭矩，N·m，换算输出扭矩时计入减速比。 |
| `wheel_rated_motor_torque_nm` | `0.50` | 轮毂电机轴额定扭矩，N·m。 |
| `steer_position_kp` | `120` | 转向位置环比例增益，N·m/rad。 |
| `steer_position_kd` | `12` | 转向位置环速度增益，N·m·s/rad。 |
| `wheel_velocity_kv` | `15` | 轮毂输出轴速度环比例增益，N·m·s/rad。 |
| `wheel_velocity_ki` | `100` | 轮毂输出轴速度环积分增益，N·m/rad；`0` 关闭积分。 |
| `wheel_velocity_integral_limit_nm` | `3.14` | 轮毂速度积分项扭矩限幅。 |
| `jiechang_steer_torque_limit_nm` | `12.285` | 转向输出扭矩限幅。 |
| `jiechang_wheel_torque_limit_nm` | `3.14` | 轮毂输出扭矩限幅。 |
| `jiechang_mit_kp_scale` | `1` | Jiechang MIT 模式位置增益倍率。 |
| `jiechang_mit_kd_scale` | `1` | Jiechang MIT 模式速度增益倍率。 |

### Kinco 转向 / 轮毂参数

本组用于 `driver=udp_mcu_kinco`，当前数值与 Platform 的 Kinco 协议驱动匹配。

支持 Platform 的 `steer_calib_mode=protocol` 回零流程：`0x6098=-17`、
`0x607C`、`0x6099`、`0x609A`、模式 `6` 的回显和回零完成状态。
仿真以 MJCF 转向关节零位作为已知标定参考，按回零搜索速度和加速度驱动关节回零，
回零控制始终按物理步计算，位置与速度均进入容差后才报告完成。
此过程不模拟真机寻找负限位再施加原点偏移的运动；
`0x607C` 等参数保留读写，但不改变模型坐标系。回零由 SDO 启动并自主执行，
停止、禁用、故障或退出回零模式会终止它；普通 RPDO 运动仍受命令超时保护。

| 参数 | 当前值 | 说明 |
| --- | --- | --- |
| `kinco_encoder_resolution` | `65536` | 电机轴每转编码器计数。 |
| `kinco_steer_gear_ratio` | `9` | 转向电机减速比。 |
| `kinco_wheel_gear_ratio` | `10` | 轮毂电机减速比。 |
| `kinco_steer_rated_current_a` | `6.5` | 转向电机额定电流，A；当前实现保留此配置值，未参与控制或反馈换算。 |
| `kinco_wheel_rated_current_a` | `8.5` | 轮毂电机额定电流，A；当前实现保留此配置值，未参与控制或反馈换算。 |
| `kinco_steer_max_current_arms` | `14.3` | 转向电流指令 / 反馈换算基准，按驱动协议使用有效值电流。 |
| `kinco_wheel_max_current_arms` | `25` | 轮毂电流换算配置值；当前协议在力矩命令换算中使用 `25 / sqrt(2)` A 有效值。 |
| `kinco_steer_output_kt_nm_per_a` | `0.9230769231` | 转向输出轴扭矩常数，N·m/A。 |
| `kinco_wheel_output_kt_nm_per_a` | `1.0588235294` | 轮毂输出轴扭矩常数，N·m/A。 |
| `kinco_steer_position_kp` | `120` | 转向位置环比例增益，N·m/rad。 |
| `kinco_steer_profile_acceleration_scale` | `0` | 可选的输出轴参考轨迹，使用已下发的 `0x6081`、`0x6083`、`0x6084` 限制；正值缩放加减速度，`0` 保持直接位置 PD。 |
| `kinco_steer_command_delay_s` | `0` | 启用转向参考轨迹时的指令延迟，单位秒，范围 `0`～`1`。 |
| `kinco_steer_velocity_feedforward` | `0` | 转向阻尼项中参考轨迹速度的前馈系数。 |
| `kinco_control_in_step` | `false` | 每个物理步根据当前关节状态计算 Kinco 电机力矩；协议收发仍使用原有线程，`false` 保持基于快照的更新方式。 |
| `kinco_steer_position_kd` | `12` | 转向位置环速度增益，N·m·s/rad。 |
| `kinco_wheel_velocity_kp` | `15` | 轮毂输出轴速度环比例增益，N·m·s/rad。 |
| `kinco_wheel_velocity_ki` | `100` | 轮毂输出轴速度环积分增益，N·m/rad；`0` 恢复纯比例控制。 |
| `kinco_wheel_velocity_integral_limit_nm` | `9` | 轮毂速度积分项扭矩限幅。 |
| `kinco_steer_continuous_torque_nm` | `6` | 转向输出轴连续扭矩，用于扭矩—转速包络。 |
| `kinco_steer_peak_torque_nm` | `13.5` | 转向输出轴峰值扭矩。 |
| `kinco_steer_peak_speed_rpm` | `245` | 转向输出轴维持峰值扭矩的转速上界，rpm。 |
| `kinco_steer_rated_speed_rpm` | `277` | 转向输出轴扭矩包络降至连续扭矩的转速，rpm。 |
| `kinco_wheel_continuous_torque_nm` | `9` | 轮毂输出轴连续扭矩，用于扭矩—转速包络。 |
| `kinco_wheel_peak_torque_nm` | `27` | 轮毂输出轴峰值扭矩。 |
| `kinco_wheel_peak_speed_rpm` | `200` | 轮毂输出轴维持峰值扭矩的转速上界，rpm。 |
| `kinco_wheel_rated_speed_rpm` | `280` | 轮毂输出轴扭矩包络降至连续扭矩的转速，rpm。 |

轮毂 PI 控制按输出轴速度误差计算：
`扭矩 = Kp × 速度误差 + Ki × 速度误差积分`，积分项受相应的
`*_velocity_integral_limit_nm` 限制；启用积分时，该限幅必须为正。
Kinco 扭矩上限在峰值转速与额定转速之间线性降至连续扭矩，最终还受模型执行器限幅约束。

## LiDAR

适用于 `[devices.chest_lidar_front]`、`[devices.chest_lidar_rear]` 和
`[devices.chassis_lidar_front]`。三路当前均关闭；启用前需确认挂载模型和标定文件。
协议及挂载开发说明见 [LiDAR 插件文档](plugins/lidar/README.zh-CN.md)。

### 通用参数

| 参数 | 说明 |
| --- | --- |
| `parent_body` | 传感器挂载的主模型 body 名称。 |
| `site_name` | 传感器模型内部的射线原点 site 名，当前均为 `ray_origin`；装配后自动加设备名作为前缀。 |
| `sensor_model` | 被挂载的传感器 MJCF 文件，必须存在。 |
| `mount_pos` | 相对父 body 的位置，三个数 `x,y,z`，单位 m。 |
| `mount_quat` | 相对父 body 的姿态四元数，顺序 `w,x,y,z`；必须有限且非零，加载时归一化。 |
| `calibration_file` | 与雷达型号匹配的扫描角度标定 CSV 文件，必须存在。 |
| `raycast_exclude_meshes` | 射线检测排除的模型 mesh 名称，用逗号分隔。 |
| `scan_rate_hz` | 扫描频率，当前均为 `10`，必须为有限正数。 |
| `min_range_m` | 最小有效距离，必须为有限正数。 |
| `max_range_m` | 最大有效距离，必须有限且大于最小距离。 |
| `raycast_threads` | 射线计算线程数，范围 `0–256`；`0` 自动分配，当前均为 `1`。 |
| `bind_ip` | 本地 UDP IP，当前均为 `127.0.0.1`。 |
| `bind_port` | 本地 UDP 端口。 |
| `peer_ip` | 接收雷达数据的对端 IP，当前均为 `127.0.0.1`。 |
| `peer_port` | 接收雷达数据的对端 UDP 端口。 |

### 各实例当前值

| 参数 | 胸前 `chest_lidar_front` | 胸后 `chest_lidar_rear` | 底盘前 `chassis_lidar_front` |
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
