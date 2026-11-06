# 底盘仿真／真机一致性采集

`chassis_motion_driver.py` 通过 `action/chassis` 发送机体系速度，并按
`state/chassis` 的原始到达频率保存反馈。默认序列是
`forward,backward,steer-sine,steer-hold,yaw-sine`，默认速度 0.15 m/s、
角速度 0.3 rad/s、单段 12 s。原有平移、自转和整圈动作仍可单独指定；
`--motions all` 包括所有动作，整圈不在默认采集序列中。

## 转向如何激励

固定方向的速度脉冲主要激励驱动轮。即使同时按相同比例增减 `vx` 和
`vtheta`，各轮速度方向也基本不变，无法充分观察连续转舵的动态。
前进／后退切换也可能由轮速反转完成，而舵角保持不变。

| 动作 | 指令 | 主要观察量 |
| --- | --- | --- |
| `forward,backward` | 固定方向的 sin² 速度脉冲，从零加速再减速至零 | 各驱动轮的响应延迟、加减速、正反向差异 |
| `steer-sine` | 保持平移速度幅值，平滑往复改变平移方向；不命令车身自转 | 四舵轮同向转动的幅值、相位滞后、换向差异 |
| `steer-hold` | 同上，方向按 0→正幅值→保持→负幅值→保持→0 变化，过渡使用五次曲线 | 转舵过程、到位时间、保持误差、回差 |
| `yaw-sine` | 保持前进速度，平滑往复改变角速度 | 前后、左右轮的不同舵角／轮速及转向与驱动的耦合 |

三种转向动作的前后各 15% 时间用于平滑加减速，中间 70% 进行转向激励。
`--steer-angle-deg` 控制前两种动作的平移方向幅值（默认 ±35°）；
`--steer-cycles` 控制两个 sine 动作的周期数（默认 2）。例如单段 12 s 时，
中间 8.4 s 内完成两次摆动，主要频率约 0.238 Hz。
`steer-hold` 的每次正／负保持持续整段时间的 10.5%，与周期数无关。

对于前两种动作，`vx=v*cos(alpha), vy=v*sin(alpha), vtheta=0`，理想四轮舵角
均为 `alpha`，理想滚动速度幅值不变。在实际 Platform 中仍会经过速度限制、
低速舵锁、舵角表示选择和舵轮同步门控，不能将 `alpha` 当作电机最终收到的目标。

对于 `yaw-sine`，轮心 `(rx, ry)` 的速度为
`(vx-vtheta*ry, vy+vtheta*rx)`，因此四个轮子的舵角和驱动速度不同。
它适合最后检查整底盘的组合响应，不能将其中的轮速变化都归因于驱动控制器。
`--steer-angle-deg` 不限制这个动作的实际舵角；该角度由速度比、轮心位置和
Platform 的舵角限制共同决定。

这里的接口无法表达“驱动轮严格零速，但四个舵机分别按指定角度运动”。
零机体速度可能保持舵角或回中。若需要独立的舵机阶跃／扫频辨识，应使用
Platform 的原始关节旁路接口和对应模式，并单独采集最终电机目标；本脚本保持
使用正常底盘速度控制链路，不自动切换到旁路模式。

## 运行

先离线查看计划，不建立 Zenoh 连接、不产生数据文件：

```bash
python3 examples/motion/chassis_motion_driver.py --dry-run
```

只测试转向时，可先低速采一段：

```bash
python3 examples/motion/chassis_motion_driver.py \
  --motions steer-sine,steer-hold \
  --linear-speed 0.10 --steer-angle-deg 25 \
  --segment-duration 12 --steer-cycles 2 \
  --label real --output-dir recordings/chassis/real_steering_01
```

真机完整采集；将端点替换为实际 Platform 的地址：

```bash
python3 examples/motion/chassis_motion_driver.py \
  --connect 'udp/<robot-ip>:7449?rel=1;multistream=1;mixed_rel=0' \
  --motions calibration --linear-speed 0.15 --angular-speed 0.3 \
  --steer-angle-deg 35 --steer-cycles 2 --segment-duration 12 \
  --repeat 3 --label real --output-dir recordings/chassis/real_01
```

仿真使用相同运动参数、更换 `--connect`、`--label sim` 和输出目录即可。
不指定端点时沿用仓库 `resources/zenoh/client.json` 的默认端点。
指定的输出目录必须尚不存在，防止覆盖旧数据；省略时自动创建
`recordings/chassis/<time_ns>/`。`--no-record` 可用于只检查动作。

运行前启动 Platform 的底盘控制和状态发布，并确保底盘处于允许速度控制的模式、
电机已使能；脚本不负责使能、校准或抢占其他命令源。仿真还需要启用
`devices.chassis.enabled`。真机与仿真应采用相同的初始舵角、载荷、轮径／几何、
伸缩位置及 Platform 控制配置，再比较同一动作、同一幅值和频率的数据。

这些是开环行进动作，不会自动返回初始世界位姿。默认每个转向段的平移路径长度
约为 `0.85*0.15*12=1.53 m`；重复序列会继续累积位移，需要相应的可用场地。
`v_l` 始终为零，不激励伸缩轴。

## 输出与比较

- `metadata.json`：完整运行参数、动作顺序、单位和接口限制。
- `events.jsonl`：每行一个 JSON 事件，包含 `command`、`state`、
  `segment_summary` 和结束时的 `result`。中断／反馈异常也保留已写入的数据。

每个事件包含 `phase`（startup/motion/settle/shutdown）、`motion`、从 1 开始的
`repeat` 和 `segment`；`elapsed_s` 是本次运行的单调时钟相对时间，
`phase_elapsed_s` 是当前阶段的相对时间，`local_time_ns` 是本机墙钟时间。
指令与反馈各自记录时间，不强行配对成一行，也不把旧反馈复制到每一个命令周期。
跨线程写入的文件行序不保证时间顺序，分析时应按 `elapsed_s` 排序。

`command` 保存实际成功发布的 `vx/vy/vtheta/v_l`、header 时间戳和序号。
`state` 保存原始关节名称、位置、速度、扭矩占位值、模式、电流、电压、温度，
以及机体速度。按名称取值；当前前八轴是
`lf_drive,lf_steer,rf_drive,rf_steer,rr_drive,rr_steer,lr_drive,lr_steer`。
`position` 中的舵角单位为 rad，`velocity` 中的轮速为 rad/s，`current` 为 A。
电气字段按 `electrical_names` 对应。后续四根伸缩轴若存在，会原样保存其归一化值。

反馈 `source_stamp_ns` 是 Platform 当前有效轴反馈的最新主机接收时间，
不是电机采样时间，也不是每根轴独立的时间戳。零、重复和倒退的时间戳仍会保存，
但其 `fresh=false`，不参与响应统计，也不能维持反馈新鲜度。
它不能证明所有轴都在更新；逐轮响应检查也只检查响应幅值，并非逐轴健康诊断。

对齐两次运行时，先按 `(repeat,segment,motion)` 分段，再按该段的命令起点对齐，
在共同的相对时间轴上比较反馈。不要直接相减两台机器的绝对时间戳来估计链路延迟。
保留 `fresh=true` 的反馈，比较驱动轮速度的上升／下降过程，以及舵角的幅值、
相位、到位过程、保持误差和正反向差异。需要转舵速度时，对舵角 unwrap 后，
平滑并按反馈时间间隔求导；当前 Platform 的舵机速度字段固定为零。

脚本对每段检查四个驱动轮是否分别有响应，并对三种转向段检查四个舵角是否分别
有足够的峰峰值；输出的 `steer_travel_deg` 是 unwrap 后的峰峰值，不是累计路程。
默认门限分别是 0.05 rad/s 和 2°，可用 `--minimum-wheel-speed` 与
`--minimum-steer-travel-deg` 调整。`--no-verify` 只跳过响应幅值检查，仍要求反馈
源时间戳持续推进。检查通过仅表示产生了响应，并不代表仿真与真机已经一致。
反馈超时、单轮无响应、写盘失败或 Ctrl-C 都会进入零速度退出路径；
停顿阶段也持续发送零速度并保留反馈。

## 能对齐到哪一层

这份数据能用于比较**正常控制链路下的驱动、转向和整底盘响应**。
有三项边界需要在分析时明确：

1. 当前 Zenoh 接口没有发布 `ChassisTarget`。脚本中的机体指令不能替代
   Platform 经过限速和舵轮门控后下发的关节目标。要区分 Platform 策略与
   电机执行器参数，需同时使用 Platform CSV recorder 记录内部目标与反馈，
   或配合仓库 `tools/monitor` 底盘 UDP 监控观察电机目标／反馈。
   该监控当前仅支持 `udp_mcu_kinco` 的仿真链路和配置，不能直接套用于
   Jiechang 或未经适配的真机链路。其显示量是电机轴单位，比较时还需
   正确换算减速比和方向。
2. 当前机体速度反馈由轮速和舵角正运动学计算而来；驱动位置和关节扭矩也有
   零占位字段。它们不能作为独立的地面速度、位置或力矩测量。
3. 若要对齐轮胎打滑、地面摩擦、车身惯量及实际轨迹，需另外同步采集
   IMU、动捕或独立定位数据，并保持地面与载荷条件一致。
