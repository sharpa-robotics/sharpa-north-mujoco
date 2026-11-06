# 躯干、手部、手臂与底盘链路实时对比监控

该工具统一监控躯干、Wave 手部、左右手臂、手套和底盘的通信链路。运行时通过一个数字选择监控部位：

| 参数 | 部位 | 对比链路 |
| --- | --- | --- |
| `1` | 躯干 | Zenoh action/state 与 CAN/UDP action/state |
| `2` | 左手 | Zenoh action/state 与 Wave UDP action/state |
| `3` | 右手 | Zenoh action/state 与 Wave UDP action/state |
| `4` | 左臂 | Zenoh action/state 与 Arm UDP action/state |
| `5` | 右臂 | Zenoh action/state 与 Arm UDP action/state |
| `6` | 左手套 | Zenoh `action/left_glove/raw` |
| `7` | 右手套 | Zenoh `action/right_glove/raw` |
| `8` | 底盘 | 左列 Zenoh 整车速度；右列 Kinco UDP 逐电机位置/转速 |

工具只接收原始协议消息，不发送控制消息、不绑定业务 UDP 端口、不读取 MuJoCo 内存，也不会修改任何链路数据。

## 运行

在 `sharpa-north-mujoco` 仓库根目录执行：

```bash
# 1：躯干
sudo .venv/bin/python tools/monitor/monitor.py 1

# 2：左手
sudo .venv/bin/python tools/monitor/monitor.py 2

# 3：右手
sudo .venv/bin/python tools/monitor/monitor.py 3

# 4：左臂
sudo .venv/bin/python tools/monitor/monitor.py 4

# 5：右臂
sudo .venv/bin/python tools/monitor/monitor.py 5

# 6：左手套 raw
sudo .venv/bin/python tools/monitor/monitor.py 6

# 7：右手套 raw
sudo .venv/bin/python tools/monitor/monitor.py 7

# 8：底盘 vx、vy、vtheta + 八个行走电机
sudo .venv/bin/python tools/monitor/monitor.py 8
```

默认显示最近 30 秒，可以保留原有的 `--time` 参数调整时间窗口：

```bash
sudo .venv/bin/python tools/monitor/monitor.py 2 --time 60
```

依赖可安装到现有虚拟环境：

```bash
.venv/bin/python -m pip install -r tools/monitor/requirements.txt
```

工具固定读取：

- `sharpa_mujoco.conf`
- 相邻仓库 `../sharpa_platform/proto/north_pb2.py`
- Zenoh UDP `127.0.0.1:7449`

## 躯干模式

躯干模式保留 8 轴、两列布局，并根据 `[devices.torso].driver` 自动选择只接收的 SocketCAN socket 或 `AF_PACKET` UDP 旁路抓包。monitor 从每个电机的 action 帧独立识别当前模式：`0x200` 为 Position/CSP，`0x210` 为 MIT；尚未收到对应电机 action 时明确显示 `Unknown`。

- Position/CSP 电机：CAN/UDP 列显示还原后的关节位置，单位为 deg；J3/J4 使用与躯干插件相同的并联腰运动学还原为 pitch/roll。
- MIT 电机：CAN/UDP 列显示该行对应的原始电机 action/state 位置，单位为 rad，不做关节转换。

每一行分别选择视图，因此同一时刻不同电机使用不同模式时不会按全局模式误画。窗口标题、子图标题和状态栏会分别标出各行模式、显示空间及单位；某个电机模式变化时，只清空受影响行的 wire 历史。

## 手部模式

左手和右手模式分别显示对应手的 22 个关节，保留原工具的 11×4 紧凑布局。Wave JointPacket V0 仍校验协议版本、方向、手别、字段有限性和 CRC32。

## 手臂模式

左臂和右臂模式还会订阅 `system/heartbeat/{side}_arm`，直接读取 platform 发布的 `ArmControlMode`。Heartbeat 是精确模式来源；在尚未收到 Heartbeat 时，UDP 驱动模式只对能无歧义识别的模式作启动兜底。

绘图会自动选择当前控制模式对应的 UDP 视图：

- `MotorPd`（模式 5）：显示原始 UDP 电机 M0–M6 的 action/state 位置，单位为 rad，不做关节传动转换。
- 其他模式：显示 J1–J7；UDP 电机位置经符号、线缆减速比和差动关系还原为关节角，单位为 deg，再与 Zenoh joint action/state 对比。

窗口标题、子图标题和状态栏会标出控制模式、模式来源、UDP 视图及单位。模式真正变化时会清空上一模式历史，避免不同空间或单位混画。每个 UDP 端点包只为它实际携带的电机所对应的关节追加采样点，避免用其他端点的旧缓存重复生成全臂曲线。控制帧和反馈帧均校验长度、命令字、节点编号和 CRC-16/CCITT-FALSE。UDP state 同时支持旧版反馈帧（单轴 35 字节、双轴 56 字节）和 `extended_feedback=true` 的扩展反馈帧（单轴 44 字节、双轴 74 字节）。

## 手套模式

左、右手套模式分别监听 `action/left_glove/raw` 和 `action/right_glove/raw`，将 `HandCommand` 中的 22 个关节位置绘制为独立曲线。该模式只监听 Zenoh，不监听 Wave UDP。

## 底盘模式

同一窗口左列显示 3 张整车速度图，右列显示 8 张电机图。所有图共享时间轴，各图纵轴独立；每张图分别对照指令和反馈。伸缩轮不参与监控。

| 位置 | 指标 | 单位与来源 |
| --- | --- | --- |
| 左列 | `vx`（前向）、`vy`（左向）、`vtheta`（逆时针偏航） | m/s、m/s、rad/s；Zenoh `action/chassis` / `state/chassis` |
| 右列 M1–M4 | 舵电机 A/LF、B/RF、C/RR、D/LR 的电机轴位置 | rad；Kinco 目标位置 `0x607A` / 反馈位置 `0x6064` |
| 右列 M5–M8 | 驱动电机 A/LF、B/RF、C/RR、D/LR 的电机轴转速 | rad/s；Kinco 目标速度 `0x60FF` / 反馈速度 `0x606C` |

左列指令直接读取 `ChassisCommand.vx/vy/vtheta`，是 Platform 限加速度、舵轮同步等处理之前的速度请求。反馈直接读取 `ChassisState.velocity.linear.x/linear.y/angular.z`，是 **Platform 计算的整车速度**。无有效反馈时间戳时显示不可用；相同 `header.stamp` 的重复发布不追加样本，也不刷新数据新鲜度。

右列直接解码固件电机反馈，**只做协议单位换算，不做整车运动学计算**：

- 电机轴位置：`raw × 2π / encoder_resolution`，保留多圈角度。
- 电机轴转速：`raw × 1875 / (512 × encoder_resolution) × 2π / 60`。
- 保留报文正负号，不除减速比，不应用 `motor_signs`、Platform 的方向映射或舵角标定零点。图中位置/转速属于电机轴，不是减速后输出轴。

右列支持当前固定映射的 RPDO1、TPDO1 和相关 expedited SDO。每个电机独立追加样本、独立判断指令和反馈是否过期；收到一个电机的数据就更新该电机，不等待其余七个电机，也不使用其他电机的缓存值。过期阈值读取 `command_timeout_s`，当前为 0.1 s。缺少数据、过期、驱动禁用或重启时，在对应图中提示原因并断开受影响曲线。横轴为本机单调时钟收包时间；Zenoh 超过 1 s 无新样本显示 stale。

电流模式仍固定显示上述位置/转速指标：对应电机的位置/转速**指令**标记为 `N/A / current control`，其位置/转速**反馈**继续绘制，其他七个电机及左列不受影响。恢复位置舵或速度轮模式后，只需收到该电机的新目标即可恢复指令曲线。控制模式从观察到的操作模式 SDO、RPDO1/RPDO2 或目标 SDO 识别；未观察到时标记 `Unknown`。电流值不叠加到位置/转速纵轴。

当前只支持 `[devices.chassis].driver=udp_mcu_kinco` 的仿真链路：舵 ID 1–4、轮 ID 5–8，A/D 在 CAN1、B/C 在 CAN0。读取配置中的 `kinco_bind_*`、`kinco_peer_*`、编码器分辨率和过期阈值，校验 MCU magic、版本、CRC32、方向、gPTP trailer、CAN 接口及相关帧长度。默认监听 `127.0.0.1:45220 ↔ 127.0.0.1:45620`，不占用这两个业务端口。本模式不支持旧 Jiechang 行走驱动或原生 SocketCAN。

`ZA/ZS/WA/WS` 分别表示 Zenoh 指令/反馈和 wire 指令/反馈；前两项计数为有效整车消息数，后两项为有效逐电机样本总数。每张图内显示对应数据的缺失、过期或模式不可用原因。保留 30 s / `--time`、50 Hz 刷新、Pause 和子图 MAX 操作。

## 权限与性能

UDP 旁路监听使用 `AF_PACKET`，需要 root 或 `CAP_NET_RAW`。使用 capability 时应注意：它授予的是所选 Python 解释器，而不仅是本脚本。

网络消息持续接收和解码，收到的有效样本都会保存；GUI 每 20 ms（50 Hz）尝试刷新一次。高消息频率或机器负载较高时，Matplotlib 的实际刷新速度可能低于 50 Hz。`Pause` 只冻结绘图刷新，后台仍持续监听；关闭绘图窗口会停止监听线程并关闭 Zenoh session 和抓包 socket。

## 测试

```bash
MPLBACKEND=Agg .venv/bin/python -m unittest discover -s tools/monitor/tests -v
```
