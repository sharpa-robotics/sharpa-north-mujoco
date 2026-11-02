# LiDAR 仿真插件

[English](README.md) | [简体中文](README.zh-CN.md)

插件从三个配置的传感器 site 对 MuJoCo 模型进行采样，并发送兼容 FTX140 或
JT16 的 UDP 数据报。FTX140 直接使用原始 UDP 报文；JT16 则封装在 POC2
传感器上行报文中。

随附的标定文件通过 `scripts/sync_lidar_calibration.py` 从 `sharpa_platform`
同步。JT16 标定文件直接复制；FTX140 从平台的 384×512 校正网格中抽取偶数行和
偶数列，以匹配仿真器的 192×256 报文网格。所有挂载位姿、网络端点和测距范围
均由 `sharpa_mujoco.conf` 配置。

JT16 扫描方向采用与 `sharpa_platform` JT16 解码器相同的方位角到 XYZ 坐标
转换约定，避免仿真点云与解码后的点云互为镜像。

## UDP 点云可视化

`sharpa_mujoco_lidar_udp_visualization_test` 是独立的 UDP 接收程序，需要与
模拟器同时运行。窗口左上为胸前 FTX140 点云（红色），右上为胸后 FTX140
点云（绿色），左下为底盘 JT16 点云（蓝色），右下为测试场景中静态障碍物的
俯视参考图。界面显示接收帧率和有效点数，按 `Esc` 或关闭窗口退出。

以下命令均在项目根目录执行，使用默认构建目录 `build-linux`。

### 1. 构建与配置

需要 Linux 图形桌面、GLFW 和 OpenGL。首次环境准备见
[项目 README](../../README.zh-CN.md)。使用默认构建命令：

```bash
./scripts/build.sh
```

该程序需要 `SHARPA_MUJOCO_BUILD_TESTS=ON` 和
`SHARPA_MUJOCO_BUILD_VIEWER=ON`，因此构建时不要使用 `--no-tests` 或
`--no-viewer`。它是交互程序，不会由 CTest 自动打开。

在 [主配置](../../sharpa_mujoco.conf) 的以下三个已有段落中，将 `enabled`
设为 `true`，保留各段的驱动、挂载、标定和网络参数：

```ini
[devices.chest_lidar_front]
enabled=true

[devices.chest_lidar_rear]
enabled=true

[devices.chassis_lidar_front]
enabled=true
```

模拟器和可视化程序都读取这份配置。可视化程序只接受 `--config` 和可选的
`--model`，不支持 `--enable-device`；三路雷达必须在配置文件中全部启用。

### 2. 启动模拟器与可视化窗口

确保 Zenoh 路由器已运行；若尚未运行，在一个终端执行：

```bash
zenohd -c resources/zenoh/router.json
```

在另一个终端启动模拟器，加载带墙面、台阶及圆柱的雷达测试场景：

```bash
./scripts/run.sh --config ./sharpa_mujoco.conf \
  --model ./tests/lidar/lidar_test_scene.xml
```

保持模拟器运行，再在一个终端打开实时点云界面：

```bash
./build-linux/tests/lidar/sharpa_mujoco_lidar_udp_visualization_test \
  --config ./sharpa_mujoco.conf \
  --model ./tests/lidar/lidar_test_scene.xml
```

两端应使用相同配置和场景。若要使用配置文件中的默认场景，同时省略两条命令的
`--model` 参数即可。右下参考图只显示静态障碍物，不同步机器人实时姿态。

### 3. 联调与排查

- 接收程序绑定三路雷达的 `peer_ip:peer_port`，当前默认分别为
  `127.0.0.1:9180`、`127.0.0.1:9181`、`127.0.0.1:9280`。
  同一组端口一次只运行一个点云接收程序，避免端口冲突或数据被其他接收者取走。
- 窗口无点云或帧率为零时，检查模拟器是否运行、三路雷达是否全部启用，
  以及两端的配置、标定文件和 UDP 端点是否一致。
- 若同时运行 `examples/motion/chassis_motion_driver.py`，保留主配置中的
  `[devices.chassis]` 完整参数并确保 `enabled=true`，同时保持 Platform 和
  Zenoh 路由器运行。`tests/data/lidar_enabled.conf` 是雷达专项测试配置，
  不含底盘驱动配置；`--config` 不会将它与主配置合并。用它启动模拟器无法为
  底盘 motion 提供所需的电机反馈，会出现 `No state/chassis feedback`。

## 集成待办

- [ ] 使用经过验证的 CAD 或标定值，替换暂定的 FTX140 光心外参和 JT16 模拟挂载参数。
- [ ] 确认 FTX140 官方标称测距范围；在此之前使用可配置的默认范围 0.1–60.0 m。
- [ ] 若目标硬件使用定制固件，使用真实抓包验证 FTX140 报文布局；否则保留 F04 标准布局。
