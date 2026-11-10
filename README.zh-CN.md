# sharpa-north-mujoco

[English](README.md) | [简体中文](README.zh-CN.md)

sharpa-north-mujoco 是面向 **Sharpa D01** 的仿真环境，基于 MuJoCo 模拟躯干、双臂和
双灵巧手，提供兼容真机的 UDP 与 CAN/CAN-FD 设备接口。

Mujoco仿真、基于Mujoco的虚幻5渲染和Sharpa North App效果展示：

https://github.com/user-attachments/assets/a71ac2f2-62c3-483b-a34a-ec6c08877c1b

## 核心特性

- **真机接口兼容**：模拟 D01 的 UDP 与 CAN/CAN-FD 通信、设备状态和关节指令。
- **全身仿真**：在同一 MuJoCo 模型中运行躯干、双臂和双灵巧手。
- **两种控制入口**：通过 North SDK 编程控制，或使用 Sharpa North App 图形界面。

## 系统架构

```text
C++ / Python 应用            Sharpa North App
        │                          │
    North SDK                      │
        └────────────┬─────────────┘
                   Zenoh
                     │
              Sharpa Platform
                     │
              UDP / CAN / CAN-FD
                     │
               D01 仿真设备插件
                     │
              MuJoCo 运行时与模型
```

SDK 和 Sharpa North App 通过 Zenoh 与 Platform 通信；Platform 通过设备协议控制仿真。
设备插件将协议指令映射到 MuJoCo 模型，并返回状态反馈。

## 快速开始

支持 Linux x86_64 和 aarch64。先克隆
[sharpa-north-mujoco](https://github.com/sharpa-robotics/sharpa-north-mujoco)
仓库，再在仓库根目录执行后续命令：

```bash
git clone https://github.com/sharpa-robotics/sharpa-north-mujoco.git
cd sharpa-north-mujoco
```

### 1. 准备环境

请安装以下基础环境：

- Linux x86_64 或 aarch64、C++17 编译器、CMake 3.22+、Git、Python 3 和 `venv`。
- 图形桌面，用于运行 Viewer。
- 显卡驱动需支持桌面 OpenGL 1.5+ 兼容功能及 `GL_ARB_framebuffer_object`、`GL_ARB_vertex_buffer_object`。
  [图形要求与已验证环境](CONFIGURATION.zh-CN.md#图形环境要求)。
- 完整的 [MuJoCo 3.8.1 Linux 发布包](https://github.com/google-deepmind/mujoco/releases/tag/3.8.1)，
  解压后包含 `include/mujoco`、`lib` 和 `simulate`。

Ubuntu/Debian 安装基础工具和项目依赖：

```bash
sudo apt-get update
sudo apt-get install build-essential cmake curl python3 python3-venv
./scripts/install_dependencies.sh
```

依赖脚本会安装 GLFW、Git LFS，并下载 Platform 运行时。

机器人模型资源见官方资产仓库 [sharpa-urdf-usd-xml](https://github.com/sharpa-robotics/sharpa-urdf-usd-xml)。本项目当前使用 D01-W HA1.2 precise 模型。

安装依赖后、构建之前，拉取固定版本的机器人资产和 North SDK：

```bash
./scripts/update_submodules.sh --pinned
```

从 [Zenoh 1.10.0 发布页](https://github.com/eclipse-zenoh/zenoh/releases/tag/1.10.0)
下载适合主机架构的 **standalone** 包：

- x86_64：`zenoh-1.10.0-x86_64-unknown-linux-gnu-standalone.zip`
- aarch64 / ARM64：`zenoh-1.10.0-aarch64-unknown-linux-gnu-standalone.zip`

解压后安装 `zenohd`，将 `/path/to/zenohd` 替换为实际路径：

```bash
install -Dm755 /path/to/zenohd "$HOME/.local/bin/zenohd"
export PATH="$HOME/.local/bin:$PATH"
zenohd --version
```

请勿使用 Snap 包，其沙箱可能阻止读取项目配置文件。

### 2. 配置 MuJoCo 和仿真设备

在 `sharpa_mujoco.conf` 中填写 MuJoCo 解压路径：

```ini
mujoco_root=/absolute/path/to/mujoco-3.8.1
```

完整参数说明见[配置参考](CONFIGURATION.zh-CN.md)。

首次运行前创建 `/userapp/sharpa/device.conf`：

```bash
sudo mkdir -p /userapp/sharpa
sudo tee /userapp/sharpa/device.conf > /dev/null <<'EOF'
[device]
sn = SIM001
model = D01W
batch = HA12
EOF
```

### 3. 构建

```bash
./scripts/build.sh
```

默认在 `build-linux` 中构建。重复执行会复用该目录，增量更新编译产物。

### 4. 启动路由器和仿真

终端 A，启动路由器，监听 UDP 7449：

```bash
export PATH="$HOME/.local/bin:$PATH"
zenohd -c resources/zenoh/router.json
```

保持终端 A 运行，在终端 B 启动模拟器：

```bash
./scripts/run.sh
```

启动器会检查环境，按需提示准备虚拟 CAN 接口，并自动启动 Platform 和 Viewer。
图形环境不可用时，会报告错误并退出。

### 5. 可选：验证启动是否正确

保持路由器和模拟器运行，在终端 C 的仓库根目录安装示例依赖：

```bash
python3 -m venv .venv
source .venv/bin/activate
python -m pip install -r requirements.txt
```

运行全身动作示例：

```bash
python examples/whole_body_motion.py
```

机器人会下蹲、抬臂并竖拇指，保持姿态直到按 Ctrl+C，随后回到起始姿态并退出。

至此，仿真环境已准备就绪。如需进一步控制，可选择下方的 North SDK 或 Sharpa North App。

## 控制仿真

### North SDK

通过官方 **C++ 或 Python API** 读取状态和控制运动，适合算法开发与程序联调。
本仓库提供状态读取、全身运动和 Home 示例。

官方项目：[sharpa-north-sdk](https://github.com/sharpa-robotics/sharpa-north-sdk)。

→ [使用 SDK 连接仿真](docs/sdk.zh-CN.md)

### Sharpa North App

通过桌面图形界面连接仿真、查看设备和关节状态，以及执行 Home，适合交互操作。
下载并安装 `production` 版本后即可使用。

官方项目：[sharpa-north-app](https://github.com/sharpa-robotics/sharpa-north-app)。

→ [使用 Sharpa North App 连接仿真](docs/sharpanorth-app.zh-CN.md)

同一时间只使用一个运动指令来源。切换或退出前，先结束当前控制任务。
关闭 Viewer 会一并退出 Platform；路由器需要在其终端按 Ctrl+C 停止。

## 仿真环境配置与启动常见问题

| 现象 | 处理方法 |
| --- | --- |
| 缺少 Platform 包 | 检查 Git LFS 访问权限，重新执行 `./scripts/install_dependencies.sh`。 |
| 缺少机器人资产，或提示 `Git LFS content not downloaded` / `Skipping object checkout, Git LFS is not installed` | 执行 `./scripts/update_submodules.sh --pinned`。除了安装 Git LFS 程序，还需在资产子模块中配置 LFS，脚本会自动完成。 |
| 启动提示 `sharpa_mujoco.conf:行号: expected key=value` | 检查报错行：配置项使用 `key=value`，注释以 `#` 开头；删除误输入的字符，例如将 `1#` 改为 `#`。修正后重新运行，无需重新构建。 |
| 找不到 MuJoCo | 将 `mujoco_root` 设置为完整发布包的解压目录；若设置了环境变量 `MUJOCO_ROOT`，检查其路径。修正后重新执行 `./scripts/build.sh`，会自动刷新 MuJoCo 路径缓存。 |
| 启动提示 `Simulator not found or not executable` | 先执行 `./scripts/build.sh` 并确认构建成功，再执行 `./scripts/run.sh`。默认使用同一个构建目录。 |
| 窗口创建失败或提示 GLX 错误 | 检查图形桌面和显卡驱动。NVIDIA 用户可运行 `nvidia-smi`；若提示 `Driver/library version mismatch`，保存工作并重启电脑。图形检查失败时不会启动设备或 Platform。 |
| 提示 `wave_tactile` 无法绑定 59998 端口 | 确认没有另一个仿真实例运行。启动器会在设备绑定端口前回收上次异常退出留下的 Platform 进程。 |
| 缺少虚拟 CAN 接口 | 执行 `sudo bash scripts/setup_vcan.sh vcan0 vcan1`。 |
| Platform 无法选择设备配置 | 检查 `device.conf` 中的型号和批次为 `D01W` / `HA12`。 |
| 7449 端口被占用 | 检查已有路由器，使用本仓库的 `resources/zenoh/router.json` 配置。 |
| Viewer 打开但控制端没有状态 | 检查 Platform 启动输出和路由器配置。 |

## UE 可视化体验版

我们还提供基于 Unreal Engine（UE）渲染的可视化仿真**体验版**，底层使用 `sharpa-north-mujoco`。用户也可基于本仓库适配自己的渲染器。

[下载 Linux 体验版（AvatarSimWorld）](https://packages.sharpa.com/repository/raw-releases/apps/sim/AvatarSimWorld-Linux.tar.zst)

## 更多文档

- [仿真配置参考](CONFIGURATION.zh-CN.md)
- [Platform 运行时说明](third_party/sharpa_platform/README.md)

## 许可证

[Apache-2.0](LICENSE)。随附 Platform 运行时的许可见其[分发说明](third_party/sharpa_platform/README.md)。
