# Sharpa MuJoCo

[English](README.md) | [简体中文](README.zh-CN.md)

Sharpa MuJoCo 是面向 **Sharpa D01** 机器人打造的高一致性仿真环境。项目基于
MuJoCo 构建，重点复现 D01 真机的硬件接口、设备生命周期、关节映射和运动控制
行为。

项目没有额外引入仅供仿真使用的控制 API，而是直接使用与真机一致的 UDP 与
CAN/CAN-FD 通信链路。因此，上层应用可以复用同一套控制栈对接仿真与真机，
从开发、联调和验证过渡到实体机器人时，通常只需切换设备端点或运行配置。

**[Sharpa Platform](third_party/sharpa_platform/README.md)** 是仿真与 D01 真机共用的
机器人运行时，上层应用可复用同一套控制接口与运行流程。本项目提供 Platform
预编译包，随仿真自动启动。

## 核心特性

- **真机协议兼容**：模拟生产环境使用的 UDP 协议及 Linux SocketCAN/CAN-FD
  接口，而不是另行定义仿真专用接口。
- **控制栈直接复用**：兼容 D01 真机使用的设备生命周期、状态反馈与指令链路。
- **覆盖主要运动控制模式**：根据不同设备支持 CSP、MIT、零力、位置和导纳等
  真机常用控制模式。
- **运动学与动力学一致性**：保留关节方向、耦合传动、软硬限位、反馈单位、
  重力、接触及标定后的执行器行为，降低 Sim-to-Real 差异。
- **全身一体化仿真**：在同一个 MuJoCo 运行时中集成躯干、双臂、双灵巧手及
  指尖力/力矩传感器。
- **灵活集成方式**：既可通过 Viewer 独立运行，也可通过 C API 嵌入已有应用和
  渲染循环。

## 系统架构

```text
上层应用 / 算法 / 遥操作
          |
 真机 UDP 或 CAN/CAN-FD
          |
  D01 仿真设备插件
          |
 共享 MuJoCo 运行时与模型
```

仿真与真机在设备协议边界上保持一致。切换运行目标时，通常只需要调整设备端点或
运行配置，上层应用与运动逻辑可以保持不变。

## 前置准备

请安装以下基础环境：

- Linux、支持 C++17 的编译器、CMake 3.22+、Git，以及支持 `venv` 的 Python 3
- [MuJoCo 3.8.1](https://github.com/google-deepmind/mujoco/releases/tag/3.8.1)

请下载并解压完整的 MuJoCo Linux 发布包，其中应包含 `include/mujoco`、`lib`
和 `simulate`。

## 快速开始

### 1. 安装依赖

在 `sharpa_mujoco` 项目根目录执行：

```bash
./scripts/install_dependencies.sh
```

从 [Zenoh 官方 1.10.0 发布页](https://github.com/eclipse-zenoh/zenoh/releases/tag/1.10.0)
下载适合主机架构的 **standalone** 二进制包：

- x86_64：`zenoh-1.10.0-x86_64-unknown-linux-gnu-standalone.zip`
- aarch64 / ARM64：`zenoh-1.10.0-aarch64-unknown-linux-gnu-standalone.zip`

解压后执行以下命令，将 `zenohd` 安装到 `~/.local/bin` 并加入当前终端的 PATH。
将 `/path/to/zenohd` 替换为解压后的实际文件路径：

```bash
install -Dm755 /path/to/zenohd "$HOME/.local/bin/zenohd"
export PATH="$HOME/.local/bin:$PATH"
```

请勿使用 Snap 包，其沙箱可能阻止读取项目配置文件。

### 2. 配置 MuJoCo

在 `sharpa_mujoco.conf` 中填写 MuJoCo 解压路径：

```ini
mujoco_root=/absolute/path/to/mujoco-3.8.1
```

`sharpa_mujoco.conf` 的完整参数说明请查看[配置参考](CONFIGURATION.zh-CN.md)。

### 3. 创建设备配置

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

### 4. 构建

```bash
./scripts/build.sh
```

### 5. 运行

启动路由器，监听 UDP 7449：

```bash
zenohd -c resources/zenoh/router.json
```

随后启动模拟器：

```bash
./scripts/run.sh
```

启动器会自动运行所需组件。关闭 Viewer 后，整个仿真环境会一并退出。

### 6. 可选：验证启动是否正确

保持模拟器运行，并在另一个终端中安装运动示例所需的 Python 依赖：

```bash
python3 -m venv .venv
source .venv/bin/activate
python -m pip install -r requirements.txt
```

运行全身动作示例：

```bash
python examples/whole_body_motion.py
```

## 许可证

Apache-2.0，详见 [LICENSE](LICENSE)。
