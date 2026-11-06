# 使用 North SDK 控制仿真

[English](sdk.md) | [简体中文](sdk.zh-CN.md) · [返回 README](../README.zh-CN.md)

使用官方 [North SDK](https://github.com/sharpa-robotics/sharpa-north-sdk) 的 C++ / Python API
读取仿真状态、执行全身动作和 Home 任务。以下命令均在 `sharpa-north-mujoco` 仓库根目录执行。

## 适用范围与版本

SDK 与仿真运行在同一台电脑，使用随附的 `sim_north` Profile 和 `D01W / HA12` 设备配置。
SDK 预编译运行时面向 Linux x86_64（Ubuntu 22.04 构建），包含 CPython 3.10 和 3.12 绑定。

| 组件 | 版本 |
| --- | --- |
| MuJoCo | 3.8.1 |
| 随附 Platform，Linux x86_64 | 1.7.10，`868484dbc138` |
| North SDK | 1.2.0，子模块提交 `33306ab4fda789b8c3fa8a48662fc4c2a3a73180` |
| Python 绑定 | CPython 3.10 / 3.12，Linux x86_64 |
| Zenoh 路由器 | 1.10.0 |

随附 Platform 的版本见[运行时清单](../third_party/sharpa_platform/linux-x86_64/sharpa-platform.manifest.json)。

## 1. 获取 SDK 子模块

```bash
./scripts/update_submodules.sh
```

默认初始化模型和 SDK，并使用父仓库记录的固定提交；`--pinned` 与默认行为相同。
只获取 SDK 可执行 `./scripts/update_submodules.sh --only sdk`。
SDK 位于 `third_party/north_sdk`，拉取后即可使用，无需解压。
脚本会校验 SDK 目录、版本清单和 SHA-256；不会覆盖所选子模块中的本地修改。

需要主动升级时执行 `./scripts/update_submodules.sh --remote`，或加 `--only sdk` 仅升级 SDK。
升级后须重新验证示例并提交父仓库中的子模块指针；上表描述的是当前固定版本。
私有仓库需要 GitHub 访问权限；HTTPS 或 SSH 认证按本机 Git 配置使用。

## 2. 加载 SDK 并检查安装

选择本机已安装且与绑定匹配的 Python（也可用 `python3.12`）：

```bash
export SDK_PYTHON=python3.10
source third_party/north_sdk/setup.bash
PYTHON="$SDK_PYTHON" ./third_party/north_sdk/check.sh
```

`check.sh` 仅检查本机库加载和 Python 导入，不连接机器人。
每个运行示例的新终端都需加载 `setup.bash`，设置 `NORTHSDK_PREFIX`、`PYTHONPATH` 和动态库路径。
仅检查 C++ 库可使用 `./third_party/north_sdk/check.sh --cpp-only`。

## 3. 启动仿真并读取状态

按[README「快速开始」](../README.zh-CN.md#快速开始)启动路由器和仿真，保持两个终端运行。
Platform 随模拟器自动启动。示例使用本机连接地址，关闭本机仿真不需要的主机对时：

```text
udp/127.0.0.1:7449?rel=1;multistream=1;mixed_rel=0
```

使用 C++17 编译器和 CMake 3.16+ 构建本仓库示例：

```bash
cmake -S examples/sdk -B build-sdk -DCMAKE_PREFIX_PATH="$NORTHSDK_PREFIX"
cmake --build build-sdk -j2
./build-sdk/sdk_read_state
```

只读示例输出 Master 阶段、模式、任务 ID 和左臂关节位置（弧度）。
源码：[read_state.cpp](../examples/sdk/read_state.cpp)。

## 4. 执行 Python 全身动作

先结束其他控制端的运动任务，确认 Master 为 `STANDBY`。
如果普通 Python 运动示例还在运行，先按 Ctrl+C，等待回位退出。
直接运行动作：

```bash
"$SDK_PYTHON" examples/sdk/whole_body_motion.py
```

动作与普通 Python 示例一致：下蹲并抬臂，再竖拇指，默认保持至 Ctrl+C。
首次 Ctrl+C 会平滑回到启动时捕获的姿态，确认回位后停止本示例启动的 inference 任务。
定时保持结束也会回位；再次 Ctrl+C 会中止回位并清理任务。
失联、反馈过期或任务状态改变等异常不会尝试回位。

| 选项 | 默认值与含义 |
| --- | --- |
| `--rate` | 50 Hz；旧名称 `--hz` 仍可用 |
| `--transition-duration` | 每段过渡至少 4 秒；旧名称 `--move-duration` 仍可用，必要时按帧间步长上限延长 |
| `--hold-duration` | 0，保持至 Ctrl+C；正数表示保持秒数 |
| `--side` | `both`；也可选 `left` / `right` |
| `--arms-only` | 仅控制所选手臂 |
| `--return-timeout` | 回位轨迹结束后等待反馈到位，默认 8 秒 |
| `--tolerance` | 回位容差，默认 1° |
| `--output` | 可选 JSON 报告路径，记录动作阶段、中断、回位、任务清理与验证结果 |

例如有限时长运行并记录报告：

```bash
"$SDK_PYTHON" examples/sdk/whole_body_motion.py --hold-duration 2 --output /tmp/sdk-motion.json
```

SDK 关节位置单位为弧度；默认直接启动任务并运动，与普通 Python 示例一致。
只读取状态、不运动时，显式添加 `--preview`。
源码：[whole_body_motion.py](../examples/sdk/whole_body_motion.py)。

## 5. 可选：通过 C++ 调用 Home

```bash
./build-sdk/sdk_home
```

默认直接执行任务，将双臂和躯干移动到 Platform 配置的 Home 姿态并停留。
如需只读检查，可使用 `./build-sdk/sdk_home --preview`。
观察反馈和 Viewer 确认到位，等待 Master 回到 `STANDBY` 后再操作。
源码：[home.cpp](../examples/sdk/home.cpp)。

## 常见问题

| 现象 | 处理方法 |
| --- | --- |
| SDK 缺失或文件校验失败 | 检查子模块本地修改，执行 `./scripts/update_submodules.sh --only sdk`；确认 GitHub 访问权限。 |
| CMake 找不到 NorthSDK | 加载 `setup.bash`，检查 `NORTHSDK_PREFIX` 指向 `third_party/north_sdk`。 |
| 无法导入 `north_sdk` | 使用 Python 3.10 或 3.12，并在当前终端加载 `setup.bash`。 |
| 加载 `.so` 提示缺少动态库 | 加载 `setup.bash`，运行 `check.sh` 查看缺失项。 |
| 连接失败或没有状态 | 检查路由器、Platform、7449 端口和可靠 UDP 参数。 |
| 运动被拒绝或任务异常 | 检查 Master 状态和 Platform 日志，在原控制端结束已有任务。 |

完整 API 见 [SDK 中文手册](../third_party/north_sdk/manual/zh/SDK/应用开发.md)。
