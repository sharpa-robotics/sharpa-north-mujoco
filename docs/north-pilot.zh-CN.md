# 使用 North Pilot 控制仿真

[English](north-pilot.md) | [简体中文](north-pilot.zh-CN.md)

North Pilot 下载并安装后即可使用。上位机与仿真运行在同一台电脑。

## 1. 下载并安装

在 [sharpa-north-app](https://github.com/sharpa-robotics/sharpa-north-app) 的 Releases 中，
下载适合本机操作系统和 CPU 架构的 **production** 安装包，完成安装后打开 **North Pilot**。

## 2. 连接仿真

先按 [README「快速开始」](../README.zh-CN.md#快速开始)启动路由器和仿真，
并结束 SDK 示例或其他控制端的运动任务。

1. 在 North Pilot 中选择**手动连接**或设备列表中的**直连**。
2. 输入 **`127.0.0.1`** 并连接。
3. 确认设备信息、状态和关节反馈正常。

结束使用时，先结束运动任务，再关闭 North Pilot 和 Viewer；在路由器终端按 Ctrl+C。

编程控制见 [SDK 教程](sdk.zh-CN.md)。
