# Control the simulation with North Pilot

[English](north-pilot.md) | [简体中文](north-pilot.zh-CN.md)

Download and install North Pilot, then open it to get started.
The app and simulation run on the same computer.

## 1. Download and install

In Releases of [sharpa-north-app](https://github.com/sharpa-robotics/sharpa-north-app),
download the **production** installer for your operating system and CPU architecture.
Install the package and open **North Pilot**.

## 2. Connect to the simulation

Follow the [README quick start](../README.md#quick-start) to start the router and simulation.
Finish motion tasks from SDK examples or other controllers.

1. In North Pilot, select **Connect manually**, or **Direct connect** from the device list.
2. Enter **`127.0.0.1`** and connect.
3. Confirm that device information, state, and joint feedback are available.

When finished, stop the motion task, then close North Pilot and the Viewer.
Press Ctrl+C in the router terminal.

For programmatic control, see the [SDK guide](sdk.md).
