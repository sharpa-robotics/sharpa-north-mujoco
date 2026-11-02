// Copyright 2026 Sharpa
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace sharpa::mujoco
{

enum class RuntimeState
{
    Empty,
    Loaded,
    Running,
    Paused,
    Faulted,
};

enum class ControlMode
{
    Actuator,
    DirectJointPosition,
};

enum class SchedulerMode
{
    Realtime,
    ActionGated,
};

struct RuntimeConfig
{
    bool realtime = true;
    std::size_t max_catch_up_steps = 4;
    double snapshot_rate_hz = 60.0;
    SchedulerMode scheduler_mode = SchedulerMode::Realtime;
    std::size_t action_hold_substeps = 1;
    bool gate_on_control_command = false;
    bool adaptive_policy_pacing = true;
    double minimum_step_interval_scale = 0.75;
    double maximum_pacing_catch_up_seconds = 0.008333;
    double action_wait_timeout_seconds = 1.0;
};

struct NamedControl
{
    std::string actuator;
    double value = 0.0;
};

struct ControlCommand
{
    ControlMode mode = ControlMode::Actuator;
    bool replace_all = true;
    bool opens_action_frame = true;
    std::vector<double> ordered_values;
    std::vector<NamedControl> named_values;
};

struct Pose
{
    std::array<double, 3> position{0.0, 0.0, 0.0};
    std::array<double, 4> quaternion{1.0, 0.0, 0.0, 0.0};
};

struct BodyInfo
{
    int id = -1;
    std::string name;
    int mocap_id = -1;
};

struct JointInfo
{
    int id = -1;
    std::string name;
    int type = -1;
    int qpos_address = -1;
    int qpos_width = 0;
    int dof_address = -1;
    int dof_width = 0;
};

struct GeomInfo
{
    int id = -1;
    std::string name;
    int body_id = -1;
    int type = -1;
};

struct ActuatorInfo
{
    int id = -1;
    std::string name;
    int transmission_type = -1;
    int transmission_id = -1;
};

struct SensorInfo
{
    int id = -1;
    std::string name;
    int type = -1;
    int data_address = -1;
    int dimension = 0;
};

struct ModelInfo
{
    std::string source_path;
    double timestep = 0.0;
    int body_count = 0;
    int geom_count = 0;
    int joint_count = 0;
    int actuator_count = 0;
    int qpos_count = 0;
    int dof_count = 0;
    int sensor_count = 0;
    std::vector<BodyInfo> bodies;
    std::vector<GeomInfo> geoms;
    std::vector<JointInfo> joints;
    std::vector<ActuatorInfo> actuators;
    std::vector<SensorInfo> sensors;
};

struct PoseSnapshot
{
    std::uint64_t version = 0;
    std::uint64_t step_index = 0;
    double simulation_time = 0.0;
    int body_count = 0;
    std::vector<double> body_positions;
    std::vector<double> body_quaternions;
    std::vector<double> body_velocities;
    int geom_count = 0;
    std::vector<double> geom_positions;
    std::vector<double> geom_quaternions;
    std::vector<double> qpos;
    std::vector<double> qvel;
    std::vector<double> controls;
    std::vector<double> actuator_forces;
    std::vector<double> sensor_values;

    bool valid() const noexcept
    {
        return body_count > 0 &&
               body_positions.size() >= static_cast<std::size_t>(body_count * 3) &&
               body_quaternions.size() >= static_cast<std::size_t>(body_count * 4);
    }
};

}  // namespace sharpa::mujoco
