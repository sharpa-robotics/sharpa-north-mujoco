// Copyright 2026 Sharpa
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include "sharpa_mujoco/device_plugin.h"

#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace sharpa::mujoco::plugins::lidar
{

enum class LidarModel
{
    Ftx140,
    Jt16,
};

struct LidarInstanceConfig
{
    std::string name;
    LidarModel model = LidarModel::Ftx140;
    std::string site_name;
    std::string bind_ip = "127.0.0.1";
    std::uint16_t bind_port = 0;
    std::string peer_ip = "127.0.0.1";
    std::uint16_t peer_port = 0;
    std::string calibration_file;
    std::vector<std::string> raycast_exclude_meshes;
    std::size_t raycast_threads = 0;
    double scan_rate_hz = 10.0;
    double min_range_m = 0.1;
    double max_range_m = 60.0;
};

struct LidarPluginConfig
{
    std::vector<LidarInstanceConfig> instances;
};

class LidarPlugin final : public IDevicePlugin
{
public:
    explicit LidarPlugin(LidarPluginConfig config);
    ~LidarPlugin() override;

    LidarPlugin(const LidarPlugin&) = delete;
    LidarPlugin& operator=(const LidarPlugin&) = delete;

    std::string name() const override;
    bool start(Runtime& runtime, std::string* error = nullptr) override;
    void stop() noexcept override;
    bool is_running() const noexcept override;

private:
    class Impl;
    std::unique_ptr<Impl> impl_;
};

}  // namespace sharpa::mujoco::plugins::lidar
