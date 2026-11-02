// Copyright 2026 Sharpa
// SPDX-License-Identifier: Apache-2.0
#pragma once

#include "sharpa_mujoco/device_plugin.h"

#include <array>
#include <cstdint>
#include <memory>
#include <string>

namespace sharpa::mujoco::plugins::tactile
{

struct TactilePluginConfig
{
    std::string name = "wave_tactile";
    std::string sdk_ip = "127.0.0.1";
    std::string right_device_ip = "127.0.50.20";
    std::string left_device_ip = "127.0.50.10";
    std::uint16_t config_port = 59998;
    std::uint16_t right_destination_port = 50001;
    std::uint16_t left_destination_port = 50011;
    double rate_hz = 30.0;
};

class TactilePlugin final : public IDevicePlugin
{
public:
    explicit TactilePlugin(TactilePluginConfig config);
    ~TactilePlugin() override;

    std::string name() const override;
    bool start(Runtime& runtime, std::string* error = nullptr) override;
    void stop() noexcept override;
    bool is_running() const noexcept override;

private:
    class Impl;
    std::unique_ptr<Impl> impl_;
};

}  // namespace sharpa::mujoco::plugins::tactile
