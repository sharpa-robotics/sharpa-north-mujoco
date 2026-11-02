// Copyright 2026 Sharpa
// SPDX-License-Identifier: Apache-2.0
#pragma once

#include "sharpa_mujoco/device_plugin.h"
#include "sharpa_mujoco/plugins/wave_hand/wave_protocol.h"

#include <array>
#include <cstdint>
#include <memory>
#include <string>

namespace sharpa::mujoco::plugins::wave_hand
{

struct WaveHandPluginConfig
{
    std::string name = "wave_hand";
    WaveHandSide side = WaveHandSide::kRight;
    std::string device_ip = "127.0.0.1";
    std::string sdk_ip = "127.0.0.1";
    std::string serial = "WAVE-HAND";
    std::string PN = "WAVE-HAND";
    std::array<std::uint8_t, 6> MAC{};

    std::uint16_t tcp_port = 2333;
    std::uint16_t command_port = 50010;
    // Zero selects the side-specific standard port (left 50000, right 50010).
    std::uint16_t feedback_destination = 0;
    std::uint16_t heartbeat_destination = 54321;
    std::uint16_t feedback_source_port = 10000;

    double feedback_rate_hz = 500.0;
    double heartbeat_rate_hz = 1.0;
    bool fingertip_tactile_supported = false;
    // Optionally validate the duplicated source/mode fields in UDP position
    // packet headers. Lifecycle, fault, side, and POSITION checks stay active.
    bool strict_command_state = false;
    std::array<float, kWaveJointCount> joint_signs = [] {
        std::array<float, kWaveJointCount> values{};
        values.fill(1.0F);
        return values;
    }();
    std::array<float, kWaveJointCount> joint_offsets{};
};

class WaveHandPlugin final : public IDevicePlugin
{
public:
    explicit WaveHandPlugin(WaveHandPluginConfig config);
    ~WaveHandPlugin() override;

    std::string name() const override;
    bool start(Runtime& runtime, std::string* error = nullptr) override;
    void stop() noexcept override;
    bool is_running() const noexcept override;

    // Test-only local fault injection. This does not add a network command.
    bool inject_fault(std::uint16_t code,
                      std::uint32_t joint_mask,
                      std::string* error = nullptr);

private:
    class Impl;
    std::unique_ptr<Impl> impl_;
};

}  // namespace sharpa::mujoco::plugins::wave_hand
