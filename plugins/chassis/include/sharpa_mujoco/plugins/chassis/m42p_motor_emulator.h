// Copyright 2026 Sharpa
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <cstdint>
#include <optional>
#include <vector>

namespace sharpa::mujoco::plugins::chassis
{

class M42pMotorEmulator
{
public:
    explicit M42pMotorEmulator(std::uint8_t slave = 1);

    std::uint8_t slave() const noexcept;
    std::int32_t position_steps() const noexcept;
    std::int32_t target_steps() const noexcept;
    float speed_rpm() const noexcept;
    std::uint16_t status() const noexcept;
    bool moving() const noexcept;

    std::optional<std::vector<std::uint8_t>> process(
        const std::uint8_t* request,
        std::size_t size);
    void advance(double seconds);
    void set_position_steps(std::int32_t steps) noexcept;
    void set_limits(std::int32_t minimum, std::int32_t maximum) noexcept;

private:
    enum class Motion
    {
        Idle,
        Position,
        Forward,
        Reverse,
        Home,
    };

    std::uint16_t read_register(std::uint16_t address) const noexcept;
    bool write_register(std::uint16_t address, std::uint16_t value);
    bool write_registers(
        std::uint16_t address,
        const std::vector<std::uint16_t>& values);
    void set_alarm(std::uint8_t alarm) noexcept;

    std::uint8_t slave_ = 1;
    std::int32_t position_steps_ = 0;
    std::int32_t target_steps_ = 0;
    float speed_rpm_ = 0.0F;
    std::uint8_t alarm_ = 0;
    std::uint16_t subdiv_ = 32;
    std::int32_t minimum_steps_ = 0;
    std::int32_t maximum_steps_ = 19200;
    Motion motion_ = Motion::Idle;
};

}  // namespace sharpa::mujoco::plugins::chassis
