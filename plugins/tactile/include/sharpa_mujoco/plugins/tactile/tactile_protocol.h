// Copyright 2026 Sharpa
// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

namespace sharpa::mujoco::plugins::tactile
{

constexpr std::size_t kF6ValueCount = 6;
// SDK 5.0.5 parses the full fixed-size receive buffer, so even an F6-only
// frame must occupy one complete tactile UDP datagram.
constexpr std::size_t kF6PacketSize = 1472;

struct F6Frame
{
    std::uint16_t channel = 0;
    std::uint16_t frame_id = 0;
    double timestamp = 0.0;
    std::array<float, kF6ValueCount> values{};
};

bool encode_f6_frame(
    const F6Frame& frame,
    std::array<std::uint8_t, kF6PacketSize>& output) noexcept;

bool decode_f6_frame(
    const std::uint8_t* data,
    std::size_t size,
    F6Frame& frame) noexcept;

}  // namespace sharpa::mujoco::plugins::tactile
