// Copyright 2026 Sharpa
// SPDX-License-Identifier: Apache-2.0
#include "sharpa_mujoco/plugins/tactile/tactile_protocol.h"

#include <algorithm>
#include <cmath>
#include <cstring>

namespace sharpa::mujoco::plugins::tactile
{
namespace
{

constexpr std::size_t kStreamHeaderSize = 16;
constexpr std::size_t kContentHeaderSize = 24;
constexpr std::size_t kBlockHeaderSize = 8;
constexpr std::size_t kTailSize = 4;
constexpr std::size_t kF6ContentSize =
    kContentHeaderSize + kBlockHeaderSize + sizeof(float) * kF6ValueCount;
constexpr std::uint32_t kF6ContentFlag = 1U << 1U;

std::uint32_t ieee_crc32(const std::uint8_t* data, std::size_t size)
{
    std::uint32_t crc = 0xFFFFFFFFU;
    for (std::size_t index = 0; index < size; ++index)
    {
        crc ^= data[index];
        for (int bit = 0; bit < 8; ++bit)
            crc = (crc >> 1U) ^
                  (0xEDB88320U & (0U - (crc & 1U)));
    }
    return ~crc;
}

void write_u16_le(std::uint8_t* data, std::uint16_t value)
{
    data[0] = static_cast<std::uint8_t>(value);
    data[1] = static_cast<std::uint8_t>(value >> 8U);
}

void write_u32_le(std::uint8_t* data, std::uint32_t value)
{
    for (unsigned index = 0; index < 4; ++index)
        data[index] = static_cast<std::uint8_t>(value >> (8U * index));
}

std::uint16_t read_u16_le(const std::uint8_t* data)
{
    return static_cast<std::uint16_t>(data[0]) |
           static_cast<std::uint16_t>(data[1]) << 8U;
}

std::uint32_t read_u32_le(const std::uint8_t* data)
{
    return static_cast<std::uint32_t>(data[0]) |
           static_cast<std::uint32_t>(data[1]) << 8U |
           static_cast<std::uint32_t>(data[2]) << 16U |
           static_cast<std::uint32_t>(data[3]) << 24U;
}

template <typename Value>
void write_scalar(std::uint8_t* data, Value value)
{
    static_assert(sizeof(Value) == 4 || sizeof(Value) == 8);
    std::memcpy(data, &value, sizeof(Value));
}

template <typename Value>
Value read_scalar(const std::uint8_t* data)
{
    Value value{};
    std::memcpy(&value, data, sizeof(Value));
    return value;
}

bool finite(const F6Frame& frame)
{
    return std::isfinite(frame.timestamp) &&
           std::all_of(frame.values.begin(), frame.values.end(),
                       [](float value) { return std::isfinite(value); });
}

}  // namespace

bool encode_f6_frame(
    const F6Frame& frame,
    std::array<std::uint8_t, kF6PacketSize>& output) noexcept
{
    if (frame.channel > 9 || !finite(frame)) return false;
    output.fill(0);

    output[0] = 0xBB;
    output[1] = 0xEE;
    output[2] = 0x40;
    output[4] = 0;
    output[5] = 2;
    write_u16_le(output.data() + 6, frame.channel);
    write_u16_le(output.data() + 8, frame.frame_id);
    write_u16_le(output.data() + 10, 1);
    write_u16_le(output.data() + 12, 0);

    std::uint8_t* payload = output.data() + kStreamHeaderSize;
    payload[0] = 0x40;
    write_u16_le(payload + 2, frame.frame_id);
    write_u32_le(payload + 4, kF6ContentFlag);
    write_scalar(payload + 8, frame.timestamp);

    std::uint8_t* block = payload + kContentHeaderSize;
    write_u16_le(block, 1);
    write_u16_le(block + 2, 6);
    block[4] = 1;
    block[7] = 5;
    std::memcpy(block + kBlockHeaderSize, frame.values.data(),
                sizeof(float) * frame.values.size());

    // Current HB1 firmware uses the protocol's 0x0000FFFF sentinel to disable
    // stream CRC validation. SDK 5.0.5 recognizes this exact wire value.
    write_u32_le(output.data() + kF6PacketSize - kTailSize, 0x0000FFFFU);
    return true;
}

bool decode_f6_frame(
    const std::uint8_t* data,
    std::size_t size,
    F6Frame& frame) noexcept
{
    if (!data || size != kF6PacketSize || data[0] != 0xBB ||
        data[1] != 0xEE || data[2] != 0x40 || data[4] != 0 ||
        data[5] != 2 || read_u16_le(data + 10) != 1 ||
        read_u16_le(data + 12) != 0)
        return false;

    const std::uint8_t* payload = data + kStreamHeaderSize;
    const std::size_t payload_size =
        kF6PacketSize - kStreamHeaderSize - kTailSize;
    const std::uint32_t wire_crc =
        read_u32_le(data + kF6PacketSize - kTailSize);
    if ((wire_crc != 0x0000FFFFU &&
         wire_crc != ieee_crc32(payload, payload_size)) ||
        payload[0] != 0x40 || read_u32_le(payload + 4) != kF6ContentFlag)
        return false;

    if (!std::all_of(payload + kF6ContentSize, payload + payload_size,
                     [](std::uint8_t value) { return value == 0; }))
        return false;

    const std::uint8_t* block = payload + kContentHeaderSize;
    if (read_u16_le(block) != 1 || read_u16_le(block + 2) != 6 ||
        block[4] != 1 || block[5] != 0 || block[6] != 0 || block[7] != 5)
        return false;

    F6Frame decoded;
    decoded.channel = read_u16_le(data + 6);
    decoded.frame_id = read_u16_le(data + 8);
    if (decoded.channel > 9 || read_u16_le(payload + 2) != decoded.frame_id)
        return false;
    decoded.timestamp = read_scalar<double>(payload + 8);
    std::memcpy(decoded.values.data(), block + kBlockHeaderSize,
                sizeof(float) * decoded.values.size());
    if (!finite(decoded)) return false;
    frame = decoded;
    return true;
}

}  // namespace sharpa::mujoco::plugins::tactile
