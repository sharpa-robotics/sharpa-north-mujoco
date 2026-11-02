// Copyright 2026 Sharpa
// SPDX-License-Identifier: Apache-2.0

#include "sharpa_mujoco/plugins/chassis/m42p_motor_emulator.h"

#include "sharpa_mujoco/plugins/common/mcu_udp_codec.h"

#include <algorithm>
#include <cmath>
#include <cstring>

namespace sharpa::mujoco::plugins::chassis
{
namespace
{

std::vector<std::uint8_t> with_crc(std::vector<std::uint8_t> frame)
{
    const std::uint16_t crc =
        common::modbus_crc16(frame.data(), frame.size());
    frame.push_back(static_cast<std::uint8_t>(crc));
    frame.push_back(static_cast<std::uint8_t>(crc >> 8U));
    return frame;
}

std::uint32_t registers_to_u32(
    const std::vector<std::uint16_t>& values)
{
    return (static_cast<std::uint32_t>(values[0]) << 16U) | values[1];
}

}  // namespace

M42pMotorEmulator::M42pMotorEmulator(std::uint8_t slave) : slave_(slave) {}
std::uint8_t M42pMotorEmulator::slave() const noexcept { return slave_; }
std::int32_t M42pMotorEmulator::position_steps() const noexcept
{
    return position_steps_;
}
std::int32_t M42pMotorEmulator::target_steps() const noexcept
{
    return target_steps_;
}
float M42pMotorEmulator::speed_rpm() const noexcept { return speed_rpm_; }
bool M42pMotorEmulator::moving() const noexcept
{
    return motion_ != Motion::Idle;
}
std::uint16_t M42pMotorEmulator::status() const noexcept
{
    const std::uint16_t run = moving() ? 1U : 0U;
    const std::uint16_t homing = motion_ == Motion::Home ? 0x08U : 0U;
    return static_cast<std::uint16_t>(
        run | homing | (static_cast<std::uint16_t>(alarm_) << 4U));
}

std::uint16_t M42pMotorEmulator::read_register(
    std::uint16_t address) const noexcept
{
    const std::uint32_t position = static_cast<std::uint32_t>(position_steps_);
    const std::uint32_t target = static_cast<std::uint32_t>(target_steps_);
    std::uint32_t speed = 0;
    const float signed_speed =
        motion_ == Motion::Reverse || motion_ == Motion::Home
            ? -std::abs(speed_rpm_)
            : std::abs(speed_rpm_);
    std::memcpy(&speed, &signed_speed, sizeof(speed));
    switch (address)
    {
        case 1000:
            return static_cast<std::uint16_t>(position >> 16U);
        case 1001:
            return static_cast<std::uint16_t>(position);
        case 1002:
            return static_cast<std::uint16_t>(target >> 16U);
        case 1003:
            return static_cast<std::uint16_t>(target);
        case 1004:
            return status();
        case 1005:
            return static_cast<std::uint16_t>(speed >> 16U);
        case 1006:
            return static_cast<std::uint16_t>(speed);
        case 1007:
            return 0;
        case 3000:
            return slave_;
        case 3001:
            return subdiv_;
        case 3002:
            return 0x0042;
        case 3003:
            return 0;
        default:
            return 0;
    }
}

void M42pMotorEmulator::set_alarm(std::uint8_t alarm) noexcept
{
    alarm_ = alarm;
    motion_ = Motion::Idle;
}

bool M42pMotorEmulator::write_register(
    std::uint16_t address,
    std::uint16_t value)
{
    switch (address)
    {
        case 2000:
            alarm_ = 0;
            target_steps_ = minimum_steps_;
            motion_ = Motion::Home;
            return true;
        case 2001:
            if (value == 0 || value == 251)
            {
                target_steps_ = position_steps_;
                motion_ = Motion::Idle;
                return true;
            }
            return false;
        case 3000:
            slave_ = static_cast<std::uint8_t>(value);
            return true;
        case 3001:
            subdiv_ = value == 0 ? 16 : value;
            return true;
        case 4002:
            return true;
        default:
            return false;
    }
}

bool M42pMotorEmulator::write_registers(
    std::uint16_t address,
    const std::vector<std::uint16_t>& values)
{
    if (values.size() != 2)
    {
        return false;
    }
    const std::uint32_t raw = registers_to_u32(values);
    switch (address)
    {
        case 2002:
        case 2202:
        case 2402:
            target_steps_ = static_cast<std::int32_t>(raw);
            motion_ = Motion::Position;
            return true;
        case 2004:
        case 2204:
        case 2404:
            target_steps_ =
                raw == 0 ? maximum_steps_ :
                position_steps_ + static_cast<std::int32_t>(raw);
            motion_ = Motion::Forward;
            return true;
        case 2006:
        case 2206:
        case 2406:
            target_steps_ =
                raw == 0 ? minimum_steps_ :
                position_steps_ - static_cast<std::int32_t>(raw);
            motion_ = Motion::Reverse;
            return true;
        case 2010:
            std::memcpy(&speed_rpm_, &raw, sizeof(speed_rpm_));
            speed_rpm_ = std::abs(speed_rpm_);
            return std::isfinite(speed_rpm_);
        default:
            return false;
    }
}

std::optional<std::vector<std::uint8_t>> M42pMotorEmulator::process(
    const std::uint8_t* request,
    std::size_t size)
{
    if (!request || size < 8 || request[0] != slave_ ||
        !common::has_valid_modbus_crc(request, size))
    {
        return std::nullopt;
    }
    const std::uint8_t function = request[1];
    const std::uint16_t address = common::read_be16(request + 2);
    if (function == 0x03)
    {
        const std::uint16_t count = common::read_be16(request + 4);
        if (count == 0 || count > 125 || size != 8)
        {
            return with_crc({slave_, 0x83, 0x03});
        }
        std::vector<std::uint8_t> response(
            3 + static_cast<std::size_t>(count) * 2, 0);
        response[0] = slave_;
        response[1] = function;
        response[2] = static_cast<std::uint8_t>(count * 2);
        for (std::uint16_t index = 0; index < count; ++index)
        {
            common::write_be16(
                response.data() + 3 + index * 2,
                read_register(static_cast<std::uint16_t>(address + index)));
        }
        return with_crc(std::move(response));
    }
    if (function == 0x06 && size == 8)
    {
        if (!write_register(address, common::read_be16(request + 4)))
        {
            return with_crc({slave_, 0x86, 0x02});
        }
        return std::vector<std::uint8_t>(request, request + size);
    }
    if (function == 0x10 && size >= 11)
    {
        const std::uint16_t count = common::read_be16(request + 4);
        const std::uint8_t byte_count = request[6];
        if (count == 0 || byte_count != count * 2 ||
            size != 9U + static_cast<std::size_t>(byte_count))
        {
            return with_crc({slave_, 0x90, 0x03});
        }
        std::vector<std::uint16_t> values(count);
        for (std::uint16_t index = 0; index < count; ++index)
        {
            values[index] = common::read_be16(request + 7 + index * 2);
        }
        if (!write_registers(address, values))
        {
            return with_crc({slave_, 0x90, 0x02});
        }
        return with_crc({
            slave_, function, request[2], request[3], request[4], request[5]});
    }
    return with_crc({
        slave_, static_cast<std::uint8_t>(function | 0x80U), 0x01});
}

void M42pMotorEmulator::advance(double seconds)
{
    if (!moving() || seconds <= 0.0)
    {
        return;
    }
    const double steps_per_second =
        std::max(1.0, static_cast<double>(speed_rpm_) *
                          static_cast<double>(subdiv_) * 200.0 / 60.0);
    const std::int32_t delta =
        std::max<std::int32_t>(1, static_cast<std::int32_t>(
            std::llround(steps_per_second * seconds)));
    const bool increasing =
        motion_ == Motion::Forward ||
        (motion_ == Motion::Position && target_steps_ > position_steps_);
    if (motion_ == Motion::Home || motion_ == Motion::Reverse || !increasing)
    {
        position_steps_ = std::max(target_steps_, position_steps_ - delta);
    }
    else
    {
        position_steps_ = std::min(target_steps_, position_steps_ + delta);
    }
    if (position_steps_ < minimum_steps_)
    {
        position_steps_ = minimum_steps_;
        set_alarm(7);
    }
    else if (position_steps_ > maximum_steps_)
    {
        position_steps_ = maximum_steps_;
        set_alarm(6);
    }
    else if (position_steps_ == target_steps_)
    {
        motion_ = Motion::Idle;
        if (target_steps_ == minimum_steps_)
        {
            position_steps_ = 0;
            target_steps_ = 0;
        }
    }
}

void M42pMotorEmulator::set_position_steps(std::int32_t steps) noexcept
{
    position_steps_ = std::max(minimum_steps_, std::min(maximum_steps_, steps));
    if (motion_ == Motion::Idle)
    {
        target_steps_ = position_steps_;
    }
}

void M42pMotorEmulator::set_limits(
    std::int32_t minimum,
    std::int32_t maximum) noexcept
{
    minimum_steps_ = std::min(minimum, maximum);
    maximum_steps_ = std::max(minimum, maximum);
    set_position_steps(position_steps_);
}

}  // namespace sharpa::mujoco::plugins::chassis
