// Copyright 2026 Sharpa
// SPDX-License-Identifier: Apache-2.0

#include "sharpa_mujoco/plugins/chassis/jiechang_stretch_emulator.h"

#include <algorithm>

namespace sharpa::mujoco::plugins::chassis
{

JiechangStretchEmulator::JiechangStretchEmulator(std::uint8_t node_id)
    : node_id_(node_id)
{
    objects_[{0x1005, 0}] = {0x80, 4};
    objects_[{0x1006, 0}] = {250000, 4};
    objects_[{0x2008, 0}] = {1, 1};
    objects_[{0x6080, 0}] = {524288, 4}; // 120 rpm, 2 mm/s CSP cruise.
}

bool JiechangStretchEmulator::enabled() const noexcept
{
    return operational_ && (control_word_ & 0x8fU) == 0x0fU;
}

std::uint16_t JiechangStretchEmulator::status() const noexcept
{
    if (enabled())
        return 0x27;
    if ((control_word_ & 0x8fU) == 0x07U)
        return 0x23;
    if ((control_word_ & 0x87U) == 0x06U)
        return 0x21;
    return 0x40;
}

double JiechangStretchEmulator::maximum_velocity() const noexcept
{
    return objects_.at({0x6080, 0}).first;
}

double JiechangStretchEmulator::command_timeout_s() const noexcept
{
    return std::max(0.1, objects_.at({0x1006, 0}).first * 3e-6);
}

std::optional<common::CanFdFrame>
JiechangStretchEmulator::process(const common::CanFdFrame& frame)
{
    using namespace common;
    if (frame.can_id == 0 && frame.data.size() == 2 &&
        (frame.data[1] == 0 || frame.data[1] == node_id_))
    {
        operational_ = frame.data[0] == 1;
        if (!operational_)
        {
            control_word_ = pending_control_word_ = 0;
            target_velocity_ = pending_velocity_ = 0;
        }
    }
    else if (operational_ && frame.can_id == 0x200U + node_id_ &&
             frame.data.size() == 6)
    {
        pending_control_word_ = read_le16(frame.data.data());
        pending_position_ = static_cast<std::int32_t>(read_le32(frame.data.data() + 2));
    }
    else if (operational_ && frame.can_id == 0x300U + node_id_ &&
             frame.data.size() == 4)
    {
        pending_velocity_ = static_cast<std::int32_t>(read_le32(frame.data.data()));
    }
    else if (operational_ && frame.can_id == 0x80 && frame.data.empty())
    {
        control_word_ = pending_control_word_;
        target_position_ = pending_position_;
        target_velocity_ = pending_velocity_;
    }
    else if (frame.can_id == 0x600U + node_id_ && frame.data.size() == 8)
    {
        const auto* data = frame.data.data();
        const std::uint16_t index = read_le16(data + 1);
        const std::uint8_t sub = data[3];
        CanFdFrame reply;
        reply.can_interface = frame.can_interface;
        reply.can_id = 0x580U + node_id_;
        reply.data = {0x60, data[1], data[2], sub, 0, 0, 0, 0};
        std::uint32_t value = 0;
        std::size_t width = 0;
        if (data[0] == 0x40)
        {
            if (sub == 0)
            {
                switch (index)
                {
                case 0x2005:
                case 0x2006:
                    value = 0x10000;
                    width = 4;
                    break;
                case 0x603f:
                    width = 2;
                    break;
                case 0x6040:
                    value = control_word_;
                    width = 2;
                    break;
                case 0x6041:
                    value = status();
                    width = 2;
                    break;
                case 0x6060:
                case 0x6061:
                    value = static_cast<std::uint8_t>(mode_);
                    width = 1;
                    break;
                case 0x6064:
                    value = static_cast<std::uint32_t>(position_);
                    width = 4;
                    break;
                case 0x606c:
                    value = static_cast<std::uint32_t>(velocity_);
                    width = 4;
                    break;
                case 0x6077:
                    value = static_cast<std::uint16_t>(torque_);
                    width = 2;
                    break;
                case 0x6078:
                    value = static_cast<std::uint16_t>(current_);
                    width = 2;
                    break;
                case 0x6079:
                    value = 48000;
                    width = 4;
                    break; // Synthetic bus voltage.
                }
            }
            const auto it = objects_.find({index, sub});
            if (width == 0 && it != objects_.end())
            {
                value = it->second.first;
                width = it->second.second;
            }
            if (width != 0)
            {
                reply.data[0] = width == 1 ? 0x4f : width == 2 ? 0x4b : 0x43;
                write_le32(reply.data.data() + 4, value);
                return reply;
            }
        }
        else
        {
            width = data[0] == 0x2f ? 1 : data[0] == 0x2b ? 2 : data[0] == 0x23 ? 4 : 0;
            value = read_le32(data + 4);
            if (width == 1)
                value &= 0xff;
            if (width == 2)
                value &= 0xffff;
            if (sub == 0 && index == 0x6060 && width == 1 && (value == 8 || value == 9))
            {
                mode_ = static_cast<std::int8_t>(value);
                target_velocity_ = pending_velocity_ = 0;
                return reply;
            }
            if (sub == 0 && index == 0x6040 && width == 2)
            {
                control_word_ = pending_control_word_ =
                    static_cast<std::uint16_t>(value);
                return reply;
            }
            const bool comm = index == 0x1400 || index == 0x1401 || index == 0x1800 ||
                              index == 0x1801;
            const bool mapping = index == 0x1600 || index == 0x1601 ||
                                 index == 0x1a00 || index == 0x1a01;
            const bool scalar = sub == 0 && (index == 0x1005 || index == 0x1006 ||
                                             index == 0x2008 || index == 0x6080);
            if (width != 0 &&
                (scalar || (comm && sub >= 1 && sub <= 2) || (mapping && sub <= 3)))
            {
                objects_[{index, sub}] = {value, width};
                return reply;
            }
        }
        reply.data[0] = 0x80;
        write_le32(reply.data.data() + 4, 0x06020000);
        return reply;
    }
    return std::nullopt;
}

void JiechangStretchEmulator::set_feedback(std::int32_t position, std::int32_t velocity,
                                           std::int16_t torque, std::int16_t current)
{
    position_ = position;
    velocity_ = velocity;
    torque_ = torque;
    current_ = current;
}

std::array<common::CanFdFrame, 2>
JiechangStretchEmulator::feedback(std::uint8_t bus) const
{
    std::array<common::CanFdFrame, 2> frames;
    for (std::size_t i = 0; i < frames.size(); ++i)
    {
        frames[i].can_interface = bus;
        frames[i].can_id = 0x180U + i * 0x100U + node_id_;
        frames[i].data.resize(8);
    }
    common::write_le16(frames[0].data.data() + 2, status());
    common::write_le32(frames[0].data.data() + 4,
                       static_cast<std::uint32_t>(position_));
    common::write_le32(frames[1].data.data(), static_cast<std::uint32_t>(velocity_));
    common::write_le16(frames[1].data.data() + 4, static_cast<std::uint16_t>(torque_));
    common::write_le16(frames[1].data.data() + 6, static_cast<std::uint16_t>(current_));
    return frames;
}

} // namespace sharpa::mujoco::plugins::chassis
