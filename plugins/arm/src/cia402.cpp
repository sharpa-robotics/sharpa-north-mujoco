// Copyright 2026 Sharpa
// SPDX-License-Identifier: Apache-2.0
#include "sharpa_mujoco/plugins/arm/cia402.h"

namespace sharpa::mujoco::plugins::arm
{

void Cia402Drive::apply(std::uint16_t control_word)
{
    if (control_word == 0x0080U && state_ == Cia402State::Fault)
    {
        state_ = Cia402State::SwitchOnDisabled;
        return;
    }
    switch (control_word & 0x008FU)
    {
        case 0x0000:
            state_ = Cia402State::SwitchOnDisabled;
            break;
        case 0x0002:
            if (state_ == Cia402State::OperationEnabled ||
                state_ == Cia402State::QuickStopActive)
            {
                state_ = Cia402State::QuickStopActive;
            }
            break;
        case 0x0006:
            state_ = Cia402State::ReadyToSwitchOn;
            break;
        case 0x0007:
            state_ = Cia402State::SwitchedOn;
            break;
        case 0x000F:
            state_ = Cia402State::OperationEnabled;
            break;
        default:
            break;
    }
}

void Cia402Drive::complete_quick_stop() noexcept
{
    if (state_ == Cia402State::QuickStopActive)
        state_ = Cia402State::SwitchOnDisabled;
}

std::uint16_t Cia402Drive::status_word() const noexcept
{
    switch (state_)
    {
        case Cia402State::NotReady: return 0x0000;
        case Cia402State::SwitchOnDisabled: return 0x0040;
        case Cia402State::ReadyToSwitchOn: return 0x0021;
        case Cia402State::SwitchedOn: return 0x0023;
        case Cia402State::OperationEnabled: return 0x0027;
        case Cia402State::QuickStopActive: return 0x0007;
        case Cia402State::FaultReactionActive: return 0x000F;
        case Cia402State::Fault: return 0x0008;
    }
    return 0;
}

}  // namespace sharpa::mujoco::plugins::arm
