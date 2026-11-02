// Copyright 2026 Sharpa
// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <cstdint>

namespace sharpa::mujoco::plugins::arm
{

enum class Cia402State
{
    NotReady,
    SwitchOnDisabled,
    ReadyToSwitchOn,
    SwitchedOn,
    OperationEnabled,
    QuickStopActive,
    FaultReactionActive,
    Fault,
};

class Cia402Drive
{
public:
    void apply(std::uint16_t control_word);
    void complete_quick_stop() noexcept;
    Cia402State state() const noexcept { return state_; }
    std::uint16_t status_word() const noexcept;
    bool operation_enabled() const noexcept
    {
        return state_ == Cia402State::OperationEnabled;
    }
    void inject_fault() noexcept { state_ = Cia402State::Fault; }

private:
    Cia402State state_ = Cia402State::SwitchOnDisabled;
};

}  // namespace sharpa::mujoco::plugins::arm
