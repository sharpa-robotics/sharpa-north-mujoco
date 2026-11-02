// Copyright 2026 Sharpa
// SPDX-License-Identifier: Apache-2.0

#include "sharpa_mujoco/plugins/common/startup_brake.h"

#include "sharpa_mujoco/runtime.h"

#include <mujoco/mujoco.h>

#include <algorithm>

namespace sharpa::mujoco::plugins::common
{
namespace
{

bool fail(const std::string& message, std::string* error)
{
    if (error)
    {
        *error = message;
    }
    return false;
}

}  // namespace

bool StartupBrake::initialize(
    Runtime& runtime,
    const std::vector<std::string>& joint_names,
    std::string* error)
{
    if (joint_names.empty())
    {
        return fail("Startup brake requires at least one joint.", error);
    }

    std::vector<std::string> equality_names;
    equality_names.reserve(joint_names.size());
    for (const std::string& joint_name : joint_names)
    {
        equality_names.push_back("brake_" + joint_name);
    }

    bool valid = true;
    std::string missing;
    if (!runtime.read_native(
            [&](const mjModel* model, const mjData*) {
                for (const std::string& equality_name : equality_names)
                {
                    if (mj_name2id(
                            model,
                            mjOBJ_EQUALITY,
                            equality_name.c_str()) < 0)
                    {
                        valid = false;
                        missing = equality_name;
                        break;
                    }
                }
            },
            error))
    {
        return false;
    }
    if (!valid)
    {
        return fail("Missing startup brake equality: " + missing, error);
    }

    std::lock_guard<std::mutex> lock(mutex_);
    runtime_ = &runtime;
    equality_names_ = std::move(equality_names);
    released_.assign(equality_names_.size(), false);
    reset_generation_ = runtime.reset_generation();
    return true;
}

bool StartupBrake::release(std::size_t axis, std::string* error)
{
    return release(std::vector<std::size_t>{axis}, error);
}

bool StartupBrake::release(
    const std::vector<std::size_t>& axes,
    std::string* error)
{
    std::lock_guard<std::mutex> lock(mutex_);
    if (!runtime_)
    {
        return fail("Startup brake is not initialized.", error);
    }
    synchronize_reset_locked();

    std::vector<std::size_t> pending;
    pending.reserve(axes.size());
    for (const std::size_t axis : axes)
    {
        if (axis >= released_.size())
        {
            return fail("Startup brake axis is out of range.", error);
        }
        if (!released_[axis] &&
            std::find(pending.begin(), pending.end(), axis) == pending.end())
        {
            pending.push_back(axis);
        }
    }
    if (pending.empty())
    {
        return true;
    }

    std::vector<std::string> equality_names;
    equality_names.reserve(pending.size());
    for (const std::size_t axis : pending)
    {
        equality_names.push_back(equality_names_[axis]);
    }
    if (!runtime_->set_equalities_active(equality_names, false, error))
    {
        return false;
    }
    for (const std::size_t axis : pending)
    {
        released_[axis] = true;
    }
    return true;
}

bool StartupBrake::engage(std::size_t axis, std::string* error)
{
    return engage(std::vector<std::size_t>{axis}, error);
}

bool StartupBrake::engage(
    const std::vector<std::size_t>& axes,
    std::string* error)
{
    std::lock_guard<std::mutex> lock(mutex_);
    if (!runtime_)
    {
        return fail("Startup brake is not initialized.", error);
    }
    synchronize_reset_locked();

    std::vector<std::size_t> pending;
    pending.reserve(axes.size());
    for (const std::size_t axis : axes)
    {
        if (axis >= released_.size())
        {
            return fail("Startup brake axis is out of range.", error);
        }
        if (released_[axis] &&
            std::find(pending.begin(), pending.end(), axis) == pending.end())
        {
            pending.push_back(axis);
        }
    }
    if (pending.empty())
    {
        return true;
    }

    std::vector<std::string> equality_names;
    equality_names.reserve(pending.size());
    for (const std::size_t axis : pending)
    {
        equality_names.push_back(equality_names_[axis]);
    }
    if (!runtime_->set_equalities_active(equality_names, true, error))
    {
        return false;
    }
    for (const std::size_t axis : pending)
    {
        released_[axis] = false;
    }
    return true;
}

bool StartupBrake::release_in_step(
    const mjModel* model,
    mjData* data,
    std::initializer_list<std::size_t> axes,
    std::string* error)
{
    return release_in_step(model, data, axes.begin(), axes.size(), error);
}

bool StartupBrake::release_in_step(
    const mjModel* model, mjData* data,
    const std::vector<std::size_t>& axes, std::string* error)
{
    return release_in_step(model, data, axes.data(), axes.size(), error);
}

bool StartupBrake::release_in_step(
    const mjModel* model, mjData* data, const std::size_t* axes,
    std::size_t count, std::string* error)
{
    std::lock_guard<std::mutex> lock(mutex_);
    if (!runtime_ || !model || !data)
    {
        return fail("Startup brake is not initialized.", error);
    }
    synchronize_reset_locked();

    bool all_released = true;
    for (std::size_t i = 0; i < count; ++i)
    {
        const std::size_t axis = axes[i];
        if (axis >= released_.size())
        {
            return fail("Startup brake axis is out of range.", error);
        }
        all_released = all_released && released_[axis];
    }
    if (all_released)
    {
        return true;
    }

    // Resolve and validate the complete group before changing any equality.
    for (std::size_t i = 0; i < count; ++i)
    {
        const std::size_t axis = axes[i];
        const int equality_id = mj_name2id(
            model, mjOBJ_EQUALITY, equality_names_[axis].c_str());
        if (equality_id < 0)
        {
            return fail(
                "Missing startup brake equality: " + equality_names_[axis], error);
        }
    }

    for (std::size_t i = 0; i < count; ++i)
    {
        const std::size_t axis = axes[i];
        const int equality_id = mj_name2id(
            model, mjOBJ_EQUALITY, equality_names_[axis].c_str());
        data->eq_active[equality_id] = 0;
        released_[axis] = true;
    }
    return true;
}

bool StartupBrake::released(std::size_t axis) const noexcept
{
    std::lock_guard<std::mutex> lock(mutex_);
    synchronize_reset_locked();
    return axis < released_.size() && released_[axis];
}

void StartupBrake::synchronize_reset_locked() const noexcept
{
    if (!runtime_) return;
    const std::uint64_t generation = runtime_->reset_generation();
    if (generation == reset_generation_) return;
    std::fill(released_.begin(), released_.end(), false);
    reset_generation_ = generation;
}

}  // namespace sharpa::mujoco::plugins::common
