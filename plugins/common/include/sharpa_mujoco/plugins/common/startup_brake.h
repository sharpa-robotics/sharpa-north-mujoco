// Copyright 2026 Sharpa
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <cstddef>
#include <cstdint>
#include <initializer_list>
#include <mutex>
#include <string>
#include <vector>

namespace sharpa::mujoco
{
class Runtime;
}

struct mjModel_;
struct mjData_;
typedef struct mjModel_ mjModel;
typedef struct mjData_ mjData;

namespace sharpa::mujoco::plugins::common
{

class StartupBrake
{
public:
    bool initialize(
        Runtime& runtime,
        const std::vector<std::string>& joint_names,
        std::string* error = nullptr);

    bool release(std::size_t axis, std::string* error = nullptr);
    bool release(
        const std::vector<std::size_t>& axes,
        std::string* error = nullptr);
    bool engage(std::size_t axis, std::string* error = nullptr);
    bool engage(
        const std::vector<std::size_t>& axes,
        std::string* error = nullptr);
    // This variant is for Runtime model-updater callbacks, where the
    // caller already owns the simulation lock. They update a group atomically
    // and do not run mj_forward; the immediately following mj_step consumes it.
    bool release_in_step(
        const mjModel* model,
        mjData* data,
        std::initializer_list<std::size_t> axes,
        std::string* error = nullptr);
    bool release_in_step(
        const mjModel* model,
        mjData* data,
        const std::vector<std::size_t>& axes,
        std::string* error = nullptr);
    bool released(std::size_t axis) const noexcept;

private:
    bool release_in_step(const mjModel* model, mjData* data,
                         const std::size_t* axes, std::size_t count,
                         std::string* error);
    void synchronize_reset_locked() const noexcept;

    Runtime* runtime_ = nullptr;
    std::vector<std::string> equality_names_;
    mutable std::vector<bool> released_;
    mutable std::uint64_t reset_generation_ = 0;
    mutable std::mutex mutex_;
};

}  // namespace sharpa::mujoco::plugins::common
