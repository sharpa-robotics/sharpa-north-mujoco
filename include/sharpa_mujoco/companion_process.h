// Copyright 2026 Sharpa
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include "sharpa_mujoco/export.h"

#include <memory>
#include <string>

namespace sharpa::mujoco
{

enum class CompanionProcessState
{
    Stopped,
    Starting,
    Running,
    Failed,
};

struct CompanionProcessStatus
{
    CompanionProcessState state = CompanionProcessState::Stopped;
    int pid = 0;
    int exit_code = 0;
    std::string error;
    std::string log_path;
};

// Owns one process group. Running means the process survived startup, not that
// its application-specific health checks passed. Destroying the owner stops it.
// A later owner safely identifies and stops a surviving group left by an
// abnormally terminated owner before launching a replacement. prepare() holds
// exclusive ownership after recovery without launching the child, so callers
// can safely bind simulator endpoints before start().
class SHARPA_MUJOCO_API CompanionProcess
{
public:
    CompanionProcess();
    ~CompanionProcess();
    CompanionProcess(const CompanionProcess&) = delete;
    CompanionProcess& operator=(const CompanionProcess&) = delete;

    bool prepare(const std::string& root, const std::string& profile,
                 std::string* error = nullptr);
    bool start(const std::string& root, const std::string& profile,
               std::string* error = nullptr);
    void stop() noexcept;
    CompanionProcessStatus status() const;

    // Resolve relative to the loaded library or executable, never the caller's
    // current directory. The named runtime must already have been staged.
    static std::string installed_root(const std::string& name);

private:
    class Impl;
    std::unique_ptr<Impl> impl_;
};

}  // namespace sharpa::mujoco
