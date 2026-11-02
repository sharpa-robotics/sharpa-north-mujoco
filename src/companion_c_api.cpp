// Copyright 2026 Sharpa
// SPDX-License-Identifier: Apache-2.0

#include "sharpa_mujoco/c_api.h"
#include "sharpa_mujoco/companion_process.h"

#include <algorithm>
#include <cstring>
#include <exception>

struct sharpa_mujoco_companion
{
    sharpa::mujoco::CompanionProcess process;
};

namespace
{
void copy_text(const std::string& text, char* buffer, size_t capacity)
{
    if (!buffer || !capacity) return;
    const size_t count = std::min(capacity - 1, text.size());
    std::memcpy(buffer, text.data(), count);
    buffer[count] = '\0';
}
}

extern "C"
{
sharpa_mujoco_companion* sharpa_mujoco_companion_create(void)
{
    try { return new sharpa_mujoco_companion; }
    catch (...) { return nullptr; }
}

void sharpa_mujoco_companion_destroy(sharpa_mujoco_companion* companion)
{
    delete companion;
}

int sharpa_mujoco_companion_prepare(sharpa_mujoco_companion* companion, const char* name,
    const char* root, const char* profile, char* error, size_t error_size)
{
    try
    {
        if (!companion || !name || !*name)
        {
            copy_text("Companion and name are required", error, error_size);
            return 0;
        }
        const std::string directory = root && *root ? root :
            sharpa::mujoco::CompanionProcess::installed_root(name);
        if (directory.empty())
        {
            copy_text("Companion runtime is not staged: " + std::string(name), error, error_size);
            return 0;
        }
        std::string detail;
        const bool result = companion->process.prepare(directory, profile ? profile : "", &detail);
        copy_text(detail, error, error_size);
        return result ? 1 : 0;
    }
    catch (const std::exception& exception) { copy_text(exception.what(), error, error_size); }
    catch (...) { copy_text("Unknown companion exception", error, error_size); }
    return 0;
}

int sharpa_mujoco_companion_start(sharpa_mujoco_companion* companion, const char* name,
    const char* root, const char* profile, char* error, size_t error_size)
{
    try
    {
        if (!companion || !name || !*name)
        {
            copy_text("Companion and name are required", error, error_size);
            return 0;
        }
        const std::string directory = root && *root ? root :
            sharpa::mujoco::CompanionProcess::installed_root(name);
        if (directory.empty())
        {
            copy_text("Companion runtime is not staged: " + std::string(name), error, error_size);
            return 0;
        }
        std::string detail;
        const bool result = companion->process.start(directory, profile ? profile : "", &detail);
        copy_text(detail, error, error_size);
        return result ? 1 : 0;
    }
    catch (const std::exception& exception) { copy_text(exception.what(), error, error_size); }
    catch (...) { copy_text("Unknown companion exception", error, error_size); }
    return 0;
}

void sharpa_mujoco_companion_stop(sharpa_mujoco_companion* companion)
{
    if (companion) companion->process.stop();
}

int sharpa_mujoco_companion_status(const sharpa_mujoco_companion* companion,
    int* state, int* pid, int* exit_code, char* error, size_t error_size)
{
    try
    {
        if (!companion) { copy_text("Companion is required", error, error_size); return 0; }
        const auto status = companion->process.status();
        if (state) *state = static_cast<int>(status.state);
        if (pid) *pid = status.pid;
        if (exit_code) *exit_code = status.exit_code;
        copy_text(status.error, error, error_size);
        return 1;
    }
    catch (...) { copy_text("Cannot read companion status", error, error_size); return 0; }
}

int sharpa_mujoco_companion_log_path(const sharpa_mujoco_companion* companion,
    char* path, size_t path_size)
{
    try
    {
        if (!companion || !path || !path_size) return 0;
        copy_text(companion->process.status().log_path, path, path_size);
        return 1;
    }
    catch (...) { return 0; }
}
}
