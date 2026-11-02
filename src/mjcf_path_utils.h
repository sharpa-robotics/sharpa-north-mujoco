// Copyright 2026 Sharpa
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <mujoco/mujoco.h>

#include <filesystem>
#include <string>

namespace sharpa::mujoco::detail
{

inline std::filesystem::path model_directory(
    const mjSpec& spec,
    const std::filesystem::path& source_xml_path,
    std::error_code& error)
{
    std::filesystem::path directory(mjs_getString(spec.modelfiledir));
    if (directory.empty())
    {
        directory = source_xml_path.parent_path();
    }
    if (directory.empty())
    {
        directory = std::filesystem::current_path(error);
    }
    else if (directory.is_relative())
    {
        directory = std::filesystem::absolute(directory, error);
    }
    return error ? std::filesystem::path{} : directory.lexically_normal();
}

inline std::filesystem::path asset_directory(
    const std::filesystem::path& model_directory,
    const mjString* configured_directory)
{
    const char* configured = mjs_getString(configured_directory);
    if (!configured || configured[0] == '\0')
    {
        return model_directory;
    }

    const std::filesystem::path directory(configured);
    return (directory.is_absolute() ? directory : model_directory / directory)
        .lexically_normal();
}

inline std::string relative_or_absolute_path(
    const std::filesystem::path& target,
    const std::filesystem::path& base)
{
    std::error_code error;
    const std::filesystem::path relative =
        std::filesystem::relative(target, base, error);
    return error ? target.generic_string() : relative.generic_string();
}

inline void rebase_asset_directories(
    mjSpec& spec,
    const std::filesystem::path& source_model_directory,
    const std::filesystem::path& output_directory)
{
    const std::filesystem::path mesh_directory =
        asset_directory(source_model_directory, spec.compiler.meshdir);
    const std::filesystem::path texture_directory =
        asset_directory(source_model_directory, spec.compiler.texturedir);
    const std::string serialized_mesh_directory =
        relative_or_absolute_path(mesh_directory, output_directory);
    const std::string serialized_texture_directory =
        relative_or_absolute_path(texture_directory, output_directory);
    mjs_setString(spec.compiler.meshdir, serialized_mesh_directory.c_str());
    mjs_setString(spec.compiler.texturedir, serialized_texture_directory.c_str());
    spec.strippath = 0;
}

}  // namespace sharpa::mujoco::detail
