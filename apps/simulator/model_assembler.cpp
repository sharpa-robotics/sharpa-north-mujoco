// Copyright 2026 Sharpa
// SPDX-License-Identifier: Apache-2.0

#include "model_assembler.h"

#include "sharpa_mujoco/runtime.h"

#include <mujoco/mujoco.h>

#include <algorithm>
#include <array>
#include <memory>
#include <set>

namespace sharpa::mujoco::simulator
{
namespace
{

bool valid_mount(const LidarMount& mount, std::string* error)
{
    if (mount.parent_body.empty() || mount.instance_prefix.empty() ||
        mount.instance.site_name.empty() || mount.sensor_model.empty())
    {
        if (error)
        {
            *error =
                "LiDAR mount requires parent_body, instance_prefix, site_name, and sensor_model.";
        }
        return false;
    }
    return true;
}

}  // namespace

bool attach_lidar_mounts(
    Runtime& runtime,
    const std::vector<LidarMount>& mounts,
    std::string* error)
{
    if (mounts.empty())
    {
        return true;
    }

    std::set<std::string> site_names;
    for (const LidarMount& mount : mounts)
    {
        if (!valid_mount(mount, error) ||
            !site_names.insert(mount.instance.site_name).second)
        {
            if (error && error->empty())
            {
                *error = "LiDAR site names must be unique.";
            }
            return false;
        }
    }

    return runtime.assemble_spec_transactional(
            [&](mjSpec* spec, std::string* assembly_error) {
                for (const LidarMount& mount : mounts)
                {
                    mjsBody* parent = mjs_findBody(spec, mount.parent_body.c_str());
                    if (!parent)
                    {
                        if (assembly_error)
                        {
                            *assembly_error =
                                "LiDAR parent body was not found: " + mount.parent_body;
                        }
                        return false;
                    }

                    std::array<char, 1024> parse_error{};
                    std::unique_ptr<mjSpec, decltype(&mj_deleteSpec)> child(
                        mj_parseXML(
                            mount.sensor_model.c_str(),
                            nullptr,
                            parse_error.data(),
                            static_cast<int>(parse_error.size())),
                        &mj_deleteSpec);
                    if (!child)
                    {
                        if (assembly_error)
                        {
                            *assembly_error = "Could not parse LiDAR sensor_model '" +
                                              mount.sensor_model + "': " +
                                              parse_error.data();
                        }
                        return false;
                    }
                    mjsBody* child_body =
                        mjs_findBody(child.get(), "mount_frame");
                    if (!child_body)
                    {
                        if (assembly_error)
                        {
                            *assembly_error =
                                "LiDAR sensor_model must contain body 'mount_frame': " +
                                mount.sensor_model;
                        }
                        return false;
                    }
                    if (!mjs_findElement(
                            child.get(),
                            mjOBJ_SITE,
                            mount.local_site_name.c_str()))
                    {
                        if (assembly_error)
                        {
                            *assembly_error = "LiDAR sensor_model site '" +
                                              mount.local_site_name +
                                              "' was not found: " +
                                              mount.sensor_model;
                        }
                        return false;
                    }
                    mjsFrame* frame = mjs_addFrame(parent, nullptr);
                    if (!frame)
                    {
                        return false;
                    }
                    std::copy(
                        mount.position.begin(), mount.position.end(), frame->pos);
                    std::copy(
                        mount.quaternion.begin(), mount.quaternion.end(), frame->quat);
                    mjs_setDeepCopy(child.get(), 1);
                    mjsBody* attached = mjs_asBody(mjs_attach(
                        frame->element,
                        child_body->element,
                        mount.instance_prefix.c_str(),
                        ""));
                    if (!attached)
                    {
                        if (assembly_error)
                        {
                            *assembly_error =
                                "Could not attach LiDAR sensor_model: " +
                                mount.sensor_model;
                        }
                        return false;
                    }
                }
                return true;
            },
            error);
}

bool attach_tactile_mounts(
    Runtime& runtime,
    const std::vector<TactileMount>& mounts,
    std::string* error)
{
    if (mounts.empty()) return true;
    return runtime.assemble_spec_transactional(
        [&](mjSpec* spec, std::string* assembly_error) {
            for (const TactileMount& mount : mounts)
            {
                mjsBody* parent = mjs_findBody(spec, mount.parent_body.c_str());
                if (!parent)
                {
                    if (assembly_error)
                        *assembly_error = "Tactile parent body was not found: " +
                                          mount.parent_body;
                    return false;
                }
                std::array<char, 1024> parse_error{};
                std::unique_ptr<mjSpec, decltype(&mj_deleteSpec)> child(
                    mj_parseXML(mount.sensor_model.c_str(), nullptr,
                                parse_error.data(),
                                static_cast<int>(parse_error.size())),
                    &mj_deleteSpec);
                if (!child)
                {
                    if (assembly_error)
                        *assembly_error = "Could not parse tactile sensor_model '" +
                                          mount.sensor_model + "': " +
                                          parse_error.data();
                    return false;
                }
                mjsBody* child_body = mjs_findBody(child.get(), "mount_frame");
                if (!child_body ||
                    !mjs_findElement(child.get(), mjOBJ_SITE,
                                     mount.local_site_name.c_str()))
                {
                    if (assembly_error)
                        *assembly_error =
                            "Tactile sensor_model requires mount_frame and site '" +
                            mount.local_site_name + "'.";
                    return false;
                }
                mjsFrame* frame = mjs_addFrame(parent, nullptr);
                if (!frame) return false;
                std::copy(mount.position.begin(), mount.position.end(), frame->pos);
                std::copy(mount.quaternion.begin(), mount.quaternion.end(),
                          frame->quat);
                mjs_setDeepCopy(child.get(), 1);
                if (!mjs_attach(frame->element, child_body->element,
                                mount.instance_prefix.c_str(), ""))
                {
                    if (assembly_error)
                        *assembly_error = "Could not attach tactile frame to " +
                                          mount.parent_body;
                    return false;
                }
            }
            return true;
        },
        error);
}

}  // namespace sharpa::mujoco::simulator
