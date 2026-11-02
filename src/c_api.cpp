// Copyright 2026 Sharpa
// SPDX-License-Identifier: Apache-2.0

#include "sharpa_mujoco/c_api.h"

#include "sharpa_mujoco/runtime.h"
#include "sharpa_mujoco/scene_builder.h"

#if SHARPA_MUJOCO_HAS_EMBEDDED_SIMULATOR
#include "sharpa_mujoco/simulator_service.h"
#include <dlfcn.h>
#include <filesystem>
#include <memory>
#endif

#include <mujoco/mujoco.h>

#include <algorithm>
#include <chrono>
#include <cstring>
#include <exception>
#include <limits>
#include <string>

struct sharpa_mujoco_runtime
{
    sharpa::mujoco::Runtime runtime;
#if SHARPA_MUJOCO_HAS_EMBEDDED_SIMULATOR
    std::unique_ptr<sharpa::mujoco::simulator::SimulatorService> simulator;
#endif
};

namespace
{

void copy_error(const std::string& source, char* destination, size_t capacity)
{
    if (!destination || capacity == 0)
    {
        return;
    }
    const size_t count = std::min(capacity - 1, source.size());
    std::memcpy(destination, source.data(), count);
    destination[count] = '\0';
}

template <typename Callback>
int guarded(Callback&& callback, char* error, size_t error_size)
{
    try
    {
        std::string detail;
        const bool result = callback(detail);
        copy_error(detail, error, error_size);
        return result ? 1 : 0;
    }
    catch (const std::exception& exception)
    {
        copy_error(exception.what(), error, error_size);
    }
    catch (...)
    {
        copy_error("Unknown sharpa_mujoco exception.", error, error_size);
    }
    return 0;
}

sharpa::mujoco::Pose convert_pose(const sharpa_mujoco_pose& source)
{
    sharpa::mujoco::Pose result;
    std::copy(std::begin(source.position), std::end(source.position), result.position.begin());
    std::copy(std::begin(source.quaternion), std::end(source.quaternion), result.quaternion.begin());
    return result;
}

#if SHARPA_MUJOCO_HAS_EMBEDDED_SIMULATOR
std::string default_simulator_config_path()
{
    Dl_info info{};
    if (dladdr(
            reinterpret_cast<const void*>(&sharpa_mujoco_version_string),
            &info) == 0 ||
        !info.dli_fname)
    {
        return {};
    }
    return (
        std::filesystem::path(info.dli_fname).parent_path() /
        "sharpa_mujoco" /
        "sharpa_mujoco.conf").lexically_normal().string();
}

int convert_simulator_state(
    sharpa::mujoco::simulator::SimulatorServiceState state)
{
    using sharpa::mujoco::simulator::SimulatorServiceState;
    switch (state)
    {
    case SimulatorServiceState::Configured:
        return SHARPA_MUJOCO_SIMULATOR_CONFIGURED;
    case SimulatorServiceState::Running:
        return SHARPA_MUJOCO_SIMULATOR_RUNNING;
    case SimulatorServiceState::Faulted:
        return SHARPA_MUJOCO_SIMULATOR_FAULTED;
    case SimulatorServiceState::Unconfigured:
    default:
        return SHARPA_MUJOCO_SIMULATOR_UNCONFIGURED;
    }
}

int convert_environment_issue_code(
    sharpa::mujoco::simulator::SimulatorEnvironmentIssueCode code)
{
    using sharpa::mujoco::simulator::SimulatorEnvironmentIssueCode;
    switch (code)
    {
    case SimulatorEnvironmentIssueCode::SocketCanMissing:
        return SHARPA_MUJOCO_ENVIRONMENT_SOCKETCAN_MISSING;
    case SimulatorEnvironmentIssueCode::SocketCanDown:
        return SHARPA_MUJOCO_ENVIRONMENT_SOCKETCAN_DOWN;
    case SimulatorEnvironmentIssueCode::SocketCanUnusable:
    default:
        return SHARPA_MUJOCO_ENVIRONMENT_SOCKETCAN_UNUSABLE;
    }
}
#else
int embedded_simulator_disabled(char* error, size_t error_size)
{
    copy_error(
        "Embedded simulator support is disabled. Rebuild with "
        "SHARPA_MUJOCO_BUILD_EMBEDDED_SIMULATOR=ON.",
        error, error_size);
    return 0;
}
#endif

bool apply_contact_parameters(
    const sharpa_mujoco_scene_geom& source,
    sharpa::mujoco::SceneGeomDescription& destination,
    std::string& detail)
{
    constexpr uint32_t supported_flags =
        SHARPA_MUJOCO_SCENE_CONTACT_SOLREF |
        SHARPA_MUJOCO_SCENE_CONTACT_SOLIMP;
    if ((source.contact.flags & ~supported_flags) != 0)
    {
        detail = "Scene geom contact flags are invalid.";
        return false;
    }
    if ((source.contact.flags & SHARPA_MUJOCO_SCENE_CONTACT_SOLREF) != 0)
    {
        destination.override_solref = true;
        std::copy(
            std::begin(source.contact.solref),
            std::end(source.contact.solref),
            destination.solref.begin());
    }
    if ((source.contact.flags & SHARPA_MUJOCO_SCENE_CONTACT_SOLIMP) != 0)
    {
        destination.override_solimp = true;
        std::copy(
            std::begin(source.contact.solimp),
            std::end(source.contact.solimp),
            destination.solimp.begin());
    }
    return true;
}

bool convert_scene(
    const sharpa_mujoco_scene& source,
    sharpa::mujoco::SceneDescription& scene,
    std::string& detail)
{
    scene.robot_xml_path = source.robot_xml_path;
    scene.robot_root_body = source.robot_root_body ? source.robot_root_body : "";
    scene.robot_world_pose = convert_pose(source.robot_world_pose);
    scene.mesh_base_directory = source.mesh_base_directory
        ? source.mesh_base_directory
        : "";
    scene.bodies.reserve(source.body_count);
    for (size_t body_index = 0; body_index < source.body_count; ++body_index)
    {
        const auto& source_body = source.bodies[body_index];
        if (!source_body.geoms && source_body.geom_count > 0)
        {
            detail = "Scene body geom pointer is null.";
            return false;
        }
        sharpa::mujoco::SceneBodyDescription body;
        body.name = source_body.name ? source_body.name : "";
        switch (source_body.motion_type)
        {
            case SHARPA_MUJOCO_MOTION_STATIC:
                body.motion_type = sharpa::mujoco::SceneMotionType::Static;
                break;
            case SHARPA_MUJOCO_MOTION_DYNAMIC:
                body.motion_type = sharpa::mujoco::SceneMotionType::Dynamic;
                break;
            default:
                detail = "Scene body motion_type is invalid.";
                return false;
        }
        body.world_pose = convert_pose(source_body.world_pose);
        body.geoms.reserve(source_body.geom_count);
        for (size_t geom_index = 0; geom_index < source_body.geom_count; ++geom_index)
        {
            const auto& source_geom = source_body.geoms[geom_index];
            sharpa::mujoco::SceneGeomDescription geom;
            geom.name = source_geom.name ? source_geom.name : "";
            switch (source_geom.type)
            {
                case SHARPA_MUJOCO_BOX:
                    geom.type = sharpa::mujoco::SceneGeomType::Box;
                    break;
                case SHARPA_MUJOCO_SPHERE:
                    geom.type = sharpa::mujoco::SceneGeomType::Sphere;
                    break;
                case SHARPA_MUJOCO_CAPSULE:
                    geom.type = sharpa::mujoco::SceneGeomType::Capsule;
                    break;
                case SHARPA_MUJOCO_MESH:
                    geom.type = sharpa::mujoco::SceneGeomType::Mesh;
                    break;
                default:
                    detail = "Scene geom type is invalid.";
                    return false;
            }
            geom.local_pose = convert_pose(source_geom.local_pose);
            std::copy(std::begin(source_geom.size), std::end(source_geom.size), geom.size.begin());
            std::copy(std::begin(source_geom.friction), std::end(source_geom.friction), geom.friction.begin());
            geom.density = source_geom.density;
            geom.mass = source_geom.mass;
            geom.mesh_file = source_geom.mesh_file ? source_geom.mesh_file : "";
            geom.mesh_name = source_geom.mesh_name ? source_geom.mesh_name : "";
            if ((source_geom.mesh_vertex_count > 0 && !source_geom.mesh_vertices) ||
                (source_geom.mesh_face_count > 0 && !source_geom.mesh_faces))
            {
                detail = "Scene geom in-memory mesh pointers are null.";
                return false;
            }
            if (source_geom.mesh_vertex_count > std::numeric_limits<size_t>::max() / 3 ||
                source_geom.mesh_face_count > std::numeric_limits<size_t>::max() / 3)
            {
                detail = "Scene geom in-memory mesh count is too large.";
                return false;
            }
            if (source_geom.mesh_vertex_count > 0)
            {
                geom.mesh_vertices.assign(
                    source_geom.mesh_vertices,
                    source_geom.mesh_vertices + source_geom.mesh_vertex_count * 3);
            }
            if (source_geom.mesh_face_count > 0)
            {
                geom.mesh_faces.assign(
                    source_geom.mesh_faces,
                    source_geom.mesh_faces + source_geom.mesh_face_count * 3);
            }
            if (!apply_contact_parameters(source_geom, geom, detail))
            {
                return false;
            }
            body.geoms.push_back(std::move(geom));
        }
        scene.bodies.push_back(std::move(body));
    }
    return true;
}

bool build_scene(
    const sharpa_mujoco_scene& source,
    const char* output_xml_path,
    const char* output_mjb_path,
    std::string& detail)
{
    sharpa::mujoco::SceneDescription scene;
    if (!convert_scene(source, scene, detail))
    {
        return false;
    }
    sharpa::mujoco::SceneBuildOptions options;
    options.output_xml_path = output_xml_path;
    options.output_mjb_path = output_mjb_path ? output_mjb_path : "";
    return sharpa::mujoco::SceneBuilder::build(scene, options, &detail);
}

}  // namespace

extern "C"
{

const char* sharpa_mujoco_version_string(void)
{
    return SHARPA_MUJOCO_VERSION;
}

const char* sharpa_mujoco_mujoco_version_string(void)
{
    return mj_versionString();
}

sharpa_mujoco_runtime* sharpa_mujoco_runtime_create(void)
{
    try
    {
        return new sharpa_mujoco_runtime;
    }
    catch (...)
    {
        return nullptr;
    }
}

void sharpa_mujoco_runtime_destroy(sharpa_mujoco_runtime* runtime)
{
    if (runtime)
    {
        runtime->runtime.pause();
#if SHARPA_MUJOCO_HAS_EMBEDDED_SIMULATOR
        if (runtime->simulator) runtime->simulator->stop();
#endif
    }
    delete runtime;
}

int sharpa_mujoco_runtime_load(sharpa_mujoco_runtime* runtime, const char* path, char* error, size_t error_size)
{
    return guarded([&](std::string& detail) {
        if (!runtime || !path)
        {
            detail = "Runtime and model path must not be null.";
            return false;
        }
        runtime->runtime.pause();
#if SHARPA_MUJOCO_HAS_EMBEDDED_SIMULATOR
        if (runtime->simulator)
        {
            runtime->simulator->stop();
            runtime->simulator.reset();
        }
#endif
        return runtime->runtime.load(path, &detail);
    }, error, error_size);
}

int sharpa_mujoco_runtime_load_scene(
    sharpa_mujoco_runtime* runtime,
    const sharpa_mujoco_scene* source,
    char* error,
    size_t error_size)
{
    return guarded([&](std::string& detail) {
        if (!runtime || !source || !source->robot_xml_path ||
            (!source->bodies && source->body_count > 0))
        {
            detail = "Runtime, scene, robot path and body array must be valid.";
            return false;
        }
        runtime->runtime.pause();
#if SHARPA_MUJOCO_HAS_EMBEDDED_SIMULATOR
        if (runtime->simulator)
        {
            runtime->simulator->stop();
            runtime->simulator.reset();
        }
#endif
        sharpa::mujoco::SceneDescription scene;
        return convert_scene(*source, scene, detail) &&
            runtime->runtime.load_scene(scene, &detail);
    }, error, error_size);
}

void sharpa_mujoco_runtime_unload(sharpa_mujoco_runtime* runtime)
{
    if (runtime)
    {
        runtime->runtime.pause();
#if SHARPA_MUJOCO_HAS_EMBEDDED_SIMULATOR
        if (runtime->simulator)
        {
            runtime->simulator->stop();
            runtime->simulator.reset();
        }
#endif
        runtime->runtime.unload();
    }
}

int sharpa_mujoco_runtime_start(sharpa_mujoco_runtime* runtime, char* error, size_t error_size)
{
    return guarded([&](std::string& detail) {
        if (!runtime)
        {
            detail = "Runtime must not be null.";
            return false;
        }
        return runtime->runtime.start(&detail);
    }, error, error_size);
}

void sharpa_mujoco_runtime_pause(sharpa_mujoco_runtime* runtime)
{
    if (runtime) runtime->runtime.pause();
}

int sharpa_mujoco_runtime_step(sharpa_mujoco_runtime* runtime, size_t count, char* error, size_t error_size)
{
    return guarded([&](std::string& detail) {
        if (!runtime)
        {
            detail = "Runtime must not be null.";
            return false;
        }
        return runtime->runtime.step(count, &detail);
    }, error, error_size);
}

int sharpa_mujoco_runtime_reset(sharpa_mujoco_runtime* runtime, char* error, size_t error_size)
{
    return guarded([&](std::string& detail) {
        if (!runtime)
        {
            detail = "Runtime must not be null.";
            return false;
        }
        return runtime->runtime.reset(&detail);
    }, error, error_size);
}

int sharpa_mujoco_runtime_save_xml(const sharpa_mujoco_runtime* runtime, const char* path, char* error, size_t error_size)
{
    return guarded([&](std::string& detail) {
        if (!runtime || !path)
        {
            detail = "Runtime and XML output path must not be null.";
            return false;
        }
        return runtime->runtime.save_xml(path, &detail);
    }, error, error_size);
}

int sharpa_mujoco_runtime_save_mjb(const sharpa_mujoco_runtime* runtime, const char* path, char* error, size_t error_size)
{
    return guarded([&](std::string& detail) {
        if (!runtime || !path)
        {
            detail = "Runtime and MJB output path must not be null.";
            return false;
        }
        return runtime->runtime.save_mjb(path, &detail);
    }, error, error_size);
}

int sharpa_mujoco_runtime_body_count(const sharpa_mujoco_runtime* runtime)
{
    return runtime ? runtime->runtime.model_info().body_count : 0;
}

int sharpa_mujoco_runtime_body_info(
    const sharpa_mujoco_runtime* runtime,
    int body_id,
    sharpa_mujoco_body_info* info,
    char* name,
    size_t name_size)
{
    if (!runtime || !info || body_id < 0)
    {
        return 0;
    }
    const sharpa::mujoco::ModelInfo model = runtime->runtime.model_info();
    if (body_id >= static_cast<int>(model.bodies.size()))
    {
        return 0;
    }
    const sharpa::mujoco::BodyInfo& body = model.bodies[static_cast<size_t>(body_id)];
    info->id = body.id;
    info->parent_id = body.parent_id;
    info->joint_address = body.joint_address;
    info->joint_count = body.joint_count;
    info->mocap_id = body.mocap_id;
    copy_error(body.name, name, name_size);
    return 1;
}

int sharpa_mujoco_runtime_copy_body_pose(
    const sharpa_mujoco_runtime* runtime,
    uint64_t* version,
    uint64_t* step_index,
    double* simulation_time,
    double* positions,
    size_t position_count,
    double* quaternions,
    size_t quaternion_count)
{
    return sharpa_mujoco_runtime_copy_body_pose_timed(
        runtime,
        version,
        step_index,
        simulation_time,
        positions,
        position_count,
        quaternions,
        quaternion_count,
        nullptr);
}

int sharpa_mujoco_runtime_copy_body_pose_timed(
    const sharpa_mujoco_runtime* runtime,
    uint64_t* version,
    uint64_t* step_index,
    double* simulation_time,
    double* positions,
    size_t position_count,
    double* quaternions,
    size_t quaternion_count,
    sharpa_mujoco_pose_timing* timing)
{
    const auto copy_started_at = timing
        ? std::chrono::steady_clock::now()
        : std::chrono::steady_clock::time_point{};
    if (!runtime || !version || !step_index || !simulation_time || !positions || !quaternions)
    {
        return 0;
    }
    sharpa::mujoco::SnapshotTiming snapshot_timing;
    const sharpa::mujoco::PoseSnapshot snapshot =
        runtime->runtime.latest_snapshot(timing ? &snapshot_timing : nullptr);
    const auto snapshot_read_finished_at = timing
        ? std::chrono::steady_clock::now()
        : std::chrono::steady_clock::time_point{};
    if (!snapshot.valid())
    {
        return 0;
    }
    if (position_count < snapshot.body_positions.size() ||
        quaternion_count < snapshot.body_quaternions.size())
    {
        return 0;
    }
    *version = snapshot.version;
    *step_index = snapshot.step_index;
    *simulation_time = snapshot.simulation_time;
    std::copy(snapshot.body_positions.begin(), snapshot.body_positions.end(), positions);
    std::copy(snapshot.body_quaternions.begin(), snapshot.body_quaternions.end(), quaternions);
    if (timing)
    {
        const auto copy_finished_at = std::chrono::steady_clock::now();
        const double pose_copy_seconds = std::chrono::duration<double>(
            copy_finished_at - copy_started_at).count();
        const double remaining_copy_seconds = std::chrono::duration<double>(
            copy_finished_at - snapshot_read_finished_at).count();
        timing->available = snapshot_timing.available ? 1 : 0;
        timing->version = snapshot_timing.version;
        timing->snapshot_age_seconds = snapshot_timing.available
            ? snapshot_timing.snapshot_age_seconds + remaining_copy_seconds
            : 0.0;
        timing->publish_interval_seconds = snapshot_timing.publish_interval_seconds;
        timing->simulation_step_seconds = snapshot_timing.simulation_step_seconds;
        timing->physics_step_seconds = snapshot_timing.physics_step_seconds;
        timing->snapshot_build_seconds = snapshot_timing.snapshot_build_seconds;
        timing->pose_copy_seconds = pose_copy_seconds;
    }
    return 1;
}

void sharpa_mujoco_runtime_set_pose_timing_enabled(
    sharpa_mujoco_runtime* runtime,
    int enabled)
{
    if (runtime)
    {
        runtime->runtime.set_timing_enabled(enabled != 0);
    }
}

int sharpa_mujoco_runtime_submit_control(
    sharpa_mujoco_runtime* runtime,
    const double* values,
    size_t value_count,
    int replace_all,
    char* error,
    size_t error_size)
{
    return guarded([&](std::string& detail) {
        if (!runtime || (!values && value_count > 0))
        {
            detail = "Runtime and control values must not be null.";
            return false;
        }
        sharpa::mujoco::ControlCommand command;
        command.replace_all = replace_all != 0;
        if (value_count > 0)
        {
            command.ordered_values.assign(values, values + value_count);
        }
        return runtime->runtime.submit_control(command, &detail);
    }, error, error_size);
}

#if SHARPA_MUJOCO_HAS_EMBEDDED_SIMULATOR
int sharpa_mujoco_runtime_configure_simulator(
    sharpa_mujoco_runtime* runtime,
    const char* config_path,
    char* error,
    size_t error_size)
{
    return guarded([&](std::string& detail) {
        if (!runtime)
        {
            detail = "Runtime must not be null.";
            return false;
        }
        const bool use_default = !config_path || config_path[0] == '\0';
        const std::string resolved = use_default
            ? default_simulator_config_path()
            : std::filesystem::absolute(config_path).lexically_normal().string();
        if (resolved.empty() || !std::filesystem::is_regular_file(resolved))
        {
            detail = "Simulator configuration does not exist: " + resolved;
            return false;
        }
        runtime->simulator =
            std::make_unique<sharpa::mujoco::simulator::SimulatorService>();
        if (!runtime->simulator->configure(
                runtime->runtime,
                resolved,
                use_default,
                &detail))
        {
            runtime->simulator.reset();
            return false;
        }
        return true;
    }, error, error_size);
}

int sharpa_mujoco_runtime_start_simulator(
    sharpa_mujoco_runtime* runtime,
    char* error,
    size_t error_size)
{
    return guarded([&](std::string& detail) {
        if (!runtime || !runtime->simulator)
        {
            detail = "Simulator service is not configured.";
            return false;
        }
        return runtime->simulator->start(runtime->runtime, &detail);
    }, error, error_size);
}

void sharpa_mujoco_runtime_stop_simulator(sharpa_mujoco_runtime* runtime)
{
    if (runtime && runtime->simulator)
    {
        runtime->simulator->stop();
    }
}

int sharpa_mujoco_runtime_simulator_status(
    const sharpa_mujoco_runtime* runtime,
    int* state,
    size_t* device_count,
    size_t* running_device_count,
    char* error,
    size_t error_size)
{
    if (!runtime || !state || !device_count || !running_device_count)
    {
        return 0;
    }
    if (!runtime->simulator)
    {
        *state = SHARPA_MUJOCO_SIMULATOR_UNCONFIGURED;
        *device_count = 0;
        *running_device_count = 0;
        copy_error({}, error, error_size);
        return 1;
    }
    const auto devices = runtime->simulator->device_status();
    *state = convert_simulator_state(runtime->simulator->state());
    *device_count = devices.size();
    *running_device_count = static_cast<size_t>(std::count_if(
        devices.begin(), devices.end(), [](const auto& device) {
            return device.running;
        }));
    copy_error(runtime->simulator->last_error(), error, error_size);
    return 1;
}

int sharpa_mujoco_runtime_simulator_device_info(
    const sharpa_mujoco_runtime* runtime,
    size_t index,
    char* name,
    size_t name_size,
    int* running)
{
    if (!runtime || !runtime->simulator || !name || name_size == 0 || !running)
    {
        return 0;
    }
    const auto devices = runtime->simulator->device_status();
    if (index >= devices.size())
    {
        return 0;
    }
    copy_error(devices[index].name, name, name_size);
    *running = devices[index].running ? 1 : 0;
    return 1;
}

int sharpa_mujoco_runtime_simulator_environment_status(
    const sharpa_mujoco_runtime* runtime,
    size_t* issue_count,
    char* error,
    size_t error_size)
{
    if (!runtime || !runtime->simulator || !issue_count)
    {
        copy_error("Simulator service is not configured.", error, error_size);
        return 0;
    }
    *issue_count = runtime->simulator->environment_issues().size();
    copy_error({}, error, error_size);
    return 1;
}

int sharpa_mujoco_runtime_simulator_environment_issue(
    const sharpa_mujoco_runtime* runtime,
    size_t index,
    int* code,
    int* can_prepare,
    char* resource,
    size_t resource_size,
    char* detail,
    size_t detail_size)
{
    if (!runtime || !runtime->simulator || !code || !can_prepare ||
        !resource || resource_size == 0 || !detail || detail_size == 0)
    {
        return 0;
    }
    const auto issues = runtime->simulator->environment_issues();
    if (index >= issues.size())
    {
        return 0;
    }
    const auto& issue = issues[index];
    *code = convert_environment_issue_code(issue.code);
    *can_prepare = issue.can_prepare_automatically ? 1 : 0;
    copy_error(issue.resource, resource, resource_size);
    copy_error(issue.detail, detail, detail_size);
    return 1;
}
#else
// Keep the C ABI available even when device simulation is not embedded.
int sharpa_mujoco_runtime_configure_simulator(
    sharpa_mujoco_runtime*, const char*, char* error, size_t error_size)
{
    return embedded_simulator_disabled(error, error_size);
}

int sharpa_mujoco_runtime_start_simulator(
    sharpa_mujoco_runtime*, char* error, size_t error_size)
{
    return embedded_simulator_disabled(error, error_size);
}

void sharpa_mujoco_runtime_stop_simulator(sharpa_mujoco_runtime*)
{
}

int sharpa_mujoco_runtime_simulator_status(
    const sharpa_mujoco_runtime*,
    int* state,
    size_t* device_count,
    size_t* running_device_count,
    char* error,
    size_t error_size)
{
    if (state) *state = SHARPA_MUJOCO_SIMULATOR_UNCONFIGURED;
    if (device_count) *device_count = 0;
    if (running_device_count) *running_device_count = 0;
    return embedded_simulator_disabled(error, error_size);
}

int sharpa_mujoco_runtime_simulator_device_info(
    const sharpa_mujoco_runtime*, size_t, char* name, size_t name_size, int* running)
{
    copy_error({}, name, name_size);
    if (running) *running = 0;
    return 0;
}

int sharpa_mujoco_runtime_simulator_environment_status(
    const sharpa_mujoco_runtime*, size_t* issue_count, char* error, size_t error_size)
{
    if (issue_count) *issue_count = 0;
    return embedded_simulator_disabled(error, error_size);
}

int sharpa_mujoco_runtime_simulator_environment_issue(
    const sharpa_mujoco_runtime*,
    size_t,
    int* code,
    int* can_prepare,
    char* resource,
    size_t resource_size,
    char* detail,
    size_t detail_size)
{
    if (code) *code = 0;
    if (can_prepare) *can_prepare = 0;
    copy_error({}, resource, resource_size);
    return embedded_simulator_disabled(detail, detail_size);
}
#endif

int sharpa_mujoco_scene_build(
    const sharpa_mujoco_scene* source,
    const char* output_xml_path,
    const char* output_mjb_path,
    char* error,
    size_t error_size)
{
    return guarded([&](std::string& detail) {
        if (!source || !source->robot_xml_path || !output_xml_path ||
            (!source->bodies && source->body_count > 0))
        {
            detail = "Scene, robot path, output path and body array must be valid.";
            return false;
        }
        return build_scene(
            *source, output_xml_path, output_mjb_path, detail);
    }, error, error_size);
}

int sharpa_mujoco_scene_validate(const char* xml_path, char* error, size_t error_size)
{
    return guarded([&](std::string& detail) {
        if (!xml_path)
        {
            detail = "XML path must not be null.";
            return false;
        }
        return sharpa::mujoco::SceneBuilder::validate_xml(xml_path, &detail);
    }, error, error_size);
}

}  // extern "C"
