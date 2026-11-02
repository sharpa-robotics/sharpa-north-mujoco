// Copyright 2026 Sharpa
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <stddef.h>
#include <stdint.h>

#if defined(_WIN32)
#  if defined(SHARPA_MUJOCO_C_EXPORTS)
#    define SHARPA_MUJOCO_C_API __declspec(dllexport)
#  else
#    define SHARPA_MUJOCO_C_API __declspec(dllimport)
#  endif
#else
#  define SHARPA_MUJOCO_C_API __attribute__((visibility("default")))
#endif

#ifdef __cplusplus
extern "C" {
#endif

typedef struct sharpa_mujoco_runtime sharpa_mujoco_runtime;

typedef struct sharpa_mujoco_companion sharpa_mujoco_companion;
enum sharpa_mujoco_companion_state
{
    SHARPA_MUJOCO_COMPANION_STOPPED = 0,
    SHARPA_MUJOCO_COMPANION_STARTING = 1,
    SHARPA_MUJOCO_COMPANION_RUNNING = 2,
    SHARPA_MUJOCO_COMPANION_FAILED = 3,
};

/* The caller owns the companion independently of its Runtime. Prepare before
 * starting simulator devices; it exclusively reserves the companion and
 * reclaims a safely identified process group left by a terminated owner
 * without launching a new child. Start after simulator devices are running;
 * stop before pausing them. A null root locates the named installed runtime
 * beside this library. A null profile selects its default. RUNNING describes
 * process liveness, not application readiness. Neither prepare nor start stops
 * a companion still locked by another live owner. */
SHARPA_MUJOCO_C_API sharpa_mujoco_companion* sharpa_mujoco_companion_create(void);
SHARPA_MUJOCO_C_API void sharpa_mujoco_companion_destroy(sharpa_mujoco_companion* companion);
SHARPA_MUJOCO_C_API int sharpa_mujoco_companion_prepare(
    sharpa_mujoco_companion* companion, const char* name, const char* root,
    const char* profile, char* error, size_t error_size);
SHARPA_MUJOCO_C_API int sharpa_mujoco_companion_start(
    sharpa_mujoco_companion* companion, const char* name, const char* root,
    const char* profile, char* error, size_t error_size);
SHARPA_MUJOCO_C_API void sharpa_mujoco_companion_stop(sharpa_mujoco_companion* companion);
SHARPA_MUJOCO_C_API int sharpa_mujoco_companion_status(
    const sharpa_mujoco_companion* companion, int* state, int* pid, int* exit_code,
    char* error, size_t error_size);
SHARPA_MUJOCO_C_API int sharpa_mujoco_companion_log_path(
    const sharpa_mujoco_companion* companion, char* path, size_t path_size);

enum sharpa_mujoco_simulator_state
{
    SHARPA_MUJOCO_SIMULATOR_UNCONFIGURED = 0,
    SHARPA_MUJOCO_SIMULATOR_CONFIGURED = 1,
    SHARPA_MUJOCO_SIMULATOR_RUNNING = 2,
    SHARPA_MUJOCO_SIMULATOR_FAULTED = 3,
};

enum sharpa_mujoco_environment_issue_code
{
    SHARPA_MUJOCO_ENVIRONMENT_SOCKETCAN_MISSING = 1,
    SHARPA_MUJOCO_ENVIRONMENT_SOCKETCAN_DOWN = 2,
    SHARPA_MUJOCO_ENVIRONMENT_SOCKETCAN_UNUSABLE = 3,
};

SHARPA_MUJOCO_C_API const char* sharpa_mujoco_version_string(void);
SHARPA_MUJOCO_C_API const char* sharpa_mujoco_mujoco_version_string(void);

typedef struct sharpa_mujoco_pose
{
    double position[3];
    double quaternion[4];
} sharpa_mujoco_pose;

typedef struct sharpa_mujoco_body_info
{
    int id;
    int parent_id;
    int joint_address;
    int joint_count;
    int mocap_id;
} sharpa_mujoco_body_info;

typedef struct sharpa_mujoco_pose_timing
{
    int available;
    uint64_t version;
    double snapshot_age_seconds;
    double publish_interval_seconds;
    double simulation_step_seconds;
    double physics_step_seconds;
    double snapshot_build_seconds;
    double pose_copy_seconds;
} sharpa_mujoco_pose_timing;

enum sharpa_mujoco_motion_type
{
    SHARPA_MUJOCO_MOTION_STATIC = 0,
    SHARPA_MUJOCO_MOTION_DYNAMIC = 1,
};

enum sharpa_mujoco_geom_type
{
    SHARPA_MUJOCO_BOX = 0,
    SHARPA_MUJOCO_SPHERE = 1,
    SHARPA_MUJOCO_CAPSULE = 2,
    SHARPA_MUJOCO_MESH = 3,
};

enum sharpa_mujoco_scene_contact_flags
{
    SHARPA_MUJOCO_SCENE_CONTACT_SOLREF = 1u << 0,
    SHARPA_MUJOCO_SCENE_CONTACT_SOLIMP = 1u << 1,
};

typedef struct sharpa_mujoco_scene_contact_parameters
{
    uint32_t flags;
    double solref[2];
    double solimp[5];
} sharpa_mujoco_scene_contact_parameters;

/* Initialize scene structures to zero. Zero contact flags preserve the model
 * defaults. This unpublished interface requires matching SDK/consumer builds. */
typedef struct sharpa_mujoco_scene_geom
{
    const char* name;
    int type;
    sharpa_mujoco_pose local_pose;
    double size[3];
    double friction[3];
    double density;
    double mass;
    const char* mesh_file;
    const char* mesh_name;
    sharpa_mujoco_scene_contact_parameters contact;
    /* Optional in-memory mesh data. Counts are vertices/faces, with three
       tightly-packed values per entry. This takes precedence over mesh_file. */
    const double* mesh_vertices;
    size_t mesh_vertex_count;
    const int* mesh_faces;
    size_t mesh_face_count;
} sharpa_mujoco_scene_geom;

typedef struct sharpa_mujoco_scene_body
{
    const char* name;
    int motion_type;
    sharpa_mujoco_pose world_pose;
    const sharpa_mujoco_scene_geom* geoms;
    size_t geom_count;
} sharpa_mujoco_scene_body;

typedef struct sharpa_mujoco_scene
{
    const char* robot_xml_path;
    const char* robot_root_body;
    sharpa_mujoco_pose robot_world_pose;
    const sharpa_mujoco_scene_body* bodies;
    size_t body_count;
    /* Base directory for relative mesh_file values. May be null when all
       scene meshes are in memory or use absolute paths. */
    const char* mesh_base_directory;
} sharpa_mujoco_scene;

SHARPA_MUJOCO_C_API sharpa_mujoco_runtime* sharpa_mujoco_runtime_create(void);
SHARPA_MUJOCO_C_API void sharpa_mujoco_runtime_destroy(sharpa_mujoco_runtime* runtime);
SHARPA_MUJOCO_C_API int sharpa_mujoco_runtime_load(
    sharpa_mujoco_runtime* runtime, const char* path, char* error, size_t error_size);
/* Assemble and compile a scene directly into the Runtime without writing XML
 * or MJB artifacts. */
SHARPA_MUJOCO_C_API int sharpa_mujoco_runtime_load_scene(
    sharpa_mujoco_runtime* runtime,
    const sharpa_mujoco_scene* scene,
    char* error,
    size_t error_size);
SHARPA_MUJOCO_C_API void sharpa_mujoco_runtime_unload(sharpa_mujoco_runtime* runtime);
SHARPA_MUJOCO_C_API int sharpa_mujoco_runtime_start(
    sharpa_mujoco_runtime* runtime, char* error, size_t error_size);
SHARPA_MUJOCO_C_API void sharpa_mujoco_runtime_pause(sharpa_mujoco_runtime* runtime);
SHARPA_MUJOCO_C_API int sharpa_mujoco_runtime_step(
    sharpa_mujoco_runtime* runtime, size_t count, char* error, size_t error_size);
SHARPA_MUJOCO_C_API int sharpa_mujoco_runtime_reset(
    sharpa_mujoco_runtime* runtime, char* error, size_t error_size);
SHARPA_MUJOCO_C_API int sharpa_mujoco_runtime_save_xml(
    const sharpa_mujoco_runtime* runtime, const char* path, char* error, size_t error_size);
SHARPA_MUJOCO_C_API int sharpa_mujoco_runtime_save_mjb(
    const sharpa_mujoco_runtime* runtime, const char* path, char* error, size_t error_size);
SHARPA_MUJOCO_C_API int sharpa_mujoco_runtime_body_count(const sharpa_mujoco_runtime* runtime);
SHARPA_MUJOCO_C_API int sharpa_mujoco_runtime_body_info(
    const sharpa_mujoco_runtime* runtime,
    int body_id,
    sharpa_mujoco_body_info* info,
    char* name,
    size_t name_size);
SHARPA_MUJOCO_C_API int sharpa_mujoco_runtime_copy_body_pose(
    const sharpa_mujoco_runtime* runtime,
    uint64_t* version,
    uint64_t* step_index,
    double* simulation_time,
    double* positions,
    size_t position_count,
    double* quaternions,
    size_t quaternion_count);
SHARPA_MUJOCO_C_API int sharpa_mujoco_runtime_copy_body_pose_timed(
    const sharpa_mujoco_runtime* runtime,
    uint64_t* version,
    uint64_t* step_index,
    double* simulation_time,
    double* positions,
    size_t position_count,
    double* quaternions,
    size_t quaternion_count,
    sharpa_mujoco_pose_timing* timing);
SHARPA_MUJOCO_C_API void sharpa_mujoco_runtime_set_pose_timing_enabled(
    sharpa_mujoco_runtime* runtime,
    int enabled);
SHARPA_MUJOCO_C_API int sharpa_mujoco_runtime_submit_control(
    sharpa_mujoco_runtime* runtime,
    const double* values,
    size_t value_count,
    int replace_all,
    char* error,
    size_t error_size);

/*
 * Configure the complete embedded device simulator around an already loaded
 * Runtime. A null or empty config_path selects the SDK's installed whole-body
 * loopback profile. Configuration may attach sensor models, so callers must
 * query body information after this function succeeds.
 *
 * The simulator functions below require the library to be built with
 * SHARPA_MUJOCO_BUILD_EMBEDDED_SIMULATOR=ON (default: OFF). Their symbols are
 * always exported. When disabled, all int-returning simulator functions return
 * 0; error/detail buffers report that the feature is disabled. Status, count
 * and flag outputs are reset to zero (state: UNCONFIGURED), and name/resource
 * buffers are emptied. stop_simulator is a no-op. Core Runtime and scene
 * functions remain available; the standalone simulator is controlled separately
 * by SHARPA_MUJOCO_BUILD_SIMULATOR.
 */
SHARPA_MUJOCO_C_API int sharpa_mujoco_runtime_configure_simulator(
    sharpa_mujoco_runtime* runtime,
    const char* config_path,
    char* error,
    size_t error_size);
SHARPA_MUJOCO_C_API int sharpa_mujoco_runtime_start_simulator(
    sharpa_mujoco_runtime* runtime,
    char* error,
    size_t error_size);
SHARPA_MUJOCO_C_API void sharpa_mujoco_runtime_stop_simulator(
    sharpa_mujoco_runtime* runtime);
SHARPA_MUJOCO_C_API int sharpa_mujoco_runtime_simulator_status(
    const sharpa_mujoco_runtime* runtime,
    int* state,
    size_t* device_count,
    size_t* running_device_count,
    char* error,
    size_t error_size);
SHARPA_MUJOCO_C_API int sharpa_mujoco_runtime_simulator_device_info(
    const sharpa_mujoco_runtime* runtime,
    size_t index,
    char* name,
    size_t name_size,
    int* running);
/* Read-only environment preflight. Query issue_count first, then retrieve each
 * issue by index. can_prepare is true only for a safe virtual-CAN interface;
 * the library itself never elevates privileges or changes host networking. */
SHARPA_MUJOCO_C_API int sharpa_mujoco_runtime_simulator_environment_status(
    const sharpa_mujoco_runtime* runtime,
    size_t* issue_count,
    char* error,
    size_t error_size);
SHARPA_MUJOCO_C_API int sharpa_mujoco_runtime_simulator_environment_issue(
    const sharpa_mujoco_runtime* runtime,
    size_t index,
    int* code,
    int* can_prepare,
    char* resource,
    size_t resource_size,
    char* detail,
    size_t detail_size);

SHARPA_MUJOCO_C_API int sharpa_mujoco_scene_build(
    const sharpa_mujoco_scene* scene,
    const char* output_xml_path,
    const char* output_mjb_path,
    char* error,
    size_t error_size);
SHARPA_MUJOCO_C_API int sharpa_mujoco_scene_validate(
    const char* xml_path, char* error, size_t error_size);

#ifdef __cplusplus
}
#endif
