// Copyright 2026 Sharpa
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include "sharpa_mujoco/export.h"
#include "sharpa_mujoco/types.h"

#include <array>
#include <string>
#include <vector>

struct mjSpec_;
typedef struct mjSpec_ mjSpec;

namespace sharpa::mujoco
{

enum class SceneMotionType
{
    Static,
    Dynamic,
};

enum class SceneGeomType
{
    Box,
    Sphere,
    Capsule,
    Mesh,
};

struct SceneGeomDescription
{
    std::string name;
    SceneGeomType type = SceneGeomType::Box;
    Pose local_pose;
    // MuJoCo units: box half extents, sphere radius, capsule radius/half-length.
    std::array<double, 3> size{0.05, 0.05, 0.05};
    std::array<double, 3> friction{1.0, 0.005, 0.0001};
    double density = 1000.0;
    // A positive value overrides density-derived mass.
    double mass = -1.0;
    bool override_solref = false;
    std::array<double, 2> solref{0.02, 1.0};
    bool override_solimp = false;
    std::array<double, 5> solimp{0.9, 0.95, 0.001, 0.5, 2.0};
    // For Mesh, mesh_name can select an existing robot mesh asset. Otherwise
    // mesh_file is resolved from SceneDescription::mesh_base_directory and
    // reused when another asset already references the same file.
    std::string mesh_file;
    std::string mesh_name;
    std::vector<double> mesh_vertices;
    std::vector<int> mesh_faces;
};

struct SceneBodyDescription
{
    std::string name;
    SceneMotionType motion_type = SceneMotionType::Static;
    Pose world_pose;
    std::vector<SceneGeomDescription> geoms;
};

struct SceneDescription
{
    std::string robot_xml_path;
    // Empty selects the first direct child body of worldbody.
    std::string robot_root_body;
    Pose robot_world_pose;
    std::vector<SceneBodyDescription> bodies;
    // Base directory for relative scene mesh_file values. Runtime assembly
    // does not depend on an exported XML file or its directory.
    std::string mesh_base_directory;
};

struct SceneBuildOptions
{
    std::string output_xml_path;
    // Optional. When present, the compiled model is saved after XML validation.
    std::string output_mjb_path;
};

class SHARPA_MUJOCO_API SceneBuilder
{
public:
    // Builds an editable MuJoCo specification without serializing it. The
    // caller owns the returned spec and must release it with mj_deleteSpec.
    static mjSpec* assemble(
        const SceneDescription& scene,
        std::string* error = nullptr);

    // File-backed mesh paths are serialized relative to the generated XML
    // whenever possible. This keeps colocated XML and mesh files portable.
    static bool build(
        const SceneDescription& scene,
        const SceneBuildOptions& options,
        std::string* error = nullptr);

    static bool validate_xml(const std::string& xml_path, std::string* error = nullptr);
};

}  // namespace sharpa::mujoco
