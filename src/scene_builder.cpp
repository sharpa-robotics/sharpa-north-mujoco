// Copyright 2026 Sharpa
// SPDX-License-Identifier: Apache-2.0

#include "sharpa_mujoco/scene_builder.h"

#include "mjcf_path_utils.h"

#include <mujoco/mujoco.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <filesystem>
#include <iterator>
#include <memory>
#include <set>
#include <unordered_map>

namespace sharpa::mujoco
{
namespace
{

using SpecPtr = std::unique_ptr<mjSpec, decltype(&mj_deleteSpec)>;
using ModelPtr = std::unique_ptr<mjModel, decltype(&mj_deleteModel)>;

struct SerializedMeshPath
{
    std::string mesh_name;
    std::filesystem::path absolute_path;
};

bool fail(const std::string& message, std::string* error)
{
    if (error)
    {
        *error = message;
    }
    return false;
}

bool create_parent_directories(
    const std::filesystem::path& output,
    const char* description,
    std::string* error)
{
    const std::filesystem::path parent = output.parent_path();
    if (parent.empty())
    {
        return true;
    }

    std::error_code filesystem_error;
    std::filesystem::create_directories(parent, filesystem_error);
    if (filesystem_error)
    {
        return fail(
            std::string("Could not create ") + description +
                " output directory: " + filesystem_error.message(),
            error);
    }
    return true;
}

SpecPtr parse_xml(const std::string& path, std::string* error)
{
    std::array<char, 2048> parse_error{};
    SpecPtr spec(
        mj_parseXML(path.c_str(), nullptr, parse_error.data(), static_cast<int>(parse_error.size())),
        &mj_deleteSpec);
    if (!spec && error)
    {
        *error = "Failed to parse MJCF '" + path + "': " + parse_error.data();
    }
    return spec;
}

mjsBody* world_body(mjSpec* spec)
{
    if (mjsBody* world = mjs_findBody(spec, "world"))
    {
        return world;
    }
    return mjs_asBody(mjs_firstElement(spec, mjOBJ_BODY));
}

mjsBody* find_robot_root(mjSpec* spec, const std::string& requested)
{
    if (!requested.empty())
    {
        return mjs_findBody(spec, requested.c_str());
    }
    mjsBody* world = world_body(spec);
    return world ? mjs_asBody(mjs_firstChild(world, mjOBJ_BODY, 0)) : nullptr;
}

void copy_pose(const Pose& pose, double* position, double* quaternion)
{
    std::copy(pose.position.begin(), pose.position.end(), position);
    std::copy(pose.quaternion.begin(), pose.quaternion.end(), quaternion);
}

mjtGeom geom_type(SceneGeomType type)
{
    switch (type)
    {
        case SceneGeomType::Sphere:
            return mjGEOM_SPHERE;
        case SceneGeomType::Capsule:
            return mjGEOM_CAPSULE;
        case SceneGeomType::Mesh:
            return mjGEOM_MESH;
        case SceneGeomType::Box:
        default:
            return mjGEOM_BOX;
    }
}

bool validate_description(const SceneDescription& scene, std::string* error)
{
    if (scene.robot_xml_path.empty())
    {
        return fail("SceneBuilder requires robot_xml_path.", error);
    }
    std::set<std::string> names;
    for (const SceneBodyDescription& body : scene.bodies)
    {
        if (body.name.empty() || !names.insert(body.name).second)
        {
            return fail("Scene body names must be non-empty and unique: " + body.name, error);
        }
        if (body.geoms.empty())
        {
            return fail("Scene body has no collision geoms: " + body.name, error);
        }
        for (const SceneGeomDescription& geom : body.geoms)
        {
            const bool has_memory_mesh =
                !geom.mesh_vertices.empty() || !geom.mesh_faces.empty();
            if (geom.type == SceneGeomType::Mesh &&
                !has_memory_mesh && geom.mesh_file.empty() && geom.mesh_name.empty())
            {
                return fail(
                    "Mesh geom requires mesh_file or an existing mesh_name on body: " +
                        body.name,
                    error);
            }
            if (has_memory_mesh &&
                (geom.mesh_vertices.size() < 9 || geom.mesh_vertices.size() % 3 != 0 ||
                 geom.mesh_faces.empty() || geom.mesh_faces.size() % 3 != 0))
            {
                return fail(
                    "In-memory mesh data must contain packed xyz vertices and triangular faces: " +
                        geom.name,
                    error);
            }
            if (has_memory_mesh)
            {
                const std::size_t vertex_count = geom.mesh_vertices.size() / 3;
                const bool finite_vertices = std::all_of(
                    geom.mesh_vertices.begin(), geom.mesh_vertices.end(),
                    [](double value) { return std::isfinite(value); });
                const bool valid_faces = std::all_of(
                    geom.mesh_faces.begin(), geom.mesh_faces.end(),
                    [vertex_count](int index)
                    {
                        return index >= 0 &&
                            static_cast<std::size_t>(index) < vertex_count;
                    });
                if (!finite_vertices || !valid_faces)
                {
                    return fail(
                        "In-memory mesh contains a non-finite vertex or out-of-range face index: " +
                            geom.name,
                        error);
                }
            }
            if (geom.override_solref &&
                (!std::isfinite(geom.solref[0]) || geom.solref[0] <= 0.0 ||
                 !std::isfinite(geom.solref[1]) || geom.solref[1] <= 0.0))
            {
                return fail(
                    "Geom solref must contain a positive finite time constant and damping ratio: " +
                        geom.name,
                    error);
            }
            if (geom.override_solimp &&
                (!std::isfinite(geom.solimp[0]) || geom.solimp[0] < 0.0 ||
                 !std::isfinite(geom.solimp[1]) || geom.solimp[1] < geom.solimp[0] ||
                 geom.solimp[1] > 1.0 ||
                 !std::isfinite(geom.solimp[2]) || geom.solimp[2] <= 0.0 ||
                 !std::isfinite(geom.solimp[3]) || geom.solimp[3] < 0.0 || geom.solimp[3] > 1.0 ||
                 !std::isfinite(geom.solimp[4]) || geom.solimp[4] < 1.0))
            {
                return fail(
                    "Geom solimp values are outside the supported MuJoCo ranges: " +
                        geom.name,
                    error);
            }
        }
    }
    return true;
}

}  // namespace

mjSpec* SceneBuilder::assemble(
    const SceneDescription& scene,
    std::string* error)
{
    if (!validate_description(scene, error))
    {
        return nullptr;
    }

    SpecPtr spec = parse_xml(scene.robot_xml_path, error);
    if (!spec)
    {
        return nullptr;
    }

    std::error_code filesystem_error;
    const std::filesystem::path model_directory = detail::model_directory(
        *spec, scene.robot_xml_path, filesystem_error);
    if (filesystem_error)
    {
        fail(
            "Could not resolve the robot model directory: " +
                filesystem_error.message(),
            error);
        return nullptr;
    }
    const std::filesystem::path source_mesh_directory =
        detail::asset_directory(model_directory, spec->compiler.meshdir);
    const std::filesystem::path scene_mesh_directory =
        scene.mesh_base_directory.empty()
            ? std::filesystem::path{}
            : std::filesystem::absolute(scene.mesh_base_directory, filesystem_error);
    if (filesystem_error)
    {
        fail(
            "Could not resolve the scene mesh directory: " +
                filesystem_error.message(),
            error);
        return nullptr;
    }

    std::unordered_map<std::string, std::string> mesh_names_by_path;
    for (mjsElement* element = mjs_firstElement(spec.get(), mjOBJ_MESH);
         element;
         element = mjs_nextElement(spec.get(), element))
    {
        mjsMesh* mesh = mjs_asMesh(element);
        const char* file = mesh ? mjs_getString(mesh->file) : nullptr;
        const char* mesh_name = mesh ? mjs_getString(mjs_getName(element)) : nullptr;
        if (!file || file[0] == '\0' || !mesh_name || mesh_name[0] == '\0')
        {
            continue;
        }
        std::filesystem::path absolute_path(file);
        if (absolute_path.is_relative())
        {
            absolute_path = source_mesh_directory / absolute_path;
        }
        mesh_names_by_path.emplace(
            absolute_path.lexically_normal().generic_string(), mesh_name);
    }

    mjsBody* robot_root = find_robot_root(spec.get(), scene.robot_root_body);
    if (!robot_root || robot_root == world_body(spec.get()))
    {
        fail(
            scene.robot_root_body.empty()
                ? "Robot MJCF has no direct root body under worldbody."
                : "Robot root body was not found: " + scene.robot_root_body,
            error);
        return nullptr;
    }
    copy_pose(scene.robot_world_pose, robot_root->pos, robot_root->quat);

    mjsBody* world = world_body(spec.get());
    if (!world)
    {
        fail("Robot MJCF has no worldbody.", error);
        return nullptr;
    }

    std::size_t generated_mesh_index = 0;
    for (const SceneBodyDescription& body_description : scene.bodies)
    {
        mjsBody* body = mjs_addBody(world, nullptr);
        if (!body || mjs_setName(body->element, body_description.name.c_str()) != 0)
        {
            fail("Could not add scene body: " + body_description.name, error);
            return nullptr;
        }
        copy_pose(body_description.world_pose, body->pos, body->quat);
        if (body_description.motion_type == SceneMotionType::Dynamic && !mjs_addFreeJoint(body))
        {
            fail("Could not add free joint to dynamic body: " + body_description.name, error);
            return nullptr;
        }

        for (std::size_t geom_index = 0; geom_index < body_description.geoms.size(); ++geom_index)
        {
            const SceneGeomDescription& description = body_description.geoms[geom_index];
            mjsGeom* geom = mjs_addGeom(body, nullptr);
            if (!geom)
            {
                fail("Could not add geom to body: " + body_description.name, error);
                return nullptr;
            }
            const std::string geom_name = description.name.empty()
                ? body_description.name + "_geom_" + std::to_string(geom_index)
                : description.name;
            if (mjs_setName(geom->element, geom_name.c_str()) != 0)
            {
                fail("Could not name scene geom: " + geom_name, error);
                return nullptr;
            }
            geom->type = geom_type(description.type);
            copy_pose(description.local_pose, geom->pos, geom->quat);
            std::copy(description.size.begin(), description.size.end(), geom->size);
            std::copy(description.friction.begin(), description.friction.end(), geom->friction);
            geom->density = description.density;
            if (description.mass > 0.0)
            {
                geom->mass = description.mass;
            }
            if (description.override_solref)
            {
                std::copy(description.solref.begin(), description.solref.end(), geom->solref);
            }
            if (description.override_solimp)
            {
                std::copy(description.solimp.begin(), description.solimp.end(), geom->solimp);
            }

            if (description.type != SceneGeomType::Mesh)
            {
                continue;
            }
            if (!description.mesh_name.empty() &&
                mjs_findElement(spec.get(), mjOBJ_MESH, description.mesh_name.c_str()))
            {
                mjs_setString(geom->meshname, description.mesh_name.c_str());
                continue;
            }

            std::string mesh_name = description.mesh_name;
            if (mesh_name.empty())
            {
                do
                {
                    mesh_name = "scene_mesh_" + std::to_string(generated_mesh_index++);
                }
                while (mjs_findElement(spec.get(), mjOBJ_MESH, mesh_name.c_str()));
            }
            else if (mjs_findElement(spec.get(), mjOBJ_MESH, mesh_name.c_str()))
            {
                fail("Mesh asset name already exists: " + mesh_name, error);
                return nullptr;
            }

            if (!description.mesh_vertices.empty())
            {
                mjsMesh* mesh = mjs_addMesh(spec.get(), nullptr);
                if (!mesh || mjs_setName(mesh->element, mesh_name.c_str()) != 0)
                {
                    fail("Could not create in-memory mesh asset: " + mesh_name, error);
                    return nullptr;
                }
                std::vector<float> vertices;
                vertices.reserve(description.mesh_vertices.size());
                std::transform(
                    description.mesh_vertices.begin(),
                    description.mesh_vertices.end(),
                    std::back_inserter(vertices),
                    [](double value) { return static_cast<float>(value); });
                mjs_setFloat(mesh->uservert, vertices.data(), static_cast<int>(vertices.size()));
                mjs_setInt(
                    mesh->userface,
                    description.mesh_faces.data(),
                    static_cast<int>(description.mesh_faces.size()));
                mjs_setString(geom->meshname, mesh_name.c_str());
                continue;
            }

            if (description.mesh_file.empty())
            {
                fail("Existing mesh asset was not found: " + mesh_name, error);
                return nullptr;
            }
            std::filesystem::path compilation_path(description.mesh_file);
            if (compilation_path.is_relative())
            {
                if (scene_mesh_directory.empty())
                {
                    fail("Relative scene mesh requires a mesh base directory: " + description.mesh_file, error);
                    return nullptr;
                }
                compilation_path = scene_mesh_directory / compilation_path;
            }
            compilation_path = compilation_path.lexically_normal();
            const std::string mesh_path_key = compilation_path.generic_string();
            auto found = mesh_names_by_path.find(mesh_path_key);
            if (found == mesh_names_by_path.end())
            {
                mjsMesh* mesh = mjs_addMesh(spec.get(), nullptr);
                if (!mesh || mjs_setName(mesh->element, mesh_name.c_str()) != 0)
                {
                    fail("Could not create mesh asset: " + mesh_name, error);
                    return nullptr;
                }
                mjs_setString(mesh->file, compilation_path.string().c_str());
                found = mesh_names_by_path.emplace(mesh_path_key, mesh_name).first;
            }
            mjs_setString(geom->meshname, found->second.c_str());
        }
    }

    if (error)
    {
        error->clear();
    }
    return spec.release();
}

bool SceneBuilder::build(
    const SceneDescription& scene,
    const SceneBuildOptions& options,
    std::string* error)
{
    if (scene.robot_xml_path.empty() || options.output_xml_path.empty())
    {
        return fail(
            "SceneBuilder requires robot_xml_path and output_xml_path.", error);
    }
    if (!validate_description(scene, error))
    {
        return false;
    }

    const std::filesystem::path xml_path(options.output_xml_path);
    if (!create_parent_directories(xml_path, "scene", error))
    {
        return false;
    }
    std::error_code filesystem_error;
    std::filesystem::path output_directory = xml_path.parent_path();
    if (output_directory.empty())
    {
        output_directory = std::filesystem::current_path(filesystem_error);
        if (filesystem_error)
        {
            return fail(
                "Could not resolve the current output directory: " +
                    filesystem_error.message(),
                error);
        }
    }
    else
    {
        output_directory = std::filesystem::absolute(
            output_directory, filesystem_error);
        if (filesystem_error)
        {
            return fail(
                "Could not resolve the scene output directory: " +
                    filesystem_error.message(),
                error);
        }
    }

    SceneDescription export_scene = scene;
    if (export_scene.mesh_base_directory.empty())
    {
        export_scene.mesh_base_directory = output_directory.string();
    }
    SpecPtr spec(SceneBuilder::assemble(export_scene, error), &mj_deleteSpec);
    if (!spec)
    {
        return false;
    }

    const std::filesystem::path model_directory = detail::model_directory(
        *spec, scene.robot_xml_path, filesystem_error);
    if (filesystem_error)
    {
        return fail(
            "Could not resolve the robot model directory: " +
                filesystem_error.message(),
            error);
    }
    const std::filesystem::path source_mesh_directory =
        detail::asset_directory(model_directory, spec->compiler.meshdir);

    std::vector<SerializedMeshPath> serialized_mesh_paths;
    for (mjsElement* element = mjs_firstElement(spec.get(), mjOBJ_MESH);
         element;
         element = mjs_nextElement(spec.get(), element))
    {
        mjsMesh* mesh = mjs_asMesh(element);
        const char* file = mesh ? mjs_getString(mesh->file) : nullptr;
        if (!file || file[0] == '\0')
        {
            continue;
        }
        const char* mesh_name = mjs_getString(mjs_getName(element));
        if (!mesh_name || mesh_name[0] == '\0')
        {
            return fail("Robot MJCF contains an unnamed file-backed mesh.", error);
        }
        std::filesystem::path absolute_path(file);
        if (absolute_path.is_relative())
        {
            absolute_path = source_mesh_directory / absolute_path;
        }
        serialized_mesh_paths.push_back(
            {
                mesh_name,
                absolute_path.lexically_normal(),
            });
    }

    ModelPtr assembled_model(mj_compile(spec.get(), nullptr), &mj_deleteModel);
    if (!assembled_model)
    {
        const char* detail = mjs_getError(spec.get());
        return fail("Assembled scene failed MuJoCo compilation: " + std::string(detail ? detail : "unknown error"), error);
    }

    for (const SerializedMeshPath& mesh_path : serialized_mesh_paths)
    {
        mjsMesh* mesh = mjs_asMesh(mjs_findElement(
            spec.get(), mjOBJ_MESH, mesh_path.mesh_name.c_str()));
        if (!mesh)
        {
            return fail(
                "Could not find mesh asset after scene compilation: " +
                    mesh_path.mesh_name,
                error);
        }
        const std::string serialized_path = detail::relative_or_absolute_path(
            mesh_path.absolute_path, output_directory);
        mjs_setString(mesh->file, serialized_path.c_str());
    }
    mjs_setString(spec->compiler.meshdir, ".");
    const std::filesystem::path source_texture_directory =
        detail::asset_directory(model_directory, spec->compiler.texturedir);
    const std::string serialized_texture_directory =
        detail::relative_or_absolute_path(
            source_texture_directory, output_directory);
    mjs_setString(
        spec->compiler.texturedir, serialized_texture_directory.c_str());
    spec->strippath = 0;
    mjs_setString(spec->modelfiledir, output_directory.string().c_str());

    ModelPtr serialized_model(mj_compile(spec.get(), nullptr), &mj_deleteModel);
    if (!serialized_model)
    {
        const char* detail = mjs_getError(spec.get());
        return fail(
            "Serialized scene failed MuJoCo compilation: " +
                std::string(detail ? detail : "unknown error"),
            error);
    }
    assembled_model = std::move(serialized_model);

    std::array<char, 2048> save_error{};
    if (mj_saveXML(spec.get(), options.output_xml_path.c_str(), save_error.data(), static_cast<int>(save_error.size())) != 0)
    {
        return fail("Could not save scene XML: " + std::string(save_error.data()), error);
    }

    if (!options.output_mjb_path.empty())
    {
        const std::filesystem::path mjb_path(options.output_mjb_path);
        if (!create_parent_directories(mjb_path, "MJB", error))
        {
            return false;
        }
        filesystem_error.clear();
        mj_saveModel(assembled_model.get(), options.output_mjb_path.c_str(), nullptr, 0);
        if (!std::filesystem::is_regular_file(mjb_path, filesystem_error) ||
            std::filesystem::file_size(mjb_path, filesystem_error) == 0)
        {
            return fail("MuJoCo did not create a valid MJB: " + options.output_mjb_path, error);
        }
    }

    if (error)
    {
        error->clear();
    }
    return true;
}

bool SceneBuilder::validate_xml(const std::string& xml_path, std::string* error)
{
    SpecPtr spec = parse_xml(xml_path, error);
    if (!spec)
    {
        return false;
    }
    ModelPtr model(mj_compile(spec.get(), nullptr), &mj_deleteModel);
    if (!model)
    {
        const char* detail = mjs_getError(spec.get());
        return fail("MJCF compilation failed: " + std::string(detail ? detail : "unknown error"), error);
    }
    if (error)
    {
        error->clear();
    }
    return true;
}

}  // namespace sharpa::mujoco
