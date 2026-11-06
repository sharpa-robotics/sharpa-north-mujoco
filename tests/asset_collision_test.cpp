// Copyright 2026 Sharpa
// SPDX-License-Identifier: Apache-2.0
#include <mujoco/mujoco.h>
#include <algorithm>
#include <array>
#include <cmath>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <vector>

namespace
{
void require(bool value, const char* message)
{
    if (!value) throw std::runtime_error(message);
}

struct Result
{
    double error = 0;
    double minimum_force = 1e9;
    double maximum_penetration = 0;
    int contact_steps = 0;
    std::array<int, 4> support_steps{};
};

Result simulate(double target, bool obstacle, bool self_contact)
{
    char error[4096]{};
    std::unique_ptr<mjSpec, decltype(&mj_deleteSpec)> spec(
        mj_parseXML(SHARPA_MUJOCO_COLLISION_SCENE, nullptr, error, sizeof(error)), mj_deleteSpec);
    if (!spec) throw std::runtime_error(error);
    if (obstacle)
    {
        auto* geom = mjs_addGeom(mjs_findBody(spec.get(), "world"), nullptr);
        mjs_setName(geom->element, "test_obstacle");
        geom->type = mjGEOM_SPHERE;
        geom->size[0] = 0.06;
        geom->pos[0] = -0.07; geom->pos[1] = 0.49; geom->pos[2] = 1.15;
        geom->contype = 4096; geom->conaffinity = 0;
    }
    std::unique_ptr<mjModel, decltype(&mj_deleteModel)> model(mj_compile(spec.get(), nullptr), mj_deleteModel);
    if (!model) throw std::runtime_error(mjs_getError(spec.get()));
    const auto* m = model.get();
    std::unique_ptr<mjData, decltype(&mj_deleteData)> data(mj_makeData(m), mj_deleteData);
    auto* d = data.get();
    require(d != nullptr, "Could not allocate collision data");
    const int joint = mj_name2id(m, mjOBJ_JOINT, "left_arm_joint_2");
    const int motor = mj_name2id(m, mjOBJ_ACTUATOR, "left_arm_joint_2_ctrl");
    const int brake = mj_name2id(m, mjOBJ_EQUALITY, "brake_left_arm_joint_2");
    const int arm = mj_name2id(m, mjOBJ_GEOM, "left_arm_link_5_collision_07");
    const int other = mj_name2id(m, mjOBJ_GEOM,
        self_contact ? "lower_body_link_2_collision_17" : "test_obstacle");
    require(joint >= 0 && motor >= 0 && brake >= 0 && arm >= 0, "Precise model interface changed");
    require(!obstacle || other >= 0, "Obstacle missing");
    const int q = m->jnt_qposadr[joint], v = m->jnt_dofadr[joint];
    const std::vector<int> types(m->geom_contype, m->geom_contype + m->ngeom);
    const std::vector<int> affinity(m->geom_conaffinity, m->geom_conaffinity + m->ngeom);
    // Initial pose only. Every subsequent state change is through mj_step/actuator torque.
    d->qpos[q] = -0.3;
    d->eq_active[brake] = 0;
    mj_forward(m, d);
    Result result;
    for (int step = 0; step < 1500; ++step)
    {
        d->ctrl[motor] = std::clamp(120 * (target - d->qpos[q]) -
            20 * d->qvel[v] + d->qfrc_bias[v], -40.0, 40.0);
        mj_step(m, d);
        require(std::isfinite(d->qpos[q]) && std::isfinite(d->qvel[v]), "Unstable collision simulation");
        double force = 0;
        std::array<bool, 4> supported{};
        for (int index = 0; index < d->ncon; ++index)
        {
            const auto& contact = d->contact[index];
            const int b1 = m->geom_bodyid[contact.geom1], b2 = m->geom_bodyid[contact.geom2];
            require(b1 == 0 || b2 == 0 || (m->body_parentid[b1] != b2 && m->body_parentid[b2] != b1),
                    "Unexpected adjacent-body contact");
            if (step < 1000) continue;
            mjtNum wrench[6]{};
            mj_contactForce(m, d, index, wrench);
            if ((contact.geom1 == arm && contact.geom2 == other) ||
                (contact.geom2 == arm && contact.geom1 == other))
            {
                require(contact.efc_address >= 0, "Detected contact has no active constraint");
                force += std::abs(wrench[0]);
                result.maximum_penetration = std::max(result.maximum_penetration, -contact.dist);
            }
            for (int wheel = 0; wheel < 4; ++wheel)
            {
                const std::string name = "chassis_wheel_link_" + std::to_string(wheel + 1);
                const int body = mj_name2id(m, mjOBJ_BODY, name.c_str());
                if ((b1 == 0 && b2 == body) || (b2 == 0 && b1 == body))
                    supported[wheel] = supported[wheel] || wrench[0] > 1;
            }
        }
        if (step >= 1000)
        {
            if (force > 1e-4) ++result.contact_steps;
            result.minimum_force = std::min(result.minimum_force, force);
            for (int wheel = 0; wheel < 4; ++wheel)
                if (supported[wheel]) ++result.support_steps[wheel];
        }
    }
    require(std::equal(types.begin(), types.end(), m->geom_contype) &&
            std::equal(affinity.begin(), affinity.end(), m->geom_conaffinity),
            "Robot collision masks changed during validation");
    result.error = std::abs(target - d->qpos[q]);
    for (const auto count : result.support_steps)
        require(count >= 450, "A wheel lacks sustained ground support");
    std::cout << "target=" << target << " obstacle=" << obstacle << " self=" << self_contact
              << " error_rad=" << result.error << " contact_steps=" << result.contact_steps
              << " min_force_N=" << result.minimum_force
              << " max_penetration_m=" << result.maximum_penetration << '\n';
    return result;
}
}

int main()
{
    try
    {
        const auto external = simulate(-1.0, true, false);
        const auto external_free = simulate(-1.0, false, false);
        const auto self = simulate(0.25, false, true);
        const auto reachable = simulate(-0.15, false, true);
        for (const auto& blocked : {external, self})
        {
            require(blocked.contact_steps >= 450 && blocked.minimum_force > 10,
                    "No sustained physical contact force");
            require(blocked.error > 0.1 && blocked.maximum_penetration < 0.002,
                    "Contact did not prevent penetration/motion");
        }
        require(external_free.error < 0.01 && external_free.contact_steps == 0 &&
                reachable.error < 0.01 && reachable.contact_steps == 0,
                "Free-motion control cannot reach the target; blocking is inconclusive");
        return 0;
    }
    catch (const std::exception& error)
    {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
