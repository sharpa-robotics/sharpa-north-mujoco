// Copyright 2026 Sharpa
// SPDX-License-Identifier: Apache-2.0
#include "sharpa_mujoco/plugins/chassis/steer_position_profile.h"
#include <cmath>
#include <iostream>
#include <limits>

int main()
{
    using sharpa::mujoco::plugins::chassis::SteerPositionProfile;
    using sharpa::mujoco::plugins::chassis::kinco_profile_ramp_rad_s2;
    // Actual Platform encoding of 11.635528756 rad/s² for a 9:1 steer drive.
    if (std::abs(kinco_profile_ramp_rad_s2(17896, 65536, 9) - 11.635528756) > .001)
        return 8;
    SteerPositionProfile profile;
    auto last = profile.update(1.0, 0.2, 0.0, 2.0, 8.0, 8.0, 0.03, 0.1);
    for (int step = 1; step <= 2000; ++step)
    {
        const double t = step * .002;
        const auto q = profile.update(t < 1.5 ? 1.0 : -0.5, 0.2, t, 2.0, 8.0, 8.0, .03, .1);
        if ((t < .03 && q.position != .2) || std::abs(q.velocity) > 2.000001 ||
            std::abs(q.velocity - last.velocity) > .016001 ||
            std::abs(q.position - last.position) > .004001)
        {
            std::cerr << "Delay or motion bounds failed at " << t << '\n';
            return 1;
        }
        const auto duplicate = profile.update(t < 1.5 ? 1.0 : -.5, .2, t, 2, 8, 8, .03, .1);
        if (duplicate.position != q.position || duplicate.velocity != q.velocity) return 2;
        last = q;
    }
    if (std::abs(last.position + .5) > 1e-3 || std::abs(last.velocity) > 1e-3) return 3;
    // Timeout / reset must discard the old reference and delayed commands.
    auto reset = profile.update(1, .7, 8, 2, 8, 8, .03, .1);
    if (reset.position != .7 || reset.velocity != 0) return 4;
    reset = profile.update(1, -.2, 0, 2, 8, 8, .03, .1);
    if (reset.position != -.2 || reset.velocity != 0) return 5;
    profile.reset();
    reset = profile.update(1, .4, 0, 2, 8, 8, .03, .1);
    if (reset.position != .4 || reset.velocity != 0) return 6;
    reset = profile.update(std::numeric_limits<double>::quiet_NaN(), .4, .01,
                           2, 8, 8, .03, .1);
    if (reset.position != .4 || reset.velocity != 0) return 7;
}
