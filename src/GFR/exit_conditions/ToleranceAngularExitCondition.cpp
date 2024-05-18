#include "gfr/exit_conditions/ToleranceAngularExitCondition.hpp"
#include "gfr/utils/angle.hpp"
#include <cmath>
#include <cstdio>

namespace gfr::controller {

ToleranceAngularExitCondition::ToleranceAngularExitCondition(
    double tolerance, double tolerance_time)
    : tolerance(gfr::to_radians(tolerance)), tolerance_time(tolerance_time) {
}

bool ToleranceAngularExitCondition::is_met(Pose current_pose, bool thru) {
    if (std::abs(gfr::norm_delta(current_pose.theta.value() -
                                  this->target_pose.theta.value())) <
        this->tolerance) {
        if (thru) {
            return true;
        }
        current_time += 10;
    } else {
        current_time = 0;
    }
    return current_time > tolerance_time;
}

void ToleranceAngularExitCondition::reset() {
    current_time = 0;
}

} // namespace gfr::controller