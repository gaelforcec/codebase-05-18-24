#include "gfr/exit_conditions/CustomExitCondition.hpp"

#include <utility>

namespace gfr::controller {
CustomExitCondition::CustomExitCondition(std::function<bool()> callback)
    : callback(std::move(callback)){};

bool CustomExitCondition::is_met(gfr::Pose current_pose, bool thru) {
    return this->callback();
}

} // namespace gfr::controller