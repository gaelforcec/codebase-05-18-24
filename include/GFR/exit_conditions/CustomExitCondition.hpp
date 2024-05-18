#pragma once
#include "../utils/Pose.hpp"
#include "AbstractExitCondition.hpp"
#include <functional>
namespace gfr::controller {
class CustomExitCondition : public AbstractExitCondition {
  private:
    std::function<bool()> callback;

  public:
    CustomExitCondition(std::function<bool()> callback);
    bool is_met(Pose current_pose, bool thru) override;
};
} // namespace gfr::controller