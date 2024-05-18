#pragma once
#include "AbstractExitCondition.hpp"
#include "gfr/utils/Pose.hpp"

namespace gfr::controller {
class PrepLineExitCondition : public AbstractExitCondition {
  private:
    double thru_smoothness;

  public:
    PrepLineExitCondition(double thru_smoothness);
    bool is_met(gfr::Pose pose, bool thru) override;
};
} // namespace gfr::controller
