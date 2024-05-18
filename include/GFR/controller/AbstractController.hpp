#pragma once

#include "gfr/chassis/ChassisCommand.hpp"
#include "gfr/exit_conditions/AbstractExitCondition.hpp"
#include "gfr/localizer/AbstractLocalizer.hpp"
#include "gfr/utils/flags.hpp"

namespace gfr::controller {

class AbstractController {

  protected:
    std::shared_ptr<localizer::AbstractLocalizer> l;
    Pose target;
    double angular_target;

  public:
    AbstractController(std::shared_ptr<localizer::AbstractLocalizer> l);

    virtual chassis::DiffChassisCommand
    get_command(bool reverse, bool thru,
                std::shared_ptr<AbstractExitCondition> ec) = 0;
    virtual chassis::DiffChassisCommand
    get_angular_command(bool reverse, bool thru,
                        gfr::AngularDirection direction,
                        std::shared_ptr<AbstractExitCondition> ec) = 0;

    virtual void reset() = 0;

    void set_target(Pose target, bool relative,
                    std::shared_ptr<AbstractExitCondition> ec);
    void set_angular_target(double angle, bool relative);
};

} // namespace gfr::controller