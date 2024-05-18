#pragma once

#include "gfr/controller/AbstractController.hpp"
#include "gfr/utils/PID.hpp"
#include <memory>

namespace gfr::controller {

class PIDController : public AbstractController {
  protected:
    std::shared_ptr<PIDController> p;

    utils::PID linear_pid, angular_pid;
    double min_error;
    bool can_reverse;

    double min_vel;
    bool turn_overshoot;

  public:
    PIDController(std::shared_ptr<localizer::AbstractLocalizer> l);

    chassis::DiffChassisCommand
    get_command(bool reverse, bool thru,
                std::shared_ptr<AbstractExitCondition> ec) override;
    chassis::DiffChassisCommand
    get_angular_command(bool reverse, bool thru,
                        gfr::AngularDirection direction,
                        std::shared_ptr<AbstractExitCondition> ec) override;

    void reset() override;

    std::shared_ptr<PIDController> modify_linear_constants(double kP, double kI,
                                                           double kD);
    std::shared_ptr<PIDController>
    modify_angular_constants(double kP, double kI, double kD);
    std::shared_ptr<PIDController> modify_min_error(double min_error);

    friend class PIDControllerBuilder;
    friend class BoomerangControllerBuilder;
};

} // namespace gfr::controller