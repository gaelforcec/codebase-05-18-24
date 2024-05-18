#pragma once

#include "gfr/controller/AbstractController.hpp"
#include "gfr/utils/PID.hpp"
#include <memory>

namespace gfr::controller {

class ArcPIDController : public AbstractController {
  protected:
    std::shared_ptr<ArcPIDController> p;
    utils::PID linear_pid;
    utils::PID angular_pid;
    double track_width;
    double min_error;
    double can_reverse;
    double arc_radius;
    Point arc_center;
    double prev_t;
    double slew;
    double prev_lin_speed;

    double close;

    double prev_lin_err, total_lin_err;

  public:
    ArcPIDController(std::shared_ptr<localizer::AbstractLocalizer> l);

    chassis::DiffChassisCommand
    get_command(bool reverse, bool thru,
                std::shared_ptr<AbstractExitCondition> ec) override;
    chassis::DiffChassisCommand
    get_angular_command(bool reverse, bool thru,
                        gfr::AngularDirection direction,
                        std::shared_ptr<AbstractExitCondition> ec) override;

    void reset() override;

    std::shared_ptr<ArcPIDController>
    modify_linear_constants(double kP, double kI, double kD);
    std::shared_ptr<ArcPIDController> modify_track_width(double track_width);
    std::shared_ptr<ArcPIDController> modify_min_error(double error);
    std::shared_ptr<ArcPIDController> modify_slew(double slew);

    friend class ArcPIDControllerBuilder;
};

} // namespace gfr::controller