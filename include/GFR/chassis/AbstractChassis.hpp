#pragma once

#include "pros/misc.hpp"
#include "pros/motors.hpp"

#include "ChassisCommand.hpp"
#include "gfr/controller/AbstractController.hpp"
#include "gfr/exit_conditions/AbstractExitCondition.hpp"

#include "gfr/utils/flags.hpp"
#include "gfr/utils/Point.hpp"
#include "gfr/utils/Pose.hpp"

namespace gfr::chassis {

using controller_ptr = std::shared_ptr<controller::AbstractController>;
using ec_ptr = std::shared_ptr<controller::AbstractExitCondition>;

class AbstractChassis {
  protected:
    controller_ptr default_controller;
    ec_ptr default_ec;
    std::unique_ptr<pros::Task> task = nullptr;
    bool task_running = false;
    pros::motor_brake_mode_e brakeMode;

    void move_task(controller_ptr controller, ec_ptr ec, double max,
                   gfr::Flags flags);

    void turn_task(controller_ptr controller, ec_ptr ec, double max,
                   gfr::Flags flags, gfr::AngularDirection direction);

  public:
    AbstractChassis(controller_ptr default_controller, ec_ptr ec);

    virtual void tank(double left_speed, double right_speed) = 0;
    virtual void arcade(double forward_speed, double turn_speed) = 0;

    virtual bool execute(DiffChassisCommand cmd, double max) = 0;
    virtual void set_brake_mode(pros::motor_brake_mode_e mode) = 0;

    void move(double distance, double max = 100.0,
              gfr::Flags flags = gfr::Flags::NONE);

    void move(double distance, controller_ptr controller, double max = 100.0,
              gfr::Flags flags = gfr::Flags::NONE);

    void move(double distance, controller_ptr controller, ec_ptr ec,
              double max = 100.0, gfr::Flags flags = gfr::Flags::NONE);

    void move(Pose target, controller_ptr controller, ec_ptr ec,
              double max = 100.0, gfr::Flags flags = gfr::Flags::NONE);

    void move(Pose target, controller_ptr controller, double max = 100.0,
              gfr::Flags flags = gfr::Flags::NONE);

    void move(Pose target, double max = 100.0,
              gfr::Flags flags = gfr::Flags::NONE);

    void turn(double target, controller_ptr controller, ec_ptr ec,
              double max = 100.0, gfr::Flags flags = gfr::Flags::NONE,
              gfr::AngularDirection direction = gfr::AngularDirection::AUTO);

    void turn(double target, controller_ptr controller, double max = 100.0,
              gfr::Flags flags = gfr::Flags::NONE,
              gfr::AngularDirection direction = gfr::AngularDirection::AUTO);

    void turn(double target, double max = 100.0,
              gfr::Flags flags = gfr::Flags::NONE,
              gfr::AngularDirection direction = gfr::AngularDirection::AUTO);

    void
    turn_to(Point target, controller_ptr controller, ec_ptr ec,
            double max = 100.0, gfr::Flags flags = gfr::Flags::NONE,
            gfr::AngularDirection direction = gfr::AngularDirection::AUTO);

    void
    turn_to(Point target, controller_ptr controller, double max = 100.0,
            gfr::Flags flags = gfr::Flags::NONE,
            gfr::AngularDirection direction = gfr::AngularDirection::AUTO);

    void
    turn_to(Point target, double max = 100.0,
            gfr::Flags flags = gfr::Flags::NONE,
            gfr::AngularDirection direction = gfr::AngularDirection::AUTO);
};

} // namespace gfr::chassis