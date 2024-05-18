#include "gfr/chassis/AbstractChassis.hpp"
#include "pros/llemu.hpp"
#include "pros/rtos.hpp"
#include "gfr/exit_conditions/AbstractExitCondition.hpp"

#include <cmath>

namespace gfr::chassis {

AbstractChassis::AbstractChassis(controller_ptr default_controller, ec_ptr ec) {
    this->default_controller = std::move(default_controller);
    this->default_ec = std::move(ec);
}

void AbstractChassis::move_task(controller_ptr controller, ec_ptr ec,
                                double max, gfr::Flags flags) {

    this->task =
        std::make_unique<pros::Task>([&, controller, ec, flags, max]() {
            ec->reset();
            controller->reset();
            while (!this->execute(
                controller->get_command(flags & gfr::Flags::REVERSE,
                                        flags & gfr::Flags::THRU, ec),
                max)) {
                if (pros::competition::is_disabled()) {
                    this->task_running = false;
                    return;
                }

                pros::delay(10);
            }
            this->task_running = false;
        });

    // Early exit for async movement
    if (flags & gfr::Flags::ASYNC) {
        return;
    }

    this->task->join();
}

void AbstractChassis::turn_task(controller_ptr controller, ec_ptr ec,
                                double max, gfr::Flags flags,
                                gfr::AngularDirection direction) {

    this->task = std::make_unique<pros::Task>(
        [&, controller, ec, flags, direction, max]() {
            ec->reset();
            controller->reset();
            while (!this->execute(controller->get_angular_command(
                                      flags & gfr::Flags::REVERSE,
                                      flags & gfr::Flags::THRU, direction, ec),
                                  max)) {
                if (pros::competition::is_disabled()) {
                    this->task_running = false;
                    return;
                }

                pros::delay(10);
            }
            this->task_running = false;
        });

    // Early exit for async movement
    if (flags & gfr::Flags::ASYNC) {
        return;
    }
    this->task->join();
}

void AbstractChassis::move(double distance, double max, gfr::Flags flags) {
    this->move({distance, 0}, this->default_controller, this->default_ec, max,
               flags | gfr::Flags::RELATIVE);
}

void AbstractChassis::move(double distance, controller_ptr controller,
                           double max, gfr::Flags flags) {
    this->move({distance, 0}, std::move(controller), this->default_ec, max,
               flags | gfr::Flags::RELATIVE);
}

void AbstractChassis::move(double distance, controller_ptr controller,
                           ec_ptr ec, double max, gfr::Flags flags) {
    this->move({distance, 0}, std::move(controller), std::move(ec), max,
               flags | Flags::RELATIVE);
}

void AbstractChassis::move(Pose target, double max, gfr::Flags flags) {
    this->move(target, this->default_controller, this->default_ec, max, flags);
}

void AbstractChassis::move(Pose target, controller_ptr controller, double max,
                           gfr::Flags flags) {
    this->move(target, std::move(controller), this->default_ec, max, flags);
}

void AbstractChassis::move(Pose target, controller_ptr controller, ec_ptr ec,
                           double max, gfr::Flags flags) {
    while (this->task_running) {
        pros::delay(10);
    }
    this->task_running = true;
    controller->set_target(target, flags & gfr::Flags::RELATIVE, ec);

    this->move_task(std::move(controller), std::move(ec), max, flags);
}

void AbstractChassis::turn(double target, double max, gfr::Flags flags,
                           gfr::AngularDirection direction) {
    this->turn(target, this->default_controller, this->default_ec, max, flags,
               direction);
}

void AbstractChassis::turn(double target, controller_ptr controller, double max,
                           gfr::Flags flags,
                           gfr::AngularDirection direction) {
    this->turn(target, std::move(controller), this->default_ec, max, flags,
               direction);
}

void AbstractChassis::turn(double target, controller_ptr controller, ec_ptr ec,
                           double max, gfr::Flags flags,
                           gfr::AngularDirection direction) {
    while (this->task_running) {
        pros::delay(10);
    }
    this->task_running = true;

    controller->set_target({NAN, NAN, target}, flags & gfr::Flags::RELATIVE,
                           ec);
    controller->set_angular_target(target, flags & gfr::Flags::RELATIVE);

    this->turn_task(std::move(controller), std::move(ec), max, flags,
                    direction);
}

void AbstractChassis::turn_to(Point target, double max, gfr::Flags flags,
                              gfr::AngularDirection direction) {
    this->turn_to(target, this->default_controller, this->default_ec, max,
                  flags, direction);
}

void AbstractChassis::turn_to(Point target, controller_ptr controller,
                              double max, gfr::Flags flags,
                              gfr::AngularDirection direction) {
    this->turn_to(target, std::move(controller), this->default_ec, max, flags,
                  direction);
}

void AbstractChassis::turn_to(Point target, controller_ptr controller,
                              ec_ptr ec, double max, gfr::Flags flags,
                              gfr::AngularDirection direction) {
    while (this->task_running) {
        pros::delay(10);
    }
    this->task_running = true;

    controller->set_target({target.x, target.y, std::nullopt},
                           flags & gfr::Flags::RELATIVE, ec);

    this->turn_task(std::move(controller), std::move(ec), max, flags,
                    direction);
}

} // namespace gfr::chassis