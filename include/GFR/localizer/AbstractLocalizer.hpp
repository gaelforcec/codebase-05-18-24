#pragma once

#include "pros/rtos.hpp"
#include "gfr/utils/Point.hpp"
#include "gfr/utils/Pose.hpp"
#include <mutex>

namespace gfr::localizer {

class AbstractLocalizer {
  protected:
    pros::Mutex mtx;
    AtomicPose pose;

  public:
    AbstractLocalizer();

    virtual void update() = 0;
    void begin_localization();

    virtual void set_pose(Pose pose);
    virtual void set_pose(double x, double y, double theta);

    Pose get_pose();
    double get_x();
    double get_y();
    double get_orientation_rad();
    double get_orientation_deg();
    Point get_position();
    virtual void calibrate() = 0;
};

} // namespace gfr::localizer