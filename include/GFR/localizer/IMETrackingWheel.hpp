#pragma once

#include "pros/motors.hpp"
#include "gfr/localizer/AbstractTrackingWheel.hpp"
#include <memory>

namespace gfr::localizer {

class IMETrackingWheel : public AbstractTrackingWheel {
  private:
    std::unique_ptr<pros::v5::Motor> encoder;

  protected:
    double get_raw_position() override;

  public:
    IMETrackingWheel(int port);
    void reset() override;
};

} // namespace gfr::localizer