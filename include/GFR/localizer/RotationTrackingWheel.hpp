#pragma once

#include "pros/rotation.hpp"
#include "gfr/localizer/AbstractTrackingWheel.hpp"
#include <memory>

namespace gfr::localizer {

class RotationTrackingWheel : public AbstractTrackingWheel {
  private:
    std::unique_ptr<pros::v5::Rotation> encoder;

  protected:
    double get_raw_position() override;

  public:
    RotationTrackingWheel(int port);
    void reset() override;
};

} // namespace gfr::localizer