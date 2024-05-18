#pragma once

#include "pros/adi.hpp"
#include "gfr/localizer/AbstractTrackingWheel.hpp"
#include <memory>

namespace gfr::localizer {

class ADITrackingWheel : public AbstractTrackingWheel {
  private:
    std::unique_ptr<pros::adi::Encoder> encoder;

  protected:
    double get_raw_position() override;

  public:
    ADITrackingWheel(int adi_port);
    ADITrackingWheel(int smart_port, int adi_port);
    void reset() override;
};

} // namespace gfr::localizer