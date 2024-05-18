#include "gfr/localizer/AbstractTrackingWheel.hpp"

namespace gfr::localizer {

AbstractTrackingWheel::AbstractTrackingWheel() {
}

double AbstractTrackingWheel::get_dist_travelled() {
    return this->get_raw_position() / this->tpi;
}

void AbstractTrackingWheel::set_tpi(double new_tpi) {
    this->tpi = new_tpi;
}

} // namespace gfr::localizer