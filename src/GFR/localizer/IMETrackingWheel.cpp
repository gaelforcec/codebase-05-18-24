#include "gfr/localizer/IMETrackingWheel.hpp"

namespace gfr::localizer {

IMETrackingWheel::IMETrackingWheel(int port) : AbstractTrackingWheel() {
    this->encoder = std::make_unique<pros::v5::Motor>(port);
}

double IMETrackingWheel::get_raw_position() {
    return this->encoder->get_position();
}

void IMETrackingWheel::reset() {
    this->encoder->tare_position();
}

} // namespace gfr::localizer