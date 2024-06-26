#include "gfr/localizer/TrackingWheelLocalizerBuilder.hpp"
#include "gfr/localizer/ADITrackingWheel.hpp"
#include "gfr/localizer/IMETrackingWheel.hpp"
#include "gfr/localizer/RotationTrackingWheel.hpp"

namespace gfr::localizer {

TrackingWheelLocalizerBuilder::TrackingWheelLocalizerBuilder()
    : left_right_dist(0.0), middle_dist(0.0), left_tracking_wheel(nullptr),
      right_tracking_wheel(nullptr), middle_tracking_wheel(nullptr),
      imu(nullptr) {
}

TrackingWheelLocalizerBuilder TrackingWheelLocalizerBuilder::new_builder() {
    return TrackingWheelLocalizerBuilder();
}

TrackingWheelLocalizerBuilder&
TrackingWheelLocalizerBuilder::with_left_encoder(int adi_port) {
    left_tracking_wheel = std::make_unique<ADITrackingWheel>(adi_port);
    return *this;
}

TrackingWheelLocalizerBuilder&
TrackingWheelLocalizerBuilder::with_left_encoder(int smart_port, int adi_port) {
    left_tracking_wheel =
        std::make_unique<ADITrackingWheel>(smart_port, adi_port);
    return *this;
}

TrackingWheelLocalizerBuilder&
TrackingWheelLocalizerBuilder::with_left_rotation(int port) {
    left_tracking_wheel = std::make_unique<RotationTrackingWheel>(port);
    return *this;
}

TrackingWheelLocalizerBuilder&
TrackingWheelLocalizerBuilder::with_left_motor(int port) {
    left_tracking_wheel = std::make_unique<IMETrackingWheel>(port);
    return *this;
}

TrackingWheelLocalizerBuilder&
TrackingWheelLocalizerBuilder::with_right_encoder(int adi_port) {
    right_tracking_wheel = std::make_unique<ADITrackingWheel>(adi_port);
    return *this;
}

TrackingWheelLocalizerBuilder&
TrackingWheelLocalizerBuilder::with_right_encoder(int smart_port,
                                                  int adi_port) {
    right_tracking_wheel =
        std::make_unique<ADITrackingWheel>(smart_port, adi_port);
    return *this;
}

TrackingWheelLocalizerBuilder&
TrackingWheelLocalizerBuilder::with_right_rotation(int port) {
    right_tracking_wheel = std::make_unique<RotationTrackingWheel>(port);
    return *this;
}

TrackingWheelLocalizerBuilder&
TrackingWheelLocalizerBuilder::with_right_motor(int port) {
    right_tracking_wheel = std::make_unique<IMETrackingWheel>(port);
    return *this;
}

TrackingWheelLocalizerBuilder&
TrackingWheelLocalizerBuilder::with_middle_encoder(int adi_port) {
    middle_tracking_wheel = std::make_unique<ADITrackingWheel>(adi_port);
    return *this;
}

TrackingWheelLocalizerBuilder&
TrackingWheelLocalizerBuilder::with_middle_encoder(int smart_port,
                                                   int adi_port) {
    middle_tracking_wheel =
        std::make_unique<ADITrackingWheel>(smart_port, adi_port);
    return *this;
}

TrackingWheelLocalizerBuilder&
TrackingWheelLocalizerBuilder::with_middle_rotation(int port) {
    middle_tracking_wheel = std::make_unique<RotationTrackingWheel>(port);
    return *this;
}

TrackingWheelLocalizerBuilder&
TrackingWheelLocalizerBuilder::with_middle_motor(int port) {
    middle_tracking_wheel = std::make_unique<IMETrackingWheel>(port);
    return *this;
}

TrackingWheelLocalizerBuilder&
TrackingWheelLocalizerBuilder::with_left_right_tpi(double tpi) {
    if (left_tracking_wheel) {
        left_tracking_wheel->set_tpi(tpi);
    }
    if (right_tracking_wheel) {
        right_tracking_wheel->set_tpi(tpi);
    }
    return *this;
}

TrackingWheelLocalizerBuilder&
TrackingWheelLocalizerBuilder::with_middle_tpi(double tpi) {
    middle_tracking_wheel->set_tpi(tpi);
    return *this;
}

TrackingWheelLocalizerBuilder&
TrackingWheelLocalizerBuilder::with_track_width(double track_width) {
    left_right_dist = track_width / 2;
    return *this;
}

TrackingWheelLocalizerBuilder&
TrackingWheelLocalizerBuilder::with_middle_dist(double middle_dist) {
    this->middle_dist = middle_dist;
    return *this;
}

TrackingWheelLocalizerBuilder&
TrackingWheelLocalizerBuilder::with_imu(int port) {
    imu = std::make_unique<pros::IMU>(port);
    return *this;
}

std::shared_ptr<TrackingWheelLocalizer> TrackingWheelLocalizerBuilder::build() {
    return std::make_shared<TrackingWheelLocalizer>(
        std::move(left_tracking_wheel), std::move(right_tracking_wheel),
        std::move(middle_tracking_wheel), std::move(imu), left_right_dist,
        middle_dist);
}

} // namespace gfr::localizer