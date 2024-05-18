#include "gfr/controller/BoomerangControllerBuilder.hpp"

#include "gfr/controller/BoomerangController.hpp"
#include "gfr/localizer/AbstractLocalizer.hpp"
#include "gfr/utils/angle.hpp"
#include <utility>

namespace gfr::controller {

BoomerangControllerBuilder::BoomerangControllerBuilder(
    std::shared_ptr<localizer::AbstractLocalizer> l)
    : ctrl(std::move(l)) {
    this->ctrl.p = nullptr;
}

BoomerangControllerBuilder BoomerangControllerBuilder::new_builder(
    std::shared_ptr<localizer::AbstractLocalizer> l) {
    BoomerangControllerBuilder builder(std::move(l));
    return builder;
}

BoomerangControllerBuilder
BoomerangControllerBuilder::from(BoomerangController bmr) {
    BoomerangControllerBuilder builder(bmr.l);
    builder.ctrl = bmr;
    return builder;
}

BoomerangControllerBuilder&
BoomerangControllerBuilder::with_linear_constants(double kP, double kI,
                                                  double kD) {
    this->ctrl.linear_pid.set_constants(kP, kI, kD);
    return *this;
}

BoomerangControllerBuilder&
BoomerangControllerBuilder::with_angular_constants(double kP, double kI,
                                                   double kD) {
    this->ctrl.angular_pid.set_constants(kP, kI, kD);
    return *this;
}

BoomerangControllerBuilder&
BoomerangControllerBuilder::with_min_error(double error) {
    this->ctrl.min_error = error;
    return *this;
}


BoomerangControllerBuilder&
BoomerangControllerBuilder::with_lead_pct(double lead_pct) {
    this->ctrl.lead_pct = lead_pct;
    return *this;
}

BoomerangControllerBuilder&
BoomerangControllerBuilder::with_min_vel_for_thru(double min_vel) {
    this->ctrl.min_vel = min_vel;
    return *this;
}

std::shared_ptr<BoomerangController> BoomerangControllerBuilder::build() {
    return std::make_shared<BoomerangController>(this->ctrl);
}

} // namespace gfr::controller