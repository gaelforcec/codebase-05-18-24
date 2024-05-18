#include "gfr/controller/SwingControllerBuilder.hpp"

#include "gfr/controller/SwingController.hpp"
#include "gfr/localizer/AbstractLocalizer.hpp"
#include "gfr/utils/angle.hpp"
#include <utility>

namespace gfr::controller {

SwingControllerBuilder::SwingControllerBuilder(
    std::shared_ptr<localizer::AbstractLocalizer> l)
    : ctrl(std::move(l)) {

    this->ctrl.p = nullptr;
}

SwingControllerBuilder SwingControllerBuilder::new_builder(
    std::shared_ptr<localizer::AbstractLocalizer> l) {
    SwingControllerBuilder builder(std::move(l));
    return builder;
}

SwingControllerBuilder SwingControllerBuilder::from(SwingController swc) {
    SwingControllerBuilder builder(swc.l);
    builder.ctrl = swc;
    return builder;
}

SwingControllerBuilder&
SwingControllerBuilder::with_angular_constants(double kP, double kI,
                                               double kD) {
    this->ctrl.angular_pid.set_constants(kP, kI, kD);
    return *this;
}

std::shared_ptr<SwingController> SwingControllerBuilder::build() {
    return std::make_shared<SwingController>(this->ctrl);
}

} // namespace gfr::controller