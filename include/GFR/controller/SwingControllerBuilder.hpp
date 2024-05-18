#pragma once

#include "gfr/controller/SwingController.hpp"
#include "gfr/controller/SwingControllerBuilder.hpp"
#include "gfr/localizer/AbstractLocalizer.hpp"

namespace gfr::controller {

class SwingControllerBuilder {
  private:
    SwingController ctrl;

  public:
    SwingControllerBuilder(std::shared_ptr<localizer::AbstractLocalizer> l);

    static SwingControllerBuilder
    new_builder(std::shared_ptr<localizer::AbstractLocalizer> l);

    static SwingControllerBuilder from(SwingController swc);

    SwingControllerBuilder& with_angular_constants(double kP, double kI,
                                                   double kD);

    std::shared_ptr<SwingController> build();
};

} // namespace gfr::controller