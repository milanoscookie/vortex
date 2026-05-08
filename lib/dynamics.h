#pragma once

#include "problem_description.h"

class CarDynamics {
  public:
    using CarSettings = problem::CarSettings;
    using VehicleState = problem::VehicleState;
    using Vec3 = problem::Vec3;

    explicit CarDynamics(const CarSettings &car_settings);

    const CarSettings &car() const noexcept {
        return car_;
    }

    Vec3 positionAt(double t_s) const noexcept;
    Vec3 velocityAt(double t_s) const noexcept;

  private:
    CarSettings car_;
};
