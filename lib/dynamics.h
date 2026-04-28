#pragma once

#include "problem_description.h"

class CarDynamics {
public:
  using CarSettings = problem::CarSettings;
  using VehicleState = problem::VehicleState;
  using Vec3 = problem::Vec3;

  explicit CarDynamics(const CarSettings &car_settings);

  [[nodiscard]] const CarSettings &car() const noexcept { return car_; }

  [[nodiscard]] Vec3 positionAt(float t_s) const noexcept;
  [[nodiscard]] Vec3 velocityAt(float t_s) const noexcept;

private:
  CarSettings car_;
};
