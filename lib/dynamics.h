#pragma once

#include "problem_description.h"

class CarDynamics {
public:
  using CarDynamicsModel = problem::CarDynamicsModel;
  using CarState = problem::CarState;
  using Vec3 = problem::Vec3;

  explicit CarDynamics(const CarDynamicsModel &target);

  const CarDynamicsModel &target() const noexcept { return target_; }

  CarState stateAt(float t_s) const noexcept;
  Vec3 positionAt(float t_s) const noexcept;
  Vec3 velocityAt(float t_s) const noexcept;

private:
  CarDynamicsModel target_;
};
