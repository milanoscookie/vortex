#include "dynamics.h"

#include <cmath>

CarDynamics::CarDynamics(const CarDynamicsModel &target) : target_(target) {}

CarDynamics::CarState CarDynamics::stateAt(float t_s) const noexcept {
  CarState state;
  state.center_m = positionAt(t_s);
  state.velocity_mps = velocityAt(t_s);
  state.yaw_rad = target_.yaw_rad;
  state.size_m = Vec3(target_.length_m, target_.width_m, target_.height_m);
  state.reflectivity = target_.reflectivity;
  return state;
}

CarDynamics::Vec3 CarDynamics::positionAt(float t_s) const noexcept {
  Vec3 position = target_.initial_position_m + target_.base_velocity_mps * t_s;
  position.z() +=
      target_.bounce_amplitude_m *
      std::sin(2.0f * problem::kPi * target_.bounce_frequency_hz * t_s +
               target_.bounce_phase_rad);
  return position;
}

CarDynamics::Vec3 CarDynamics::velocityAt(float t_s) const noexcept {
  Vec3 velocity = target_.base_velocity_mps;
  velocity.z() +=
      2.0f * problem::kPi * target_.bounce_frequency_hz *
      target_.bounce_amplitude_m *
      std::cos(2.0f * problem::kPi * target_.bounce_frequency_hz * t_s +
               target_.bounce_phase_rad);
  return velocity;
}
