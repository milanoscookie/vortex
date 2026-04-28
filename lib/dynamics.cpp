#include "dynamics.h"

#include <cmath>

namespace {

float bouncePhaseRadians(const problem::CarSettings &car, float t_s) noexcept {
  return 2.0f * problem::Constants::kPi * car.bounce_frequency_hz * t_s +
         car.bounce_phase_rad;
}

} // namespace

CarDynamics::CarDynamics(const CarSettings &car_settings) : car_(car_settings) {}

CarDynamics::Vec3 CarDynamics::positionAt(float t_s) const noexcept {
  Vec3 position = car_.initial_position_m + car_.base_velocity_mps * t_s;
  position.z() += car_.bounce_amplitude_m * std::sin(bouncePhaseRadians(car_, t_s));
  return position;
}

CarDynamics::Vec3 CarDynamics::velocityAt(float t_s) const noexcept {
  Vec3 velocity = car_.base_velocity_mps;
  velocity.z() +=
      2.0f * problem::Constants::kPi * car_.bounce_frequency_hz *
      car_.bounce_amplitude_m * std::cos(bouncePhaseRadians(car_, t_s));
  return velocity;
}
