#pragma once

#include "dynamics.h"
#include "problem_description.h"

#include <algorithm>

namespace radar {

struct TargetObservation {
  problem::Vec3 position_m = problem::Vec3::Zero();
  problem::Vec3 velocity_mps = problem::Vec3::Zero();
  problem::Vec3 line_of_sight = problem::Vec3::Zero();
  float range_m = 0.0f;
  float safe_range_m = 0.0f;
  float delay_s = 0.0f;
  float radial_velocity_mps = 0.0f;
  float doppler_hz = 0.0f;
};

[[nodiscard]] inline TargetObservation observeTarget(
    const CarDynamics &dynamics,
    const problem::RadarSettings &radar_settings,
    float t_s) noexcept {
  TargetObservation observation;
  observation.position_m = dynamics.positionAt(t_s);
  observation.velocity_mps = dynamics.velocityAt(t_s);
  observation.range_m = observation.position_m.norm();
  observation.safe_range_m =
      std::max(observation.range_m, radar_settings.min_range_m);
  observation.line_of_sight = observation.position_m / observation.safe_range_m;
  observation.delay_s =
      2.0f * observation.range_m / problem::Constants::kSpeedOfLightMps;
  observation.radial_velocity_mps =
      observation.velocity_mps.dot(observation.line_of_sight);
  const float lambda_m =
      problem::Constants::kSpeedOfLightMps / radar_settings.carrier_hz;
  observation.doppler_hz = 2.0f * observation.radial_velocity_mps / lambda_m;
  return observation;
}

[[nodiscard]] inline problem::SimulationMetrics
makeSimulationMetrics(float t_s, const TargetObservation &observation) noexcept {
  problem::SimulationMetrics metrics;
  metrics.time_s = t_s;
  metrics.range_m = observation.range_m;
  metrics.delay_s = observation.delay_s;
  metrics.radial_velocity_mps = observation.radial_velocity_mps;
  metrics.doppler_hz = observation.doppler_hz;
  metrics.position_m = observation.position_m;
  metrics.velocity_mps = observation.velocity_mps;
  metrics.line_of_sight = observation.line_of_sight;
  return metrics;
}

} // namespace radar
