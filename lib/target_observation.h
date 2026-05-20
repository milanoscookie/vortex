#pragma once

#include "dynamics.h"
#include "problem_description.h"

#include <algorithm>

namespace radar {

using Vec3 = dsp::Vec3;
using RadarSettings = dsp::RadarSettings;
using SimulationMetrics = dsp::SimulationMetrics;
using Constants = dsp::Constants;

struct TargetObservation {
    Vec3 position_m = Vec3::Zero();
    Vec3 velocity_mps = Vec3::Zero();
    Vec3 line_of_sight = Vec3::Zero();
    double range_m = 0.0f;
    double safe_range_m = 0.0f;
    double delay_s = 0.0f;
    double radial_velocity_mps = 0.0f;
    double doppler_hz = 0.0f;
};

inline TargetObservation observeTarget(const CarDynamics &dynamics,
                                       const RadarSettings &radar_settings,
                                       double t_s) noexcept {
    TargetObservation observation;
    observation.position_m = dynamics.positionAt(t_s);
    observation.velocity_mps = dynamics.velocityAt(t_s);
    observation.range_m = observation.position_m.norm();
    observation.safe_range_m = std::max(observation.range_m, radar_settings.min_range_m);
    observation.line_of_sight = observation.position_m / observation.safe_range_m;
    observation.delay_s = 2.0 * observation.range_m / Constants::kSpeedOfLightMps;
    observation.radial_velocity_mps = observation.velocity_mps.dot(observation.line_of_sight);
    const double lambda_m = Constants::kSpeedOfLightMps / radar_settings.carrier_hz;
    observation.doppler_hz = 2.0 * observation.radial_velocity_mps / lambda_m;
    return observation;
}

inline SimulationMetrics makeSimulationMetrics(double t_s,
                                               const TargetObservation &observation) noexcept {
    SimulationMetrics metrics;
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
