#pragma once

#include "problem_description.h"
#include "target_observation.h"
#include "tx_history_buffer.h"

#include <algorithm>
#include <cmath>
#include <complex>

namespace radar {

template <typename Complex, std::size_t HistorySize>
[[nodiscard]] inline Complex makeMonostaticBaseReturn(
    const TargetObservation &observation,
    const problem::RadarSettings &radar_settings,
    const Complex &reflectivity,
    const TxHistoryBuffer<Complex, HistorySize> &tx_history,
    std::size_t sample_index) noexcept {
  const float path_gain =
      radar_settings.field_gain /
      std::max(observation.safe_range_m * observation.safe_range_m, 1.0e-6f);
  return tx_history.delayedSample(sample_index, observation.delay_s,
                                  radar_settings.sample_rate_hz) *
         reflectivity * path_gain;
}

[[nodiscard]] inline float makeMonostaticBasePhase(
    const TargetObservation &observation,
    float wave_number) noexcept {
  return -2.0f * wave_number * observation.range_m;
}

template <typename Complex>
[[nodiscard]] inline Complex applySpatialPhase(
    const Complex &base_return,
    float base_phase,
    const problem::Vec3 &line_of_sight,
    const problem::Vec3 &element_position_m,
    float weight,
    float wave_number) noexcept {
  const float total_phase =
      base_phase + wave_number * line_of_sight.dot(element_position_m);
  return weight * base_return *
         Complex(std::cos(total_phase), std::sin(total_phase));
}

template <typename Complex, std::size_t HistorySize>
[[nodiscard]] inline Complex sampleBistaticTargetReturn(
    const TargetObservation &observation,
    const problem::Vec3 &element_position_m,
    float element_delay_s,
    float weight,
    float wave_number,
    const problem::RadarSettings &radar_settings,
    const Complex &reflectivity,
    const TxHistoryBuffer<Complex, HistorySize> &tx_history,
    std::size_t sample_index) noexcept {
  if (weight == 0.0f) {
    return Complex(0.0f, 0.0f);
  }

  const problem::Vec3 delta_m = observation.position_m - element_position_m;
  const float range_rx_m = delta_m.norm();
  if (range_rx_m > radar_settings.max_range_m) {
    return Complex(0.0f, 0.0f);
  }

  const float total_range_m = observation.range_m + range_rx_m;
  const float safe_range_rx_m = std::max(range_rx_m, radar_settings.min_range_m);
  const float delay_s = total_range_m / problem::Constants::kSpeedOfLightMps +
                        element_delay_s;
  const float path_gain =
      radar_settings.field_gain / (safe_range_rx_m * safe_range_rx_m);
  const float phase = -wave_number * total_range_m;

  return (weight * path_gain) *
         tx_history.delayedSample(sample_index, delay_s,
                                  radar_settings.sample_rate_hz) *
         reflectivity * Complex(std::cos(phase), std::sin(phase));
}

} // namespace radar
