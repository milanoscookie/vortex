#pragma once

#include "problem_description.h"
#include "target_observation.h"
#include "tx_history_buffer.h"

#include <algorithm>
#include <cmath>
#include <complex>

namespace radar {

template <typename Complex> inline Complex unitPhasor(problem::Real phase) noexcept {
    problem::Real sin_phase = 0.0;
    problem::Real cos_phase = 0.0;
#if defined(__GNUC__) || defined(__clang__)
    __builtin_sincos(phase, &sin_phase, &cos_phase);
#else
    sin_phase = std::sin(phase);
    cos_phase = std::cos(phase);
#endif
    return Complex(cos_phase, sin_phase);
}

template <typename Complex, std::size_t HistorySize>
inline Complex makeMonostaticBaseReturn(const TargetObservation &observation,
                                        const problem::RadarSettings &radar_settings,
                                        const Complex &reflectivity,
                                        const TxHistoryBuffer<Complex, HistorySize> &tx_history,
                                        std::size_t sample_index) noexcept {
    const problem::Real delay_samples = observation.delay_s * radar_settings.sample_rate_hz;
    const problem::Real path_gain =
        radar_settings.field_gain /
        std::max(observation.safe_range_m * observation.safe_range_m, problem::Real(1.0e-6));
    return tx_history.delayedSample(sample_index, delay_samples) * reflectivity * path_gain;
}

inline problem::Real makeMonostaticBasePhase(const TargetObservation &observation,
                                             problem::Real wave_number) noexcept {
    return -2.0 * wave_number * observation.range_m;
}

template <typename Complex>
inline Complex applySpatialPhase(const Complex &base_return,
                                 problem::Real base_phase,
                                 const problem::Vec3 &line_of_sight,
                                 const problem::Vec3 &element_position_m,
                                 problem::Real weight,
                                 problem::Real wave_number) noexcept {
    const problem::Real total_phase =
        base_phase + wave_number * line_of_sight.dot(element_position_m);
    return weight * base_return * unitPhasor<Complex>(total_phase);
}

template <typename Complex, std::size_t HistorySize>
inline Complex sampleBistaticTargetReturn(const TargetObservation &observation,
                                          const problem::Vec3 &element_position_m,
                                          problem::Real element_delay_samples,
                                          problem::Real weight,
                                          problem::Real wave_number,
                                          const problem::RadarSettings &radar_settings,
                                          const Complex &reflectivity,
                                          const TxHistoryBuffer<Complex, HistorySize> &tx_history,
                                          std::size_t sample_index) noexcept {
    if (weight == 0.0) {
        return Complex(0.0, 0.0);
    }

    const problem::Vec3 delta_m = observation.position_m - element_position_m;
    const problem::Real range_rx_sq_m = delta_m.squaredNorm();
    const problem::Real max_range_sq_m = radar_settings.max_range_m * radar_settings.max_range_m;
    if (range_rx_sq_m > max_range_sq_m) {
        return Complex(0.0, 0.0);
    }

    const problem::Real range_rx_m = std::sqrt(range_rx_sq_m);

    const problem::Real total_range_m = observation.range_m + range_rx_m;
    const problem::Real safe_range_rx_m = std::max(range_rx_m, radar_settings.min_range_m);
    const problem::Real delay_samples =
        total_range_m * (radar_settings.sample_rate_hz / problem::Constants::kSpeedOfLightMps) +
        element_delay_samples;
    const problem::Real path_gain = radar_settings.field_gain / (safe_range_rx_m * safe_range_rx_m);
    const problem::Real phase = -wave_number * total_range_m;

    return (weight * path_gain) * tx_history.delayedSample(sample_index, delay_samples) *
           reflectivity * unitPhasor<Complex>(phase);
}

} // namespace radar
