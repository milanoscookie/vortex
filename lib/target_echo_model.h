#pragma once

#include "problem_description.h"
#include "target_observation.h"
#include "tx_history_buffer.h"

#include <algorithm>
#include <cmath>
#include <complex>

namespace radar {

using Constants = dsp::Constants;
using RadarSettings = dsp::RadarSettings;
using Vec3 = dsp::Vec3;

template <typename Complex> inline Complex unitPhasor(double phase) noexcept {
    double sin_phase = 0.0;
    double cos_phase = 0.0;
#if defined(__GNUC__) || defined(__clang__)
    __builtin_sincos(phase, &sin_phase, &cos_phase);
#else
    sin_phase = std::sin(phase);
    cos_phase = std::cos(phase);
#endif
    return Complex(cos_phase, sin_phase);
}

template <typename Complex, std::size_t HistorySize>
inline Complex sampleBistaticTargetReturn(const TargetObservation &observation,
                                          const Vec3 &element_position_m,
                                          double element_delay_samples,
                                          double weight,
                                          double wave_number,
                                          const RadarSettings &radar_settings,
                                          const Complex &reflectivity,
                                          const TxHistoryBuffer<Complex, HistorySize> &tx_history,
                                          std::size_t sample_index) noexcept {
    if (weight == 0.0) {
        return Complex(0.0, 0.0);
    }

    const Vec3 delta_m = observation.position_m - element_position_m;
    const double range_rx_sq_m = delta_m.squaredNorm();
    const double max_range_sq_m = radar_settings.max_range_m * radar_settings.max_range_m;
    if (range_rx_sq_m > max_range_sq_m) {
        return Complex(0.0, 0.0);
    }

    const double range_rx_m = std::sqrt(range_rx_sq_m);

    const double total_range_m = observation.range_m + range_rx_m;
    const double safe_range_rx_m = std::max(range_rx_m, radar_settings.min_range_m);
    const double delay_samples =
        total_range_m * (radar_settings.sample_rate_hz / Constants::kSpeedOfLightMps) +
        element_delay_samples;
    const double path_gain = radar_settings.field_gain / (safe_range_rx_m * safe_range_rx_m);
    const double phase = -wave_number * total_range_m;

    return (weight * path_gain) * tx_history.delayedSample(sample_index, delay_samples) *
           reflectivity * unitPhasor<Complex>(phase);
}

} // namespace radar
