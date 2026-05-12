#include "environment.h"

namespace {

Environment::Complex cis(problem::Real phase_rad) noexcept {
    return {std::cos(phase_rad), std::sin(phase_rad)};
}

} // namespace

Environment::Environment(const RadarSettings &radar_settings) noexcept
    : radar_settings_(radar_settings) {}

Environment::Environment(const RadarSettings &radar_settings,
                         const FloorplaneClutterSettings &floorplane_settings) noexcept
    : Environment(radar_settings) {
    initializeFloorplane(floorplane_settings);
}

void Environment::clearFloorplane() noexcept {
    floorplane_beat_frequency_hz_ = problem::Real(0.0);
    floorplane_amplitude_ = problem::Real(0.0);
    floorplane_base_phase_rad_ = problem::Real(0.0);
    floorplane_samples_.fill({problem::Real(0.0), problem::Real(0.0)});
    floorplane_valid_ = false;
}

void Environment::initializeFloorplane(
    const FloorplaneClutterSettings &floorplane_settings) noexcept {
    if (!floorplane_settings.enable_static_floorplane ||
        floorplane_settings.range_m <= problem::Real(0.0)) {
        clearFloorplane();
        return;
    }

    const problem::Real lambda_m = Constants::kSpeedOfLightMps / radar_settings_.carrier_hz;
    const problem::Real chirp_duration_s =
        static_cast<problem::Real>(problem::RadarSettings::kRadarBlockSize) /
        radar_settings_.sample_rate_hz;
    const problem::Real chirp_slope_hz_per_s = radar_settings_.bandwidth_hz / chirp_duration_s;
    const problem::Real beat_frequency_hz = problem::Real(2.0) * chirp_slope_hz_per_s *
                                            floorplane_settings.range_m /
                                            Constants::kSpeedOfLightMps;
    const problem::Real amplitude =
        floorplane_settings.amplitude_ref *
        std::pow(floorplane_settings.reference_range_m / floorplane_settings.range_m,
                 floorplane_settings.range_exponent);
    const problem::Real base_phase_rad =
        -problem::Real(4.0) * Constants::kPi * floorplane_settings.range_m / lambda_m +
        floorplane_settings.phase_rad;

    floorplane_beat_frequency_hz_ = beat_frequency_hz;
    floorplane_amplitude_ = amplitude;
    floorplane_base_phase_rad_ = base_phase_rad;
    floorplane_valid_ = amplitude > problem::Real(0.0);
    if (!floorplane_valid_) {
        floorplane_samples_.fill({problem::Real(0.0), problem::Real(0.0)});
        return;
    }

    const problem::Real phase_step_rad =
        problem::Real(2.0) * Constants::kPi * beat_frequency_hz / radar_settings_.sample_rate_hz;
    Complex sample = amplitude * cis(base_phase_rad);
    const Complex step = cis(phase_step_rad);
    for (Complex &floorplane_sample : floorplane_samples_) {
        floorplane_sample = sample;
        sample *= step;
    }
}

Environment::Complex Environment::sampleStaticFloorplane(size_t fast_time_index) const noexcept {
    if (!floorplane_valid_) {
        return {problem::Real(0.0), problem::Real(0.0)};
    }

    return floorplane_samples_[fast_time_index % floorplane_samples_.size()];
}
