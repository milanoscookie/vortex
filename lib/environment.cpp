#include "environment.h"

#include <cmath>
#include <stdexcept>

namespace {

Environment::Complex cis(double phase_rad) noexcept {
    return {std::cos(phase_rad), std::sin(phase_rad)};
}

void validateRadarSettings(const Environment::RadarSettings &radar_settings) {
    if (!std::isfinite(radar_settings.carrier_hz) || radar_settings.carrier_hz <= 0.0) {
        throw std::invalid_argument("radar carrier_hz must be positive and finite");
    }
    if (!std::isfinite(radar_settings.sample_rate_hz) || radar_settings.sample_rate_hz <= 0.0) {
        throw std::invalid_argument("radar sample_rate_hz must be positive and finite");
    }
    if (!std::isfinite(radar_settings.bandwidth_hz) || radar_settings.bandwidth_hz <= 0.0) {
        throw std::invalid_argument("radar bandwidth_hz must be positive and finite");
    }
}

void validateFloorplaneSettings(const Environment::FloorplaneClutterSettings &floorplane_settings) {
    if (!floorplane_settings.enable_static_floorplane) {
        return;
    }
    if (!std::isfinite(floorplane_settings.range_m) || floorplane_settings.range_m <= 0.0) {
        throw std::invalid_argument("floorplane range_m must be positive and finite when enabled");
    }
    if (!std::isfinite(floorplane_settings.reference_range_m) ||
        floorplane_settings.reference_range_m <= 0.0) {
        throw std::invalid_argument(
            "floorplane reference_range_m must be positive and finite when enabled");
    }
    if (!std::isfinite(floorplane_settings.amplitude_ref) ||
        floorplane_settings.amplitude_ref < 0.0) {
        throw std::invalid_argument(
            "floorplane amplitude_ref must be non-negative and finite when enabled");
    }
    if (!std::isfinite(floorplane_settings.range_exponent) ||
        !std::isfinite(floorplane_settings.phase_rad)) {
        throw std::invalid_argument(
            "floorplane range_exponent and phase_rad must be finite when enabled");
    }
}

} // namespace

Environment::Environment(const RadarSettings &radar_settings,
                         const FloorplaneClutterSettings &floorplane_settings)
    : radar_settings_(radar_settings) {
    validateRadarSettings(radar_settings_);
    validateFloorplaneSettings(floorplane_settings);
    initializeFloorplane(floorplane_settings);
}

void Environment::clearFloorplane() noexcept {
    floorplane_samples_.fill({0.0, 0.0});
    floorplane_valid_ = false;
}

void Environment::initializeFloorplane(
    const FloorplaneClutterSettings &floorplane_settings) noexcept {
    if (!floorplane_settings.enable_static_floorplane || floorplane_settings.range_m <= 0.0) {
        clearFloorplane();
        return;
    }

    const double lambda_m = Constants::kSpeedOfLightMps / radar_settings_.carrier_hz;
    const double chirp_duration_s =
        static_cast<double>(dsp::RadarSettings::kRadarBlockSize) / radar_settings_.sample_rate_hz;
    const double chirp_slope_hz_per_s = radar_settings_.bandwidth_hz / chirp_duration_s;
    const double beat_frequency_hz =
        2.0 * chirp_slope_hz_per_s * floorplane_settings.range_m / Constants::kSpeedOfLightMps;
    const double amplitude =
        floorplane_settings.amplitude_ref *
        std::pow(floorplane_settings.reference_range_m / floorplane_settings.range_m,
                 floorplane_settings.range_exponent);
    const double base_phase_rad = -4.0 * Constants::kPi * floorplane_settings.range_m / lambda_m +
                                  floorplane_settings.phase_rad;

    floorplane_valid_ = amplitude > 0.0;
    if (!floorplane_valid_) {
        floorplane_samples_.fill({0.0, 0.0});
        return;
    }

    const double phase_step_rad =
        2.0 * Constants::kPi * beat_frequency_hz / radar_settings_.sample_rate_hz;
    Complex sample = amplitude * cis(base_phase_rad);
    const Complex step = cis(phase_step_rad);
    for (Complex &floorplane_sample : floorplane_samples_) {
        floorplane_sample = sample;
        sample *= step;
    }
}

Environment::Complex Environment::sampleStaticFloorplane(std::size_t fastTimeIndex) const noexcept {
    if (!floorplane_valid_) {
        return {0.0, 0.0};
    }

    return floorplane_samples_[fastTimeIndex % floorplane_samples_.size()];
}
