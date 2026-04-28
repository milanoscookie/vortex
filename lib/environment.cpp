#include "environment.h"

Environment::Environment(const RadarSettings &radar_settings) noexcept
    : radar_settings_(radar_settings) {}

Environment::Environment(
    const RadarSettings &radar_settings,
    const FloorplaneClutterSettings &floorplane_settings) noexcept
    : Environment(radar_settings) {
  initializeFloorplane(floorplane_settings);
}

void Environment::clearFloorplane() noexcept {
  floorplane_beat_frequency_hz_ = 0.0f;
  floorplane_amplitude_ = 0.0f;
  floorplane_base_phase_rad_ = 0.0f;
  floorplane_valid_ = false;
}

void Environment::initializeFloorplane(
    const FloorplaneClutterSettings &floorplane_settings) noexcept {
  if (!floorplane_settings.enable_static_floorplane ||
      floorplane_settings.range_m <= 0.0f) {
    clearFloorplane();
    return;
  }

  const float lambda_m =
      Constants::kSpeedOfLightMps / radar_settings_.carrier_hz;
  const float chirp_duration_s =
      static_cast<float>(Constants::kRadarBlockSize) /
      radar_settings_.sample_rate_hz;
  const float chirp_slope_hz_per_s =
      radar_settings_.bandwidth_hz / chirp_duration_s;
  const float beat_frequency_hz = 2.0f * chirp_slope_hz_per_s *
                                  floorplane_settings.range_m /
                                  Constants::kSpeedOfLightMps;
  const float amplitude = floorplane_settings.amplitude_ref *
                          std::pow(floorplane_settings.reference_range_m /
                                       floorplane_settings.range_m,
                                   floorplane_settings.range_exponent);
  const float base_phase_rad =
      -4.0f * Constants::kPi * floorplane_settings.range_m / lambda_m +
      floorplane_settings.phase_rad;

  floorplane_beat_frequency_hz_ = beat_frequency_hz;
  floorplane_amplitude_ = amplitude;
  floorplane_base_phase_rad_ = base_phase_rad;
  floorplane_valid_ = amplitude > 0.0f;
}

Environment::Complex
Environment::sampleStaticFloorplane(size_t fast_time_index) const noexcept {
  if (!floorplane_valid_) {
    return {0.0f, 0.0f};
  }

  const float sample_period_s = 1.0f / radar_settings_.sample_rate_hz;
  const float fast_time_s =
      static_cast<float>(fast_time_index) * sample_period_s;
  const float phase_rad =
      2.0f * Constants::kPi * floorplane_beat_frequency_hz_ * fast_time_s +
      floorplane_base_phase_rad_;
  return std::polar(floorplane_amplitude_, phase_rad);
}
