#include "environment.h"

#include <cmath>

Environment::Environment(const Config &config, std::uint32_t /* random_seed */) noexcept
    : config_(config) {}

Environment::Environment(const Config &config,
                         const FloorplaneClutterConfig &floorplane_config,
                         std::uint32_t /* random_seed */) noexcept
    : config_(config) {
  initializeFloorplane(floorplane_config);
}

void Environment::initializeFloorplane(
    const FloorplaneClutterConfig &floorplane_config) noexcept {
  if (!floorplane_config.enable_static_floorplane ||
      floorplane_config.range_m <= 0.0f) {
    floorplane_beat_frequency_hz_ = 0.0f;
    floorplane_amplitude_ = 0.0f;
    floorplane_base_phase_rad_ = 0.0f;
    floorplane_valid_ = false;
    return;
  }

  const float lambda_m = config_.speed_of_light_mps / config_.carrier_hz;
  const float chirp_duration_s =
      static_cast<float>(problem::kRadarBlockSize) / config_.sample_rate_hz;
  const float chirp_slope_hz_per_s = config_.bandwidth_hz / chirp_duration_s;
  const float beat_frequency_hz =
      2.0f * chirp_slope_hz_per_s * floorplane_config.range_m /
      config_.speed_of_light_mps;
  const float amplitude =
      floorplane_config.amplitude_ref *
      std::pow(floorplane_config.reference_range_m / floorplane_config.range_m,
               floorplane_config.range_exponent);
  const float base_phase_rad =
      -4.0f * problem::kPi * floorplane_config.range_m / lambda_m +
      floorplane_config.phase_rad;

  floorplane_beat_frequency_hz_ = beat_frequency_hz;
  floorplane_amplitude_ = amplitude;
  floorplane_base_phase_rad_ = base_phase_rad;
  floorplane_valid_ = amplitude > 0.0f;
}

Environment::Complex Environment::sampleStaticFloorplane(
    std::size_t fast_time_index) const noexcept {
  if (!floorplane_valid_) {
    return {0.0f, 0.0f};
  }

  const float sample_period_s = 1.0f / config_.sample_rate_hz;
  const float fast_time_s = static_cast<float>(fast_time_index) * sample_period_s;
  const float phase_rad =
      2.0f * problem::kPi * floorplane_beat_frequency_hz_ * fast_time_s +
      floorplane_base_phase_rad_;
  return std::polar(floorplane_amplitude_, phase_rad);
}
