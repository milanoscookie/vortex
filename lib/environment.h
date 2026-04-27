#pragma once

#include "problem_description.h"

#include <complex>
#include <cstddef>
#include <cstdint>

class Environment {
public:
  using Complex = std::complex<float>;
  using Config = problem::Config;
  using FloorplaneClutterConfig = problem::FloorplaneClutterConfig;

  Environment(const Config &config, std::uint32_t random_seed = 0U) noexcept;
  Environment(const Config &config, const FloorplaneClutterConfig &floorplane_config,
              std::uint32_t random_seed = 0U) noexcept;

  bool hasStaticFloorplane() const noexcept { return floorplane_valid_; }
  Complex sampleStaticFloorplane(std::size_t fast_time_index) const noexcept;

private:
  void initializeFloorplane(const FloorplaneClutterConfig &floorplane_config) noexcept;

  Config config_;
  float floorplane_beat_frequency_hz_ = 0.0f;
  float floorplane_amplitude_ = 0.0f;
  float floorplane_base_phase_rad_ = 0.0f;
  bool floorplane_valid_ = false;
};
