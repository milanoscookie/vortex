#pragma once

#include "problem_description.h"

#include <array>
#include <complex>
#include <cstddef>
#include <cstdint>
#include <cmath>

class Environment {
  public:
    using size_t = std::size_t;
    using Complex = std::complex<float>;
    using RadarSettings = problem::RadarSettings;
    using FloorplaneClutterSettings = problem::FloorplaneClutterSettings;
    using Constants = problem::Constants;

    explicit Environment(const RadarSettings &radar_settings) noexcept;
    Environment(const RadarSettings &radar_settings,
                const FloorplaneClutterSettings &floorplane_settings) noexcept;

    bool hasStaticFloorplane() const noexcept {
        return floorplane_valid_;
    }
    Complex sampleStaticFloorplane(size_t fast_time_index) const noexcept;

  private:
    void clearFloorplane() noexcept;
    void initializeFloorplane(const FloorplaneClutterSettings &floorplane_settings) noexcept;

    RadarSettings radar_settings_;
    float floorplane_beat_frequency_hz_ = 0.0f;
    float floorplane_amplitude_ = 0.0f;
    float floorplane_base_phase_rad_ = 0.0f;
    std::array<Complex, Constants::kRadarBlockSize> floorplane_samples_{};
    bool floorplane_valid_ = false;
};
