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
    using Complex = std::complex<problem::Real>;
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
    problem::Real floorplane_beat_frequency_hz_ = problem::Real(0.0);
    problem::Real floorplane_amplitude_ = problem::Real(0.0);
    problem::Real floorplane_base_phase_rad_ = problem::Real(0.0);
    std::array<Complex, problem::RadarSettings::kRadarBlockSize> floorplane_samples_{};
    bool floorplane_valid_ = false;
};
