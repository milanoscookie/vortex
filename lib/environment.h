#pragma once

#include "problem_description.h"

#include <array>
#include <complex>
#include <cstddef>
#include <cstdint>
#include <cmath>

class Environment {
  public:
    using Complex = dsp::Complex;
    using RadarSettings = dsp::RadarSettings;
    using FloorplaneClutterSettings = dsp::FloorplaneClutterSettings;
    using Constants = dsp::Constants;

    // RT-unsafe.
    // Copies configuration and precomputes one chirp of floorplane clutter state.
    explicit Environment(const RadarSettings &radar_settings,
                         const FloorplaneClutterSettings &floorplane_settings = {});

    // RT-safe.
    bool hasStaticFloorplane() const noexcept {
        return floorplane_valid_;
    }

    // RT-safe.
    // Returns one precomputed floorplane clutter sample for the given fast-time index.
    [[nodiscard]] Complex sampleStaticFloorplane(std::size_t fastTimeIndex) const noexcept;

  private:
    void clearFloorplane() noexcept;
    void initializeFloorplane(const FloorplaneClutterSettings &floorplane_settings) noexcept;

    RadarSettings radar_settings_;
    std::array<Complex, dsp::RadarSettings::kRadarBlockSize> floorplane_samples_{};
    bool floorplane_valid_ = false;
};
