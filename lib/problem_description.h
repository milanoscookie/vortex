#pragma once

#include <Eigen/Dense>

#include <complex>
#include <cstddef>
#include <cstdint>
#include <vector>

namespace problem {

using Real = float;
using AmpReal = double;
using Vec3 = Eigen::Vector3f;
using SignalComplex = std::complex<Real>;
using AmplitudeComplex = std::complex<AmpReal>;
using size_t = std::size_t;

struct Constants {
    static inline constexpr Real kPi = 3.14159265358979323846f;
    static inline constexpr Real kSpeedOfLightMps = 299'792'458.0f;
    static inline constexpr Real kInvSqrt2 = 0.70710678f;
};

struct RadarSettings {
    static inline constexpr size_t kProbeNumX = 4;
    static inline constexpr size_t kProbeNumY = 4;
    static inline constexpr size_t kProbeNumElements = kProbeNumX * kProbeNumY;
    // // 400 MHz gives a crisp 0.375m theoretical range bin resolution
    // static inline constexpr double kDefaultBandwidthHz = 400.0e6f;
    //
    // // We need 4096 samples to prevent aliasing out to 500m.
    // // To process 4096 samples in a reasonable chirp time, we need a fast ADC.
    // static inline constexpr double kDefaultSampleRateHz = 160.0e6f;
    //
    // // 4096 samples / 160 MHz = 25.6 microseconds
    // static inline constexpr double kDefaultChirpDurationS = 25.6e-6f;
    //
    // // This will perfectly compute to 4096
    // static inline constexpr size_t kRadarBlockSize =
    //     static_cast<size_t>(kDefaultSampleRateHz * kDefaultChirpDurationS + 0.5f);
    // static inline constexpr double kDefaultCarrierHz = 77.0e9f;
    // static inline constexpr double kDefaultMinRangeM = 1.0f;
    // static inline constexpr double kDefaultMaxRangeM = 500.0f;
    // static inline constexpr double kDefaultFieldGain = 1.0f;
    // static inline constexpr double kDefaultReceiverNoiseStddev = 0.0f;
    // static inline constexpr double kDefaultReceiverNoiseMean = 0.0f;
    // static inline constexpr double kDefaultReceiverNoiseDistributionStddev = 1.0f;
    // static inline constexpr bool kDefaultUseStdNormalDistribution = true;

    static inline constexpr Real kDefaultBandwidthHz = 10.0e6f;
    static inline constexpr Real kDefaultSampleRateHz = 2.0f * kDefaultBandwidthHz;
    static inline constexpr Real kDefaultChirpDurationS = 12.8e-6f;
    static inline constexpr size_t kRadarBlockSize =
        static_cast<size_t>(kDefaultSampleRateHz * kDefaultChirpDurationS + 0.5f);
    static inline constexpr Real kDefaultCarrierHz = 77.0e9f;
    static inline constexpr Real kDefaultMinRangeM = 1.0f;
    static inline constexpr Real kDefaultMaxRangeM = 500.0f;
    static inline constexpr Real kDefaultFieldGain = 1.0f;
    static inline constexpr AmpReal kDefaultReceiverNoiseStddev = 0.0;
    static inline constexpr AmpReal kDefaultReceiverNoiseMean = 0.0;
    static inline constexpr AmpReal kDefaultReceiverNoiseDistributionStddev = 1.0;
    static inline constexpr bool kDefaultUseStdNormalDistribution = true;

    Real bandwidth_hz = kDefaultBandwidthHz;
    Real sample_rate_hz = kDefaultSampleRateHz;
    Real carrier_hz = kDefaultCarrierHz;
    Real min_range_m = kDefaultMinRangeM;
    Real max_range_m = kDefaultMaxRangeM;

    Real field_gain = kDefaultFieldGain;
    AmpReal receiver_noiselevel_stddev = kDefaultReceiverNoiseStddev;
    AmpReal receiver_noiselevel_mean = kDefaultReceiverNoiseMean;
    AmpReal receiver_noise_distribution_stddev = kDefaultReceiverNoiseDistributionStddev;
    bool receiver_noise_use_std_normal_distribution = kDefaultUseStdNormalDistribution;
};

struct ProbeSettings {
    Vec3 center_m = Vec3::Zero();
    Real spacing_x_wavelengths = 0.5f;
    Real spacing_y_wavelengths = 0.5f;
};

struct FloorplaneClutterSettings {
    bool enable_static_floorplane = false;
    Real range_m = 101.5f;
    AmpReal amplitude_ref = 0.001;
    Real reference_range_m = 100.0f;
    Real range_exponent = 1.0f;
    Real phase_rad = 0.7f;
};

struct SimulatorSettings {
    Real burst_duration_s = 1.00f;
    Real tracking_duration_s = 1.00f;
    Real split_duration_s = 15.0f;
    Real prediction_duration_s = 15.0f;
    std::uint32_t random_seed = 1U;
    size_t vehicle_count = 1;
};

struct CarSettings {
    Vec3 initial_position_m = Vec3(50.0f, 50.0f, 100.0f);
    Vec3 base_velocity_mps = Vec3(20.0f, 20.0f, 0.0f);
    Real yaw_rad = 0.0f;
    Real length_m = 4.5f;
    Real width_m = 1.8f;
    Real height_m = 1.5f;
    Real bounce_amplitude_m = 0.01f;  // heavy: 0.002
    Real bounce_frequency_hz = 30.0f; // heavy: 4
    Real bounce_phase_rad = 0.0f;     // each car should be different
    SignalComplex reflectivity = std::polar(1.0f, 0.0f);
};

struct VehicleState {
    Vec3 center_m = Vec3::Zero();
    Vec3 velocity_mps = Vec3::Zero();
    Real yaw_rad = 0.0f;
    SignalComplex reflectivity = {1.0f, 0.0f};
};

struct SimulationMetrics {
    Real time_s = 0.0f;
    Real range_m = 0.0f;
    Real delay_s = 0.0f;
    Real radial_velocity_mps = 0.0f;
    Real doppler_hz = 0.0f;
    Vec3 position_m = Vec3::Zero();
    Vec3 velocity_mps = Vec3::Zero();
    Vec3 line_of_sight = Vec3::Zero();
};

struct ProblemDescription {
    RadarSettings radar;
    ProbeSettings probe;
    FloorplaneClutterSettings floorplane_clutter;
    SimulatorSettings simulator;
    std::vector<CarSettings> cars;
};

inline const ProblemDescription kDefaultProblemDescription{
    .radar = {},
    .probe = {},
    .floorplane_clutter = {},
    .simulator = {.vehicle_count = 3},
    .cars = {CarSettings{
                 .initial_position_m = Vec3(107.0f, 107.0f, 150.0f),
                 .reflectivity = std::polar(1.0f, 3.1f),
             },
             CarSettings{
                 .initial_position_m = Vec3(114.0f, 114.0f, 150.0f),
                 .reflectivity = std::polar(1.0f, 3.1f),
             },
             CarSettings{
                 .initial_position_m = Vec3(100.0f, 100.0f, 150.0f),
                 .base_velocity_mps = Vec3(20.0f, 20.0f, 0.0f),
                 .yaw_rad = 0.3f,
                 .bounce_amplitude_m = 0.002f,
                 .bounce_frequency_hz = 4.0f,
                 .bounce_phase_rad = 1.1f,
                 .reflectivity = std::polar(0.9f, 0.4f),
             }}};

} // namespace problem
