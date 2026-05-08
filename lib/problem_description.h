#pragma once

#include <Eigen/Dense>

#include <complex>
#include <cstddef>
#include <cstdint>
#include <vector>

namespace problem {

using Vec3 = Eigen::Vector3d;
using size_t = std::size_t;

struct Constants {
    static inline constexpr double kPi = 3.14159265358979323846f;
    static inline constexpr double kSpeedOfLightMps = 299'792'458.0f;
    static inline constexpr double kInvSqrt2 = 0.70710678f;
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

    static inline constexpr double kDefaultBandwidthHz = 10.0e6f;
    static inline constexpr double kDefaultSampleRateHz = 2.0f * kDefaultBandwidthHz;
    static inline constexpr double kDefaultChirpDurationS = 12.8e-6f;
    static inline constexpr size_t kRadarBlockSize =
        static_cast<size_t>(kDefaultSampleRateHz * kDefaultChirpDurationS + 0.5f);
    static inline constexpr double kDefaultCarrierHz = 77.0e9f;
    static inline constexpr double kDefaultMinRangeM = 1.0f;
    static inline constexpr double kDefaultMaxRangeM = 500.0f;
    static inline constexpr double kDefaultFieldGain = 1.0f;
    static inline constexpr double kDefaultReceiverNoiseStddev = 0.0f;
    static inline constexpr double kDefaultReceiverNoiseMean = 0.0f;
    static inline constexpr double kDefaultReceiverNoiseDistributionStddev = 1.0f;
    static inline constexpr bool kDefaultUseStdNormalDistribution = true;

    double bandwidth_hz = kDefaultBandwidthHz;
    double sample_rate_hz = kDefaultSampleRateHz;
    double carrier_hz = kDefaultCarrierHz;
    double min_range_m = kDefaultMinRangeM;
    double max_range_m = kDefaultMaxRangeM;

    double field_gain = kDefaultFieldGain;
    double receiver_noiselevel_stddev = kDefaultReceiverNoiseStddev;
    double receiver_noiselevel_mean = kDefaultReceiverNoiseMean;
    double receiver_noise_distribution_stddev = kDefaultReceiverNoiseDistributionStddev;
    bool receiver_noise_use_std_normal_distribution = kDefaultUseStdNormalDistribution;
};

struct ProbeSettings {
    Vec3 center_m = Vec3::Zero();
    double spacing_x_wavelengths = 0.5f;
    double spacing_y_wavelengths = 0.5f;
};

struct FloorplaneClutterSettings {
    bool enable_static_floorplane = false;
    double range_m = 101.5f;
    double amplitude_ref = 0.001f;
    double reference_range_m = 100.0f;
    double range_exponent = 1.0f;
    double phase_rad = 0.7f;
};

struct SimulatorSettings {
    double burst_duration_s = 1.00f;
    double tracking_duration_s = 1.00f;
    double split_duration_s = 15.0f;
    double prediction_duration_s = 15.0f;
    std::uint32_t random_seed = 1U;
    size_t vehicle_count = 1;
};

struct CarSettings {
    Vec3 initial_position_m = Vec3(50.0f, 50.0f, 100.0f);
    Vec3 base_velocity_mps = Vec3(20.0f, 20.0f, 0.0f);
    double yaw_rad = 0.0f;
    double length_m = 4.5f;
    double width_m = 1.8f;
    double height_m = 1.5f;
    double bounce_amplitude_m = 0.01f;  // heavy: 0.002
    double bounce_frequency_hz = 30.0f; // heavy: 4
    double bounce_phase_rad = 0.0f;     // each car should be different
    std::complex<double> reflectivity = std::polar(1.0f, 0.0f);
};

struct VehicleState {
    Vec3 center_m = Vec3::Zero();
    Vec3 velocity_mps = Vec3::Zero();
    double yaw_rad = 0.0f;
    std::complex<double> reflectivity = {1.0f, 0.0f};
};

struct SimulationMetrics {
    double time_s = 0.0f;
    double range_m = 0.0f;
    double delay_s = 0.0f;
    double radial_velocity_mps = 0.0f;
    double doppler_hz = 0.0f;
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
