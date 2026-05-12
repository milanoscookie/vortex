#pragma once

#include <Eigen/Dense>

#include <complex>
#include <cstddef>
#include <cstdint>
#include <vector>

namespace problem {

using Real = double;
using AmpReal = double;
using Vec3 = Eigen::Vector3d;
using SignalComplex = std::complex<Real>;
using AmplitudeComplex = std::complex<AmpReal>;
using size_t = std::size_t;

struct Constants {
    static constexpr Real kPi = 3.14159265358979323846f;
    static constexpr Real kSpeedOfLightMps = 299'792'458.0f;
    static constexpr Real kInvSqrt2 = 0.70710678f;
};
struct RadarSettings {
    static constexpr size_t kProbeNumX = 4;
    static constexpr size_t kProbeNumY = 4;
    static constexpr size_t kProbeNumElements = kProbeNumX * kProbeNumY;

    // 400 MHz gives a crisp 0.375m theoretical range bin resolution
    static constexpr Real kDefaultBandwidthHz = 1000.0e6f;

    // 160 MHz sample rate prevents aliasing out to 500m
    static constexpr Real kDefaultSampleRateHz = 160.0e6f;
    // 4096 samples / 160 MHz = 25.6 microseconds
    static constexpr Real kDefaultChirpDurationS = 25.6e-6f;

    static constexpr size_t kRadarBlockSize =
        static_cast<size_t>(kDefaultSampleRateHz * kDefaultChirpDurationS + 0.5f);

    static constexpr Real kDefaultCarrierHz = 77.0e9f;
    static constexpr Real kDefaultMinRangeM = 1.0f;
    static constexpr Real kDefaultMaxRangeM = 500.0f;

    static constexpr Real kDefaultFieldGain = 1.0f;
    static constexpr AmpReal kDefaultReceiverNoiseStddev = 4.0e-7;
    // static constexpr AmpReal kDefaultReceiverNoiseStddev = 2.0e-7;
    static constexpr AmpReal kDefaultReceiverNoiseMean = 0.0;
    static constexpr AmpReal kDefaultReceiverNoiseDistributionStddev = 1.0;
    static constexpr bool kDefaultUseStdNormalDistribution = true;

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
// struct RadarSettings {
//     static constexpr size_t kProbeNumX = 4;
//     static constexpr size_t kProbeNumY = 4;
//     static constexpr size_t kProbeNumElements = kProbeNumX * kProbeNumY;
//     // // 400 MHz gives a crisp 0.375m theoretical range bin resolution
//     // static constexpr double kDefaultBandwidthHz = 400.0e6f;
//     //
//     // // We need 4096 samples to prevent aliasing out to 500m.
//     // // To process 4096 samples in a reasonable chirp time, we need a fast ADC.
//     // static constexpr double kDefaultSampleRateHz = 160.0e6f;
//     //
//     // // 4096 samples / 160 MHz = 25.6 microseconds
//     // static constexpr double kDefaultChirpDurationS = 25.6e-6f;
//     //
//     // // This will perfectly compute to 4096
//     // static constexpr size_t kRadarBlockSize =
//     //     static_cast<size_t>(kDefaultSampleRateHz * kDefaultChirpDurationS + 0.5f);
//     // static constexpr double kDefaultCarrierHz = 77.0e9f;
//     // static constexpr double kDefaultMinRangeM = 1.0f;
//     // static constexpr double kDefaultMaxRangeM = 500.0f;
//     // static constexpr double kDefaultFieldGain = 1.0f;
//     // static constexpr double kDefaultReceiverNoiseStddev = 0.0f;
//     // static constexpr double kDefaultReceiverNoiseMean = 0.0f;
//     // static constexpr double kDefaultReceiverNoiseDistributionStddev = 1.0f;
//     // static constexpr bool kDefaultUseStdNormalDistribution = true;
//
//     static constexpr Real kDefaultBandwidthHz = 10.0e6f;
//     static constexpr Real kDefaultSampleRateHz = 2.0f * kDefaultBandwidthHz;
//     static constexpr Real kDefaultChirpDurationS = 12.8e-6f;
//     static constexpr size_t kRadarBlockSize =
//         static_cast<size_t>(kDefaultSampleRateHz * kDefaultChirpDurationS + 0.5f);
//     static constexpr Real kDefaultCarrierHz = 77.0e9f;
//     static constexpr Real kDefaultMinRangeM = 1.0f;
//     static constexpr Real kDefaultMaxRangeM = 500.0f;
//     static constexpr Real kDefaultFieldGain = 1.0f;
//     static constexpr AmpReal kDefaultReceiverNoiseStddev = 0.0;
//     static constexpr AmpReal kDefaultReceiverNoiseMean = 0.0;
//     static constexpr AmpReal kDefaultReceiverNoiseDistributionStddev = 1.0;
//     static constexpr bool kDefaultUseStdNormalDistribution = true;
//
//     Real bandwidth_hz = kDefaultBandwidthHz;
//     Real sample_rate_hz = kDefaultSampleRateHz;
//     Real carrier_hz = kDefaultCarrierHz;
//     Real min_range_m = kDefaultMinRangeM;
//     Real max_range_m = kDefaultMaxRangeM;
//
//     Real field_gain = kDefaultFieldGain;
//     AmpReal receiver_noiselevel_stddev = kDefaultReceiverNoiseStddev;
//     AmpReal receiver_noiselevel_mean = kDefaultReceiverNoiseMean;
//     AmpReal receiver_noise_distribution_stddev = kDefaultReceiverNoiseDistributionStddev;
//     bool receiver_noise_use_std_normal_distribution = kDefaultUseStdNormalDistribution;
// };

struct ProbeSettings {
    Vec3 center_m = Vec3::Zero();
    Real spacing_x_wavelengths = 0.5f;
    Real spacing_y_wavelengths = 0.5f;
};

struct FloorplaneClutterSettings {
    bool enable_static_floorplane = true;
    Real range_m = 101.5f;
    AmpReal amplitude_ref = 1.0e-5;
    Real reference_range_m = 100.0f;
    Real range_exponent = 1.0f;
    Real phase_rad = 0.7f;
};

struct SimulatorSettings {
    Real burst_duration_s = 0.10f;
    Real tracking_duration_s = 0.10f;
    Real split_duration_s = 15.0f;
    Real prediction_duration_s = 15.0f;
    std::uint32_t random_seed = 1U;
    size_t vehicle_count = 1;
};

struct CarSettings {
    Vec3 initial_position_m = Vec3(0.0f, 0.0f, 100.0f);
    Vec3 base_velocity_mps = Vec3(4.0f, 4.0f, 0.0f);
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
                 .initial_position_m = Vec3(82.0f, -38.0f, 145.0f),
                 .base_velocity_mps = Vec3(18.0f, 6.0f, 0.0f),
                 // .base_velocity_mps = Vec3::Zero(),
                 .bounce_amplitude_m = 0.0010f,
                 .bounce_frequency_hz = 510.0f,
                 .bounce_phase_rad = 0.2f,
                 .reflectivity = std::polar(1.0f, 3.1f),
             },
             CarSettings{
                 .initial_position_m = Vec3(122.0f, 26.0f, 152.0f),
                 .base_velocity_mps = Vec3(12.0f, -5.0f, 0.0f),
                 // .base_velocity_mps = Vec3::Zero(),
                 .bounce_amplitude_m = 0.0010f,
                 .bounce_frequency_hz = 1000.0f,
                 .bounce_phase_rad = 0.8f,
                 .reflectivity = std::polar(1.0f, 1.9f),
             },
             CarSettings{
                 .initial_position_m = Vec3(-108.0f, 84.0f, 138.0f),
                 .base_velocity_mps = Vec3(24.0f, 13.0f, 0.0f),
                 // .base_velocity_mps = Vec3::Zero(),
                 .bounce_amplitude_m = 0.0010f,
                 .bounce_frequency_hz = 1700.0f,
                 .bounce_phase_rad = 0.0f,
                 .reflectivity = std::polar(1.0f, 0.4f),
             }}};

} // namespace problem
