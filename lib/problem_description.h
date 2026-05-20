#pragma once

#include <Eigen/Dense>

#include <complex>
#include <cstddef>
#include <cstdint>
#include <vector>

namespace dsp {

using Vec3 = Eigen::Vector3d;
using Complex = std::complex<double>;

struct Constants {
    static constexpr double kPi = 3.14159265358979323846;
    static constexpr double kSpeedOfLightMps = 299'792'458.0;
    static constexpr double kInvSqrt2 = 0.70710678;
};

inline constexpr size_t kProbeNumX = 4;
inline constexpr size_t kProbeNumY = 4;
inline constexpr size_t kProbeNumElements = kProbeNumX * kProbeNumY;

inline constexpr double kDefaultBandwidthHz = 10.0e6;
inline constexpr double kDefaultSampleRateHz = 2.0 * kDefaultBandwidthHz;
inline constexpr double kDefaultChirpDurationS = 12.8e-6;
inline constexpr size_t kRadarBlockSize =
    static_cast<size_t>(kDefaultSampleRateHz * kDefaultChirpDurationS + 0.5);
inline constexpr double kDefaultCarrierHz = 77.0e9;
inline constexpr double kDefaultMinRangeM = 1.0;
inline constexpr double kDefaultMaxRangeM = 500.0;
inline constexpr double kDefaultFieldGain = 1.0;
inline constexpr double kDefaultReceiverNoiseStddev = 0.0;
inline constexpr double kDefaultReceiverNoiseMean = 0.0;
inline constexpr double kDefaultReceiverNoiseDistributionStddev = 1.0;
inline constexpr bool kDefaultUseStdNormalDistribution = false;

struct RadarSettings {
    static constexpr size_t kProbeNumX = dsp::kProbeNumX;
    static constexpr size_t kProbeNumY = dsp::kProbeNumY;
    static constexpr size_t kProbeNumElements = dsp::kProbeNumElements;

    static constexpr double kDefaultBandwidthHz = dsp::kDefaultBandwidthHz;
    static constexpr double kDefaultSampleRateHz = dsp::kDefaultSampleRateHz;
    static constexpr double kDefaultChirpDurationS = dsp::kDefaultChirpDurationS;
    static constexpr size_t kRadarBlockSize = dsp::kRadarBlockSize;
    static constexpr double kDefaultCarrierHz = dsp::kDefaultCarrierHz;
    static constexpr double kDefaultMinRangeM = dsp::kDefaultMinRangeM;
    static constexpr double kDefaultMaxRangeM = dsp::kDefaultMaxRangeM;
    static constexpr double kDefaultFieldGain = dsp::kDefaultFieldGain;
    static constexpr double kDefaultReceiverNoiseStddev = dsp::kDefaultReceiverNoiseStddev;
    static constexpr double kDefaultReceiverNoiseMean = dsp::kDefaultReceiverNoiseMean;
    static constexpr double kDefaultReceiverNoiseDistributionStddev =
        dsp::kDefaultReceiverNoiseDistributionStddev;
    static constexpr bool kDefaultUseStdNormalDistribution =
        dsp::kDefaultUseStdNormalDistribution;

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
    double spacing_x_wavelengths = 0.5;
    double spacing_y_wavelengths = 0.5;
};

struct FloorplaneClutterSettings {
    bool enable_static_floorplane = false;
    double range_m = 101.5;
    double amplitude_ref = 0.001;
    double reference_range_m = 100.0;
    double range_exponent = 1.0;
    double phase_rad = 0.7;
};

struct SimulatorSettings {
    double burst_duration_s = 1.0;
    double tracking_duration_s = 1.0;
    double split_duration_s = 15.0;
    double prediction_duration_s = 15.0;
    std::uint32_t random_seed = 1U;
    size_t vehicle_count = 1;
};

struct CarSettings {
    Vec3 initial_position_m = Vec3(50.0, 50.0, 100.0);
    Vec3 base_velocity_mps = Vec3(20.0, 20.0, 0.0);
    double yaw_rad = 0.0;
    double length_m = 4.5;
    double width_m = 1.8;
    double height_m = 1.5;
    double bounce_amplitude_m = 0.01;
    double bounce_frequency_hz = 30.0;
    double bounce_phase_rad = 0.0;
    Complex reflectivity = std::polar(1.0, 0.0);
};

struct VehicleState {
    Vec3 center_m = Vec3::Zero();
    Vec3 velocity_mps = Vec3::Zero();
    double yaw_rad = 0.0;
    Complex reflectivity = {1.0, 0.0};
};

struct SimulationMetrics {
    double time_s = 0.0;
    double range_m = 0.0;
    double delay_s = 0.0;
    double radial_velocity_mps = 0.0;
    double doppler_hz = 0.0;
    Vec3 position_m = Vec3::Zero();
    Vec3 velocity_mps = Vec3::Zero();
    Vec3 line_of_sight = Vec3::Zero();
};

inline const ProbeSettings kDefaultProbeSettings{};

inline constexpr FloorplaneClutterSettings kDefaultFloorplaneClutterSettings{};

inline constexpr SimulatorSettings kDefaultSimulatorSettings{};

inline const CarSettings kDefaultCarSettings{
    .initial_position_m = Vec3(50.0, 50.0, 100.0),
    .base_velocity_mps = Vec3(20.0, 20.0, 0.0),
    .yaw_rad = 0.0,
    .length_m = 4.5,
    .width_m = 1.8,
    .height_m = 1.5,
    .bounce_amplitude_m = 0.01,
    .bounce_frequency_hz = 30.0,
    .bounce_phase_rad = 0.0,
    .reflectivity = std::polar(1.0, 0.0),
};

inline constexpr RadarSettings kDefaultRadarSettings{};

inline const CarSettings kDefaultSceneCarA{
    .initial_position_m = Vec3(107.0, 107.0, 150.0),
    .reflectivity = std::polar(1.0, 3.1),
};

inline const CarSettings kDefaultSceneCarB{
    .initial_position_m = Vec3(114.0, 114.0, 150.0),
    .reflectivity = std::polar(1.0, 3.1),
};

inline const CarSettings kDefaultSceneCarC{
    .initial_position_m = Vec3(100.0, 100.0, 150.0),
    .base_velocity_mps = Vec3(20.0, 20.0, 0.0),
    .yaw_rad = 0.3,
    .bounce_amplitude_m = 0.002,
    .bounce_frequency_hz = 4.0,
    .bounce_phase_rad = 1.1,
    .reflectivity = std::polar(0.9, 0.4),
};

struct ProblemDescription {
    RadarSettings radar = kDefaultRadarSettings;
    ProbeSettings probe = kDefaultProbeSettings;
    FloorplaneClutterSettings floorplane_clutter = kDefaultFloorplaneClutterSettings;
    SimulatorSettings simulator = kDefaultSimulatorSettings;
    std::vector<CarSettings> cars;
};

inline const ProblemDescription kDefaultProblemDescription{
    .radar = kDefaultRadarSettings,
    .probe = kDefaultProbeSettings,
    .floorplane_clutter = kDefaultFloorplaneClutterSettings,
    .simulator = {.vehicle_count = 3},
    .cars = {kDefaultSceneCarA, kDefaultSceneCarB, kDefaultSceneCarC},
};

} // namespace dsp
