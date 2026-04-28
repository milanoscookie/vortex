#pragma once

#include <Eigen/Dense>

#include <complex>
#include <cstddef>
#include <cstdint>

namespace problem {

    using Vec3 = Eigen::Vector3f;
    using size_t = std::size_t;

    struct Constants {
        static inline constexpr float kPi = 3.14159265358979323846f;
        static inline constexpr float kSpeedOfLightMps = 299'792'458.0f;
        static inline constexpr size_t kRadarBlockSize = 256;
        static inline constexpr float kInvSqrt2 = 0.70710678f;
    };

    struct RadarSettings {
        static inline constexpr size_t kProbeNumX = 8;
        static inline constexpr size_t kProbeNumY = 8;
        static inline constexpr size_t kProbeNumElements =
        kProbeNumX * kProbeNumY;

        float sample_rate_hz = 20.0e6f;
        float bandwidth_hz = 10.0e6f;

        float carrier_hz = 77.0e9f;
        float min_range_m = 1.0f;
        float max_range_m = 500.0f;

        float field_gain = 1.0f;
        float receiver_noiselevel_stddev = 1e-5f;
        float receiver_noiselevel_mean = 0.0f;
        float receiver_noise_distribution_stddev = 1.0f;

    };

    struct ProbeSettings {
        Vec3 center_m = Vec3::Zero();
        float spacing_x_wavelengths = 0.5f;
        float spacing_y_wavelengths = 0.5f;
    };

    struct FloorplaneClutterSettings {
        bool enable_static_floorplane = true;
        float range_m = 100.0f;
        float amplitude_ref = 0.001f;
        float reference_range_m = 100.0f;
        float range_exponent = 1.0f;
        float phase_rad = 0.7f;
    };

    struct SimulatorSettings {
        float burst_duration_s = 1.0f;
        float tracking_duration_s = 1.00f;
        float split_duration_s = 15.0f;
        float prediction_duration_s = 15.0f;
        std::uint32_t random_seed = 1U;
        size_t vehicle_count = 1;
    };

    struct CarSettings {
        Vec3 initial_position_m = Vec3(100.0f, 100.0f, 150.0f);
        Vec3 base_velocity_mps = Vec3(20.0f, 20.0f, 0.0f);
        float yaw_rad = 0.0f;
        float length_m = 4.5f;
        float width_m = 1.8f;
        float height_m = 1.5f;
        float bounce_amplitude_m = 0.01f; // heavy: 0.002
        float bounce_frequency_hz = 12.0f; // heavy: 4
        float bounce_phase_rad = 0.5f; // each car should be different
        std::complex<float> reflectivity = std::polar(1.0f, 1.2f);
    };

    struct VehicleState {
        Vec3 center_m = Vec3::Zero();
        Vec3 velocity_mps = Vec3::Zero();
        float yaw_rad = 0.0f;
        std::complex<float> reflectivity = {1.0f, 0.0f};
    };

    struct SimulationMetrics {
        float time_s = 0.0f;
        float range_m = 0.0f;
        float delay_s = 0.0f;
        float radial_velocity_mps = 0.0f;
        float doppler_hz = 0.0f;
        Vec3 position_m = Vec3::Zero();
        Vec3 velocity_mps = Vec3::Zero();
        Vec3 line_of_sight = Vec3::Zero();
    };

    struct ProblemDescription {
        RadarSettings radar;
        ProbeSettings probe;
        FloorplaneClutterSettings floorplane_clutter;
        SimulatorSettings simulator;
        CarSettings car;
    };

    inline const ProblemDescription kDefaultProblemDescription{};

} // namespace problem
