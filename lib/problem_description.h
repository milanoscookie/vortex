#pragma once

#include <Eigen/Dense>

#include <complex>
#include <cstddef>
#include <cstdint>

namespace problem {

using Vec3 = Eigen::Vector3f;

inline constexpr float kDefaultNoiseStddev = 0.1f;
inline constexpr float kNoiseDistributionMean = 0.0f;
inline constexpr float kNoiseDistributionStddev = 0.1f;
inline constexpr float kComplexNoiseQuadratureScale = 0.7071067811865475f;

inline constexpr float kPi = 3.14159265358979323846f;
inline constexpr std::size_t kRadarBlockSize = 256;
inline constexpr std::size_t kRadarChirpCount = 256;
inline constexpr std::size_t kProbeNumX = 8;
inline constexpr std::size_t kProbeNumY = 8;
inline constexpr std::size_t kProbeNumElements = kProbeNumX * kProbeNumY;

struct CarDynamicsModel {
  Vec3 initial_position_m = Vec3(80.0f, -18.0f, 1.5f);
  Vec3 base_velocity_mps = Vec3(0.0f, 20.0f, 0.0f);
  float yaw_rad = 0.0f;
  float length_m = 4.5f;
  float width_m = 1.8f;
  float height_m = 1.5f;
  float bounce_amplitude_m = 0.05f;
  float bounce_frequency_hz = 3.0f;
  float bounce_phase_rad = 0.0f;
  std::complex<float> reflectivity = {1.0f, 0.0f};
};

using TargetModel = CarDynamicsModel;

struct CarState {
  Vec3 center_m = Vec3(0.0f, 0.0f, 20.0f);
  Vec3 velocity_mps = Vec3(0.0f, 30.0f, 0.0f);
  float yaw_rad = 0.0f;
  Vec3 size_m = Vec3(4.5f, 1.8f, 1.5f);
  std::complex<float> reflectivity = {1.0f, 0.0f};
};

struct Config {
  float sample_rate_hz = 20'000'000.0f;
  float carrier_hz = 77.0e9f;
  float max_range_m = 500.0f;
  float noise_stddev = 0.01f;
  float field_gain = 1.0f;
  float min_range_m = 1.0f;
  float speed_of_light_mps = 299'792'458.0f;
  float bandwidth_hz = 100.0e6f;
};

struct Metrics {
  float time_s = 0.0f;
  float range_m = 0.0f;
  float delay_s = 0.0f;
  float radial_velocity_mps = 0.0f;
  float doppler_hz = 0.0f;
  Vec3 position_m = Vec3::Zero();
  Vec3 velocity_mps = Vec3::Zero();
  Vec3 line_of_sight = Vec3::Zero();
};

struct FloorplaneClutterConfig {
  bool enable_static_floorplane = true;
  float range_m = 100.0f;
  float amplitude_ref = 0.02f;
  float reference_range_m = 100.0f;
  float range_exponent = 1.0f;
  float phase_rad = 0.0f;
};

struct ProblemDescription {
  Config simulation;
  std::uint32_t random_seed = 1U;
  std::size_t vehicle_count = 1;
  float tracking_duration_s = 0.128f;
  float split_duration_s = 15.0f;
  float prediction_duration_s = 15.0f;
  TargetModel target;
  FloorplaneClutterConfig floorplane = {};
};

inline const ProblemDescription kDefaultProblemDescription{};

} // namespace problem
