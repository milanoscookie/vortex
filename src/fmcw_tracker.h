#pragma once

#include "problem_description.h"
#include "utils/RingBuffer.h"

#include <Eigen/Dense>

#include <array>
#include <complex>
#include <cstddef>
#include <memory>
#include <span>
#include <vector>

namespace fmcw_tracker {

using Complex = std::complex<float>;

struct RadarConfig {
    float sample_rate_hz = 0.0f;
    float carrier_hz = 0.0f;
    float bandwidth_hz = 0.0f;
    float chirp_duration_s = 0.0f;
    float speed_of_light_mps = 0.0f;
    std::size_t block_size = 0;
    std::size_t chirp_count = 0;
    std::size_t probe_num_x = 0;
    std::size_t probe_num_y = 0;
    float probe_dx_m = 0.0f;
    float probe_dy_m = 0.0f;

    float chirpSlopeHzPerS() const noexcept {
        return bandwidth_hz / chirp_duration_s;
    }

    float wavelengthM() const noexcept {
        return speed_of_light_mps / carrier_hz;
    }

    std::size_t numRx() const noexcept {
        return probe_num_x * probe_num_y;
    }
};

struct DetectionConfig {
    float min_range_m = 20.0f;
    float max_range_m = 500.0f;
    std::size_t coherent_processing_interval_chirps = 64;
    std::size_t hop_chirps = 64;
    std::size_t zero_doppler_guard_bins = 2;
    std::size_t nfft_range_min = 4096;
    bool static_clutter_suppression_enable = true;
    bool aoa_enable = true;
    float azimuth_min_deg = -90.0f;
    float azimuth_max_deg = 90.0f;
    std::size_t azimuth_count = 181;
    float elevation_min_deg = 0.0f;
    float elevation_max_deg = 90.0f;
    std::size_t elevation_count = 5001;
    std::size_t range_gate_bins = 30;
    std::size_t doppler_gate_bins = 10;
    float range_association_sigma_m = 1.0f;
    float doppler_association_sigma_hz = 800.0f;
    float doppler_interp_gate_hz = 5000.0f;
    float range_interp_gate_m = 10.0f;
    float azimuth_association_sigma_deg = 12.0f;
    float elevation_association_sigma_deg = 6.0f;
};

struct BatchResult {
    static constexpr int kDopplerSliceSize = 2 * 64;
    static constexpr int kSlowTimeSize = 64;
    float time_s = 0.0f;
    float range_m = 0.0f;
    float doppler_hz = 0.0f;
    float phase_rad = 0.0f;
    float predicted_range_m = 0.0f;
    float predicted_doppler_hz = 0.0f;
    float range_bin_offset = 0.0f;
    float doppler_bin_offset = 0.0f;
    bool valid = false;
    problem::Vec3 direction = problem::Vec3::Zero();
    problem::Vec3 predicted_direction = problem::Vec3::Zero();
    std::size_t range_bin = 0;
    std::size_t doppler_bin = 0;
    std::size_t azimuth_bin = 0;
    std::size_t elevation_bin = 0;
    Eigen::Matrix<float, kDopplerSliceSize, 1> doppler_slice_power =
        Eigen::Matrix<float, kDopplerSliceSize, 1>::Zero();
    Eigen::Matrix<float, kSlowTimeSize, 1> slow_time_phase_rad =
        Eigen::Matrix<float, kSlowTimeSize, 1>::Zero();
};

struct TrackSummary {
    std::vector<BatchResult> batch_results;
    std::vector<float> times_s;
    std::vector<problem::Vec3> raw_positions_m;
    std::vector<problem::Vec3> smoothed_positions_m;
    std::vector<float> ranges_m;
    std::vector<float> radial_velocity_mps;
    std::vector<float> unwrapped_phase_rad;
    std::vector<float> detrended_phase_rad;
    std::vector<problem::Vec3> cartesian_velocity_mps;
    std::vector<float> velocity_axis_mps;
    std::vector<problem::SimulationMetrics> truth_metrics;
    float microdoppler_phase_frequency_hz = 0.0f;
    float microdoppler_truth_frequency_hz = 0.0f;
    float microdoppler_frequency_rmse_hz = 0.0f;
    float microdoppler_residual_phase_mean_rad = 0.0f;
    float microdoppler_residual_phase_rms_rad = 0.0f;
    float microdoppler_residual_phase_stddev_rad = 0.0f;
    float microdoppler_peak_power = 0.0f;
    std::size_t microdoppler_valid_cpi_count = 0;
    std::vector<float> microdoppler_candidate_frequency_hz;
    std::vector<float> microdoppler_candidate_power;
};

class StreamingTracker {
  public:
    static constexpr std::size_t kBlockSize = problem::Constants::kRadarBlockSize;
    static constexpr std::size_t kNumRx = problem::RadarSettings::kProbeNumElements;
    static constexpr std::size_t kMaxCpiChirps = 256U;
    static constexpr std::size_t kFixedCpiChirps = 64U;
    static constexpr std::size_t kFixedRangeNfft = 4096U;
    static constexpr std::size_t kFixedDopplerFftSize = 2U * kFixedCpiChirps;
    static constexpr std::size_t kFixedAzimuthCount = 181U;
    static constexpr std::size_t kFixedElevationCount = 5001U;
    static constexpr std::size_t kFixedRangeBinCount = 512U;
    using ChirpBlock = std::array<Complex, kBlockSize * kNumRx>;
    using ChirpReference = std::array<Complex, kBlockSize>;

    explicit StreamingTracker(const problem::ProblemDescription &description,
                              DetectionConfig detection_config = {});

    void pushChirp(std::size_t chirp_index,
                   std::span<const Complex> tx_chirp,
                   std::span<const Complex> rx_block);

    const RadarConfig &radarConfig() const noexcept {
        return radar_config_;
    }

    const DetectionConfig &detectionConfig() const noexcept {
        return detection_config_;
    }

    const std::vector<BatchResult> &batchResults() const noexcept {
        return batch_results_;
    }

    TrackSummary buildSummary() const;

  private:
    BatchResult processCurrentWindow(std::size_t start_chirp);

    struct TrackingState {
        bool initialized = false;
        float time_s = 0.0f;
        float range_m = 0.0f;
        float radial_velocity_mps = 0.0f;
        float doppler_hz = 0.0f;
        problem::Vec3 position_m = problem::Vec3::Zero();
        problem::Vec3 velocity_mps = problem::Vec3::Zero();
        problem::Vec3 direction = problem::Vec3::UnitX();
    };
    problem::ProblemDescription description_;
    RadarConfig radar_config_;
    DetectionConfig detection_config_;
    std::array<int, kFixedRangeBinCount> range_indices_{};
    Eigen::Matrix<float, kFixedRangeBinCount, 1> range_axis_sliced_m_ =
        Eigen::Matrix<float, kFixedRangeBinCount, 1>::Zero();
    std::size_t range_bin_count_ = 0;
    ChirpReference range_window_{};
    Eigen::Matrix<Complex, kFixedCpiChirps, 1> doppler_window_ =
        Eigen::Matrix<Complex, kFixedCpiChirps, 1>::Zero();
    Eigen::Matrix<float, kFixedDopplerFftSize, 1> doppler_axis_hz_ =
        Eigen::Matrix<float, kFixedDopplerFftSize, 1>::Zero();
    Eigen::Matrix<float, kFixedDopplerFftSize, 1> velocity_axis_mps_ =
        Eigen::Matrix<float, kFixedDopplerFftSize, 1>::Zero();
    std::vector<Complex> steering_conj_;
    std::vector<problem::Vec3> directions_;
    ChirpReference tx_conj_{};
    bool tx_conj_initialized_ = false;
    std::unique_ptr<RingBuffer<ChirpBlock, kMaxCpiChirps>> chirp_window_;
    std::vector<BatchResult> batch_results_;
    TrackingState tracking_state_;
    std::size_t range_wrap_guard_samples_ = 0;
};

problem::SimulationMetrics truthAtTime(const problem::ProblemDescription &description,
                                       float time_s);

} // namespace fmcw_tracker
