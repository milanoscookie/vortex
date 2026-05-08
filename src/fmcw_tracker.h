#pragma once

#include "problem_description.h"
#include "utils/RingBuffer.h"

#include <Eigen/Dense>
#include <unsupported/Eigen/FFT>

#include <array>
#include <complex>
#include <cstddef>
#include <memory>
#include <span>
#include <vector>

namespace fmcw_tracker {

using Complex = std::complex<double>;

struct RadarConfig {
    double sample_rate_hz = 0.0f;
    double carrier_hz = 0.0f;
    double bandwidth_hz = 0.0f;
    double chirp_duration_s = 0.0f;
    double speed_of_light_mps = 0.0f;
    std::size_t block_size = 0;
    std::size_t chirp_count = 0;
    std::size_t probe_num_x = 0;
    std::size_t probe_num_y = 0;
    double probe_dx_m = 0.0f;
    double probe_dy_m = 0.0f;

    double chirpSlopeHzPerS() const noexcept {
        return bandwidth_hz / chirp_duration_s;
    }

    double wavelengthM() const noexcept {
        return speed_of_light_mps / carrier_hz;
    }

    std::size_t numRx() const noexcept {
        return probe_num_x * probe_num_y;
    }
};

struct DetectionConfig {
    double min_range_m = 20.0f;
    double max_range_m = 500.0f;
    std::size_t coherent_processing_interval_chirps = 64;
    std::size_t hop_chirps = 64;
    std::size_t zero_doppler_guard_bins = 2;
    std::size_t nfft_range_min = 4096;
    bool static_clutter_suppression_enable = true;
    bool aoa_enable = true;
    double azimuth_min_deg = -90.0f;
    double azimuth_max_deg = 90.0f;
    std::size_t azimuth_count = 181;
    double elevation_min_deg = 0.0f;
    double elevation_max_deg = 90.0f;
    std::size_t elevation_count = 181;
    std::size_t range_gate_bins = 30;
    std::size_t doppler_gate_bins = 10;
    double range_association_sigma_m = 1.0f;
    double doppler_association_sigma_hz = 800.0f;
    double doppler_interp_gate_hz = 5000.0f;
    double range_interp_gate_m = 10.0f;
    double azimuth_association_sigma_deg = 12.0f;
    double elevation_association_sigma_deg = 50.0f;
};

struct BatchResult {
    double time_s = 0.0f;
    double range_m = 0.0f;
    double doppler_hz = 0.0f;
    double phase_rad = 0.0f;
    double predicted_range_m = 0.0f;
    double predicted_doppler_hz = 0.0f;
    double range_bin_offset = 0.0f;
    double doppler_bin_offset = 0.0f;
    bool valid = false;
    problem::Vec3 direction = problem::Vec3::Zero();
    problem::Vec3 predicted_direction = problem::Vec3::Zero();
    std::size_t range_bin = 0;
    std::size_t doppler_bin = 0;
    std::size_t azimuth_bin = 0;
    std::size_t elevation_bin = 0;
    std::vector<double> doppler_slice_power;
    std::vector<double> slow_time_phase_rad;
};

struct TrackSummary {
    std::vector<BatchResult> batch_results;
    std::vector<double> times_s;
    std::vector<problem::Vec3> raw_positions_m;
    std::vector<problem::Vec3> smoothed_positions_m;
    std::vector<double> ranges_m;
    std::vector<double> radial_velocity_mps;
    std::vector<double> unwrapped_phase_rad;
    std::vector<double> detrended_phase_rad;
    std::vector<problem::Vec3> cartesian_velocity_mps;
    std::vector<double> velocity_axis_mps;
    std::vector<problem::SimulationMetrics> truth_metrics;
    double microdoppler_phase_frequency_hz = 0.0f;
    double microdoppler_truth_frequency_hz = 0.0f;
    double microdoppler_frequency_rmse_hz = 0.0f;
    double microdoppler_residual_phase_mean_rad = 0.0f;
    double microdoppler_residual_phase_rms_rad = 0.0f;
    double microdoppler_residual_phase_stddev_rad = 0.0f;
    double microdoppler_peak_power = 0.0f;
    std::size_t microdoppler_valid_cpi_count = 0;
    std::vector<double> microdoppler_candidate_frequency_hz;
    std::vector<double> microdoppler_candidate_power;
};

class StreamingTracker {
  public:
    static constexpr std::size_t kBlockSize = problem::RadarSettings::kRadarBlockSize;
    static constexpr std::size_t kNumRx = problem::RadarSettings::kProbeNumElements;
    static constexpr std::size_t kMaxCpiChirps = 256U;
    using ChirpBlock = std::array<Complex, kBlockSize * kNumRx>;

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
    struct CandidateScoreScratch {
        std::size_t doppler_bin = 0;
        std::size_t range_bin = 0;
        double range_m = 0.0f;
        double doppler_hz = 0.0f;
        double score = 0.0f;
    };

    BatchResult processCurrentWindow(std::size_t start_chirp);

    struct TrackingState {
        bool initialized = false;
        double time_s = 0.0f;
        double range_m = 0.0f;
        double radial_velocity_mps = 0.0f;
        double doppler_hz = 0.0f;
        problem::Vec3 position_m = problem::Vec3::Zero();
        problem::Vec3 velocity_mps = problem::Vec3::Zero();
        problem::Vec3 direction = problem::Vec3::UnitX();
    };
    problem::ProblemDescription description_;
    RadarConfig radar_config_;
    DetectionConfig detection_config_;
    std::vector<int> range_indices_;
    std::vector<double> range_axis_sliced_m_;
    std::size_t range_bin_count_ = 0;
    std::vector<Complex> doppler_window_;
    std::vector<double> doppler_axis_hz_;
    std::vector<double> velocity_axis_mps_;
    std::vector<Complex> steering_conj_;
    std::vector<problem::Vec3> directions_;
    std::vector<double> direction_azimuth_deg_;
    std::vector<double> direction_elevation_deg_;
    std::vector<Complex> range_mix_coeff_;
    Eigen::FFT<double> fft_;
    std::vector<Complex> range_fft_input_;
    std::vector<Complex> range_fft_output_;
    std::vector<Complex> doppler_fft_input_;
    std::vector<Complex> doppler_fft_output_;
    std::vector<Complex> spec_scratch_;
    std::vector<Complex> rd_cube_scratch_;
    std::vector<Complex> clutter_mean_scratch_;
    std::vector<double> rd_power_scratch_;
    std::vector<CandidateScoreScratch> rd_candidates_scratch_;
    bool range_mix_coeff_initialized_ = false;
    std::unique_ptr<RingBuffer<ChirpBlock, kMaxCpiChirps>> chirp_window_;
    std::vector<BatchResult> batch_results_;
    TrackingState tracking_state_;
    std::size_t range_wrap_guard_samples_ = 0;
};

problem::SimulationMetrics truthAtTime(const problem::ProblemDescription &description,
                                       double time_s);

} // namespace fmcw_tracker
