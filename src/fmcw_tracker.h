#pragma once

#include "problem_description.h"

#include <complex>
#include <cstddef>
#include <deque>
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
    bool static_clutter_suppression_enable = false;
    bool aoa_enable = true;
    float azimuth_min_deg = -90.0f;
    float azimuth_max_deg = 90.0f;
    std::size_t azimuth_count = 181;
    float elevation_min_deg = 0.0f;
    float elevation_max_deg = 90.0f;
    std::size_t elevation_count = 91;
};

struct BatchResult {
    float time_s = 0.0f;
    float range_m = 0.0f;
    float doppler_hz = 0.0f;
    float phase_rad = 0.0f;
    problem::Vec3 direction = problem::Vec3::Zero();
    std::vector<float> doppler_slice_power;
};

struct TrackSummary {
    std::vector<BatchResult> batch_results;
    std::vector<float> times_s;
    std::vector<problem::Vec3> raw_positions_m;
    std::vector<problem::Vec3> smoothed_positions_m;
    std::vector<float> ranges_m;
    std::vector<float> radial_velocity_mps;
    std::vector<float> unwrapped_phase_rad;
    std::vector<problem::Vec3> cartesian_velocity_mps;
    std::vector<float> velocity_axis_mps;
    std::vector<problem::SimulationMetrics> truth_metrics;
};

class StreamingTracker {
  public:
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
    BatchResult processCurrentWindow(std::size_t start_chirp) const;

    problem::ProblemDescription description_;
    RadarConfig radar_config_;
    DetectionConfig detection_config_;
    std::vector<std::size_t> range_indices_;
    std::vector<float> range_axis_sliced_m_;
    std::vector<Complex> range_window_;
    std::vector<Complex> doppler_window_;
    std::vector<Complex> steering_conj_;
    std::vector<problem::Vec3> directions_;
    std::vector<float> doppler_axis_hz_;
    std::vector<float> velocity_axis_mps_;
    std::vector<Complex> tx_conj_;
    std::deque<std::vector<Complex>> chirp_window_;
    std::vector<BatchResult> batch_results_;
};

problem::SimulationMetrics truthAtTime(const problem::ProblemDescription &description,
                                       float time_s);

} // namespace fmcw_tracker
