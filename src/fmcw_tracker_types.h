#pragma once

#include "problem_description.h"

#include <complex>
#include <cstddef>
#include <cstdint>
#include <vector>

namespace fmcw_tracker {

using Real = problem::Real;
using Complex = problem::SignalComplex;

struct RadarConfig {
    Real sample_rate_hz = 0.0f;
    Real carrier_hz = 0.0f;
    Real bandwidth_hz = 0.0f;
    Real chirp_duration_s = 0.0f;
    Real speed_of_light_mps = 0.0f;
    std::size_t block_size = 0;
    std::size_t chirp_count = 0;
    std::size_t probe_num_x = 0;
    std::size_t probe_num_y = 0;
    Real probe_dx_m = 0.0f;
    Real probe_dy_m = 0.0f;

    Real chirpSlopeHzPerS() const noexcept {
        return bandwidth_hz / chirp_duration_s;
    }

    Real wavelengthM() const noexcept {
        return speed_of_light_mps / carrier_hz;
    }

    std::size_t numRx() const noexcept {
        return probe_num_x * probe_num_y;
    }
};

struct DetectionConfig {
    Real min_range_m = 20.0f;
    Real max_range_m = 500.0f;
    std::size_t coherent_processing_interval_chirps = 256;
    std::size_t hop_chirps = 64;
    std::size_t zero_doppler_guard_bins = 2;
    std::size_t nfft_range_min = 4096;
    bool static_clutter_suppression_enable = true;
    bool aoa_enable = true;
    Real azimuth_min_deg = -90.0f;
    Real azimuth_max_deg = 90.0f;
    std::size_t azimuth_count = 181;
    Real elevation_min_deg = 0.0f;
    Real elevation_max_deg = 90.0f;
    std::size_t elevation_count = 181;
    std::size_t range_gate_bins = 30;
    std::size_t doppler_gate_bins = 10;
    Real range_association_sigma_m = 5.0f;
    Real doppler_association_sigma_hz = 4000.0f;
    Real doppler_interp_gate_hz = 5000.0f;
    Real range_interp_gate_m = 20.0f;
    Real azimuth_association_sigma_deg = 10.0f;
    Real elevation_association_sigma_deg = 15.0f;
    Real rd_detection_threshold_scale = 12.0f;
    std::size_t cfar_guard_range_bins = 2;
    std::size_t cfar_guard_doppler_bins = 2;
    std::size_t cfar_training_range_bins = 6;
    std::size_t cfar_training_doppler_bins = 6;
    Real cfar_min_snr_db = 8.0f;
    std::size_t max_active_tracks = 32;
    std::size_t max_measurements_per_cpi = 32;
    std::size_t max_detection_cells = 256;
    std::size_t cluster_range_gate_bins = 2;
    std::size_t cluster_doppler_gate_bins = 2;
    Real measurement_merge_range_m = 4.0f;
    Real measurement_merge_doppler_hz = 800.0f;
    Real measurement_merge_azimuth_deg = 4.0f;
    Real measurement_merge_elevation_deg = 6.0f;
    Real birth_min_peak_power = 2.0f;
    Real birth_min_integrated_power = 0.0f;
    std::size_t birth_min_cluster_size = 1;
    Real track_max_range_error_m = 45.0f;
    Real track_max_doppler_error_hz = 15000.0f;
    Real track_max_azimuth_error_deg = 15.0f;
    Real track_max_elevation_error_deg = 20.0f;
    Real track_distance_gate = 45.0f;
    Real track_alpha = 0.1f;
    Real track_beta = 0.01f;
    Real track_alpha_range = 0.10f;
    Real track_alpha_angle = 0.02f;
    Real track_beta_angle = 0.0001f;
    Real track_gamma_radial = 0.02f;
    Real track_birth_distance_gate = 9.0f;
    Real deleted_track_birth_distance_gate = 9.0f;
    Real deleted_track_retention_s = 1.00f;
    Real assignment_miss_cost = 35.0f;
    std::size_t tentative_confirm_hits = 3;
    std::size_t tentative_confirm_window = 6;
    std::size_t tentative_max_misses = 3;
    std::size_t confirmed_max_misses = 15;
};

struct DetectionCell {
    std::size_t doppler_bin = 0;
    std::size_t range_bin = 0;
    Real power = 0.0f;
};

struct MeasurementCandidate {
    Real time_s = 0.0f;
    Real range_m = 0.0f;
    Real doppler_hz = 0.0f;
    Real radial_velocity_mps = 0.0f;
    Real phase_rad = 0.0f;
    Real range_bin_offset = 0.0f;
    Real doppler_bin_offset = 0.0f;
    Real peak_power = 0.0f;
    Real integrated_power = 0.0f;
    problem::Vec3 direction = problem::Vec3::UnitX();
    std::size_t range_bin = 0;
    std::size_t doppler_bin = 0;
    std::size_t azimuth_bin = 0;
    std::size_t elevation_bin = 0;
    std::size_t cluster_size = 0;
    std::vector<DetectionCell> cells;
    std::vector<Real> doppler_slice_power;
    std::vector<Real> slow_time_phase_rad;
};

enum class TrackStatus : std::uint8_t {
    Tentative = 0,
    Confirmed = 1,
    Deleted = 2,
};

struct TrackReport {
    std::size_t id = 0;
    TrackStatus status = TrackStatus::Tentative;
    bool matched = false;
    bool valid = false;
    bool deleted = false;
    std::size_t hit_count = 0;
    std::size_t miss_count = 0;
    std::size_t measurement_index = static_cast<std::size_t>(-1);
    Real time_s = 0.0f;
    Real predicted_range_m = 0.0f;
    Real predicted_doppler_hz = 0.0f;
    Real range_m = 0.0f;
    Real doppler_hz = 0.0f;
    Real radial_velocity_mps = 0.0f;
    Real phase_rad = 0.0f;
    Real range_bin_offset = 0.0f;
    Real doppler_bin_offset = 0.0f;
    problem::Vec3 position_m = problem::Vec3::Zero();
    problem::Vec3 velocity_mps = problem::Vec3::Zero();
    problem::Vec3 direction = problem::Vec3::UnitX();
    std::size_t range_bin = 0;
    std::size_t doppler_bin = 0;
    std::size_t azimuth_bin = 0;
    std::size_t elevation_bin = 0;
    std::vector<Real> doppler_slice_power;
    std::vector<Real> slow_time_phase_rad;
};

struct SceneBatchResult {
    Real time_s = 0.0f;
    std::vector<MeasurementCandidate> measurements;
    std::vector<TrackReport> tracks;
    std::vector<problem::SimulationMetrics> truth_metrics;
};

struct TrackHistory {
    std::size_t id = 0;
    TrackStatus final_status = TrackStatus::Tentative;
    std::vector<TrackReport> reports;
    double microdoppler_frequency_hz = 0.0;
    double microdoppler_truth_frequency_hz = 0.0;
    double microdoppler_frequency_error_hz = 0.0;
    double microdoppler_peak_power = 0.0;
    std::size_t microdoppler_valid_cpi_count = 0;
    std::size_t matched_truth_car_index = static_cast<std::size_t>(-1);
    std::vector<double> microdoppler_candidate_frequency_hz;
    std::vector<double> microdoppler_candidate_power;
};

struct DeletedTrackGhost {
    Real time_s = 0.0f;
    Real range_m = 0.0f;
    Real radial_velocity_mps = 0.0f;
    problem::Vec3 direction = problem::Vec3::UnitX();
};

struct SceneSummary {
    std::vector<SceneBatchResult> scene_batches;
    std::vector<TrackHistory> track_histories;
};

struct BatchResult {
    Real time_s = 0.0f;
    Real range_m = 0.0f;
    Real doppler_hz = 0.0f;
    Real phase_rad = 0.0f;
    Real predicted_range_m = 0.0f;
    Real predicted_doppler_hz = 0.0f;
    Real range_bin_offset = 0.0f;
    Real doppler_bin_offset = 0.0f;
    bool valid = false;
    problem::Vec3 direction = problem::Vec3::Zero();
    problem::Vec3 predicted_direction = problem::Vec3::Zero();
    std::size_t range_bin = 0;
    std::size_t doppler_bin = 0;
    std::size_t azimuth_bin = 0;
    std::size_t elevation_bin = 0;
    std::vector<Real> doppler_slice_power;
    std::vector<Real> slow_time_phase_rad;
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

} // namespace fmcw_tracker
