#include "fmcw_tracker.h"

#include "dynamics.h"
#include "target_observation.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace fmcw_tracker {

TrackSummary StreamingTracker::buildSummary() const {
    TrackSummary summary;
    summary.batch_results.reserve(scene_batch_results_.size());
    std::size_t matched_car_index = 0;
    if (!description_.cars.empty()) {
        const auto valid_it = std::find_if(
            scene_batch_results_.begin(),
            scene_batch_results_.end(),
            [](const SceneBatchResult &scene) {
                return std::any_of(scene.tracks.begin(),
                                   scene.tracks.end(),
                                   [](const TrackReport &track) { return track.valid; });
            });
        if (valid_it != scene_batch_results_.end()) {
            const auto best_track_it =
                std::max_element(valid_it->tracks.begin(),
                                 valid_it->tracks.end(),
                                 [](const TrackReport &lhs, const TrackReport &rhs) {
                                     if (lhs.valid != rhs.valid) {
                                         return rhs.valid;
                                     }
                                     if (lhs.status != rhs.status) {
                                         return lhs.status < rhs.status;
                                     }
                                     return lhs.hit_count < rhs.hit_count;
                                 });
            if (best_track_it != valid_it->tracks.end() && best_track_it->valid) {
                matched_car_index = detail::nearestTruthCarIndex(
                    description_, valid_it->time_s, best_track_it->position_m);
                if (matched_car_index >= description_.cars.size()) {
                    matched_car_index = 0;
                }
            }
        }
    }

    summary.microdoppler_truth_frequency_hz =
        description_.cars.empty() ? 0.0f : description_.cars[matched_car_index].bounce_frequency_hz;
    summary.velocity_axis_mps.assign(velocity_axis_mps_.begin(), velocity_axis_mps_.end());

    for (const SceneBatchResult &scene : scene_batch_results_) {
        BatchResult batch;
        batch.time_s = scene.time_s;
        const TrackReport *best_track = nullptr;
        for (const TrackReport &track : scene.tracks) {
            if (!track.valid) {
                continue;
            }
            if (best_track == nullptr ||
                (best_track->status != TrackStatus::Confirmed &&
                 track.status == TrackStatus::Confirmed) ||
                (best_track->status == track.status && track.hit_count > best_track->hit_count)) {
                best_track = &track;
            }
        }
        if (best_track != nullptr) {
            batch.valid = best_track->valid;
            batch.range_m = best_track->range_m;
            batch.doppler_hz = best_track->doppler_hz;
            batch.predicted_range_m = best_track->predicted_range_m;
            batch.predicted_doppler_hz = best_track->predicted_doppler_hz;
            batch.direction = best_track->direction;
            batch.predicted_direction = best_track->direction;
            batch.phase_rad = best_track->phase_rad;
            batch.range_bin_offset = best_track->range_bin_offset;
            batch.doppler_bin_offset = best_track->doppler_bin_offset;
            batch.range_bin = best_track->range_bin;
            batch.doppler_bin = best_track->doppler_bin;
            batch.azimuth_bin = best_track->azimuth_bin;
            batch.elevation_bin = best_track->elevation_bin;
            batch.doppler_slice_power = best_track->doppler_slice_power;
            batch.slow_time_phase_rad = best_track->slow_time_phase_rad;
        }
        summary.batch_results.push_back(std::move(batch));
    }

    if (summary.batch_results.empty()) {
        return summary;
    }

    const double wavelength_m = radar_config_.wavelengthM();
    summary.times_s.reserve(summary.batch_results.size());
    summary.raw_positions_m.reserve(summary.batch_results.size());
    summary.smoothed_positions_m.reserve(summary.batch_results.size());
    summary.ranges_m.reserve(summary.batch_results.size());
    summary.radial_velocity_mps.reserve(summary.batch_results.size());
    summary.unwrapped_phase_rad.reserve(summary.batch_results.size());
    summary.detrended_phase_rad.reserve(summary.batch_results.size());
    summary.cartesian_velocity_mps.reserve(summary.batch_results.size());
    summary.truth_metrics.reserve(summary.batch_results.size());

    std::vector<double> raw_x(summary.batch_results.size());
    std::vector<double> raw_y(summary.batch_results.size());
    std::vector<double> raw_z(summary.batch_results.size());
    for (std::size_t i = 0; i < summary.batch_results.size(); ++i) {
        const BatchResult &batch = summary.batch_results[i];
        summary.times_s.push_back(batch.time_s);
        const problem::Vec3 raw_xyz = batch.range_m * batch.direction;
        summary.raw_positions_m.push_back(raw_xyz);
        summary.ranges_m.push_back(batch.range_m);
        summary.radial_velocity_mps.push_back(0.5f * batch.doppler_hz * wavelength_m);
        raw_x[i] = raw_xyz.x();
        raw_y[i] = raw_xyz.y();
        raw_z[i] = raw_xyz.z();
        summary.truth_metrics.push_back(truthAtTime(description_, batch.time_s, matched_car_index));
    }

    /*
    std::vector<double> microdoppler_times_s;
    std::vector<double> microdoppler_unwrapped_phase;
    std::vector<double> residual_phase_example;
    std::vector<double> microdoppler_candidate_frequency_hz;
    std::vector<double> microdoppler_candidate_power;
    const double phase_microdoppler_frequency_hz = detail::estimateDominantCpiResidualFrequencyHz(
        summary.batch_results,
        static_cast<double>(radar_config_.chirp_duration_s),
        &microdoppler_times_s,
        &microdoppler_unwrapped_phase,
        &residual_phase_example);
    summary.microdoppler_phase_frequency_hz = detail::estimateDominantBatchDopplerFrequencyHz(
        summary.batch_results,
        static_cast<double>(detection_config_.hop_chirps) * radar_config_.chirp_duration_s,
        &microdoppler_candidate_frequency_hz,
        &microdoppler_candidate_power,
        &summary.microdoppler_peak_power,
        &summary.microdoppler_valid_cpi_count);
    if (summary.microdoppler_phase_frequency_hz <= 0.0f) {
        summary.microdoppler_phase_frequency_hz = phase_microdoppler_frequency_hz;
    }
    summary.unwrapped_phase_rad = microdoppler_unwrapped_phase;
    summary.detrended_phase_rad = residual_phase_example;
    summary.microdoppler_candidate_frequency_hz = microdoppler_candidate_frequency_hz;
    summary.microdoppler_candidate_power = microdoppler_candidate_power;
    */

    const double microdoppler_error_hz =
        summary.microdoppler_phase_frequency_hz - summary.microdoppler_truth_frequency_hz;
    summary.microdoppler_frequency_rmse_hz =
        std::sqrt(microdoppler_error_hz * microdoppler_error_hz);
    if (!summary.detrended_phase_rad.empty()) {
        double sum = 0.0f;
        double squared_sum = 0.0f;
        for (double sample : summary.detrended_phase_rad) {
            sum += sample;
            squared_sum += sample * sample;
        }
        summary.microdoppler_residual_phase_mean_rad =
            sum / static_cast<double>(summary.detrended_phase_rad.size());
        summary.microdoppler_residual_phase_rms_rad =
            std::sqrt(squared_sum / static_cast<double>(summary.detrended_phase_rad.size()));
        double variance_sum = 0.0f;
        for (double sample : summary.detrended_phase_rad) {
            const double centered = sample - summary.microdoppler_residual_phase_mean_rad;
            variance_sum += centered * centered;
        }
        summary.microdoppler_residual_phase_stddev_rad =
            std::sqrt(variance_sum / static_cast<double>(summary.detrended_phase_rad.size()));
    }

    for (std::size_t i = 0; i < summary.batch_results.size(); ++i) {
        summary.smoothed_positions_m.emplace_back(raw_x[i], raw_y[i], raw_z[i]);
    }

    const std::vector<double> vx = detail::gradient(raw_x, summary.times_s);
    const std::vector<double> vy = detail::gradient(raw_y, summary.times_s);
    const std::vector<double> vz = detail::gradient(raw_z, summary.times_s);
    for (std::size_t i = 0; i < summary.batch_results.size(); ++i) {
        summary.cartesian_velocity_mps.emplace_back(vx[i], vy[i], vz[i]);
    }
    return summary;
}

SceneSummary StreamingTracker::buildSceneSummary() const {
    SceneSummary summary;
    summary.scene_batches = scene_batch_results_;
    summary.track_histories.reserve(active_tracks_.size() + finished_tracks_.size());

    auto append_history = [&](const detail::TrackState &track) {
        if (track.id == 0U || track.history.empty()) {
            return;
        }
        TrackHistory history;
        history.id = track.id;
        history.final_status = track.status;
        history.reports = track.history;

        std::vector<BatchResult> microdoppler_batches;
        microdoppler_batches.reserve(track.history.size());
        for (const TrackReport &report : track.history) {
            if (!report.valid || report.slow_time_phase_rad.empty() ||
                report.doppler_slice_power.empty()) {
                continue;
            }
            BatchResult batch;
            batch.time_s = report.time_s;
            batch.range_m = report.range_m;
            batch.doppler_hz = report.doppler_hz;
            batch.phase_rad = report.phase_rad;
            batch.predicted_range_m = report.predicted_range_m;
            batch.predicted_doppler_hz = report.predicted_doppler_hz;
            batch.range_bin_offset = report.range_bin_offset;
            batch.doppler_bin_offset = report.doppler_bin_offset;
            batch.valid = true;
            batch.direction = report.direction;
            batch.predicted_direction = report.direction;
            batch.range_bin = report.range_bin;
            batch.doppler_bin = report.doppler_bin;
            batch.azimuth_bin = report.azimuth_bin;
            batch.elevation_bin = report.elevation_bin;
            batch.doppler_slice_power = report.doppler_slice_power;
            batch.slow_time_phase_rad = report.slow_time_phase_rad;
            microdoppler_batches.push_back(std::move(batch));
        }

        /*
        const double phase_microdoppler_frequency_hz =
            detail::estimateDominantCpiResidualFrequencyHz(
                microdoppler_batches,
                static_cast<double>(radar_config_.chirp_duration_s),
                nullptr,
                nullptr,
                nullptr);
        history.microdoppler_frequency_hz = detail::estimateDominantBatchDopplerFrequencyHz(
            microdoppler_batches,
            static_cast<double>(detection_config_.hop_chirps) * radar_config_.chirp_duration_s,
            &history.microdoppler_candidate_frequency_hz,
            &history.microdoppler_candidate_power,
            &history.microdoppler_peak_power,
            &history.microdoppler_valid_cpi_count);
        if (history.microdoppler_frequency_hz <= 0.0) {
            history.microdoppler_frequency_hz = phase_microdoppler_frequency_hz;
        }
        */

        const auto valid_report_it =
            std::find_if(history.reports.rbegin(),
                         history.reports.rend(),
                         [](const TrackReport &report) { return report.valid; });
        if (valid_report_it != history.reports.rend() && !description_.cars.empty()) {
            history.matched_truth_car_index = detail::nearestTruthCarIndex(
                description_, valid_report_it->time_s, valid_report_it->position_m);
            if (history.matched_truth_car_index < description_.cars.size()) {
                history.microdoppler_truth_frequency_hz =
                    description_.cars[history.matched_truth_car_index].bounce_frequency_hz;
                history.microdoppler_frequency_error_hz = std::abs(
                    history.microdoppler_frequency_hz - history.microdoppler_truth_frequency_hz);
            }
        }

        summary.track_histories.push_back(std::move(history));
    };

    for (const detail::TrackState &track : finished_tracks_) {
        append_history(track);
    }
    for (const detail::TrackState &track : active_tracks_) {
        append_history(track);
    }

    std::sort(summary.track_histories.begin(),
              summary.track_histories.end(),
              [](const TrackHistory &lhs, const TrackHistory &rhs) { return lhs.id < rhs.id; });
    return summary;
}

std::vector<problem::SimulationMetrics> StreamingTracker::truthSceneAtTime(Real time_s) const {
    std::vector<problem::SimulationMetrics> truth;
    truth.reserve(description_.cars.size());
    for (const problem::CarSettings &car : description_.cars) {
        const CarDynamics dynamics(car);
        const radar::TargetObservation observation =
            radar::observeTarget(dynamics, description_.radar, time_s);
        truth.push_back(radar::makeSimulationMetrics(time_s, observation));
    }
    return truth;
}

problem::SimulationMetrics
truthAtTime(const problem::ProblemDescription &description, Real time_s, std::size_t car_index) {
    if (car_index >= description.cars.size()) {
        throw std::runtime_error("truthAtTime requires a valid car index");
    }
    const CarDynamics dynamics(description.cars[car_index]);
    const radar::TargetObservation observation =
        radar::observeTarget(dynamics, description.radar, time_s);
    return radar::makeSimulationMetrics(time_s, observation);
}

} // namespace fmcw_tracker
