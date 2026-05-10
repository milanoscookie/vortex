#include "fmcw_tracker.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace fmcw_tracker {

using detail::AssignmentSolution;
using detail::KinematicEstimate;
using detail::Vec3;

void StreamingTracker::updateTracks(const detail::SceneObservation &observation) {
    struct PredictedTrack {
        std::size_t track_index = 0;
        Real dt = 0.0f;
        Vec3 position_m = Vec3::Zero();
        Vec3 velocity_mps = Vec3::Zero();
        KinematicEstimate estimate;
    };

    const std::vector<Vec3> rx_positions = detail::elementPositions(radar_config_);
    const Real wave_number = 2.0f * problem::Constants::kPi / radar_config_.wavelengthM();
    const auto fillTrackBeamformedImage = [&](TrackReport &report) {
        if (observation.spec_cube.empty() || observation.rd_cube.empty() ||
            observation.num_rx == 0U || observation.range_count == 0U ||
            observation.n_chirps == 0U || observation.nfft_doppler == 0U) {
            return;
        }

        const std::size_t range_bin = detail::nearestAxisBin(range_axis_sliced_m_, report.range_m);
        report.range_bin = range_bin;
        report.slow_time_phase_rad.assign(observation.n_chirps, 0.0f);
        report.doppler_slice_power.assign(observation.nfft_doppler, 0.0f);

        std::vector<Complex> steering_conj(observation.num_rx, Complex(0.0f, 0.0f));
        for (std::size_t rx = 0; rx < observation.num_rx; ++rx) {
            const Real phase = wave_number * report.direction.dot(rx_positions[rx]);
            steering_conj[rx] = Complex(std::cos(-phase), std::sin(-phase));
        }

        Complex peak_response(0.0f, 0.0f);
        Real best_power = -1.0f;
        std::size_t best_doppler_bin = 0U;
        for (std::size_t dbin = 0; dbin < observation.nfft_doppler; ++dbin) {
            Complex response(0.0f, 0.0f);
            for (std::size_t rx = 0; rx < observation.num_rx; ++rx) {
                response += steering_conj[rx] *
                            observation.rd_cube[detail::cubeIndex(
                                dbin, range_bin, rx, observation.range_count, observation.num_rx)];
            }
            const Real power = std::norm(response);
            report.doppler_slice_power[dbin] = power;
            if (power > best_power) {
                best_power = power;
                best_doppler_bin = dbin;
                peak_response = response;
            }
        }

        for (std::size_t chirp = 0; chirp < observation.n_chirps; ++chirp) {
            Complex response(0.0f, 0.0f);
            for (std::size_t rx = 0; rx < observation.num_rx; ++rx) {
                response += steering_conj[rx] *
                            observation.spec_cube[detail::cubeIndex(
                                chirp, range_bin, rx, observation.range_count, observation.num_rx)];
            }
            report.slow_time_phase_rad[chirp] = std::arg(response);
        }

        report.phase_rad = std::arg(peak_response);
        report.doppler_bin = best_doppler_bin;
    };

    deleted_track_ghosts_.erase(
        std::remove_if(deleted_track_ghosts_.begin(),
                       deleted_track_ghosts_.end(),
                       [&](const DeletedTrackGhost &ghost) {
                           return observation.time_s - ghost.time_s >
                                  detection_config_.deleted_track_retention_s;
                       }),
        deleted_track_ghosts_.end());

    const std::size_t target_track_count = detection_config_.max_active_tracks;
    if (active_tracks_.size() < target_track_count) {
        active_tracks_.resize(target_track_count);
    }

    const auto resetSlot = [&](detail::TrackState &track) {
        track.id = 0U;
        track.status = TrackStatus::Tentative;
        track.initialized = false;
        track.deleted = false;
        track.time_s = observation.time_s;
        track.range_m = 0.0f;
        track.radial_velocity_mps = 0.0f;
        track.doppler_hz = 0.0f;
        track.position_m = Vec3::Zero();
        track.velocity_mps = Vec3::Zero();
        track.direction = Vec3::UnitX();
        track.hit_count = 0U;
        track.miss_count = 0U;
        track.recent_matches.clear();
        track.history.clear();
    };

    const auto retireSlot = [&](detail::TrackState &track) {
        if (track.initialized && !track.history.empty()) {
            deleted_track_ghosts_.push_back(DeletedTrackGhost{
                .time_s = track.time_s,
                .range_m = track.range_m,
                .radial_velocity_mps = track.radial_velocity_mps,
                .direction = track.direction,
            });
            finished_tracks_.push_back(track);
        }
        resetSlot(track);
    };

    const auto initializeSlotFromMeasurement =
        [&](detail::TrackState &track, const MeasurementCandidate &measurement, Real time_s) {
            if (track.initialized && !track.history.empty()) {
                retireSlot(track);
            }
            track.id = next_track_id_++;
            track.status = TrackStatus::Tentative;
            track.initialized = true;
            track.deleted = false;
            track.time_s = time_s;
            track.position_m = detail::measurementPosition(measurement);
            track.velocity_mps = detail::measurementVelocity(measurement);
            track.velocity_mps.z() = 0.0f;
            const KinematicEstimate estimate = detail::makeKinematicEstimate(
                radar_config_, track.position_m, track.velocity_mps, measurement.direction);
            track.direction = estimate.direction;
            track.range_m = estimate.range_m;
            track.radial_velocity_mps = estimate.radial_velocity_mps;
            track.doppler_hz = estimate.doppler_hz;
            track.hit_count = 1U;
            track.miss_count = 0U;
            track.recent_matches.clear();
            detail::updateRecentMatches(
                track.recent_matches, true, detection_config_.tentative_confirm_window);
        };

    std::vector<PredictedTrack> predicted_tracks;
    predicted_tracks.reserve(active_tracks_.size());
    for (std::size_t track_index = 0; track_index < active_tracks_.size(); ++track_index) {
        detail::TrackState &track = active_tracks_[track_index];
        if (!track.initialized) {
            continue;
        }
        const Real dt = std::max(0.0f, observation.time_s - track.time_s);
        const Vec3 predicted_position = track.position_m + track.velocity_mps * dt;
        const KinematicEstimate estimate = detail::makeKinematicEstimate(
            radar_config_, predicted_position, track.velocity_mps, track.direction);
        predicted_tracks.push_back(
            PredictedTrack{track_index, dt, predicted_position, track.velocity_mps, estimate});
    }

    std::vector<std::vector<Real>> cost_matrix(
        predicted_tracks.size(),
        std::vector<Real>(observation.measurements.size(), std::numeric_limits<Real>::infinity()));
    for (std::size_t predicted_index = 0; predicted_index < predicted_tracks.size();
         ++predicted_index) {
        const PredictedTrack &predicted = predicted_tracks[predicted_index];
        const Real predicted_azimuth_deg = detail::azimuthDeg(predicted.estimate.direction);
        const Real predicted_elevation_deg = detail::elevationDeg(predicted.estimate.direction);
        for (std::size_t measurement_index = 0; measurement_index < observation.measurements.size();
             ++measurement_index) {
            const MeasurementCandidate &measurement = observation.measurements[measurement_index];
            const Real delta_range = measurement.range_m - predicted.estimate.range_m;
            const Real delta_radial_velocity =
                measurement.radial_velocity_mps - predicted.estimate.radial_velocity_mps;
            const Real delta_azimuth = detail::angleDifferenceDeg(
                detail::measurementAzimuthDeg(measurement), predicted_azimuth_deg);
            const Real delta_elevation = detail::angleDifferenceDeg(
                detail::measurementElevationDeg(measurement), predicted_elevation_deg);
            const Real delta_doppler = measurement.doppler_hz - predicted.estimate.doppler_hz;
            if (std::abs(delta_range) > detection_config_.track_max_range_error_m ||
                std::abs(delta_doppler) > detection_config_.track_max_doppler_error_hz ||
                std::abs(delta_azimuth) > detection_config_.track_max_azimuth_error_deg ||
                std::abs(delta_elevation) > detection_config_.track_max_elevation_error_deg) {
                if (active_tracks_.size() > 0 && (scene_batch_results_.size() + 1) % 100 == 0) {
                     // Only log failed candidates for existing tracks periodically to avoid noise
                     // but here we just keep it simple.
                }
                continue;
            }

            const Real distance_sq = detail::computeAssociationDistanceSq(radar_config_,
                                                                          delta_range,
                                                                          delta_radial_velocity,
                                                                          delta_azimuth,
                                                                          delta_elevation,
                                                                          detection_config_);
            if (distance_sq <= detection_config_.track_distance_gate) {
                cost_matrix[predicted_index][measurement_index] = distance_sq;
            }
        }
    }

    const AssignmentSolution assignments =
        detail::solveAssignments(cost_matrix, detection_config_.assignment_miss_cost);

    SceneBatchResult scene;
    scene.time_s = observation.time_s;
    scene.measurements = observation.measurements;
    scene.truth_metrics = truthSceneAtTime(observation.time_s);

    std::vector<bool> measurement_used(observation.measurements.size(), false);
    std::vector<bool> slot_reported(active_tracks_.size(), false);
    for (std::size_t predicted_index = 0; predicted_index < predicted_tracks.size();
         ++predicted_index) {
        const PredictedTrack &predicted = predicted_tracks[predicted_index];
        detail::TrackState &track = active_tracks_[predicted.track_index];
        const int assigned_measurement = assignments.measurement_for_track[predicted_index];
        if (assigned_measurement >= 0) {
            const MeasurementCandidate &measurement =
                observation.measurements[static_cast<std::size_t>(assigned_measurement)];
            const Real dt = std::max(radar_config_.chirp_duration_s, predicted.dt);
            const Vec3 u_pred =
                detail::normalizedOr(predicted.position_m, predicted.estimate.direction);
            const Real r_pred = predicted.position_m.norm();
            const Real range_residual = measurement.range_m - r_pred;
            const Vec3 residual_los = range_residual * u_pred;
            const Vec3 measured_position = detail::measurementPosition(measurement);
            const Vec3 residual_cart = measured_position - predicted.position_m;
            const Vec3 residual_cross = residual_cart - residual_cart.dot(u_pred) * u_pred;

            track.position_m = predicted.position_m +
                               detection_config_.track_alpha_range * residual_los +
                               detection_config_.track_alpha_angle * residual_cross;
            track.velocity_mps =
                predicted.velocity_mps + (detection_config_.track_beta_angle / dt) * residual_cross;
            const Real radial_velocity_pred = predicted.velocity_mps.dot(u_pred);
            const Real radial_velocity_residual =
                measurement.radial_velocity_mps - radial_velocity_pred;
            track.velocity_mps +=
                detection_config_.track_gamma_radial * radial_velocity_residual * u_pred;
            track.velocity_mps.z() = 0.0f;

            const KinematicEstimate estimate = detail::makeKinematicEstimate(
                radar_config_, track.position_m, track.velocity_mps, measurement.direction);
            track.time_s = observation.time_s;
            track.direction = estimate.direction;
            track.range_m = estimate.range_m;
            track.radial_velocity_mps = estimate.radial_velocity_mps;
            track.doppler_hz = estimate.doppler_hz;
            track.hit_count += 1U;
            track.miss_count = 0U;
            detail::updateRecentMatches(
                track.recent_matches, true, detection_config_.tentative_confirm_window);
            if (track.status == TrackStatus::Tentative &&
                track.recent_matches.size() >= detection_config_.tentative_confirm_window &&
                detail::recentHitCount(track.recent_matches) >=
                    detection_config_.tentative_confirm_hits) {
                track.status = TrackStatus::Confirmed;
            }
            measurement_used[static_cast<std::size_t>(assigned_measurement)] = true;

            TrackReport report;
            report.id = track.id;
            report.status = track.status;
            report.matched = true;
            report.valid = true;
            report.deleted = false;
            report.hit_count = track.hit_count;
            report.miss_count = track.miss_count;
            report.measurement_index = static_cast<std::size_t>(assigned_measurement);
            report.time_s = observation.time_s;
            report.predicted_range_m = predicted.estimate.range_m;
            report.predicted_doppler_hz = predicted.estimate.doppler_hz;
            report.range_m = track.range_m;
            report.doppler_hz = track.doppler_hz;
            report.radial_velocity_mps = track.radial_velocity_mps;
            report.position_m = track.position_m;
            report.velocity_mps = track.velocity_mps;
            report.direction = track.direction;
            report.range_bin_offset = measurement.range_bin_offset;
            report.doppler_bin_offset = measurement.doppler_bin_offset;
            report.azimuth_bin = measurement.azimuth_bin;
            report.elevation_bin = measurement.elevation_bin;
            fillTrackBeamformedImage(report);
            track.history.push_back(report);
            scene.tracks.push_back(report);
            slot_reported[predicted.track_index] = true;
            continue;
        }

        track.position_m = predicted.position_m;
        track.velocity_mps = predicted.velocity_mps;
        track.time_s = observation.time_s;
        track.direction = predicted.estimate.direction;
        track.range_m = predicted.estimate.range_m;
        track.radial_velocity_mps = predicted.estimate.radial_velocity_mps;
        track.doppler_hz = predicted.estimate.doppler_hz;
        track.miss_count += 1U;
        detail::updateRecentMatches(
            track.recent_matches, false, detection_config_.tentative_confirm_window);
        const std::size_t max_misses = track.status == TrackStatus::Confirmed
                                           ? detection_config_.confirmed_max_misses
                                           : detection_config_.tentative_max_misses;
        TrackReport report;
        report.id = track.id;
        report.status = track.status;
        report.matched = false;
        report.valid = true;
        report.deleted = false;
        report.hit_count = track.hit_count;
        report.miss_count = track.miss_count;
        report.time_s = observation.time_s;
        report.predicted_range_m = predicted.estimate.range_m;
        report.predicted_doppler_hz = predicted.estimate.doppler_hz;
        report.range_m = track.range_m;
        report.doppler_hz = track.doppler_hz;
        report.radial_velocity_mps = track.radial_velocity_mps;
        report.position_m = track.position_m;
        report.velocity_mps = track.velocity_mps;
        report.direction = track.direction;
        fillTrackBeamformedImage(report);
        if (track.miss_count > max_misses) {
            report.valid = false;
            report.deleted = true;
            report.status = TrackStatus::Deleted;
            scene.tracks.push_back(report);
            slot_reported[predicted.track_index] = true;
            retireSlot(track);
            continue;
        }
        track.history.push_back(report);
        scene.tracks.push_back(report);
        slot_reported[predicted.track_index] = true;
    }

    for (std::size_t measurement_index = 0; measurement_index < observation.measurements.size();
         ++measurement_index) {
        if (measurement_used[measurement_index]) {
            continue;
        }
        const MeasurementCandidate &measurement = observation.measurements[measurement_index];
        if (!detail::isBirthCandidateValid(measurement, detection_config_)) {
            continue;
        }
        if (detail::shouldSuppressBirth(
                measurement, scene.tracks, detection_config_, radar_config_) ||
            detail::shouldSuppressBirthFromGhost(measurement,
                                                 deleted_track_ghosts_,
                                                 observation.time_s,
                                                 detection_config_,
                                                 radar_config_)) {
            continue;
        }
        auto empty_it =
            std::find_if(active_tracks_.begin(),
                         active_tracks_.end(),
                         [](const detail::TrackState &track) { return !track.initialized; });
        if (empty_it == active_tracks_.end()) {
            break;
        }
        const std::size_t track_index =
            static_cast<std::size_t>(std::distance(active_tracks_.begin(), empty_it));
        detail::TrackState &track = *empty_it;
        initializeSlotFromMeasurement(track, measurement, observation.time_s);

        TrackReport report;
        report.id = track.id;
        report.status = track.status;
        report.matched = true;
        report.valid = true;
        report.deleted = false;
        report.hit_count = track.hit_count;
        report.miss_count = track.miss_count;
        report.measurement_index = measurement_index;
        report.time_s = observation.time_s;
        report.predicted_range_m = measurement.range_m;
        report.predicted_doppler_hz = measurement.doppler_hz;
        report.range_m = track.range_m;
        report.doppler_hz = track.doppler_hz;
        report.radial_velocity_mps = track.radial_velocity_mps;
        report.position_m = track.position_m;
        report.velocity_mps = track.velocity_mps;
        report.direction = track.direction;
        report.range_bin_offset = measurement.range_bin_offset;
        report.doppler_bin_offset = measurement.doppler_bin_offset;
        report.azimuth_bin = measurement.azimuth_bin;
        report.elevation_bin = measurement.elevation_bin;
        fillTrackBeamformedImage(report);
        track.history.push_back(report);
        scene.tracks.push_back(report);
        slot_reported[track_index] = true;
    }

    for (std::size_t track_index = 0; track_index < active_tracks_.size(); ++track_index) {
        if (slot_reported[track_index]) {
            continue;
        }
        const detail::TrackState &track = active_tracks_[track_index];
        if (!track.initialized) {
            continue;
        }
        TrackReport report;
        report.id = track.id;
        report.status = track.status;
        report.matched = false;
        report.valid = true;
        report.deleted = false;
        report.hit_count = track.hit_count;
        report.miss_count = track.miss_count;
        report.time_s = observation.time_s;
        report.range_m = track.range_m;
        report.doppler_hz = track.doppler_hz;
        report.radial_velocity_mps = track.radial_velocity_mps;
        report.position_m = track.position_m;
        report.velocity_mps = track.velocity_mps;
        report.direction = track.direction;
        fillTrackBeamformedImage(report);
        scene.tracks.push_back(report);
    }

    std::sort(active_tracks_.begin(),
              active_tracks_.end(),
              [](const detail::TrackState &lhs, const detail::TrackState &rhs) {
                  if (lhs.initialized != rhs.initialized) {
                      return lhs.initialized > rhs.initialized;
                  }
                  return lhs.id < rhs.id;
              });
    std::sort(scene.tracks.begin(),
              scene.tracks.end(),
              [](const TrackReport &lhs, const TrackReport &rhs) {
                  if (lhs.valid != rhs.valid) {
                      return lhs.valid > rhs.valid;
                  }
                  if (lhs.status != rhs.status) {
                      return lhs.status > rhs.status;
                  }
                  return lhs.id < rhs.id;
              });

    scene_batch_results_.push_back(scene);
}

} // namespace fmcw_tracker
