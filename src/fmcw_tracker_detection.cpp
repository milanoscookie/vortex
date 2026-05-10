#include "fmcw_tracker.h"

#include "dynamics.h"
#include "target_observation.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace fmcw_tracker {
namespace detail {

namespace {

bool isZeroDopplerExcluded(std::size_t dbin,
                           std::size_t nfft_doppler,
                           std::size_t zero_doppler_guard_bins) {
    if (nfft_doppler == 0U || zero_doppler_guard_bins == 0U) {
        return false;
    }
    const std::size_t zero_doppler_bin = nfft_doppler / 2U;
    const std::size_t low = zero_doppler_bin > zero_doppler_guard_bins
                                ? zero_doppler_bin - zero_doppler_guard_bins
                                : 0U;
    const std::size_t high =
        std::min(nfft_doppler - 1U, zero_doppler_bin + zero_doppler_guard_bins);
    return dbin >= low && dbin <= high;
}

bool isLocalMaximum(const std::vector<Real> &rd_power,
                    std::size_t range_count,
                    std::size_t nfft_doppler,
                    std::size_t dbin,
                    std::size_t rbin) {
    const Real center = rd_power[dbin * range_count + rbin];
    const std::size_t d0 = dbin > 0U ? dbin - 1U : dbin;
    const std::size_t d1 = std::min(nfft_doppler - 1U, dbin + 1U);
    const std::size_t r0 = rbin > 0U ? rbin - 1U : rbin;
    const std::size_t r1 = std::min(range_count - 1U, rbin + 1U);
    for (std::size_t d = d0; d <= d1; ++d) {
        for (std::size_t r = r0; r <= r1; ++r) {
            if (d == dbin && r == rbin) {
                continue;
            }
            if (rd_power[d * range_count + r] >= center) {
                return false;
            }
        }
    }
    return true;
}

bool caCfarDecision(const std::vector<Real> &rd_power,
                    std::size_t range_count,
                    std::size_t nfft_doppler,
                    std::size_t dbin,
                    std::size_t rbin,
                    const DetectionConfig &config,
                    Real *noise_out,
                    Real *snr_linear_out) {
    if (range_count == 0U || nfft_doppler == 0U) {
        return false;
    }

    const std::size_t outer_d = config.cfar_guard_doppler_bins + config.cfar_training_doppler_bins;
    const std::size_t outer_r = config.cfar_guard_range_bins + config.cfar_training_range_bins;
    const std::size_t d_start = dbin > outer_d ? dbin - outer_d : 0U;
    const std::size_t d_stop = std::min(nfft_doppler - 1U, dbin + outer_d);
    const std::size_t r_start = rbin > outer_r ? rbin - outer_r : 0U;
    const std::size_t r_stop = std::min(range_count - 1U, rbin + outer_r);
    const std::size_t guard_d_start =
        dbin > config.cfar_guard_doppler_bins ? dbin - config.cfar_guard_doppler_bins : 0U;
    const std::size_t guard_d_stop =
        std::min(nfft_doppler - 1U, dbin + config.cfar_guard_doppler_bins);
    const std::size_t guard_r_start =
        rbin > config.cfar_guard_range_bins ? rbin - config.cfar_guard_range_bins : 0U;
    const std::size_t guard_r_stop =
        std::min(range_count - 1U, rbin + config.cfar_guard_range_bins);

    double noise_sum = 0.0;
    std::size_t noise_count = 0U;
    for (std::size_t d = d_start; d <= d_stop; ++d) {
        if (isZeroDopplerExcluded(d, nfft_doppler, config.zero_doppler_guard_bins)) {
            continue;
        }
        for (std::size_t r = r_start; r <= r_stop; ++r) {
            const bool in_guard_d = d >= guard_d_start && d <= guard_d_stop;
            const bool in_guard_r = r >= guard_r_start && r <= guard_r_stop;
            if (in_guard_d && in_guard_r) {
                continue;
            }
            noise_sum += rd_power[d * range_count + r];
            ++noise_count;
        }
    }

    if (noise_count == 0U) {
        return false;
    }

    const Real noise_power = static_cast<Real>(noise_sum / static_cast<double>(noise_count));
    const Real cell_power = rd_power[dbin * range_count + rbin];
    const Real threshold_power = std::max(0.0f, config.rd_detection_threshold_scale) * noise_power;
    const Real snr_linear = cell_power / std::max(noise_power, 1.0e-12f);
    const Real min_snr_linear = std::pow(10.0f, config.cfar_min_snr_db / 10.0f);
    if (noise_out != nullptr) {
        *noise_out = noise_power;
    }
    if (snr_linear_out != nullptr) {
        *snr_linear_out = snr_linear;
    }
    return cell_power > threshold_power && snr_linear >= min_snr_linear;
}

bool shouldMergeMeasurements(const MeasurementCandidate &lhs,
                             const MeasurementCandidate &rhs,
                             const DetectionConfig &config) {
    const Real delta_range = std::abs(lhs.range_m - rhs.range_m);
    const Real delta_doppler = std::abs(lhs.doppler_hz - rhs.doppler_hz);
    const Real delta_azimuth =
        std::abs(angleDifferenceDeg(measurementAzimuthDeg(lhs), measurementAzimuthDeg(rhs)));
    const Real delta_elevation =
        std::abs(measurementElevationDeg(lhs) - measurementElevationDeg(rhs));
    return delta_range <= config.measurement_merge_range_m &&
           delta_doppler <= config.measurement_merge_doppler_hz &&
           delta_azimuth <= config.measurement_merge_azimuth_deg &&
           delta_elevation <= config.measurement_merge_elevation_deg;
}

} // namespace

std::vector<DetectionCell> detectLocalPeakCells(const std::vector<Real> &rd_power,
                                               std::size_t range_count,
                                               std::size_t nfft_doppler,
                                               const DetectionConfig &config) {
    std::vector<DetectionCell> detections;
    for (std::size_t dbin = 0; dbin < nfft_doppler; ++dbin) {
        if (isZeroDopplerExcluded(dbin, nfft_doppler, config.zero_doppler_guard_bins)) {
            continue;
        }
        for (std::size_t rbin = 0; rbin < range_count; ++rbin) {
            Real noise_power = 0.0f;
            Real snr_linear = 0.0f;
            if (!caCfarDecision(rd_power,
                                range_count,
                                nfft_doppler,
                                dbin,
                                rbin,
                                config,
                                &noise_power,
                                &snr_linear)) {
                continue;
            }
            const Real power = rd_power[dbin * range_count + rbin];
            if (!isLocalMaximum(rd_power, range_count, nfft_doppler, dbin, rbin)) {
                continue;
            }
            detections.push_back(DetectionCell{dbin, rbin, power});
        }
    }
    std::sort(
        detections.begin(),
        detections.end(),
        [](const DetectionCell &lhs, const DetectionCell &rhs) { return lhs.power > rhs.power; });
    if (detections.size() > config.max_detection_cells) {
        detections.resize(config.max_detection_cells);
    }
    return detections;
}

std::vector<std::vector<DetectionCell>>
clusterNearbyPeaks(const std::vector<DetectionCell> &detections, const DetectionConfig &config) {
    std::vector<std::vector<DetectionCell>> clusters;
    if (detections.empty()) {
        return clusters;
    }
    std::vector<bool> visited(detections.size(), false);
    for (std::size_t i = 0; i < detections.size(); ++i) {
        if (visited[i]) {
            continue;
        }
        std::vector<std::size_t> queue{i};
        visited[i] = true;
        std::vector<DetectionCell> cluster;
        while (!queue.empty()) {
            const std::size_t current = queue.back();
            queue.pop_back();
            cluster.push_back(detections[current]);
            for (std::size_t j = 0; j < detections.size(); ++j) {
                if (visited[j]) {
                    continue;
                }
                const std::size_t range_diff =
                    detections[current].range_bin > detections[j].range_bin
                        ? detections[current].range_bin - detections[j].range_bin
                        : detections[j].range_bin - detections[current].range_bin;
                const std::size_t doppler_diff =
                    detections[current].doppler_bin > detections[j].doppler_bin
                        ? detections[current].doppler_bin - detections[j].doppler_bin
                        : detections[j].doppler_bin - detections[current].doppler_bin;
                if (range_diff <= config.cluster_range_gate_bins &&
                    doppler_diff <= config.cluster_doppler_gate_bins) {
                    visited[j] = true;
                    queue.push_back(j);
                }
            }
        }
        clusters.push_back(std::move(cluster));
    }
    std::sort(clusters.begin(), clusters.end(), [](const auto &lhs, const auto &rhs) {
        const auto lhs_peak = std::max_element(
            lhs.begin(), lhs.end(), [](const DetectionCell &a, const DetectionCell &b) {
                return a.power < b.power;
            });
        const auto rhs_peak = std::max_element(
            rhs.begin(), rhs.end(), [](const DetectionCell &a, const DetectionCell &b) {
                return a.power < b.power;
            });
        return lhs_peak->power > rhs_peak->power;
    });
    return clusters;
}

Real computeAssociationDistanceSq(const RadarConfig &radar_config,
                                  Real delta_range_m,
                                  Real delta_radial_velocity_mps,
                                  Real delta_azimuth_deg,
                                  Real delta_elevation_deg,
                                  const DetectionConfig &config) {
    const Real sigma_range = std::max(config.range_association_sigma_m, 1.0e-3f);
    const Real sigma_velocity =
        std::max(0.5f * config.doppler_association_sigma_hz * radar_config.wavelengthM(), 1.0e-3f);
    const Real sigma_azimuth = std::max(config.azimuth_association_sigma_deg, 1.0e-3f);
    const Real sigma_elevation = std::max(config.elevation_association_sigma_deg, 1.0e-3f);
    return std::pow(delta_range_m / sigma_range, 2.0f) +
           std::pow(delta_radial_velocity_mps / sigma_velocity, 2.0f) +
           std::pow(delta_azimuth_deg / sigma_azimuth, 2.0f) +
           std::pow(delta_elevation_deg / sigma_elevation, 2.0f);
}

bool shouldSuppressBirth(const MeasurementCandidate &measurement,
                         const std::vector<TrackReport> &scene_tracks,
                         const DetectionConfig &config,
                         const RadarConfig &radar_config) {
    for (const TrackReport &track : scene_tracks) {
        if (!track.valid || track.deleted) {
            continue;
        }
        const Real delta_range = measurement.range_m - track.range_m;
        const Real delta_radial_velocity =
            measurement.radial_velocity_mps - track.radial_velocity_mps;
        const Real delta_azimuth =
            angleDifferenceDeg(measurementAzimuthDeg(measurement), azimuthDeg(track.direction));
        const Real delta_elevation =
            angleDifferenceDeg(measurementElevationDeg(measurement), elevationDeg(track.direction));
        const Real gate = track.status == TrackStatus::Confirmed
                              ? config.track_birth_distance_gate
                              : 0.5f * config.track_birth_distance_gate;
        if (computeAssociationDistanceSq(radar_config,
                                         delta_range,
                                         delta_radial_velocity,
                                         delta_azimuth,
                                         delta_elevation,
                                         config) < gate) {
            return true;
        }
    }
    return false;
}

bool shouldSuppressBirthFromGhost(const MeasurementCandidate &measurement,
                                  const std::vector<DeletedTrackGhost> &ghosts,
                                  Real time_s,
                                  const DetectionConfig &config,
                                  const RadarConfig &radar_config) {
    for (const DeletedTrackGhost &ghost : ghosts) {
        if (time_s - ghost.time_s > config.deleted_track_retention_s) {
            continue;
        }
        const Real delta_range = measurement.range_m - ghost.range_m;
        const Real delta_radial_velocity =
            measurement.radial_velocity_mps - ghost.radial_velocity_mps;
        const Real delta_azimuth =
            angleDifferenceDeg(measurementAzimuthDeg(measurement), azimuthDeg(ghost.direction));
        const Real delta_elevation =
            angleDifferenceDeg(measurementElevationDeg(measurement), elevationDeg(ghost.direction));
        if (computeAssociationDistanceSq(radar_config,
                                         delta_range,
                                         delta_radial_velocity,
                                         delta_azimuth,
                                         delta_elevation,
                                         config) < config.deleted_track_birth_distance_gate) {
            return true;
        }
    }
    return false;
}

bool isBirthCandidateValid(const MeasurementCandidate &measurement, const DetectionConfig &config) {
    return measurement.cluster_size >= config.birth_min_cluster_size &&
           measurement.peak_power >= config.birth_min_peak_power &&
           measurement.integrated_power >= config.birth_min_integrated_power;
}

std::vector<MeasurementCandidate>
mergeNearbyMeasurements(const std::vector<MeasurementCandidate> &measurements,
                        const DetectionConfig &config) {
    std::vector<MeasurementCandidate> merged;
    merged.reserve(measurements.size());
    for (const MeasurementCandidate &candidate : measurements) {
        bool absorbed = false;
        for (MeasurementCandidate &existing : merged) {
            if (!shouldMergeMeasurements(existing, candidate, config)) {
                continue;
            }
            if (candidate.peak_power > existing.peak_power) {
                existing = candidate;
            } else {
                existing.integrated_power += candidate.integrated_power;
                existing.peak_power = std::max(existing.peak_power, candidate.peak_power);
                existing.cluster_size += candidate.cluster_size;
            }
            absorbed = true;
            break;
        }
        if (!absorbed) {
            merged.push_back(candidate);
        }
    }
    std::sort(merged.begin(),
              merged.end(),
              [](const MeasurementCandidate &lhs, const MeasurementCandidate &rhs) {
                  if (lhs.integrated_power == rhs.integrated_power) {
                      return lhs.peak_power > rhs.peak_power;
                  }
                  return lhs.integrated_power > rhs.integrated_power;
              });
    return merged;
}

std::vector<MeasurementCandidate>
selectTopMeasurements(std::vector<MeasurementCandidate> measurements, std::size_t max_count) {
    if (max_count == 0U || measurements.size() <= max_count) {
        return measurements;
    }
    measurements.resize(max_count);
    return measurements;
}

void updateRecentMatches(std::vector<std::uint8_t> &recent_matches,
                         bool matched,
                         std::size_t max_window) {
    if (max_window == 0U) {
        recent_matches.clear();
        return;
    }
    recent_matches.push_back(matched ? 1U : 0U);
    if (recent_matches.size() > max_window) {
        recent_matches.erase(recent_matches.begin());
    }
}

std::size_t recentHitCount(const std::vector<std::uint8_t> &recent_matches) {
    return static_cast<std::size_t>(std::count(recent_matches.begin(), recent_matches.end(), 1U));
}

AssignmentSolution solveAssignments(const std::vector<std::vector<Real>> &cost_matrix,
                                    Real miss_cost) {
    const std::size_t track_count = cost_matrix.size();
    const std::size_t measurement_count = track_count == 0U ? 0U : cost_matrix.front().size();
    AssignmentSolution solution;
    solution.measurement_for_track.assign(track_count, -1);
    solution.track_for_measurement.assign(measurement_count, -1);
    if (track_count == 0U || measurement_count == 0U) {
        return solution;
    }
    if (measurement_count <= 20U) {
        const std::size_t state_count = 1U << measurement_count;
        std::vector<Real> memo((track_count + 1U) * state_count, -1.0f);
        std::vector<int> choice((track_count + 1U) * state_count, -2);
        const auto solve =
            [&](const auto &self, std::size_t track_index, std::size_t used_mask) -> Real {
            const std::size_t flat = track_index * state_count + used_mask;
            if (track_index == track_count) {
                return 0.0f;
            }
            if (memo[flat] >= 0.0f) {
                return memo[flat];
            }
            Real best_cost = miss_cost + self(self, track_index + 1U, used_mask);
            int best_choice = -1;
            for (std::size_t measurement_index = 0; measurement_index < measurement_count;
                 ++measurement_index) {
                if ((used_mask & (1U << measurement_index)) != 0U) {
                    continue;
                }
                const Real pair_cost = cost_matrix[track_index][measurement_index];
                if (!std::isfinite(pair_cost)) {
                    continue;
                }
                const Real total_cost =
                    pair_cost + self(self, track_index + 1U, used_mask | (1U << measurement_index));
                if (total_cost < best_cost) {
                    best_cost = total_cost;
                    best_choice = static_cast<int>(measurement_index);
                }
            }
            memo[flat] = best_cost;
            choice[flat] = best_choice;
            return best_cost;
        };
        solve(solve, 0U, 0U);
        std::size_t used_mask = 0U;
        for (std::size_t track_index = 0; track_index < track_count; ++track_index) {
            const std::size_t flat = track_index * state_count + used_mask;
            const int selected = choice[flat];
            if (selected >= 0) {
                solution.measurement_for_track[track_index] = selected;
                solution.track_for_measurement[static_cast<std::size_t>(selected)] =
                    static_cast<int>(track_index);
                used_mask |= 1U << static_cast<std::size_t>(selected);
            }
        }
        return solution;
    }

    struct Pair {
        std::size_t track_index = 0;
        std::size_t measurement_index = 0;
        Real cost = 0.0f;
    };
    std::vector<Pair> feasible_pairs;
    for (std::size_t track_index = 0; track_index < track_count; ++track_index) {
        for (std::size_t measurement_index = 0; measurement_index < measurement_count;
             ++measurement_index) {
            const Real cost = cost_matrix[track_index][measurement_index];
            if (std::isfinite(cost) && cost < miss_cost) {
                feasible_pairs.push_back(Pair{track_index, measurement_index, cost});
            }
        }
    }
    std::sort(feasible_pairs.begin(), feasible_pairs.end(), [](const Pair &lhs, const Pair &rhs) {
        return lhs.cost < rhs.cost;
    });
    for (const Pair &pair : feasible_pairs) {
        if (solution.measurement_for_track[pair.track_index] >= 0 ||
            solution.track_for_measurement[pair.measurement_index] >= 0) {
            continue;
        }
        solution.measurement_for_track[pair.track_index] = static_cast<int>(pair.measurement_index);
        solution.track_for_measurement[pair.measurement_index] = static_cast<int>(pair.track_index);
    }
    return solution;
}

std::size_t nearestTruthCarIndex(const problem::ProblemDescription &description,
                                 Real time_s,
                                 const Vec3 &position_m) {
    if (description.cars.empty()) {
        return static_cast<std::size_t>(-1);
    }
    std::size_t best_index = 0U;
    Real best_error = std::numeric_limits<Real>::max();
    for (std::size_t car_index = 0; car_index < description.cars.size(); ++car_index) {
        const CarDynamics dynamics(description.cars[car_index]);
        const radar::TargetObservation observation =
            radar::observeTarget(dynamics, description.radar, time_s);
        const Real error = (observation.position_m - position_m).norm();
        if (error < best_error) {
            best_error = error;
            best_index = car_index;
        }
    }
    return best_index;
}

} // namespace detail
} // namespace fmcw_tracker
