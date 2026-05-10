#include "test_harness.h"

#include "fmcw_tracker.h"
#include "problem_description.h"
#include "radar_algo.h"

#include <algorithm>
#include <cmath>
#include <iostream>

namespace {

problem::ProblemDescription makeMultiCarDescription() {
    problem::ProblemDescription description = problem::kDefaultProblemDescription;
    description.simulator.vehicle_count = 3;
    description.simulator.random_seed = 0U;
    description.simulator.burst_duration_s = 0.20f;
    description.radar.receiver_noiselevel_stddev = 1e-9f;
    description.radar.receiver_noiselevel_mean = 0.0f;
    description.radar.receiver_noise_distribution_stddev = 1e-6f;
    description.cars = {
        problem::CarSettings{.initial_position_m = problem::Vec3(70.0f, -35.0f, 130.0f),
                             .base_velocity_mps = problem::Vec3(18.0f, 6.0f, 0.0f),
                             .bounce_amplitude_m = 0.001f,
                             .bounce_frequency_hz = 8.0f,
                             .reflectivity = std::polar(1.2f, 0.1f)},
        problem::CarSettings{.initial_position_m = problem::Vec3(125.0f, 28.0f, 145.0f),
                             .base_velocity_mps = problem::Vec3(14.0f, -4.0f, 0.0f),
                             .bounce_amplitude_m = 0.001f,
                             .bounce_frequency_hz = 11.0f,
                             .reflectivity = std::polar(1.0f, 1.3f)},
        problem::CarSettings{.initial_position_m = problem::Vec3(165.0f, 82.0f, 120.0f),
                             .base_velocity_mps = problem::Vec3(22.0f, 10.0f, 0.0f),
                             .bounce_amplitude_m = 0.001f,
                             .bounce_frequency_hz = 14.0f,
                             .reflectivity = std::polar(0.95f, 2.2f)},
    };
    return description;
}

fmcw_tracker::DetectionConfig makeDetectionConfig() {
    fmcw_tracker::DetectionConfig config;
    config.rd_detection_threshold_scale = 4.0f;
    config.cluster_range_gate_bins = 2U;
    config.cluster_doppler_gate_bins = 1U;
    config.tentative_confirm_hits = 3U;
    config.tentative_max_misses = 1U;
    config.confirmed_max_misses = 3U;
    config.track_max_range_error_m = 15.0f;
    config.track_max_doppler_error_hz = 1500.0f;
    config.track_max_azimuth_error_deg = 15.0f;
    config.track_max_elevation_error_deg = 25.0f;
    config.track_distance_gate = 16.0f;
    config.assignment_miss_cost = 12.0f;
    return config;
}

std::size_t countConfirmedTrackHistories(const fmcw_tracker::SceneSummary &summary) {
    std::size_t count = 0U;
    for (const fmcw_tracker::TrackHistory &history : summary.track_histories) {
        bool confirmed = history.final_status == fmcw_tracker::TrackStatus::Confirmed;
        if (!confirmed) {
            for (const fmcw_tracker::TrackReport &report : history.reports) {
                if (report.status == fmcw_tracker::TrackStatus::Confirmed) {
                    confirmed = true;
                    break;
                }
            }
        }
        if (confirmed) {
            ++count;
        }
    }
    return count;
}

double greedyFinalScenePositionError(const fmcw_tracker::SceneBatchResult &scene) {
    std::vector<problem::Vec3> track_positions;
    for (const fmcw_tracker::TrackReport &track : scene.tracks) {
        if (track.valid && track.status == fmcw_tracker::TrackStatus::Confirmed) {
            track_positions.push_back(track.position_m);
        }
    }
    if (track_positions.empty() || scene.truth_metrics.empty()) {
        return 1.0e9;
    }

    std::vector<bool> used(track_positions.size(), false);
    double error_sum = 0.0;
    std::size_t match_count = 0U;
    for (const problem::SimulationMetrics &truth : scene.truth_metrics) {
        double best_error = 1.0e9;
        std::size_t best_index = track_positions.size();
        for (std::size_t i = 0; i < track_positions.size(); ++i) {
            if (used[i]) {
                continue;
            }
            const double error = (track_positions[i] - truth.position_m).norm();
            if (error < best_error) {
                best_error = error;
                best_index = i;
            }
        }
        if (best_index < track_positions.size()) {
            used[best_index] = true;
            error_sum += best_error;
            ++match_count;
        }
    }
    return match_count == 0U ? 1.0e9 : error_sum / static_cast<double>(match_count);
}

} // namespace

TEST(multicar_tracker_produces_scene_batches) {
    const problem::ProblemDescription description = makeMultiCarDescription();
    const fmcw_tracker::DetectionConfig config = makeDetectionConfig();

    fmcw_tracker::StreamingTracker tracker(description, config);
    radar_algo::streamRadarChirps(description,
                                  [&tracker](std::size_t chirp_index,
                                             std::span<const radar_algo::Complex> tx_chirp,
                                             std::span<const radar_algo::Complex> rx_block) {
                                      tracker.pushChirp(chirp_index, tx_chirp, rx_block);
                                  });

    const fmcw_tracker::SceneSummary summary = tracker.buildSceneSummary();
    const std::size_t chirp_count = radar_algo::computeChirpCount(description);
    const std::size_t cpi = tracker.detectionConfig().coherent_processing_interval_chirps;
    const std::size_t hop = tracker.detectionConfig().hop_chirps;
    const std::size_t expected_batches = chirp_count >= cpi ? ((chirp_count - cpi) / hop) + 1U : 0U;

    ASSERT_EQ(summary.scene_batches.size(), expected_batches);
    ASSERT_TRUE(!summary.track_histories.empty());

    std::size_t nonempty_measurement_batches = 0U;
    for (const fmcw_tracker::SceneBatchResult &scene : summary.scene_batches) {
        if (!scene.measurements.empty()) {
            ++nonempty_measurement_batches;
        }
        ASSERT_EQ(scene.truth_metrics.size(), description.cars.size());
    }

    std::cout << "Scene batches: " << summary.scene_batches.size()
              << ", nonempty measurement batches: " << nonempty_measurement_batches
              << ", track histories: " << summary.track_histories.size() << std::endl;
    ASSERT_TRUE(nonempty_measurement_batches > summary.scene_batches.size() / 2U);
}

TEST(multicar_tracker_confirms_multiple_tracks) {
    const problem::ProblemDescription description = makeMultiCarDescription();
    const fmcw_tracker::DetectionConfig config = makeDetectionConfig();

    fmcw_tracker::StreamingTracker tracker(description, config);
    radar_algo::streamRadarChirps(description,
                                  [&tracker](std::size_t chirp_index,
                                             std::span<const radar_algo::Complex> tx_chirp,
                                             std::span<const radar_algo::Complex> rx_block) {
                                      tracker.pushChirp(chirp_index, tx_chirp, rx_block);
                                  });

    const fmcw_tracker::SceneSummary summary = tracker.buildSceneSummary();
    const std::size_t confirmed_tracks = countConfirmedTrackHistories(summary);

    std::cout << "Confirmed track histories: " << confirmed_tracks << std::endl;
    ASSERT_TRUE(confirmed_tracks >= 2U);
}

TEST(multicar_tracker_final_scene_stays_near_truth) {
    const problem::ProblemDescription description = makeMultiCarDescription();
    const fmcw_tracker::DetectionConfig config = makeDetectionConfig();

    fmcw_tracker::StreamingTracker tracker(description, config);
    radar_algo::streamRadarChirps(description,
                                  [&tracker](std::size_t chirp_index,
                                             std::span<const radar_algo::Complex> tx_chirp,
                                             std::span<const radar_algo::Complex> rx_block) {
                                      tracker.pushChirp(chirp_index, tx_chirp, rx_block);
                                  });

    const fmcw_tracker::SceneSummary summary = tracker.buildSceneSummary();
    ASSERT_TRUE(!summary.scene_batches.empty());
    const double mean_final_error_m = greedyFinalScenePositionError(summary.scene_batches.back());

    std::cout << "Mean final-scene position error: " << mean_final_error_m << " m" << std::endl;
    ASSERT_TRUE(mean_final_error_m < 35.0);
}

int main() {
    RUN_TEST(multicar_tracker_produces_scene_batches);
    RUN_TEST(multicar_tracker_confirms_multiple_tracks);
    RUN_TEST(multicar_tracker_final_scene_stays_near_truth);
    PRINT_RESULTS();
    return g_fails > 0 ? 1 : 0;
}
