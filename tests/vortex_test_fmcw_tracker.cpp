#include "test_harness.h"

#include "fmcw_tracker.h"
#include "problem_description.h"
#include "radar_algo.h"

#include <cmath>
#include <iostream>


namespace {

float positionErrorNorm(const problem::Vec3 &estimated, const problem::Vec3 &truth) {
    return (estimated - truth).norm();
}

} // namespace

TEST(single_target_raw_position_tracks_truth) {
    const problem::ProblemDescription description =
        radar_algo::makeSingleTargetTrackingDescription(problem::kDefaultProblemDescription);

    fmcw_tracker::StreamingTracker tracker(description);
    radar_algo::streamRadarChirps(
        description,
        [&tracker](std::size_t chirp_index,
                   std::span<const radar_algo::Complex> tx_chirp,
                   std::span<const radar_algo::Complex> rx_block) {
            tracker.pushChirp(chirp_index, tx_chirp, rx_block);
        });

    const fmcw_tracker::TrackSummary summary = tracker.buildSummary();
    const std::size_t chirp_count = radar_algo::computeChirpCount(description);
    const std::size_t cpi = tracker.detectionConfig().coherent_processing_interval_chirps;
    const std::size_t hop = tracker.detectionConfig().hop_chirps;
    const std::size_t expected_batches =
        chirp_count >= cpi ? ((chirp_count - cpi) / hop) + 1U : 0U;

    ASSERT_EQ(summary.batch_results.size(), expected_batches);
    ASSERT_EQ(summary.raw_positions_m.size(), summary.truth_metrics.size());

    float squared_error_sum = 0.0f;
    float max_error_m = 0.0f;
    std::size_t valid_batch_count = 0;
    for (std::size_t i = 0; i < summary.raw_positions_m.size(); ++i) {
        const auto &batch = summary.batch_results[i];
        if (batch.valid) {
            ++valid_batch_count;
        } else {
            ASSERT_TRUE(batch.time_s > 0.0f);
            ASSERT_TRUE(batch.range_m > 0.0f);
            ASSERT_TRUE(std::isfinite(batch.doppler_hz));
        }
        const float error_m =
            positionErrorNorm(summary.raw_positions_m[i], summary.truth_metrics[i].position_m);
        squared_error_sum += error_m * error_m;
        max_error_m = std::max(max_error_m, error_m);
    }

    const float rmse_m =
        std::sqrt(squared_error_sum / static_cast<float>(summary.raw_positions_m.size()));

    std::cout << "RMSE: " << rmse_m << " m, max error: " << max_error_m << " m" << std::endl;
    ASSERT_TRUE(valid_batch_count > 0);
    ASSERT_TRUE(rmse_m < 8.0f);
    ASSERT_TRUE(max_error_m < 10.0f);
}

int main() {
    RUN_TEST(single_target_raw_position_tracks_truth);
    PRINT_RESULTS();
    return g_fails > 0 ? 1 : 0;
}
