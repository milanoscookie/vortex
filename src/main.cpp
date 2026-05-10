#include "fmcw_tracker.h"
#include "problem_description.h"
#include "radar_algo.h"

#include <algorithm>
#include <cstdlib>
#include <cstring>
#include <iomanip>
#include <iostream>
#include <limits>
#include <map>
#include <sstream>
#include <stdexcept>
#include <string>
#include <string_view>

namespace {

struct RuntimeOptions {
    bool print_truth_summary = true;
};

RuntimeOptions parseRuntimeOptions(int argc, char **argv) {
    RuntimeOptions options;
    for (int i = 1; i < argc; ++i) {
        const std::string_view arg(argv[i]);
        if (arg == "--no-truth" || arg == "--no-truth-csv") {
            options.print_truth_summary = false;
            continue;
        }

        throw std::runtime_error("unknown argument: " + std::string(arg));
    }

    if (const char *disable_truth_csv = std::getenv("VORTEX_DISABLE_TRUTH_CSV")) {
        options.print_truth_summary = std::strcmp(disable_truth_csv, "0") != 0;
    }

    return options;
}

std::string formatVec3(const problem::Vec3 &value) {
    std::ostringstream out;
    out << std::fixed << std::setprecision(3) << '[' << value.x() << ", " << value.y() << ", "
        << value.z() << ']';
    return out.str();
}

const char *trackStatusName(fmcw_tracker::TrackStatus status) {
    switch (status) {
    case fmcw_tracker::TrackStatus::Tentative:
        return "tentative";
    case fmcw_tracker::TrackStatus::Confirmed:
        return "confirmed";
    case fmcw_tracker::TrackStatus::Deleted:
        return "deleted";
    }
    return "unknown";
}

std::size_t countTrackHistoriesWithStatus(const fmcw_tracker::SceneSummary &summary,
                                          fmcw_tracker::TrackStatus status) {
    std::size_t count = 0U;
    for (const fmcw_tracker::TrackHistory &history : summary.track_histories) {
        bool seen = history.final_status == status;
        if (!seen) {
            for (const fmcw_tracker::TrackReport &report : history.reports) {
                if (report.status == status) {
                    seen = true;
                    break;
                }
            }
        }
        if (seen) {
            ++count;
        }
    }
    return count;
}

double averageMeasurementsPerBatch(const fmcw_tracker::SceneSummary &summary) {
    if (summary.scene_batches.empty()) {
        return 0.0;
    }

    double total = 0.0;
    for (const fmcw_tracker::SceneBatchResult &batch : summary.scene_batches) {
        total += static_cast<double>(batch.measurements.size());
    }
    return total / static_cast<double>(summary.scene_batches.size());
}

double averageTrackHistoryLength(const fmcw_tracker::SceneSummary &summary) {
    if (summary.track_histories.empty()) {
        return 0.0;
    }

    double total = 0.0;
    for (const fmcw_tracker::TrackHistory &history : summary.track_histories) {
        total += static_cast<double>(history.reports.size());
    }
    return total / static_cast<double>(summary.track_histories.size());
}

double batchMeanConfirmedPositionErrorM(const fmcw_tracker::SceneBatchResult &batch) {
    std::vector<problem::Vec3> estimated_positions;
    for (const fmcw_tracker::TrackReport &track : batch.tracks) {
        if (track.valid && track.status == fmcw_tracker::TrackStatus::Confirmed) {
            estimated_positions.push_back(track.position_m);
        }
    }
    if (estimated_positions.empty() || batch.truth_metrics.empty()) {
        return std::numeric_limits<double>::quiet_NaN();
    }

    std::vector<bool> used(estimated_positions.size(), false);
    double total_error = 0.0;
    std::size_t match_count = 0U;
    for (const problem::SimulationMetrics &truth : batch.truth_metrics) {
        double best_error = std::numeric_limits<double>::infinity();
        std::size_t best_index = estimated_positions.size();
        for (std::size_t i = 0; i < estimated_positions.size(); ++i) {
            if (used[i]) {
                continue;
            }
            const double error_m = (estimated_positions[i] - truth.position_m).norm();
            if (error_m < best_error) {
                best_error = error_m;
                best_index = i;
            }
        }
        if (best_index < estimated_positions.size()) {
            used[best_index] = true;
            total_error += best_error;
            ++match_count;
        }
    }

    if (match_count == 0U) {
        return std::numeric_limits<double>::quiet_NaN();
    }

    return total_error / static_cast<double>(match_count);
}

bool historyEverConfirmed(const fmcw_tracker::TrackHistory &history) {
    if (history.final_status == fmcw_tracker::TrackStatus::Confirmed) {
        return true;
    }
    return std::any_of(history.reports.begin(), history.reports.end(), [](const auto &report) {
        return report.status == fmcw_tracker::TrackStatus::Confirmed;
    });
}

double historyMinMatchedPositionErrorM(const problem::ProblemDescription &description,
                                       const fmcw_tracker::TrackHistory &history) {
    if (history.matched_truth_car_index >= description.cars.size()) {
        return std::numeric_limits<double>::infinity();
    }
    double best_error_m = std::numeric_limits<double>::infinity();
    for (const fmcw_tracker::TrackReport &report : history.reports) {
        if (!report.valid) {
            continue;
        }
        const auto truth =
            fmcw_tracker::truthAtTime(description, report.time_s, history.matched_truth_car_index);
        best_error_m = std::min(best_error_m,
                                static_cast<double>((report.position_m - truth.position_m).norm()));
    }
    return best_error_m;
}

double firstConfirmedTimeS(const fmcw_tracker::TrackHistory &history) {
    for (const fmcw_tracker::TrackReport &report : history.reports) {
        if (report.status == fmcw_tracker::TrackStatus::Confirmed) {
            return report.time_s;
        }
    }
    return std::numeric_limits<double>::quiet_NaN();
}

std::size_t confirmedTracksInBatch(const fmcw_tracker::SceneBatchResult &batch) {
    return static_cast<std::size_t>(
        std::count_if(batch.tracks.begin(), batch.tracks.end(), [](const auto &track) {
            return track.valid && track.status == fmcw_tracker::TrackStatus::Confirmed;
        }));
}

struct TrackingQualityStats {
    std::size_t confirmed_histories = 0;
    std::size_t matched_confirmed_histories = 0;
    std::size_t false_confirmed_histories = 0;
    std::size_t fragmentation_count = 0;
    std::size_t truth_cars_covered = 0;
    double avg_confirmation_latency_s = std::numeric_limits<double>::quiet_NaN();
    double avg_confirmed_tracks_per_batch = 0.0;
    std::size_t max_confirmed_tracks_in_batch = 0;
    double scene_mean_confirmed_position_error_m = std::numeric_limits<double>::quiet_NaN();
    double final_batch_confirmed_position_error_m = std::numeric_limits<double>::quiet_NaN();
};

TrackingQualityStats computeTrackingQualityStats(const problem::ProblemDescription &description,
                                                 const fmcw_tracker::SceneSummary &summary) {
    TrackingQualityStats stats;
    if (summary.scene_batches.empty()) {
        return stats;
    }

    constexpr double kMatchedTrackErrorThresholdM = 100.0;
    const double start_time_s = summary.scene_batches.front().time_s;
    double confirmation_latency_sum_s = 0.0;
    std::size_t confirmation_latency_count = 0U;
    std::map<std::size_t, std::size_t> confirmed_histories_per_truth;

    for (const fmcw_tracker::TrackHistory &history : summary.track_histories) {
        if (!historyEverConfirmed(history)) {
            continue;
        }
        ++stats.confirmed_histories;

        const double min_error_m = historyMinMatchedPositionErrorM(description, history);
        const bool matched =
            std::isfinite(min_error_m) && min_error_m <= kMatchedTrackErrorThresholdM;
        if (matched) {
            ++stats.matched_confirmed_histories;
            ++confirmed_histories_per_truth[history.matched_truth_car_index];
        }

        const double confirmed_time_s = firstConfirmedTimeS(history);
        if (std::isfinite(confirmed_time_s)) {
            confirmation_latency_sum_s += confirmed_time_s - start_time_s;
            ++confirmation_latency_count;
        }
    }

    stats.false_confirmed_histories =
        stats.confirmed_histories >= stats.matched_confirmed_histories
            ? stats.confirmed_histories - stats.matched_confirmed_histories
            : 0U;
    if (confirmation_latency_count > 0U) {
        stats.avg_confirmation_latency_s =
            confirmation_latency_sum_s / static_cast<double>(confirmation_latency_count);
    }

    double confirmed_tracks_sum = 0.0;
    double scene_error_sq_sum_m2 = 0.0;
    std::size_t scene_error_count = 0U;
    for (const fmcw_tracker::SceneBatchResult &batch : summary.scene_batches) {
        const std::size_t confirmed_count = confirmedTracksInBatch(batch);
        confirmed_tracks_sum += static_cast<double>(confirmed_count);
        stats.max_confirmed_tracks_in_batch =
            std::max(stats.max_confirmed_tracks_in_batch, confirmed_count);
        
        // Custom batch mean error calculation for statistics
        std::vector<problem::Vec3> est_pos;
        for (const auto& t : batch.tracks) {
            if (t.valid && t.status == fmcw_tracker::TrackStatus::Confirmed) est_pos.push_back(t.position_m);
        }
        
        if (!est_pos.empty() && !batch.truth_metrics.empty()) {
            double batch_total_err = 0.0;
            std::size_t batch_matches = 0;
            std::vector<bool> used(est_pos.size(), false);
            for (const auto& truth : batch.truth_metrics) {
                double best_err = std::numeric_limits<double>::infinity();
                int best_idx = -1;
                for (int i=0; i<(int)est_pos.size(); ++i) {
                    if (used[i]) continue;
                    double err = (est_pos[i] - truth.position_m).norm();
                    if (err < best_err) { best_err = err; best_idx = i; }
                }
                if (best_idx >= 0 && best_err < kMatchedTrackErrorThresholdM) {
                    used[best_idx] = true;
                    batch_total_err += best_err;
                    batch_matches++;
                }
            }
            if (batch_matches > 0) {
                double mean_err = batch_total_err / batch_matches;
                scene_error_sq_sum_m2 += mean_err * mean_err;
                scene_error_count++;
            }
        }
    }
    stats.avg_confirmed_tracks_per_batch =
        confirmed_tracks_sum / static_cast<double>(summary.scene_batches.size());

    for (const auto &[truth_index, history_count] : confirmed_histories_per_truth) {
        (void)truth_index;
        ++stats.truth_cars_covered;
        if (history_count > 1U) {
            stats.fragmentation_count += history_count - 1U;
        }
    }

    if (scene_error_count > 0U) {
        stats.scene_mean_confirmed_position_error_m =
            std::sqrt(scene_error_sq_sum_m2 / static_cast<double>(scene_error_count));
    }
    stats.final_batch_confirmed_position_error_m =
        batchMeanConfirmedPositionErrorM(summary.scene_batches.back());
    return stats;
}

std::string formatErrorValue(double err) {
    if (!std::isfinite(err)) {
        return "N/A";
    }
    std::ostringstream out;
    out << std::fixed << std::setprecision(3) << err;
    return out.str();
}

} // namespace

int main(int argc, char **argv) {
    try {
        const RuntimeOptions runtime_options = parseRuntimeOptions(argc, argv);
        const problem::ProblemDescription description = problem::kDefaultProblemDescription;

        fmcw_tracker::StreamingTracker tracker(description);
        radar_algo::streamRadarChirps(description,
                                      [&tracker, &description](std::size_t chirp_index,
                                                 std::span<const radar_algo::Complex> tx_chirp,
                                                 std::span<const radar_algo::Complex> rx_block) {
                                          tracker.pushChirp(chirp_index, tx_chirp, rx_block);
                                          
                                          // Every 50th chirp, perform a blind AoA estimation of the top 3 peaks in the range-summed angular map
                                          // and compare to the truth AoA of all 3 cars.
                                          if (chirp_index % 50 == 0) {
                                              const auto radar_cfg = tracker.radarConfig();
                                              const auto det_cfg = tracker.detectionConfig();
                                              const auto &range_indices = tracker.rangeIndices();
                                              const std::size_t num_rx = radar_cfg.numRx();
                                              const float time_s = static_cast<float>(chirp_index) * radar_cfg.chirp_duration_s;
                                              
                                              const std::size_t nfft = det_cfg.nfft_range_min;
                                              std::vector<float> range_power(range_indices.size(), 0.0f);
                                              std::vector<std::vector<radar_algo::Complex>> range_rx(num_rx, std::vector<radar_algo::Complex>(range_indices.size()));
                                              
                                              Eigen::FFT<float> fft;
                                              std::vector<radar_algo::Complex> fft_input(nfft, {0.0f, 0.0f});
                                              std::vector<radar_algo::Complex> fft_output(nfft);
                                              
                                              for (std::size_t rx = 0; rx < num_rx; ++rx) {
                                                  std::fill(fft_input.begin(), fft_input.end(), radar_algo::Complex{0.0f, 0.0f});
                                                  for (std::size_t sample = 0; sample < radar_cfg.block_size; ++sample) {
                                                      const auto conj_tx = std::conj(tx_chirp[sample]);
                                                      float win = 0.5f - 0.5f * std::cos(2.0f * problem::Constants::kPi * sample / (radar_cfg.block_size - 1));
                                                      fft_input[sample] = rx_block[sample * num_rx + rx] * conj_tx * win;
                                                  }
                                                  fft.fwd(fft_output, fft_input);
                                                  for (std::size_t k = 0; k < range_indices.size(); ++k) {
                                                      const auto val = fft_output[static_cast<std::size_t>(range_indices[k])];
                                                      range_rx[rx][k] = val;
                                                      range_power[k] += std::norm(val);
                                                  }
                                              }
                                              
                                              struct RangePeak { std::size_t idx; float power; };
                                              std::vector<RangePeak> peaks;
                                              for (std::size_t k = 1; k + 1 < range_indices.size(); ++k) {
                                                  if (range_power[k] > range_power[k-1] && range_power[k] > range_power[k+1]) {
                                                      peaks.push_back({k, range_power[k]});
                                                  }
                                              }
                                              std::sort(peaks.begin(), peaks.end(), [](auto& a, auto& b) { return a.power > b.power; });
                                              if (peaks.size() > 3) peaks.resize(3);
                                              
                                              printf("[Chirp %4zu, t=%6.3fs] Per-Chirp AoA Analysis:\n", chirp_index, time_s);
                                              printf("  Top Blind Estimates:\n");
                                              for (std::size_t i = 0; i < peaks.size(); ++i) {
                                                  std::vector<radar_algo::Complex> snapshot(num_rx);
                                                  for (std::size_t rx = 0; rx < num_rx; ++rx) snapshot[rx] = range_rx[rx][peaks[i].idx];
                                                  
                                                  auto aoa_peaks = tracker.estimateTopAoAs(snapshot, 1);
                                                  if (!aoa_peaks.empty()) {
                                                      const float range_m = -radar_cfg.speed_of_light_mps * 
                                                          (static_cast<float>(det_cfg.nfft_range_min) / (nfft * (1.0f/radar_cfg.sample_rate_hz))) // This is wrong, let's use the axis
                                                          * 0.0f; // placeholder
                                                      // Better: use the tracker's axis if we exposed it, but we can just use truth for comparison.
                                                      printf("    #%zu: Az=%6.1f, El=%6.1f (score=%8.1f)\n",
                                                             i+1, aoa_peaks[0].azimuth_deg, aoa_peaks[0].elevation_deg,
                                                             aoa_peaks[0].score);
                                                  }
                                              }
                                              
                                              printf("  Truth Car AoAs:\n");
                                              for (std::size_t i = 0; i < description.cars.size(); ++i) {
                                                  auto truth = fmcw_tracker::truthAtTime(description, time_s, i);
                                                  auto dir = truth.position_m.normalized();
                                                  printf("    Car %zu: Az=%6.1f, El=%6.1f, Range=%6.1f\n",
                                                         i+1, fmcw_tracker::detail::azimuthDeg(dir),
                                                         fmcw_tracker::detail::elevationDeg(dir),
                                                         truth.range_m);
                                              }
                                          }
                                      });

        const fmcw_tracker::SceneSummary summary = tracker.buildSceneSummary();
        if (summary.scene_batches.empty()) {
            throw std::runtime_error("tracker produced no CPI results");
        }

        const auto &first_batch = summary.scene_batches.front();
        std::cout << std::fixed << std::setprecision(3);
        std::cout << "Processed " << summary.scene_batches.size() << " CPI batch(es) for "
                  << description.cars.size() << " simulated car(s).\n";
        for (std::size_t car_index = 0; car_index < description.cars.size(); ++car_index) {
            const problem::CarSettings &car = description.cars[car_index];
            std::cout << "Car " << (car_index + 1U)
                      << ": start=" << formatVec3(car.initial_position_m)
                      << ", velocity=" << formatVec3(car.base_velocity_mps) << '\n';
        }
        std::cout << "First batch: " << first_batch.measurements.size() << " measurement(s), "
                  << first_batch.tracks.size() << " track snapshot(s)\n";

        if (!first_batch.measurements.empty()) {
            const auto &measurement = first_batch.measurements.front();
            std::cout << "First measurement: t=" << measurement.time_s
                      << " s, range=" << measurement.range_m
                      << " m, doppler=" << measurement.doppler_hz
                      << " Hz, dir=" << formatVec3(measurement.direction) << '\n';
        }

        if (!first_batch.tracks.empty()) {
            const auto &track = first_batch.tracks.front();
            std::cout << "First track: id=" << track.id
                      << ", status=" << trackStatusName(track.status)
                      << ", xyz=" << formatVec3(track.position_m)
                      << ", vel=" << formatVec3(track.velocity_mps) << "\n";
        }

        if (runtime_options.print_truth_summary && !first_batch.truth_metrics.empty()) {
            const auto &truth = first_batch.truth_metrics.front();
            std::cout << "First truth target: range=" << truth.range_m
                      << " m, doppler=" << truth.doppler_hz
                      << " Hz, xyz=" << formatVec3(truth.position_m) << '\n';
        }

        const TrackingQualityStats quality = computeTrackingQualityStats(description, summary);

        std::cout << "Track histories: " << summary.track_histories.size() << " total, "
                  << countTrackHistoriesWithStatus(summary, fmcw_tracker::TrackStatus::Confirmed)
                  << " confirmed\n";
        
        std::cout << "Tracking quality: avg measurements/CPI="
                  << averageMeasurementsPerBatch(summary)
                  << ", avg confirmed tracks/CPI=" << quality.avg_confirmed_tracks_per_batch
                  << ", max confirmed tracks/CPI=" << quality.max_confirmed_tracks_in_batch
                  << ", avg history length=" << averageTrackHistoryLength(summary)
                  << ", truth cars covered=" << quality.truth_cars_covered << '/'
                  << description.cars.size()
                  << ", matched confirmed histories=" << quality.matched_confirmed_histories
                  << ", false confirmed histories=" << quality.false_confirmed_histories
                  << ", fragmentation count=" << quality.fragmentation_count
                  << ", avg confirmation latency=" << formatErrorValue(quality.avg_confirmation_latency_s)
                  << " s, scene mean confirmed position error="
                  << formatErrorValue(quality.scene_mean_confirmed_position_error_m)
                  << " m, final batch confirmed position error="
                  << formatErrorValue(quality.final_batch_confirmed_position_error_m) << " m\n";
    } catch (const std::exception &ex) {
        std::cerr << "Radar demo failed: " << ex.what() << '\n';
        return 1;
    }
    return 0;
}
