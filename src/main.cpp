#include "fmcw_tracker.h"
#include "problem_description.h"
#include "radar_algo.h"

#include <cstdlib>
#include <cstring>
#include <iomanip>
#include <iostream>
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
    out << std::fixed << std::setprecision(3) << '[' << value.x() << ", " << value.y()
        << ", " << value.z() << ']';
    return out.str();
}

problem::Vec3 meanVec3(const std::vector<problem::Vec3> &values) {
    if (values.empty()) {
        return problem::Vec3::Zero();
    }

    problem::Vec3 mean = problem::Vec3::Zero();
    for (const problem::Vec3 &value : values) {
        mean += value;
    }
    return mean / static_cast<float>(values.size());
}

} // namespace

int main(int argc, char **argv) {
    try {
        const RuntimeOptions runtime_options = parseRuntimeOptions(argc, argv);

        const problem::ProblemDescription description =
            radar_algo::makeSingleTargetTrackingDescription(
                problem::kDefaultProblemDescription);

        fmcw_tracker::StreamingTracker tracker(description);
        radar_algo::streamRadarChirps(
            description,
            [&tracker](std::size_t chirp_index,
                       std::span<const radar_algo::Complex> tx_chirp,
                       std::span<const radar_algo::Complex> rx_block) {
                tracker.pushChirp(chirp_index, tx_chirp, rx_block);
            });

        const fmcw_tracker::TrackSummary summary = tracker.buildSummary();
        if (summary.batch_results.empty()) {
            throw std::runtime_error("tracker produced no CPI results");
        }

        const auto &first_estimate = summary.batch_results.front();
        std::cout << std::fixed << std::setprecision(3);
        std::cout << "Processed " << summary.batch_results.size()
                  << " CPI batch(es) for one target.\n";
        std::cout << "First batch estimate: "
                  << "t=" << first_estimate.time_s << " s, "
                  << "range=" << first_estimate.range_m << " m, "
                  << "doppler=" << first_estimate.doppler_hz << " Hz, "
                  << "dir=" << formatVec3(first_estimate.direction) << '\n';

        if (runtime_options.print_truth_summary) {
            const auto &first_truth = summary.truth_metrics.front();
            std::cout << "First batch truth: "
                      << "range=" << first_truth.range_m << " m, "
                      << "doppler=" << first_truth.doppler_hz << " Hz, "
                      << "xyz=" << formatVec3(first_truth.position_m) << '\n';
        }

        std::cout << "Mean smoothed XYZ: "
                  << formatVec3(meanVec3(summary.smoothed_positions_m)) << '\n';
        std::cout << "Mean Cartesian velocity: "
                  << formatVec3(meanVec3(summary.cartesian_velocity_mps)) << " m/s\n";
    } catch (const std::exception &ex) {
        std::cerr << "Radar demo failed: " << ex.what() << '\n';
        return 1;
    }
    return 0;
}
