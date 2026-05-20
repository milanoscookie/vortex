#include "radar.h"
#include "student_tracker.h"

#include <cstdlib>
#include <cstring>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <string>
#include <string_view>

namespace {

using ProblemDescription = dsp::ProblemDescription;
using TrackerResult = radar::TrackerResult;

struct RuntimeOptions {
    bool print_truth = true;
    double duration_s = 1.0;
};

RuntimeOptions parseRuntimeOptions(int argc, char **argv) {
    RuntimeOptions options;
    for (int i = 1; i < argc; ++i) {
        const std::string_view arg(argv[i]);
        if (arg == "--no-truth") {
            options.print_truth = false;
            continue;
        }
        if (arg == "--duration" && i + 1 < argc) {
            options.duration_s = std::stod(argv[++i]);
            continue;
        }
        throw std::runtime_error("unknown argument: " + std::string(arg));
    }

    if (const char *disable_truth = std::getenv("VORTEX_DISABLE_TRUTH_CSV")) {
        options.print_truth = std::strcmp(disable_truth, "0") != 0;
    }
    return options;
}

ProblemDescription makeSingleTargetTrackingDescription(const ProblemDescription &base) {
    ProblemDescription description = base;
    if (description.cars.empty()) {
        throw std::runtime_error("single-target tracking requires at least one car");
    }

    description.cars = {description.cars.front()};
    description.simulator.vehicle_count = 1;
    description.simulator.random_seed = 0U;
    description.radar.receiver_noiselevel_stddev = 1e-9;
    description.radar.receiver_noiselevel_mean = 0.0;
    description.radar.receiver_noise_distribution_stddev = 1e-6;
    return description;
}

double positionError(const TrackerResult &estimate, const TrackerResult &truth) {
    const double dx = estimate.x - truth.x;
    const double dy = estimate.y - truth.y;
    const double dz = estimate.z - truth.z;
    return std::sqrt(dx * dx + dy * dy + dz * dz);
}

void printState(const char *label, const TrackerResult &result) {
    std::cout << label << " t=" << result.time_s << " s"
              << " pos=[" << result.x << ", " << result.y << ", " << result.z << "]"
              << " vel=[" << result.vx << ", " << result.vy << ", " << result.vz << "]"
              << " valid=" << (result.valid ? "true" : "false") << '\n';
}

} // namespace

int main(int argc, char **argv) {
    try {
        const RuntimeOptions runtime_options = parseRuntimeOptions(argc, argv);
        const ProblemDescription description =
            makeSingleTargetTrackingDescription(dsp::kDefaultProblemDescription);

        radar::Session session(description);
        radar::StudentTracker tracker(description);
        
        session.init();

        const double chirp_duration_s = static_cast<double>(dsp::RadarSettings::kRadarBlockSize) /
                                       description.radar.sample_rate_hz;
        const std::size_t num_chirps =
            static_cast<std::size_t>(std::ceil(runtime_options.duration_s / chirp_duration_s));

        std::cout << std::fixed << std::setprecision(3);
        std::cout << "Simulating " << num_chirps << " chirps (" << runtime_options.duration_s
                  << " s).\n";

        for (std::size_t chirp_idx = 0; chirp_idx < num_chirps; ++chirp_idx) {
            // Get data from simulator
            const radar::SignalBlock block = session.nextChirp();
            
            // Student processes the data
            tracker.processChirp(block);

            // Periodically compare student estimate against ground truth
            if (chirp_idx % 1000 == 0 || chirp_idx == num_chirps - 1) {
                const TrackerResult truth = session.truth();
                const TrackerResult estimate = tracker.getLatestEstimate();

                std::cout << "Chirp " << chirp_idx << " (t=" << block.start_time_s << "s)\n";
                if (estimate.valid) {
                    printState("  estimate", estimate);
                } else {
                    std::cout << "  estimate: (not implemented/invalid)\n";
                }

                if (runtime_options.print_truth) {
                    printState("  truth   ", truth);
                    if (estimate.valid) {
                        std::cout << "  error=" << positionError(estimate, truth) << " m\n";
                    }
                }
            }
        }
    } catch (const std::exception &ex) {
        std::cerr << "Radar demo failed: " << ex.what() << '\n';
        return 1;
    }
    return 0;
}
