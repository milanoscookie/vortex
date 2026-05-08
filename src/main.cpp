#include "fmcw_tracker.h"
#include "problem_description.h"
#include "radar_algo.h"

#include <cstdlib>
#include <cstring>
#include <iomanip>
#include <iostream>
#include <cmath>
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

problem::Vec3 meanVec3(const std::vector<problem::Vec3> &values) {
    if (values.empty()) {
        return problem::Vec3::Zero();
    }

    problem::Vec3 mean = problem::Vec3::Zero();
    for (const problem::Vec3 &value : values) {
        mean += value;
    }
    return mean / static_cast<problem::Real>(values.size());
}

double meanScalar(const std::vector<double> &values) {
    if (values.empty()) {
        return 0.0f;
    }

    double mean = 0.0f;
    for (double value : values) {
        mean += value;
    }
    return mean / static_cast<double>(values.size());
}

std::vector<problem::Vec3> lineOfSightVelocityVectors(const fmcw_tracker::TrackSummary &summary) {
    std::vector<problem::Vec3> los_velocity_mps;
    const std::size_t count =
        std::min(summary.batch_results.size(), summary.radial_velocity_mps.size());
    los_velocity_mps.reserve(count);
    for (std::size_t i = 0; i < count; ++i) {
        los_velocity_mps.push_back(summary.radial_velocity_mps[i] *
                                   summary.batch_results[i].direction);
    }
    return los_velocity_mps;
}

std::vector<problem::Vec3>
truthVelocityVectors(const std::vector<problem::SimulationMetrics> &truth_metrics) {
    std::vector<problem::Vec3> velocities;
    velocities.reserve(truth_metrics.size());
    for (const problem::SimulationMetrics &truth : truth_metrics) {
        velocities.push_back(truth.velocity_mps);
    }
    return velocities;
}

std::vector<problem::Vec3>
truthLineOfSightVelocityVectors(const std::vector<problem::SimulationMetrics> &truth_metrics) {
    std::vector<problem::Vec3> velocities;
    velocities.reserve(truth_metrics.size());
    for (const problem::SimulationMetrics &truth : truth_metrics) {
        velocities.push_back(truth.radial_velocity_mps * truth.line_of_sight);
    }
    return velocities;
}

double rmseVec3(const std::vector<problem::Vec3> &estimates,
                const std::vector<problem::SimulationMetrics> &truth) {
    if (estimates.empty() || estimates.size() != truth.size()) {
        return 0.0f;
    }

    double squared_error_sum = 0.0f;
    for (std::size_t i = 0; i < estimates.size(); ++i) {
        squared_error_sum += (estimates[i] - truth[i].position_m).squaredNorm();
    }
    return std::sqrt(squared_error_sum / static_cast<double>(estimates.size()));
}

double rmseRange(const std::vector<double> &ranges_m,
                 const std::vector<problem::SimulationMetrics> &truth) {
    if (ranges_m.empty() || ranges_m.size() != truth.size()) {
        return 0.0f;
    }

    double squared_error_sum = 0.0f;
    for (std::size_t i = 0; i < ranges_m.size(); ++i) {
        const double error_m = ranges_m[i] - truth[i].range_m;
        squared_error_sum += error_m * error_m;
    }
    return std::sqrt(squared_error_sum / static_cast<double>(ranges_m.size()));
}

double degrees(double radians) {
    return radians * 180.0f / problem::Constants::kPi;
}

double azimuthDeg(const problem::Vec3 &direction) {
    return degrees(std::atan2(direction.y(), direction.x()));
}

double elevationDeg(const problem::Vec3 &direction) {
    return degrees(std::atan2(direction.z(), std::hypot(direction.x(), direction.y())));
}

double wrapAngleDeg(double angle_deg) {
    while (angle_deg > 180.0f) {
        angle_deg -= 360.0f;
    }
    while (angle_deg < -180.0f) {
        angle_deg += 360.0f;
    }
    return angle_deg;
}

double rmseAzimuthDeg(const std::vector<fmcw_tracker::BatchResult> &batches,
                      const std::vector<problem::SimulationMetrics> &truth) {
    if (batches.empty() || batches.size() != truth.size()) {
        return 0.0f;
    }

    double squared_error_sum = 0.0f;
    for (std::size_t i = 0; i < batches.size(); ++i) {
        const double error_deg =
            wrapAngleDeg(azimuthDeg(batches[i].direction) - azimuthDeg(truth[i].line_of_sight));
        squared_error_sum += error_deg * error_deg;
    }
    return std::sqrt(squared_error_sum / static_cast<double>(batches.size()));
}

double rmseElevationDeg(const std::vector<fmcw_tracker::BatchResult> &batches,
                        const std::vector<problem::SimulationMetrics> &truth) {
    if (batches.empty() || batches.size() != truth.size()) {
        return 0.0f;
    }

    double squared_error_sum = 0.0f;
    for (std::size_t i = 0; i < batches.size(); ++i) {
        const double error_deg =
            elevationDeg(batches[i].direction) - elevationDeg(truth[i].line_of_sight);
        squared_error_sum += error_deg * error_deg;
    }
    return std::sqrt(squared_error_sum / static_cast<double>(batches.size()));
}

std::string formatFrequencyCandidates(const std::vector<double> &frequencies_hz,
                                      const std::vector<double> &powers) {
    std::ostringstream out;
    out << '[';
    const std::size_t count = std::min(frequencies_hz.size(), powers.size());
    for (std::size_t i = 0; i < count; ++i) {
        if (i > 0) {
            out << ", ";
        }
        out << std::fixed << std::setprecision(3) << frequencies_hz[i] << " Hz @ "
            << std::scientific << std::setprecision(3) << powers[i];
    }
    out << ']';
    return out.str();
}

} // namespace

int main(int argc, char **argv) {
    try {
        const RuntimeOptions runtime_options = parseRuntimeOptions(argc, argv);

        const problem::ProblemDescription description =
            radar_algo::makeSingleTargetTrackingDescription(problem::kDefaultProblemDescription);

        fmcw_tracker::StreamingTracker tracker(description);
        radar_algo::streamRadarChirps(description,
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

        std::cout << "Mean radial velocity: " << meanScalar(summary.radial_velocity_mps)
                  << " m/s\n";
        std::cout << "Mean LOS velocity vector: "
                  << formatVec3(meanVec3(lineOfSightVelocityVectors(summary))) << " m/s\n";
        if (runtime_options.print_truth_summary) {
            std::cout << "Mean truth LOS velocity vector: "
                      << formatVec3(
                             meanVec3(truthLineOfSightVelocityVectors(summary.truth_metrics)))
                      << " m/s\n";
            std::cout << "Mean truth Cartesian velocity: "
                      << formatVec3(meanVec3(truthVelocityVectors(summary.truth_metrics)))
                      << " m/s\n";
        }
        std::cout << "Micro-Doppler frequency: " << summary.microdoppler_phase_frequency_hz
                  << " Hz\n";
        std::cout << "Micro-Doppler truth frequency: " << summary.microdoppler_truth_frequency_hz
                  << " Hz\n";
        std::cout << "Micro-Doppler frequency RMSE: " << summary.microdoppler_frequency_rmse_hz
                  << " Hz\n";
        std::cout << "Micro-Doppler valid CPI count: " << summary.microdoppler_valid_cpi_count
                  << '\n';
        std::cout << "Micro-Doppler peak power: " << summary.microdoppler_peak_power << '\n';
        std::cout << "Micro-Doppler phase residual mean/std/rms: "
                  << summary.microdoppler_residual_phase_mean_rad << " / "
                  << summary.microdoppler_residual_phase_stddev_rad << " / "
                  << summary.microdoppler_residual_phase_rms_rad << " rad\n";
        std::cout << "Micro-Doppler top candidates: "
                  << formatFrequencyCandidates(summary.microdoppler_candidate_frequency_hz,
                                               summary.microdoppler_candidate_power)
                  << '\n';
        std::cout << "RMSE raw XYZ: " << rmseVec3(summary.raw_positions_m, summary.truth_metrics)
                  << " m\n";
        std::cout << "RMSE range: " << rmseRange(summary.ranges_m, summary.truth_metrics) << " m\n";
        std::cout << "RMSE azimuth: "
                  << rmseAzimuthDeg(summary.batch_results, summary.truth_metrics) << " deg\n";
        std::cout << "RMSE elevation: "
                  << rmseElevationDeg(summary.batch_results, summary.truth_metrics) << " deg\n";
    } catch (const std::exception &ex) {
        std::cerr << "Radar demo failed: " << ex.what() << '\n';
        return 1;
    }
    return 0;
}
