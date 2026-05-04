#include "radar_algo.h"

#include "probe.h"
#include "simulation.h"

#include <cmath>
#include <stdexcept>
#include <vector>

namespace radar_algo {
namespace {

using Probe64 =
    Probe<problem::RadarSettings::kProbeNumX, problem::RadarSettings::kProbeNumY>;

constexpr std::size_t kInitialTrackingBatchCount = 11;
constexpr std::size_t kInitialTrackingCpiChirps = 64;

Complex cis(float phase_rad) {
    return {std::cos(phase_rad), std::sin(phase_rad)};
}

std::vector<Complex> computeChirpReference(const problem::RadarSettings &radar_settings) {
    std::vector<Complex> chirp(RadarSimulator::kBlockSize);
    const float chirp_duration_s =
        static_cast<float>(RadarSimulator::kBlockSize) / radar_settings.sample_rate_hz;
    const float chirp_slope_hz_per_s = radar_settings.bandwidth_hz / chirp_duration_s;
    const float sample_period_s = 1.0f / radar_settings.sample_rate_hz;
    const float a = problem::Constants::kPi * chirp_slope_hz_per_s * sample_period_s *
                    sample_period_s;
    const float b = -problem::Constants::kPi * radar_settings.bandwidth_hz * sample_period_s;

    Complex sample(1.0f, 0.0f);
    Complex phase_step = cis(a + b);
    const Complex phase_acceleration = cis(2.0f * a);

    for (std::size_t n = 0; n < RadarSimulator::kBlockSize; ++n) {
        chirp[n] = sample;
        sample *= phase_step;
        phase_step *= phase_acceleration;
    }

    return chirp;
}

Probe64 makeProbe(const problem::ProblemDescription &description) {
    const auto &radar_settings = description.radar;
    const auto &probe_settings = description.probe;
    const float lambda_m = problem::Constants::kSpeedOfLightMps / radar_settings.carrier_hz;
    const float spacing_x_m = probe_settings.spacing_x_wavelengths * lambda_m;
    const float spacing_y_m = probe_settings.spacing_y_wavelengths * lambda_m;
    return Probe64(probe_settings.center_m, spacing_x_m, spacing_y_m);
}

} // namespace

std::size_t computeChirpCount(const problem::ProblemDescription &description) {
    const float chirp_duration_s =
        static_cast<float>(RadarSimulator::kBlockSize) / description.radar.sample_rate_hz;
    return static_cast<std::size_t>(
        std::llround(description.simulator.burst_duration_s / chirp_duration_s));
}

problem::ProblemDescription
makeSingleTargetTrackingDescription(const problem::ProblemDescription &base) {
    problem::ProblemDescription description = base;
    if (description.cars.empty()) {
        throw std::runtime_error("single-target tracking requires at least one car");
    }

    description.cars = {description.cars.front()};
    description.simulator.vehicle_count = 1;
    description.simulator.random_seed = 0U;
    description.floorplane_clutter.enable_static_floorplane = false;
    description.radar.receiver_noiselevel_stddev = 0.0f;
    description.radar.receiver_noiselevel_mean = 0.0f;
    description.radar.receiver_noise_distribution_stddev = 0.0f;

    const float chirp_duration_s =
        static_cast<float>(RadarSimulator::kBlockSize) / description.radar.sample_rate_hz;
    description.simulator.burst_duration_s =
        static_cast<float>(kInitialTrackingBatchCount * kInitialTrackingCpiChirps) *
        chirp_duration_s;

    return description;
}

void streamRadarChirps(const problem::ProblemDescription &description,
                       const ChirpCallback &callback) {
    const std::size_t chirp_count = computeChirpCount(description);
    const std::size_t num_rx = problem::RadarSettings::kProbeNumElements;

    RadarSimulator simulator(description);
    const Probe64 probe = makeProbe(description);
    const auto probe_state = simulator.prepareProbeState(probe);
    const std::vector<Complex> tx_chirp = computeChirpReference(description.radar);
    RadarSimulator::ElementVector step_output;
    std::vector<Complex> rx_chirp(RadarSimulator::kBlockSize * num_rx);

    for (std::size_t chirp_index = 0; chirp_index < chirp_count; ++chirp_index) {
        for (std::size_t sample_index = 0; sample_index < RadarSimulator::kBlockSize;
             ++sample_index) {
            simulator.step<problem::RadarSettings::kProbeNumX,
                           problem::RadarSettings::kProbeNumY>(
                probe_state, step_output, tx_chirp[sample_index]);

            const std::size_t base_offset = sample_index * num_rx;
            for (std::size_t rx_index = 0; rx_index < num_rx; ++rx_index) {
                rx_chirp[base_offset + rx_index] =
                    step_output(static_cast<Eigen::Index>(rx_index));
            }
        }

        callback(chirp_index, tx_chirp, rx_chirp);
    }
}

} // namespace radar_algo
