#include "radar_algo.h"

#include "probe.h"
#include "simulation.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <vector>

namespace radar_algo {
namespace {

using Complex = RadarSimulator::Complex;
using Probe64 =
    Probe<problem::RadarSettings::kProbeNumX, problem::RadarSettings::kProbeNumY>;

constexpr std::size_t kWriteBatchSize = 256;

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

std::size_t computeChirpCount(const problem::ProblemDescription &description) {
    const float chirp_duration_s =
        static_cast<float>(RadarSimulator::kBlockSize) / description.radar.sample_rate_hz;
    return static_cast<std::size_t>(
        std::llround(description.simulator.burst_duration_s / chirp_duration_s));
}

} // namespace

void runRadarAlgorithm(const problem::ProblemDescription &description) {
    const std::size_t chirp_count = computeChirpCount(description);
    RadarSimulator simulator(description);
    const Probe64 probe = makeProbe(description);
    const auto probe_state = simulator.prepareProbeState(probe);
    const std::vector<Complex> tx_chirp = computeChirpReference(description.radar);
    RadarSimulator::ElementVector step_output;

    for (std::size_t batch_start = 0; batch_start < chirp_count;
         batch_start += kWriteBatchSize) {
        const std::size_t current_batch_size =
            std::min(kWriteBatchSize, chirp_count - batch_start);

        for (std::size_t b = 0; b < current_batch_size; ++b) {
            for (std::size_t sample_index = 0; sample_index < RadarSimulator::kBlockSize;
                 ++sample_index) {
                simulator.step<problem::RadarSettings::kProbeNumX,
                               problem::RadarSettings::kProbeNumY>(
                    probe_state, step_output, tx_chirp[sample_index]);
            }
        }
    }
}

} // namespace radar_algo
