#include "simulation.h"

#include <stdexcept>

namespace {

    RadarSimulator::DefaultProbe makeDefaultProbe(
        const RadarSimulator::RadarSettings &radar_settings,
        const problem::ProbeSettings &probe_settings) {
            const float lambda_m =
                problem::Constants::kSpeedOfLightMps / radar_settings.carrier_hz;
            const float spacing_x_m = probe_settings.spacing_x_wavelengths * lambda_m;
            const float spacing_y_m = probe_settings.spacing_y_wavelengths * lambda_m;
            return RadarSimulator::DefaultProbe(probe_settings.center_m, spacing_x_m,
                spacing_y_m);

        }

} // namespace

RadarSimulator::RadarSimulator(const ProblemDescription &description)
: RadarSimulator(description.radar, description.car,
    description.floorplane_clutter,
    description.simulator.random_seed) {
        default_probe_state_ = prepareDefaultProbeState(description.probe);
    }

    RadarSimulator::RadarSimulator(const RadarSettings &radar_settings,
        const CarSettings &car_settings,
        std::uint32_t random_seed)
    : radar_settings_(radar_settings),
    dynamics_(car_settings),
    environment_(radar_settings),
    receiver_noise_model_(
        radar_settings.receiver_noiselevel_stddev,
        radar_settings.receiver_noiselevel_mean,
        radar_settings.receiver_noise_distribution_stddev, random_seed) {
            default_probe_state_ = prepareDefaultProbeState(ProbeSettings{});
        }

        RadarSimulator::RadarSimulator(
            const RadarSettings &radar_settings, const CarSettings &car_settings,
            const FloorplaneClutterSettings &floorplane_settings,
            std::uint32_t random_seed)
        : radar_settings_(radar_settings),
        dynamics_(car_settings),
        environment_(radar_settings, floorplane_settings),
        receiver_noise_model_(
            radar_settings.receiver_noiselevel_stddev,
            radar_settings.receiver_noiselevel_mean,
            radar_settings.receiver_noise_distribution_stddev, random_seed) {
                default_probe_state_ = prepareDefaultProbeState(problem::ProbeSettings{});
            }

            auto RadarSimulator::prepareDefaultProbeState(
                const ProbeSettings &probe_settings) const -> DefaultProbeState {
                    return makeProbeState(makeDefaultProbe(radar_settings_, probe_settings));
                }

                float RadarSimulator::sampleIntervalSeconds() const noexcept {
                    return 1.0f / radar_settings_.sample_rate_hz;
                }

                void RadarSimulator::step(
                    RadarSimulator::ElementVector &output,
                    Complex tx_sample) {
                        step<RadarSettings::kProbeNumX, RadarSettings::kProbeNumY>(
                            default_probe_state_, output, tx_sample);
                    }

                    RadarSimulator::SimulationMetrics RadarSimulator::metricsFromObservation(
                        float t_s, const radar::TargetObservation &observation) const noexcept {
                            return radar::makeSimulationMetrics(t_s, observation);
                        }
