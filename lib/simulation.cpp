#include "simulation.h"

#include <stdexcept>

namespace {

RadarSimulator::DefaultProbe makeDefaultProbe(const RadarSimulator::RadarSettings &radar_settings,
                                              const problem::ProbeSettings &probe_settings) {
    const problem::Real lambda_m = problem::Constants::kSpeedOfLightMps / radar_settings.carrier_hz;
    const problem::Real spacing_x_m = probe_settings.spacing_x_wavelengths * lambda_m;
    const problem::Real spacing_y_m = probe_settings.spacing_y_wavelengths * lambda_m;
    return RadarSimulator::DefaultProbe(probe_settings.center_m, spacing_x_m, spacing_y_m);
}

} // namespace

std::vector<CarDynamics> RadarSimulator::makeDynamics(const CarList &car_settings) {
    std::vector<CarDynamics> dynamics;
    dynamics.reserve(car_settings.size());
    for (const CarSettings &car_setting : car_settings) {
        dynamics.emplace_back(car_setting);
    }
    return dynamics;
}

RadarSimulator::RadarSimulator(const ProblemDescription &description)
    : RadarSimulator(description.radar,
                     description.cars,
                     description.floorplane_clutter,
                     description.simulator.random_seed) {
    simulator_settings_ = description.simulator;
    default_probe_state_ = prepareDefaultProbeState(description.probe);
}

RadarSimulator::RadarSimulator(const RadarSettings &radar_settings,
                               const CarSettings &car_settings,
                               std::uint32_t random_seed)
    : RadarSimulator(radar_settings, CarList{car_settings}, random_seed) {}

RadarSimulator::RadarSimulator(const RadarSettings &radar_settings,
                               const CarList &car_settings,
                               std::uint32_t random_seed)
    : radar_settings_(radar_settings), dynamics_(makeDynamics(car_settings)),
      environment_(radar_settings),
      receiver_noise_model_(radar_settings.receiver_noiselevel_stddev,
                            radar_settings.receiver_noiselevel_mean,
                            radar_settings.receiver_noise_distribution_stddev,
                            radar_settings.receiver_noise_use_std_normal_distribution,
                            random_seed),
      observation_scratch_(dynamics_.size()), last_metrics_(dynamics_.size()) {
    default_probe_state_ = prepareDefaultProbeState(ProbeSettings{});
}

RadarSimulator::RadarSimulator(const RadarSettings &radar_settings,
                               const CarSettings &car_settings,
                               const FloorplaneClutterSettings &floorplane_settings,
                               std::uint32_t random_seed)
    : RadarSimulator(radar_settings, CarList{car_settings}, floorplane_settings, random_seed) {}

RadarSimulator::RadarSimulator(const RadarSettings &radar_settings,
                               const CarList &car_settings,
                               const FloorplaneClutterSettings &floorplane_settings,
                               std::uint32_t random_seed)
    : radar_settings_(radar_settings), dynamics_(makeDynamics(car_settings)),
      environment_(radar_settings, floorplane_settings),
      receiver_noise_model_(radar_settings.receiver_noiselevel_stddev,
                            radar_settings.receiver_noiselevel_mean,
                            radar_settings.receiver_noise_distribution_stddev,
                            radar_settings.receiver_noise_use_std_normal_distribution,
                            random_seed),
      observation_scratch_(dynamics_.size()), last_metrics_(dynamics_.size()) {
    default_probe_state_ = prepareDefaultProbeState(problem::ProbeSettings{});
}

auto RadarSimulator::prepareDefaultProbeState(const ProbeSettings &probe_settings) const
    -> DefaultProbeState {
    return makeProbeState(makeDefaultProbe(radar_settings_, probe_settings));
}

problem::Real RadarSimulator::sampleIntervalSeconds() const noexcept {
    return 1.0f / radar_settings_.sample_rate_hz;
}

void RadarSimulator::step(RadarSimulator::ElementVector &output, Complex tx_sample) {
    step<RadarSettings::kProbeNumX, RadarSettings::kProbeNumY>(
        default_probe_state_, output, tx_sample);
}

RadarSimulator::SimulationMetrics
RadarSimulator::metricsFromObservation(problem::Real t_s,
                                       const radar::TargetObservation &observation) const noexcept {
    return radar::makeSimulationMetrics(t_s, observation);
}

bool RadarSimulator::isGuessLocationWithinPredictedCar(const Vec3 &guesslocation) const noexcept {
    if (dynamics_.empty()) {
        return false;
    }

    const CarDynamics &car_dynamics = dynamics_.front();
    const CarSettings &car_settings = car_dynamics.car();
    const problem::Real prediction_time_s = time_s_ + simulator_settings_.prediction_duration_s;
    const Vec3 actual_location = car_dynamics.positionAt(prediction_time_s);
    const Vec3 delta = guesslocation - actual_location;

    const problem::Real yaw_rad = car_dynamics.yawAt(prediction_time_s);
    const problem::Real cos_yaw = std::cos(yaw_rad);
    const problem::Real sin_yaw = std::sin(yaw_rad);
    const problem::Real longitudinal_offset_m = cos_yaw * delta.x() + sin_yaw * delta.y();
    const problem::Real lateral_offset_m = -sin_yaw * delta.x() + cos_yaw * delta.y();

    return std::abs(longitudinal_offset_m) <= 0.5f * car_settings.length_m &&
           std::abs(lateral_offset_m) <= 0.5f * car_settings.width_m;
}
