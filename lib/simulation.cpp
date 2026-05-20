#include "simulation.h"

#include <cmath>
#include <stdexcept>

namespace {

RadarSimulator::DefaultProbe makeDefaultProbe(const RadarSimulator::RadarSettings &radar_settings,
                                              const dsp::ProbeSettings &probe_settings) {
    const double lambda_m = dsp::Constants::kSpeedOfLightMps / radar_settings.carrier_hz;
    const double spacing_x_m = probe_settings.spacing_x_wavelengths * lambda_m;
    const double spacing_y_m = probe_settings.spacing_y_wavelengths * lambda_m;
    return RadarSimulator::DefaultProbe(probe_settings.center_m, spacing_x_m, spacing_y_m);
}

bool isFiniteVec3(const dsp::Vec3 &value) {
    return value.allFinite();
}

void validateCarSettings(const dsp::CarSettings &car_settings) {
    if (!isFiniteVec3(car_settings.initial_position_m) ||
        !isFiniteVec3(car_settings.base_velocity_mps)) {
        throw std::invalid_argument("car position and velocity must be finite");
    }
    if (!std::isfinite(car_settings.yaw_rad) || !std::isfinite(car_settings.length_m) ||
        !std::isfinite(car_settings.width_m) || !std::isfinite(car_settings.height_m) ||
        !std::isfinite(car_settings.bounce_amplitude_m) ||
        !std::isfinite(car_settings.bounce_frequency_hz) ||
        !std::isfinite(car_settings.bounce_phase_rad) ||
        !std::isfinite(car_settings.reflectivity.real()) ||
        !std::isfinite(car_settings.reflectivity.imag())) {
        throw std::invalid_argument("car settings must be finite");
    }
    if (car_settings.length_m <= 0.0 || car_settings.width_m <= 0.0 ||
        car_settings.height_m <= 0.0) {
        throw std::invalid_argument("car dimensions must be positive");
    }
}

void validateProblemDescription(const RadarSimulator::ProblemDescription &description) {
    const auto &radar = description.radar;
    if (!std::isfinite(radar.bandwidth_hz) || radar.bandwidth_hz <= 0.0) {
        throw std::invalid_argument("radar bandwidth_hz must be positive and finite");
    }
    if (!std::isfinite(radar.sample_rate_hz) || radar.sample_rate_hz <= 0.0) {
        throw std::invalid_argument("radar sample_rate_hz must be positive and finite");
    }
    if (!std::isfinite(radar.carrier_hz) || radar.carrier_hz <= 0.0) {
        throw std::invalid_argument("radar carrier_hz must be positive and finite");
    }
    if (!std::isfinite(radar.min_range_m) || !std::isfinite(radar.max_range_m) ||
        radar.min_range_m <= 0.0 || radar.max_range_m < radar.min_range_m) {
        throw std::invalid_argument("radar range limits must be finite and ordered");
    }
    if (!std::isfinite(radar.field_gain) || radar.field_gain < 0.0) {
        throw std::invalid_argument("radar field_gain must be non-negative and finite");
    }
    if (!std::isfinite(radar.receiver_noiselevel_stddev) ||
        radar.receiver_noiselevel_stddev < 0.0 ||
        !std::isfinite(radar.receiver_noise_distribution_stddev) ||
        radar.receiver_noise_distribution_stddev < 0.0 ||
        !std::isfinite(radar.receiver_noiselevel_mean)) {
        throw std::invalid_argument("radar noise settings must be finite with non-negative stddev");
    }

    const auto &probe = description.probe;
    if (!isFiniteVec3(probe.center_m) || !std::isfinite(probe.spacing_x_wavelengths) ||
        !std::isfinite(probe.spacing_y_wavelengths) || probe.spacing_x_wavelengths <= 0.0 ||
        probe.spacing_y_wavelengths <= 0.0) {
        throw std::invalid_argument("probe settings must be finite with positive spacings");
    }

    const auto &simulator = description.simulator;
    if (!std::isfinite(simulator.burst_duration_s) || simulator.burst_duration_s <= 0.0 ||
        !std::isfinite(simulator.tracking_duration_s) ||
        !std::isfinite(simulator.split_duration_s) ||
        !std::isfinite(simulator.prediction_duration_s) || simulator.tracking_duration_s < 0.0 ||
        simulator.split_duration_s < 0.0 || simulator.prediction_duration_s < 0.0) {
        throw std::invalid_argument("simulator durations must be finite and non-negative");
    }

    if (description.cars.empty()) {
        throw std::invalid_argument("problem description must contain at least one car");
    }
    for (const dsp::CarSettings &car_settings : description.cars) {
        validateCarSettings(car_settings);
    }
}

const RadarSimulator::ProblemDescription &
validatedProblemDescription(const RadarSimulator::ProblemDescription &description) {
    validateProblemDescription(description);
    return description;
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

RadarSimulator::RadarSimulator(const RadarSimulator::ProblemDescription &description)
    : RadarSimulator(validatedProblemDescription(description), true) {}

RadarSimulator::RadarSimulator(const RadarSimulator::ProblemDescription &description,
                               bool /*already_validated*/)
    : radar_settings_(description.radar), dynamics_(makeDynamics(description.cars)),
      environment_(description.radar, description.floorplane_clutter),
      receiver_noise_model_(description.radar.receiver_noiselevel_stddev,
                            description.radar.receiver_noiselevel_mean,
                            description.radar.receiver_noise_distribution_stddev,
                            description.radar.receiver_noise_use_std_normal_distribution,
                            description.simulator.random_seed),
      observation_scratch_(dynamics_.size()), last_metrics_(dynamics_.size()) {
    default_probe_state_ = prepareDefaultProbeState(description.probe);
}

auto RadarSimulator::prepareDefaultProbeState(const ProbeSettings &probe_settings) const
    -> DefaultProbeState {
    return makeProbeState(makeDefaultProbe(radar_settings_, probe_settings));
}

double RadarSimulator::sampleIntervalSeconds() const noexcept {
    return 1.0f / radar_settings_.sample_rate_hz;
}

void RadarSimulator::step(RadarSimulator::ElementVector &output, Complex tx_sample) {
    step<RadarSettings::kProbeNumX, RadarSettings::kProbeNumY>(
        default_probe_state_, output, tx_sample);
}

RadarSimulator::SimulationMetrics
RadarSimulator::metricsFromObservation(double t_s,
                                       const radar::TargetObservation &observation) const noexcept {
    return radar::makeSimulationMetrics(t_s, observation);
}
