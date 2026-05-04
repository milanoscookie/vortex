#pragma once

#include "dynamics.h"
#include "environment.h"
#include "probe.h"
#include "problem_description.h"
#include "receiver_noise_model.h"
#include "target_echo_model.h"
#include "target_observation.h"
#include "tx_history_buffer.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <complex>
#include <cstddef>
#include <cstdint>
#include <vector>

class RadarSimulator {
  public:
    using size_t = std::size_t;

    static constexpr size_t kTxHistorySize = 8192U;
    static constexpr size_t kBlockSize = problem::Constants::kRadarBlockSize;

    using Complex = std::complex<float>;

    using Vec3 = problem::Vec3;
    using CarSettings = problem::CarSettings;
    using CarList = std::vector<CarSettings>;
    using RadarSettings = problem::RadarSettings;
    using FloorplaneClutterSettings = problem::FloorplaneClutterSettings;
    using SimulationMetrics = problem::SimulationMetrics;
    using ProblemDescription = problem::ProblemDescription;
    using Constants = problem::Constants;
    using ProbeSettings = problem::ProbeSettings;

    using ElementVector =
        Eigen::Matrix<Complex, static_cast<int>(RadarSettings::kProbeNumElements), 1>;

    template <size_t NumX, size_t NumY> using ProbeState = typename Probe<NumX, NumY>::Compiled;

    using DefaultProbe = Probe<RadarSettings::kProbeNumX, RadarSettings::kProbeNumY>;
    using DefaultProbeState = ProbeState<RadarSettings::kProbeNumX, RadarSettings::kProbeNumY>;

    explicit RadarSimulator(const ProblemDescription &description);
    explicit RadarSimulator(const RadarSettings &radar_settings,
                            const CarSettings &car_settings,
                            std::uint32_t random_seed = 0U);
    explicit RadarSimulator(const RadarSettings &radar_settings,
                            const CarList &car_settings,
                            std::uint32_t random_seed = 0U);
    RadarSimulator(const RadarSettings &radar_settings,
                   const CarSettings &car_settings,
                   const FloorplaneClutterSettings &floorplane_settings,
                   std::uint32_t random_seed = 0U);
    RadarSimulator(const RadarSettings &radar_settings,
                   const CarList &car_settings,
                   const FloorplaneClutterSettings &floorplane_settings,
                   std::uint32_t random_seed = 0U);

    void step(ElementVector &output, Complex tx_sample);

    template <size_t NumX, size_t NumY>
    void step(const Probe<NumX, NumY> &probe, ElementVector &output, Complex tx_sample);
    template <size_t NumX, size_t NumY>
    ProbeState<NumX, NumY> prepareProbeState(const Probe<NumX, NumY> &probe) const;

    template <size_t NumX, size_t NumY>
    void step(const ProbeState<NumX, NumY> &probe_state,
              Eigen::Ref<ElementVector> output,
              Complex tx_sample);

    const RadarSettings &config() const noexcept {
        return radar_settings_;
    }
    const CarSettings &car() const noexcept {
        return dynamics_.front().car();
    }
    const std::vector<CarDynamics> &cars() const noexcept {
        return dynamics_;
    }
    float timeSeconds() const noexcept {
        return time_s_;
    }
    const SimulationMetrics &lastMetrics() const noexcept {
        return last_metrics_.front();
    }
    const std::vector<SimulationMetrics> &lastMetricsPerTarget() const noexcept {
        return last_metrics_;
    }

  private:
    template <size_t NumX, size_t NumY>
    ProbeState<NumX, NumY> makeProbeState(const Probe<NumX, NumY> &probe) const;
    DefaultProbeState prepareDefaultProbeState(const ProbeSettings &probe_settings) const;
    float sampleIntervalSeconds() const noexcept;
    SimulationMetrics
    metricsFromObservation(float t_s, const radar::TargetObservation &observation) const noexcept;
    static std::vector<CarDynamics> makeDynamics(const CarList &car_settings);
    RadarSettings radar_settings_;
    std::vector<CarDynamics> dynamics_;
    Environment environment_;
    radar::TxHistoryBuffer<Complex, kTxHistorySize> tx_history_;
    radar::ReceiverNoiseModel receiver_noise_model_;
    DefaultProbeState default_probe_state_;
    size_t sample_index_ = 0;
    float time_s_ = 0.0f;
    std::vector<SimulationMetrics> last_metrics_;
};

template <size_t NumX, size_t NumY>
auto RadarSimulator::makeProbeState(const Probe<NumX, NumY> &probe) const
    -> ProbeState<NumX, NumY> {
    return probe.compile();
}

template <size_t NumX, size_t NumY>
auto RadarSimulator::prepareProbeState(const Probe<NumX, NumY> &probe) const
    -> ProbeState<NumX, NumY> {
    return makeProbeState(probe);
}

template <size_t NumX, size_t NumY>
void RadarSimulator::step(const Probe<NumX, NumY> &probe,
                          ElementVector &output,
                          Complex tx_sample) {
    const ProbeState<NumX, NumY> probe_state = makeProbeState(probe);
    step(probe_state, output, tx_sample);
}

template <size_t NumX, size_t NumY>
void RadarSimulator::step(const ProbeState<NumX, NumY> &probe_state,
                          Eigen::Ref<ElementVector> output,
                          Complex tx_sample) {
    constexpr size_t element_count = NumX * NumY;

    tx_history_.store(sample_index_, tx_sample);

    const float t_s = time_s_;
    const float wave_number =
        2.0f * Constants::kPi * radar_settings_.carrier_hz / Constants::kSpeedOfLightMps;
    const bool has_floor = environment_.hasStaticFloorplane();
    std::array<Complex, element_count> noise_samples{};
    std::vector<radar::TargetObservation> observations(dynamics_.size());
    std::vector<Complex> reflectivities(dynamics_.size());

    receiver_noise_model_.fill(noise_samples);

    const Eigen::Map<const ElementVector> noise_map(noise_samples.data());
    output = noise_map;
    const Complex floor_sample = environment_.sampleStaticFloorplane(sample_index_ % kBlockSize);
    if (has_floor) {
        for (int element_index = 0; element_index < static_cast<int>(element_count);
             ++element_index) {
            output(element_index) += floor_sample;
        }
    }

    last_metrics_.resize(dynamics_.size());
    for (std::size_t car_index = 0; car_index < dynamics_.size(); ++car_index) {
        const CarDynamics &car_dynamics = dynamics_[car_index];
        observations[car_index] = radar::observeTarget(car_dynamics, radar_settings_, t_s);
        reflectivities[car_index] = car_dynamics.car().reflectivity;
        last_metrics_[car_index] = metricsFromObservation(t_s, observations[car_index]);
    }

    for (int element_index = 0; element_index < static_cast<int>(element_count); ++element_index) {
        const float weight = probe_state.element_weight(element_index);
        if (weight == 0.0f) {
            output(element_index) = Complex(0.0f, 0.0f);
            continue;
        }

        const auto element_position =
            probe_state.element_positions_m.col(static_cast<Eigen::Index>(element_index));
        const float element_delay_samples =
            probe_state.element_delay_samples(element_index);

        for (std::size_t car_index = 0; car_index < dynamics_.size(); ++car_index) {
            output(static_cast<Eigen::Index>(element_index)) += radar::sampleBistaticTargetReturn(
                observations[car_index],
                element_position,
                element_delay_samples,
                weight,
                wave_number,
                radar_settings_,
                reflectivities[car_index],
                tx_history_,
                sample_index_);
        }
    }

    ++sample_index_;
    time_s_ += sampleIntervalSeconds();
}
