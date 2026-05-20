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
    static constexpr std::size_t kTxHistorySize = 8192U;
    static constexpr std::size_t kBlockSize = dsp::RadarSettings::kRadarBlockSize;

    using RadarSettings = dsp::RadarSettings;
    using Complex = dsp::Complex;
    using Vec3 = dsp::Vec3;
    using CarSettings = dsp::CarSettings;
    using CarList = std::vector<CarSettings>;
    using FloorplaneClutterSettings = dsp::FloorplaneClutterSettings;
    using SimulationMetrics = dsp::SimulationMetrics;
    using ProblemDescription = dsp::ProblemDescription;
    using Constants = dsp::Constants;
    using ProbeSettings = dsp::ProbeSettings;

    using ElementVector =
        Eigen::Matrix<Complex, static_cast<int>(RadarSettings::kProbeNumElements), 1>;

    template <std::size_t NumX, std::size_t NumY>
    using ProbeState = typename Probe<NumX, NumY>::Compiled;

    using DefaultProbe = Probe<RadarSettings::kProbeNumX, RadarSettings::kProbeNumY>;
    using DefaultProbeState = ProbeState<RadarSettings::kProbeNumX, RadarSettings::kProbeNumY>;

    // RT-unsafe.
    // Copies simulation configuration, allocates working storage, and prepares default probe state.
    explicit RadarSimulator(const ProblemDescription &description);

    // RT-safe.
    // Processes one sample for the default probe using only preallocated state.
    void step(ElementVector &output, Complex tx_sample);

    template <std::size_t NumX, std::size_t NumY>
    // RT-unsafe.
    // Compiles probe state on demand, then processes one sample.
    void step(const Probe<NumX, NumY> &probe, ElementVector &output, Complex tx_sample);

    template <std::size_t NumX, std::size_t NumY>
    // RT-unsafe.
    // Compiles probe geometry into reusable state before streaming starts.
    ProbeState<NumX, NumY> prepareProbeState(const Probe<NumX, NumY> &probe) const;

    template <std::size_t NumX, std::size_t NumY>
    // RT-safe.
    // Processes one sample using caller-prepared probe state and preallocated scratch buffers.
    void step(const ProbeState<NumX, NumY> &probe_state,
              Eigen::Ref<ElementVector> output,
              Complex tx_sample);

    // RT-safe.
    const RadarSettings &config() const noexcept {
        return radar_settings_;
    }

    // RT-safe.
    const CarSettings &car() const noexcept {
        return dynamics_.front().car();
    }

    // RT-safe.
    const std::vector<CarDynamics> &cars() const noexcept {
        return dynamics_;
    }

    // RT-safe.
    double timeSeconds() const noexcept {
        return time_s_;
    }

    // RT-safe.
    const SimulationMetrics &lastMetrics() const noexcept {
        return last_metrics_.front();
    }

    // RT-safe.
    const std::vector<SimulationMetrics> &lastMetricsPerTarget() const noexcept {
        return last_metrics_;
    }

  private:
    RadarSimulator(const ProblemDescription &description, bool already_validated);
    template <std::size_t NumX, std::size_t NumY>
    ProbeState<NumX, NumY> makeProbeState(const Probe<NumX, NumY> &probe) const;
    DefaultProbeState prepareDefaultProbeState(const ProbeSettings &probe_settings) const;
    double sampleIntervalSeconds() const noexcept;
    SimulationMetrics
    metricsFromObservation(double t_s, const radar::TargetObservation &observation) const noexcept;
    static std::vector<CarDynamics> makeDynamics(const CarList &car_settings);
    RadarSettings radar_settings_;
    std::vector<CarDynamics> dynamics_;
    Environment environment_;
    radar::TxHistoryBuffer<Complex, kTxHistorySize> tx_history_;
    radar::ReceiverNoiseModel receiver_noise_model_;
    DefaultProbeState default_probe_state_;
    std::size_t sample_index_ = 0U;
    double time_s_ = 0.0f;
    std::vector<radar::TargetObservation> observation_scratch_;
    std::vector<SimulationMetrics> last_metrics_;
};

template <std::size_t NumX, std::size_t NumY>
auto RadarSimulator::makeProbeState(const Probe<NumX, NumY> &probe) const
    -> ProbeState<NumX, NumY> {
    return probe.compile();
}

template <std::size_t NumX, std::size_t NumY>
auto RadarSimulator::prepareProbeState(const Probe<NumX, NumY> &probe) const
    -> ProbeState<NumX, NumY> {
    return makeProbeState(probe);
}

template <std::size_t NumX, std::size_t NumY>
void RadarSimulator::step(const Probe<NumX, NumY> &probe,
                          ElementVector &output,
                          Complex tx_sample) {
    const ProbeState<NumX, NumY> probe_state = makeProbeState(probe);
    step<NumX, NumY>(probe_state, output, tx_sample);
}

template <std::size_t NumX, std::size_t NumY>
void RadarSimulator::step(const ProbeState<NumX, NumY> &probe_state,
                          Eigen::Ref<ElementVector> output,
                          Complex tx_sample) {
    constexpr std::size_t elementCount = NumX * NumY;

    tx_history_.store(sample_index_, tx_sample);

    const double t_s = time_s_;
    const double wave_number =
        2.0f * Constants::kPi * radar_settings_.carrier_hz / Constants::kSpeedOfLightMps;
    const bool has_floor = environment_.hasStaticFloorplane();
    std::array<Complex, elementCount> noiseSamples{};

    receiver_noise_model_.fill(noiseSamples);

    const Eigen::Map<const ElementVector> noiseMap(noiseSamples.data());
    output = noiseMap;
    const Complex floorSample = environment_.sampleStaticFloorplane(sample_index_ % kBlockSize);
    if (has_floor) {
        for (int elementIndex = 0; elementIndex < static_cast<int>(elementCount); ++elementIndex) {
            output(elementIndex) += floorSample;
        }
    }

    for (std::size_t carIndex = 0; carIndex < dynamics_.size(); ++carIndex) {
        const CarDynamics &carDynamics = dynamics_[carIndex];
        observation_scratch_[carIndex] = radar::observeTarget(carDynamics, radar_settings_, t_s);
        last_metrics_[carIndex] = metricsFromObservation(t_s, observation_scratch_[carIndex]);
    }

    for (int elementIndex = 0; elementIndex < static_cast<int>(elementCount); ++elementIndex) {
        const double weight = probe_state.element_weight(elementIndex);
        if (weight == 0.0f) {
            output(elementIndex) = Complex(0.0f, 0.0f);
            continue;
        }

        const auto elementPosition =
            probe_state.element_positions_m.col(static_cast<Eigen::Index>(elementIndex));
        const double elementDelaySamples = probe_state.element_delay_samples(elementIndex);

        for (std::size_t carIndex = 0; carIndex < dynamics_.size(); ++carIndex) {
            output(static_cast<Eigen::Index>(elementIndex)) +=
                radar::sampleBistaticTargetReturn(observation_scratch_[carIndex],
                                                  elementPosition,
                                                  elementDelaySamples,
                                                  weight,
                                                  wave_number,
                                                  radar_settings_,
                                                  dynamics_[carIndex].car().reflectivity,
                                                  tx_history_,
                                                  sample_index_);
        }
    }

    ++sample_index_;
    time_s_ += sampleIntervalSeconds();
}
