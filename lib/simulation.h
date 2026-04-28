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

class RadarSimulator {
public:
  using size_t = std::size_t;


  static constexpr size_t kTxHistorySize = 8192U;
  static constexpr size_t kBlockSize = problem::Constants::kRadarBlockSize;

  using Complex = std::complex<float>;

  using Vec3 = problem::Vec3;
  using CarSettings = problem::CarSettings;
  using RadarSettings = problem::RadarSettings;
  using FloorplaneClutterSettings = problem::FloorplaneClutterSettings;
  using SimulationMetrics = problem::SimulationMetrics;
  using ProblemDescription = problem::ProblemDescription;
  using Constants = problem::Constants;
  using ProbeSettings = problem::ProbeSettings;

  using ElementVector = Eigen::Matrix<Complex,
                                      static_cast<int>(RadarSettings::kProbeNumElements),
                                      1>;

  template <size_t NumX, size_t NumY>
  using ProbeState = typename Probe<NumX, NumY>::Compiled;

  using DefaultProbe = Probe<RadarSettings::kProbeNumX,
                             RadarSettings::kProbeNumY>;
  using DefaultProbeState =
      ProbeState<RadarSettings::kProbeNumX,
                 RadarSettings::kProbeNumY>;

  explicit RadarSimulator(const ProblemDescription &description);
  explicit RadarSimulator(const RadarSettings &radar_settings,
                          const CarSettings &car_settings,
                          std::uint32_t random_seed = 0U);
  RadarSimulator(
      const RadarSettings &radar_settings, const CarSettings &car_settings,
      const FloorplaneClutterSettings &floorplane_settings,
      std::uint32_t random_seed = 0U);

  void step(ElementVector &output,
             Complex tx_sample);

  template <size_t NumX, size_t NumY>
  void step(const Probe<NumX, NumY> &probe, ElementVector &output,
             Complex tx_sample);
  template <size_t NumX, size_t NumY>
  ProbeState<NumX, NumY>
  prepareProbeState(const Probe<NumX, NumY> &probe) const;

  template <size_t NumX, size_t NumY>
  void step(const ProbeState<NumX, NumY> &probe_state,
            Eigen::Ref<ElementVector> output, Complex tx_sample);

  const RadarSettings &config() const noexcept { return radar_settings_; }
  const CarSettings &car() const noexcept { return dynamics_.car(); }
  float timeSeconds() const noexcept { return time_s_; }
  const SimulationMetrics &lastMetrics() const noexcept {
    return last_metrics_;
  }

private:
  template <size_t NumX, size_t NumY>
  ProbeState<NumX, NumY> makeProbeState(const Probe<NumX, NumY> &probe) const;
  DefaultProbeState
  prepareDefaultProbeState(const ProbeSettings &probe_settings) const;
  [[nodiscard]] float sampleIntervalSeconds() const noexcept;
  [[nodiscard]] SimulationMetrics
  metricsFromObservation(float t_s, const radar::TargetObservation &observation) const
      noexcept;

  RadarSettings radar_settings_;
  CarDynamics dynamics_;
  Environment environment_;
  radar::TxHistoryBuffer<Complex, kTxHistorySize> tx_history_;
  radar::ReceiverNoiseModel receiver_noise_model_;
  DefaultProbeState default_probe_state_;
  size_t sample_index_ = 0;
  float time_s_ = 0.0f;
  SimulationMetrics last_metrics_;
};

template <size_t NumX, size_t NumY>
auto RadarSimulator::makeProbeState(const Probe<NumX, NumY> &probe) const
    -> ProbeState<NumX, NumY> {
  return probe.compile(radar_settings_.sample_rate_hz);
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
  const radar::TargetObservation observation =
      radar::observeTarget(dynamics_, radar_settings_, t_s);

  const Complex reflectivity = dynamics_.car().reflectivity;
  const float wave_number =
      2.0f * Constants::kPi * radar_settings_.carrier_hz /
      Constants::kSpeedOfLightMps;
  const bool has_floor = environment_.hasStaticFloorplane();
  std::array<Complex, element_count> noise_samples{};

  receiver_noise_model_.fill(noise_samples);

  const Eigen::Map<const ElementVector> noise_map(noise_samples.data());
  output = noise_map;
  const Complex floor_sample =
      environment_.sampleStaticFloorplane(sample_index_ % kBlockSize);
  if (has_floor) {
    for (int element_index = 0; element_index < static_cast<int>(element_count);
         ++element_index) {
      output(element_index) += floor_sample;
    }
  }

  for (int element_index = 0; element_index < static_cast<int>(element_count);
       ++element_index) {
    const float weight = probe_state.element_weight(element_index);
    if (weight == 0.0f) {
      output(element_index) = Complex(0.0f, 0.0f);
      continue;
    }

    output(static_cast<Eigen::Index>(element_index)) +=
        radar::sampleBistaticTargetReturn(
        observation,
        probe_state.element_positions_m.col(static_cast<Eigen::Index>(element_index)),
        probe_state.element_delay_s(element_index), weight, wave_number,
        radar_settings_,
        reflectivity, tx_history_, sample_index_);
  }

  last_metrics_ = metricsFromObservation(t_s, observation);
  ++sample_index_;
  time_s_ += sampleIntervalSeconds();
}
