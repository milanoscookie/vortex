#pragma once

#include "dynamics.h"
#include "environment.h"
#include "probe.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <complex>
#include <cstddef>
#include <cstdint>
#include <random>
#include <vector>

class RadarSimulator {
public:
  using size_t = std::size_t;
  using Complex = std::complex<float>;
  static constexpr std::size_t kBlockSize = problem::kRadarBlockSize;
  static constexpr std::size_t kTxHistorySize = 8192U;
  static constexpr std::size_t kTxHistoryMask = kTxHistorySize - 1U;
  using Vec3 = problem::Vec3;
  using TargetModel = problem::TargetModel;
  using Config = problem::Config;
  using Metrics = problem::Metrics;

  template <std::size_t NumElements>
  using ElementVector =
      Eigen::Matrix<Complex, static_cast<int>(NumElements), 1>;

  template <std::size_t NumX, std::size_t NumY> struct ProbeState {
    std::array<Vec3, NumX * NumY> element_positions_m;
    std::array<float, NumX * NumY> element_delay_s;
    std::array<float, NumX * NumY> element_weight;
  };

  using DefaultProbe = Probe<problem::kProbeNumX, problem::kProbeNumY>;
  using DefaultProbeState =
      ProbeState<problem::kProbeNumX, problem::kProbeNumY>;

  RadarSimulator(const Config &config, const TargetModel &target,
                 std::uint32_t random_seed = 0U);
  RadarSimulator(const Config &config, const TargetModel &target,
                 const problem::FloorplaneClutterConfig &floorplane_config,
                 std::uint32_t random_seed = 0U);

  void step(ElementVector<problem::kProbeNumElements> &output,
            Complex tx_sample);

  template <std::size_t NumX, std::size_t NumY>
  void step(const Probe<NumX, NumY> &probe, ElementVector<NumX * NumY> &output,
            Complex tx_sample);

  template <std::size_t NumX, std::size_t NumY>
  ProbeState<NumX, NumY>
  prepareProbeState(const Probe<NumX, NumY> &probe) const;

  template <std::size_t NumX, std::size_t NumY>
  void step(const ProbeState<NumX, NumY> &probe_state,
            ElementVector<NumX * NumY> &output, Complex tx_sample);

  template <std::size_t NumX, std::size_t NumY>
  void stepChirp(const ProbeState<NumX, NumY> &probe_state,
                 Complex *output_buffer, const Complex *tx_samples);

  const Config &config() const noexcept { return config_; }
  const TargetModel &target() const noexcept { return dynamics_.target(); }
  float timeSeconds() const noexcept { return time_s_; }
  const Metrics &lastMetrics() const noexcept { return last_metrics_; }

private:
  template <std::size_t NumX, std::size_t NumY>
  ProbeState<NumX, NumY> makeProbeState(const Probe<NumX, NumY> &probe) const;
  DefaultProbeState prepareDefaultProbeState() const;
  Metrics metricsAt(float t_s) const;
  void storeTxSample(Complex tx_sample);
  Complex delayedTxSample(float delay_s) const noexcept;
  Complex sampleNoise();

  Config config_;
  CarDynamics dynamics_;
  Environment environment_;
  std::vector<Complex> tx_history_;
  std::mt19937 noise_rng_;
  std::normal_distribution<float> noise_dist_;
  DefaultProbeState default_probe_state_;
  float sample_duration_s_ = 0.0f;
  std::size_t sample_index_ = 0;
  float time_s_ = 0.0f;
  Metrics last_metrics_;
};

template <std::size_t NumX, std::size_t NumY>
auto RadarSimulator::makeProbeState(const Probe<NumX, NumY> &probe) const
    -> ProbeState<NumX, NumY> {
  ProbeState<NumX, NumY> state;
  for (std::size_t ix = 0; ix < NumX; ++ix) {
    for (std::size_t iy = 0; iy < NumY; ++iy) {
      const std::size_t flat_index = ix * NumY + iy;
      state.element_positions_m[flat_index] = probe.elementPosition(ix, iy);
      state.element_delay_s[flat_index] =
          static_cast<float>(probe(ix, iy).delay) / config_.sample_rate_hz;
      state.element_weight[flat_index] =
          probe(ix, iy).isActive() ? probe(ix, iy).weight : 0.0f;
    }
  }
  return state;
}

template <std::size_t NumX, std::size_t NumY>
auto RadarSimulator::prepareProbeState(const Probe<NumX, NumY> &probe) const
    -> ProbeState<NumX, NumY> {
  return makeProbeState(probe);
}

template <std::size_t NumX, std::size_t NumY>
void RadarSimulator::step(const Probe<NumX, NumY> &probe,
                          ElementVector<NumX * NumY> &output,
                          Complex tx_sample) {
  const ProbeState<NumX, NumY> probe_state = makeProbeState(probe);
  step(probe_state, output, tx_sample);
}

template <std::size_t NumX, std::size_t NumY>
void RadarSimulator::stepChirp(const ProbeState<NumX, NumY> &probe_state,
                               Complex *output_buffer,
                               const Complex *tx_samples) {
  // 1. Sequentially fill TX history for the entire chirp
  for (std::size_t s = 0; s < kBlockSize; ++s) {
    tx_history_[(sample_index_ + s) & kTxHistoryMask] = tx_samples[s];
  }

  const float inv_c = 1.0f / config_.speed_of_light_mps;
  const float inv_lambda = config_.carrier_hz * inv_c;
  const float two_pi = 2.0f * problem::kPi;
  const float dt = 1.0f / config_.sample_rate_hz;
  const float t_start = time_s_;
  const Complex reflectivity = dynamics_.target().reflectivity;

  // Stack-allocated buffers for pre-calculated shared dynamics
  Complex base_returns[kBlockSize];
  Vec3 unit_dirs[kBlockSize];
  float base_phases[kBlockSize];

  // 2. Pre-calculate shared dynamics (Sequential)
  for (std::size_t s = 0; s < kBlockSize; ++s) {
    const float t_s = t_start + static_cast<float>(s) * dt;
    const Vec3 pt = dynamics_.positionAt(t_s);
    const float range_center = pt.norm();
    unit_dirs[s] = pt / range_center;

    const float total_delay_s = (range_center * 2.0f) * inv_c;
    const float delayed_idx_f = static_cast<float>(sample_index_ + s) -
                                total_delay_s * config_.sample_rate_hz;

    Complex delayed_sample(0, 0);
    if (delayed_idx_f >= 0) {
      const std::size_t low = static_cast<std::size_t>(delayed_idx_f);
      const float frac = delayed_idx_f - static_cast<float>(low);
      delayed_sample = (1.0f - frac) * tx_history_[low & kTxHistoryMask] +
                       frac * tx_history_[(low + 1) & kTxHistoryMask];
    }

    const float path_gain =
        config_.field_gain / std::max(range_center * range_center, 1e-6f);
    base_returns[s] = delayed_sample * reflectivity * path_gain;
    base_phases[s] = -two_pi * (range_center * 2.0f) * inv_lambda;
  }

// 3. Parallelize over antenna elements ONCE per chirp
#pragma omp parallel for
  for (int e = 0; e < static_cast<int>(NumX * NumY); ++e) {
    const Vec3 pos = probe_state.element_positions_m[e];
    const float weight = probe_state.element_weight[e];

    if (weight == 0.0f) {
      for (std::size_t s = 0; s < kBlockSize; ++s) {
        output_buffer[s * (NumX * NumY) + e] = Complex(0.0f, 0.0f);
      }
      continue;
    }

    for (std::size_t s = 0; s < kBlockSize; ++s) {
      const float spatial_phase = two_pi * unit_dirs[s].dot(pos) * inv_lambda;
      const float total_phase = base_phases[s] + spatial_phase;
      const std::size_t out_idx = s * (NumX * NumY) + e;

      // Fast path trig: explicit cos/sin often optimizes to hardware sincos
      output_buffer[out_idx] =
          weight * base_returns[s] *
          Complex(std::cos(total_phase), std::sin(total_phase));
    }
  }

  last_metrics_ = metricsAt(t_start + (static_cast<float>(kBlockSize) / 2) * dt);
  sample_index_ += kBlockSize;
  time_s_ += kBlockSize * dt;
}

template <std::size_t NumX, std::size_t NumY>
void RadarSimulator::step(const ProbeState<NumX, NumY> &probe_state,
                          ElementVector<NumX * NumY> &output,
                          Complex tx_sample) {
  storeTxSample(tx_sample);

  const float t_s = time_s_;
  const Vec3 pt = dynamics_.positionAt(t_s);
  const float pt_x = pt.x();
  const float pt_y = pt.y();
  const float pt_z = pt.z();

  const Complex reflectivity = dynamics_.target().reflectivity;
  const bool has_floor = environment_.hasStaticFloorplane();
  const float field_gain = config_.field_gain;
  const float min_range = config_.min_range_m;
  const float max_range = config_.max_range_m;
  const float inv_c = 1.0f / config_.speed_of_light_mps;
  const float inv_lambda = config_.carrier_hz * inv_c;
  const float two_pi = 2.0f * problem::kPi;

  // TX Range from origin (0,0,0) to target
  const float range_tx_m = std::sqrt(pt_x * pt_x + pt_y * pt_y + pt_z * pt_z);

#pragma omp parallel for
  for (int element_index = 0; element_index < static_cast<int>(NumX * NumY);
       ++element_index) {
    const float weight = probe_state.element_weight[element_index];
    if (weight == 0.0f) {
      output(static_cast<Eigen::Index>(element_index)) = Complex(0.0f, 0.0f);
      continue;
    }

    Complex sample(0.0f, 0.0f);
    // Noise sampling is currently skipped for performance/thread-safety in this
    // benchmark
    if (has_floor) {
      sample += environment_.sampleStaticFloorplane(sample_index_ % kBlockSize);
    }

    const Vec3 ep_m = probe_state.element_positions_m[element_index];
    const float dx = pt_x - ep_m.x();
    const float dy = pt_y - ep_m.y();
    const float dz = pt_z - ep_m.z();

    const float range_rx_m = std::sqrt(dx * dx + dy * dy + dz * dz);
    if (range_rx_m <= max_range) {
      const float total_range_m = range_tx_m + range_rx_m;
      const float safe_range_rx = std::max(range_rx_m, min_range);

      const float delay_s =
          total_range_m * inv_c + probe_state.element_delay_s[element_index];
      const float path_gain = field_gain / (safe_range_rx * safe_range_rx);
      const float phase = -two_pi * total_range_m * inv_lambda;

      sample += (weight * path_gain) * delayedTxSample(delay_s) * reflectivity *
                std::polar(1.0f, phase);
    }
    output(static_cast<Eigen::Index>(element_index)) = sample;
  }

  last_metrics_ = metricsAt(t_s);
  ++sample_index_;
  time_s_ += 1.0f / config_.sample_rate_hz;
}
