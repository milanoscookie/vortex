#include "simulation.h"


namespace {

} // namespace

RadarSimulator::RadarSimulator(const Config &config, const TargetModel &target,
                               std::uint32_t /* random_seed */)
    : config_(config),
      dynamics_(target),
      environment_(config),
      noise_rng_(0U),
      noise_dist_(problem::kNoiseDistributionMean, problem::kNoiseDistributionStddev),
      default_probe_state_(prepareDefaultProbeState()),
      sample_duration_s_(1.0f / config.sample_rate_hz) {
  tx_history_.assign(kTxHistorySize, Complex(0.0f, 0.0f));
}

RadarSimulator::RadarSimulator(const Config &config, const TargetModel &target,
                               const problem::FloorplaneClutterConfig &floorplane_config,
                               std::uint32_t random_seed)
    : config_(config),
      dynamics_(target),
      environment_(config, floorplane_config, random_seed),
      noise_rng_(random_seed),
      noise_dist_(problem::kNoiseDistributionMean, problem::kNoiseDistributionStddev),
      default_probe_state_(prepareDefaultProbeState()),
      sample_duration_s_(1.0f / config.sample_rate_hz) {
  tx_history_.assign(kTxHistorySize, Complex(0.0f, 0.0f));
}

auto RadarSimulator::prepareDefaultProbeState() const -> DefaultProbeState {
  const float lambda_m = config_.speed_of_light_mps / config_.carrier_hz;
  const float spacing_m = 0.5f * lambda_m;
  DefaultProbe probe(coords::Coords::Zero(), spacing_m, spacing_m);
  return makeProbeState(probe);
}

void RadarSimulator::step(
    RadarSimulator::ElementVector<problem::kProbeNumElements> &output,
    Complex tx_sample) {
  step(default_probe_state_, output, tx_sample);
}

RadarSimulator::Metrics RadarSimulator::metricsAt(float t_s) const {
  Metrics metrics;
  metrics.time_s = t_s;
  metrics.position_m = dynamics_.positionAt(t_s);
  metrics.velocity_mps = dynamics_.velocityAt(t_s);

  metrics.range_m = metrics.position_m.norm();
  const float safe_range_m = std::max(metrics.range_m, config_.min_range_m);
  metrics.line_of_sight = metrics.position_m / safe_range_m;

  metrics.delay_s = 2.0f * metrics.range_m / config_.speed_of_light_mps;
  metrics.radial_velocity_mps = metrics.velocity_mps.dot(metrics.line_of_sight);
  const float lambda_m = config_.speed_of_light_mps / config_.carrier_hz;
  metrics.doppler_hz = 2.0f * metrics.radial_velocity_mps / lambda_m;
  return metrics;
}

void RadarSimulator::storeTxSample(Complex tx_sample) {
  tx_history_[sample_index_ & kTxHistoryMask] = tx_sample;
}

RadarSimulator::Complex RadarSimulator::delayedTxSample(float delay_s) const noexcept {
  const float delayed_index =
      static_cast<float>(sample_index_) - delay_s * config_.sample_rate_hz;
  if (delayed_index < 0.0f) {
    return Complex(0.0f, 0.0f);
  }

  const std::size_t lower_index =
      static_cast<std::size_t>(std::floor(delayed_index));
  const std::size_t upper_index = lower_index + 1U;
  if (sample_index_ - lower_index >= kTxHistorySize) {
    return Complex(0.0f, 0.0f);
  }

  const float fraction = delayed_index - static_cast<float>(lower_index);
  const Complex lower_sample = tx_history_[lower_index & kTxHistoryMask];
  Complex upper_sample = lower_sample;
  if (upper_index <= sample_index_ && sample_index_ - upper_index < kTxHistorySize) {
    upper_sample = tx_history_[upper_index & kTxHistoryMask];
  }

  return (1.0f - fraction) * lower_sample + fraction * upper_sample;
}

RadarSimulator::Complex RadarSimulator::sampleNoise() {
  if (config_.noise_stddev <= 0.0f) {
    return Complex(0.0f, 0.0f);
  }

  const float sigma = config_.noise_stddev * problem::kComplexNoiseQuadratureScale;
  return Complex(noise_dist_(noise_rng_) * sigma, noise_dist_(noise_rng_) * sigma);
}
