#include "radar.h"

#include "probe.h"
#include "simulation.h"

#include <cmath>
#include <memory>
#include <stdexcept>

namespace radar {
namespace {

dsp::Complex cis(double phase_rad) {
    return {std::cos(phase_rad), std::sin(phase_rad)};
}

TrackerResult toTrackerResult(const dsp::SimulationMetrics &metrics, bool valid) {
    return TrackerResult{.time_s = metrics.time_s,
                         .x = metrics.position_m.x(),
                         .y = metrics.position_m.y(),
                         .z = metrics.position_m.z(),
                         .vx = metrics.velocity_mps.x(),
                         .vy = metrics.velocity_mps.y(),
                         .vz = metrics.velocity_mps.z(),
                         .valid = valid};
}

} // namespace

struct Session::Impl {
    explicit Impl(const dsp::ProblemDescription &description)
        : Impl(description,
               Probe(description.probe.center_m,
                     description.probe.spacing_x_wavelengths *
                         (dsp::Constants::kSpeedOfLightMps / description.radar.carrier_hz),
                     description.probe.spacing_y_wavelengths *
                         (dsp::Constants::kSpeedOfLightMps / description.radar.carrier_hz))) {}

    Impl(const dsp::ProblemDescription &description, const Probe &probe)
        : description(description), simulator(description), probe(probe) {}

    void reset() {
        simulator = RadarSimulator(description);
        rx_vector.setZero();
        latest_truth = TrackerResult{};
        chirp_index = 0;
        initialized = true;

        const double chirp_duration_s = static_cast<double>(dsp::RadarSettings::kRadarBlockSize) /
                                        description.radar.sample_rate_hz;
        const double chirp_slope_hz_per_s = description.radar.bandwidth_hz / chirp_duration_s;
        const double sample_period_s = 1.0 / description.radar.sample_rate_hz;
        const double a =
            dsp::Constants::kPi * chirp_slope_hz_per_s * sample_period_s * sample_period_s;
        const double b = -dsp::Constants::kPi * description.radar.bandwidth_hz * sample_period_s;

        tx_sample = dsp::Complex(1.0, 0.0);
        tx_phase_step = cis(a + b);
        tx_phase_acceleration = cis(2.0 * a);
    }

    dsp::Complex nextTxSample() {
        const dsp::Complex sample = tx_sample;
        tx_sample *= tx_phase_step;
        tx_phase_step *= tx_phase_acceleration;
        return sample;
    }

    dsp::ProblemDescription description;
    RadarSimulator simulator;
    Probe probe;
    RxVector rx_vector = RxVector::Zero();
    TrackerResult latest_truth;
    std::size_t chirp_index = 0;
    bool initialized = false;
    dsp::Complex tx_sample = dsp::Complex(1.0, 0.0);
    dsp::Complex tx_phase_step = dsp::Complex(1.0, 0.0);
    dsp::Complex tx_phase_acceleration = dsp::Complex(1.0, 0.0);
};

Session::Session(const dsp::ProblemDescription &description)
    : impl_(std::make_unique<Impl>(description)) {}

Session::Session(const dsp::ProblemDescription &description, const Probe &probe)
    : impl_(std::make_unique<Impl>(description, probe)) {}

Session::~Session() = default;
Session::Session(Session &&) noexcept = default;
Session &Session::operator=(Session &&) noexcept = default;

void Session::init() {
    impl_->reset();
}

SignalBlock Session::nextChirp() {
    if (!impl_->initialized) {
        throw std::runtime_error("radar::Session::init must be called before nextChirp()");
    }

    SignalBlock block;
    block.start_time_s = impl_->simulator.timeSeconds();
    block.chirp_index = impl_->chirp_index++;

    for (std::size_t sample_index = 0; sample_index < dsp::RadarSettings::kRadarBlockSize;
         ++sample_index) {
        const dsp::Complex tx = impl_->nextTxSample();
        impl_->simulator.step(impl_->probe, impl_->rx_vector, tx);
        
        block.tx_ref[sample_index] = tx;
        block.rx_data[sample_index] = impl_->rx_vector;
    }

    impl_->latest_truth = toTrackerResult(impl_->simulator.lastMetrics(), true);
    return block;
}

TrackerResult Session::truth() const {
    return impl_->latest_truth;
}

const Probe& Session::probe() const {
    return impl_->probe;
}

} // namespace radar
