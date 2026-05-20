#pragma once

#include "problem_description.h"
#include "probe.h"

#include <Eigen/Dense>
#include <array>
#include <memory>
#include <vector>

namespace radar {

using Probe = ::Probe<dsp::RadarSettings::kProbeNumX, dsp::RadarSettings::kProbeNumY>;
using RxVector = Eigen::Matrix<dsp::Complex, static_cast<int>(dsp::RadarSettings::kProbeNumElements), 1>;

struct TrackerResult {
    double time_s = 0.0;
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
    double vx = 0.0;
    double vy = 0.0;
    double vz = 0.0;
    bool valid = false;
};

// Option C: Coherent Reference Provided
// A block represents one full chirp/block of radar time.
struct SignalBlock {
    double start_time_s = 0.0;
    std::size_t chirp_index = 0;
    
    // Coherent TX reference for de-chirping (mixing)
    std::array<dsp::Complex, dsp::RadarSettings::kRadarBlockSize> tx_ref;
    
    // Multi-channel RX data
    std::array<RxVector, dsp::RadarSettings::kRadarBlockSize> rx_data;
};

class Session {
  public:
    explicit Session(const dsp::ProblemDescription &description);
    Session(const dsp::ProblemDescription &description, const Probe &probe);
    ~Session();

    Session(Session &&) noexcept;
    Session &operator=(Session &&) noexcept;

    Session(const Session &) = delete;
    Session &operator=(const Session &) = delete;

    void init();

    // Returns a coherent block of TX and RX data.
    // The student is expected to mix these (rx * conj(tx)) to get the beat signal.
    SignalBlock nextChirp();

    // Get ground truth state for the last simulation step.
    TrackerResult truth() const;

    // Get the physical probe geometry used by the session.
    const Probe& probe() const;

  private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

} // namespace radar
