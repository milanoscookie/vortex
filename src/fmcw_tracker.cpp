#include "fmcw_tracker.h"

#include "dynamics.h"
#include "target_observation.h"

#include <unsupported/Eigen/FFT>

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <vector>

namespace fmcw_tracker {
namespace {

using Vec3 = problem::Vec3;

std::size_t computeChirpCount(const problem::ProblemDescription &description) {
    const float chirp_duration_s =
        static_cast<float>(problem::Constants::kRadarBlockSize) /
        description.radar.sample_rate_hz;
    return static_cast<std::size_t>(
        std::llround(description.simulator.burst_duration_s / chirp_duration_s));
}

RadarConfig makeRadarConfig(const problem::ProblemDescription &description) {
    const float chirp_duration_s =
        static_cast<float>(problem::Constants::kRadarBlockSize) /
        description.radar.sample_rate_hz;
    const float lambda_m =
        problem::Constants::kSpeedOfLightMps / description.radar.carrier_hz;

    RadarConfig config;
    config.sample_rate_hz = description.radar.sample_rate_hz;
    config.carrier_hz = description.radar.carrier_hz;
    config.bandwidth_hz = description.radar.bandwidth_hz;
    config.chirp_duration_s = chirp_duration_s;
    config.speed_of_light_mps = problem::Constants::kSpeedOfLightMps;
    config.block_size = problem::Constants::kRadarBlockSize;
    config.chirp_count = computeChirpCount(description);
    config.probe_num_x = problem::RadarSettings::kProbeNumX;
    config.probe_num_y = problem::RadarSettings::kProbeNumY;
    config.probe_dx_m = description.probe.spacing_x_wavelengths * lambda_m;
    config.probe_dy_m = description.probe.spacing_y_wavelengths * lambda_m;
    return config;
}

std::vector<float> linspace(float start, float stop, std::size_t count) {
    std::vector<float> values(count);
    if (count == 0) {
        return values;
    }
    if (count == 1) {
        values[0] = start;
        return values;
    }

    const float step = (stop - start) / static_cast<float>(count - 1);
    for (std::size_t i = 0; i < count; ++i) {
        values[i] = start + static_cast<float>(i) * step;
    }
    return values;
}

std::vector<float> hannWindow(std::size_t length) {
    std::vector<float> window(length, 1.0f);
    if (length <= 1) {
        return window;
    }

    for (std::size_t n = 0; n < length; ++n) {
        window[n] = 0.5f - 0.5f * std::cos(
                                   2.0f * problem::Constants::kPi *
                                   static_cast<float>(n) /
                                   static_cast<float>(length - 1));
    }
    return window;
}

std::vector<float> fftFreq(std::size_t nfft, float sample_period_s) {
    std::vector<float> freqs(nfft, 0.0f);
    const float scale = 1.0f / (static_cast<float>(nfft) * sample_period_s);
    const std::size_t half = nfft / 2;
    for (std::size_t k = 0; k < nfft; ++k) {
        if (k < half) {
            freqs[k] = static_cast<float>(k) * scale;
        } else {
            freqs[k] =
                static_cast<float>(static_cast<long long>(k) -
                                   static_cast<long long>(nfft)) *
                scale;
        }
    }
    return freqs;
}

template <typename T> void fftShiftInPlace(std::vector<T> &values) {
    if (values.empty()) {
        return;
    }
    std::rotate(values.begin(),
                values.begin() + static_cast<std::ptrdiff_t>(values.size() / 2),
                values.end());
}

std::size_t cubeIndex(std::size_t outer,
                      std::size_t middle,
                      std::size_t inner,
                      std::size_t middle_size,
                      std::size_t inner_size) {
    return (outer * middle_size + middle) * inner_size + inner;
}

std::vector<Vec3> elementPositions(const RadarConfig &cfg) {
    std::vector<Vec3> positions(cfg.numRx(), Vec3::Zero());
    for (std::size_t ix = 0; ix < cfg.probe_num_x; ++ix) {
        for (std::size_t iy = 0; iy < cfg.probe_num_y; ++iy) {
            const std::size_t flat = ix * cfg.probe_num_y + iy;
            positions[flat].x() =
                (static_cast<float>(ix) -
                 0.5f * static_cast<float>(cfg.probe_num_x - 1)) *
                cfg.probe_dx_m;
            positions[flat].y() =
                (static_cast<float>(iy) -
                 0.5f * static_cast<float>(cfg.probe_num_y - 1)) *
                cfg.probe_dy_m;
        }
    }
    return positions;
}

struct SteeringGrid {
    std::vector<Complex> steering_conj;
    std::vector<Vec3> directions;
};

SteeringGrid makeSteeringGrid(const RadarConfig &cfg, const DetectionConfig &det) {
    const std::vector<Vec3> positions_m = elementPositions(cfg);
    const std::vector<float> az_rad =
        linspace(det.azimuth_min_deg, det.azimuth_max_deg, det.azimuth_count);
    const std::vector<float> el_rad =
        linspace(det.elevation_min_deg, det.elevation_max_deg, det.elevation_count);

    SteeringGrid grid;
    grid.steering_conj.reserve(det.azimuth_count * det.elevation_count * cfg.numRx());
    grid.directions.reserve(det.azimuth_count * det.elevation_count);

    for (float az_deg : az_rad) {
        const float az = az_deg * problem::Constants::kPi / 180.0f;
        for (float el_deg : el_rad) {
            const float el = el_deg * problem::Constants::kPi / 180.0f;
            Vec3 direction;
            direction.x() = std::cos(el) * std::cos(az);
            direction.y() = std::cos(el) * std::sin(az);
            direction.z() = std::sin(el);
            grid.directions.push_back(direction);

            for (const Vec3 &position : positions_m) {
                const float phase =
                    (2.0f * problem::Constants::kPi / cfg.wavelengthM()) *
                    direction.dot(position);
                grid.steering_conj.emplace_back(std::cos(-phase), std::sin(-phase));
            }
        }
    }

    return grid;
}

std::vector<Complex> makeComplexWindow(const std::vector<float> &real_window) {
    std::vector<Complex> window(real_window.size(), Complex(0.0f, 0.0f));
    for (std::size_t i = 0; i < real_window.size(); ++i) {
        window[i] = Complex(real_window[i], 0.0f);
    }
    return window;
}

float degrees(float radians) {
    return radians * 180.0f / problem::Constants::kPi;
}

float azimuthDeg(const Vec3 &direction) {
    return degrees(std::atan2(direction.y(), direction.x()));
}

float elevationDeg(const Vec3 &direction) {
    return degrees(std::atan2(direction.z(), std::hypot(direction.x(), direction.y())));
}

float wrapAngleDeg(float angle_deg) {
    while (angle_deg > 180.0f) {
        angle_deg -= 360.0f;
    }
    while (angle_deg < -180.0f) {
        angle_deg += 360.0f;
    }
    return angle_deg;
}

float angleDifferenceDeg(float lhs_deg, float rhs_deg) {
    return wrapAngleDeg(lhs_deg - rhs_deg);
}

float associationPenalty(float error, float sigma) {
    if (sigma <= 0.0f) {
        return 0.0f;
    }
    const float normalized_error = error / sigma;
    return -0.5f * normalized_error * normalized_error;
}

struct PeakInterpResult {
    float delta = 0.0f;
    bool valid = true;
};

PeakInterpResult quadraticPeakOffsetChecked(float power_minus,
                                            float power_center,
                                            float power_plus) {
    constexpr float kLogEpsilon = 1.0e-12f;
    const float y_minus = std::log(power_minus + kLogEpsilon);
    const float y_center = std::log(power_center + kLogEpsilon);
    const float y_plus = std::log(power_plus + kLogEpsilon);
    const float denom = y_minus - 2.0f * y_center + y_plus;

    PeakInterpResult out{};

    if (!(denom < -1.0e-12f)) {
        out.delta = 0.0f;
        out.valid = false;
        return out;
    }

    const float raw_delta = 0.5f * (y_minus - y_plus) / denom;

    if (!std::isfinite(raw_delta) || std::abs(raw_delta) >= 0.45f) {
        out.delta = 0.0f;
        out.valid = false;
        return out;
    }

    out.delta = raw_delta;
    out.valid = true;
    return out;
}

float interpolateAxis(const std::vector<float> &axis,
                      std::size_t bin,
                      float offset) {
    if (axis.empty() || bin >= axis.size()) {
        return 0.0f;
    }
    if (offset > 0.0f && bin + 1 < axis.size()) {
        return axis[bin] + offset * (axis[bin + 1] - axis[bin]);
    }
    if (offset < 0.0f && bin > 0) {
        return axis[bin] + offset * (axis[bin] - axis[bin - 1]);
    }
    return axis[bin];
}

std::vector<float> unwrapPhases(const std::vector<float> &phases) {
    if (phases.empty()) {
        return {};
    }

    std::vector<float> unwrapped(phases.size(), 0.0f);
    unwrapped[0] = phases[0];
    float offset = 0.0f;
    for (std::size_t i = 1; i < phases.size(); ++i) {
        const float delta = phases[i] - phases[i - 1];
        if (delta > problem::Constants::kPi) {
            offset -= 2.0f * problem::Constants::kPi;
        } else if (delta < -problem::Constants::kPi) {
            offset += 2.0f * problem::Constants::kPi;
        }
        unwrapped[i] = phases[i] + offset;
    }
    return unwrapped;
}

std::vector<float> convolveSame(const std::vector<float> &signal,
                                const std::vector<float> &kernel) {
    if (signal.empty() || kernel.empty()) {
        return {};
    }

    std::vector<float> full(signal.size() + kernel.size() - 1, 0.0f);
    for (std::size_t i = 0; i < signal.size(); ++i) {
        for (std::size_t k = 0; k < kernel.size(); ++k) {
            full[i + k] += signal[i] * kernel[k];
        }
    }

    const std::size_t start = (kernel.size() - 1) / 2;
    return std::vector<float>(full.begin() + static_cast<std::ptrdiff_t>(start),
                              full.begin() + static_cast<std::ptrdiff_t>(start + signal.size()));
}

std::vector<float> gradient(const std::vector<float> &values,
                            const std::vector<float> &times_s) {
    const std::size_t count = values.size();
    std::vector<float> grad(count, 0.0f);
    if (count < 2) {
        return grad;
    }

    const float first_dt = times_s[1] - times_s[0];
    const float last_dt = times_s[count - 1] - times_s[count - 2];
    grad[0] = (values[1] - values[0]) / first_dt;
    grad[count - 1] = (values[count - 1] - values[count - 2]) / last_dt;

    for (std::size_t i = 1; i + 1 < count; ++i) {
        const float dt = times_s[i + 1] - times_s[i - 1];
        grad[i] = (values[i + 1] - values[i - 1]) / dt;
    }

    return grad;
}

} // namespace

StreamingTracker::StreamingTracker(const problem::ProblemDescription &description,
                                   DetectionConfig detection_config)
    : description_(description),
      radar_config_(makeRadarConfig(description)),
      detection_config_(detection_config) {
    if (description_.cars.size() != 1) {
        throw std::runtime_error("StreamingTracker currently supports exactly one car");
    }

    // Range axis: use only negative-frequency FFT bins (down-chirp produces
    // negative beat frequencies for targets). Map to range via
    // R = -c * f_beat / (2 * chirp_slope).
    const std::vector<float> freqs =
        fftFreq(detection_config_.nfft_range_min, 1.0f / radar_config_.sample_rate_hz);
    range_indices_.reserve(freqs.size());
    range_axis_sliced_m_.reserve(freqs.size());
    for (std::size_t i = 0; i < freqs.size(); ++i) {
        const float beat_frequency_hz = freqs[i];
        if (beat_frequency_hz > 0.0f) {
            continue;
        }

        const float range_m =
            -radar_config_.speed_of_light_mps * beat_frequency_hz /
            (2.0f * radar_config_.chirpSlopeHzPerS());
        if (range_m >= detection_config_.min_range_m &&
            range_m <= detection_config_.max_range_m) {
            range_indices_.push_back(i);
            range_axis_sliced_m_.push_back(range_m);
        }
    }
    if (range_indices_.empty()) {
        throw std::runtime_error("tracker range gate produced no bins");
    }

    range_window_ = makeComplexWindow(hannWindow(radar_config_.block_size));
    doppler_window_ = makeComplexWindow(
        hannWindow(detection_config_.coherent_processing_interval_chirps));

    const SteeringGrid grid = makeSteeringGrid(radar_config_, detection_config_);
    steering_conj_ = grid.steering_conj;
    directions_ = grid.directions;

    // Doppler axis: zero-pad to 2x CPI for finer frequency resolution
    // (~305 Hz/bin instead of ~610 Hz/bin). The truth model uses
    // f_D = 2*v_r/lambda (positive = approaching). The FFT on the beat-signal
    // slow-time produces the opposite sign, so we negate the axis.
    const std::size_t nfft_doppler =
        2U * detection_config_.coherent_processing_interval_chirps;
    doppler_axis_hz_ = fftFreq(nfft_doppler, radar_config_.chirp_duration_s);
    fftShiftInPlace(doppler_axis_hz_);
    for (float &doppler_hz : doppler_axis_hz_) {
        doppler_hz = -doppler_hz;
    }

    velocity_axis_mps_.resize(doppler_axis_hz_.size(), 0.0f);
    for (std::size_t i = 0; i < doppler_axis_hz_.size(); ++i) {
        velocity_axis_mps_[i] = 0.5f * doppler_axis_hz_[i] * radar_config_.wavelengthM();
    }

    // The simulator keeps a continuous TX history, so the first fast-time
    // samples of a chirp can include delayed energy from the previous chirp.
    // Blanking that prefix avoids wrap-around contamination. Cap the guard
    // to a fraction of the block so extreme-range configs don't starve the
    // range FFT.
    const float max_tracked_range_m =
        std::min(description_.radar.max_range_m, detection_config_.max_range_m);
    const float max_delay_samples =
        (2.0f * max_tracked_range_m / radar_config_.speed_of_light_mps) *
        radar_config_.sample_rate_hz;
    range_wrap_guard_samples_ = std::min(
        radar_config_.block_size / 8U,
        static_cast<std::size_t>(std::ceil(max_delay_samples)) + 2U);
}

void StreamingTracker::pushChirp(std::size_t chirp_index,
                                 std::span<const Complex> tx_chirp,
                                 std::span<const Complex> rx_block) {
    if (tx_conj_.empty()) {
        tx_conj_.resize(tx_chirp.size(), Complex(0.0f, 0.0f));
        for (std::size_t i = 0; i < tx_chirp.size(); ++i) {
            tx_conj_[i] = std::conj(tx_chirp[i]);
        }
    }

    if (tx_chirp.size() != radar_config_.block_size) {
        throw std::runtime_error("unexpected TX chirp size");
    }
    if (rx_block.size() != radar_config_.block_size * radar_config_.numRx()) {
        throw std::runtime_error("unexpected RX block size");
    }

    chirp_window_.emplace_back(rx_block.begin(), rx_block.end());
    if (chirp_window_.size() > detection_config_.coherent_processing_interval_chirps) {
        chirp_window_.pop_front();
    }

    const std::size_t received_chirps = chirp_index + 1;
    const std::size_t cpi = detection_config_.coherent_processing_interval_chirps;
    if (received_chirps < cpi) {
        return;
    }

    const std::size_t start_chirp = received_chirps - cpi;
    if (start_chirp % detection_config_.hop_chirps != 0) {
        return;
    }

    BatchResult result = processCurrentWindow(start_chirp);

    if (!result.valid) {
        if (tracking_state_.initialized) {
            const float dt = static_cast<float>(detection_config_.hop_chirps) *
                             radar_config_.chirp_duration_s;
            tracking_state_.range_m += tracking_state_.radial_velocity_mps * dt;

            result.time_s =
                (static_cast<float>(start_chirp) +
                 0.5f * static_cast<float>(detection_config_.coherent_processing_interval_chirps)) *
                radar_config_.chirp_duration_s;
            result.range_m = tracking_state_.range_m;
            result.doppler_hz = tracking_state_.doppler_hz;
            result.direction = tracking_state_.direction;
            result.predicted_range_m = tracking_state_.range_m;
            result.predicted_doppler_hz = tracking_state_.doppler_hz;
            result.predicted_direction = tracking_state_.direction;
        }
        batch_results_.push_back(result);
        return;
    }

    const float measured_radial_velocity_mps =
        0.5f * result.doppler_hz * radar_config_.wavelengthM();

    if (!tracking_state_.initialized) {
        tracking_state_.initialized = true;
        tracking_state_.range_m = result.range_m;
        tracking_state_.radial_velocity_mps = measured_radial_velocity_mps;
        tracking_state_.doppler_hz = result.doppler_hz;
        tracking_state_.direction = result.direction;
    } else {
        const float dt = static_cast<float>(detection_config_.hop_chirps) *
                         radar_config_.chirp_duration_s;
        const float pred_range =
            tracking_state_.range_m + tracking_state_.radial_velocity_mps * dt;
        tracking_state_.range_m =
            pred_range + 0.25f * (result.range_m - pred_range);
        tracking_state_.radial_velocity_mps =
            0.65f * tracking_state_.radial_velocity_mps +
            0.35f * measured_radial_velocity_mps;
        tracking_state_.doppler_hz =
            2.0f * tracking_state_.radial_velocity_mps / radar_config_.wavelengthM();
        tracking_state_.direction =
            (0.9f * tracking_state_.direction + 0.1f * result.direction).normalized();
    }

    batch_results_.push_back(result);
}

BatchResult StreamingTracker::processCurrentWindow(std::size_t start_chirp) {
    const std::size_t n_chirps = detection_config_.coherent_processing_interval_chirps;
    const std::size_t nfft_doppler = n_chirps * 2U;
    const std::size_t num_rx = radar_config_.numRx();
    const std::size_t nfft = detection_config_.nfft_range_min;
    const std::size_t range_count = range_indices_.size();

    // Range FFT: dechirp, window, and transform each chirp.
    std::vector<Complex> spec(n_chirps * range_count * num_rx, Complex(0.0f, 0.0f));
    Eigen::FFT<float> fft;
    std::vector<Complex> fft_input(nfft, Complex(0.0f, 0.0f));
    std::vector<Complex> fft_output;

    for (std::size_t chirp = 0; chirp < n_chirps; ++chirp) {
        const std::vector<Complex> &rx_block = chirp_window_[chirp];
        for (std::size_t rx = 0; rx < num_rx; ++rx) {
            std::fill(fft_input.begin(), fft_input.end(), Complex(0.0f, 0.0f));
            for (std::size_t sample = range_wrap_guard_samples_;
                 sample < radar_config_.block_size; ++sample) {
                const Complex beat =
                    rx_block[sample * num_rx + rx] * tx_conj_[sample];
                fft_input[sample] = beat * range_window_[sample];
            }

            fft.fwd(fft_output, fft_input);
            for (std::size_t r = 0; r < range_count; ++r) {
                spec[cubeIndex(chirp, r, rx, range_count, num_rx)] =
                    fft_output[range_indices_[r]];
            }
        }
    }

    // Static-clutter suppression: remove DC (mean) across slow-time.
    if (detection_config_.static_clutter_suppression_enable) {
        for (std::size_t r = 0; r < range_count; ++r) {
            for (std::size_t rx = 0; rx < num_rx; ++rx) {
                Complex mean(0.0f, 0.0f);
                for (std::size_t chirp = 0; chirp < n_chirps; ++chirp) {
                    mean += spec[cubeIndex(chirp, r, rx, range_count, num_rx)];
                }
                mean /= static_cast<float>(n_chirps);
                for (std::size_t chirp = 0; chirp < n_chirps; ++chirp) {
                    spec[cubeIndex(chirp, r, rx, range_count, num_rx)] -= mean;
                }
            }
        }
    }

    // Doppler FFT: zero-padded to 2x CPI for finer Doppler resolution.
    std::vector<Complex> rd_cube(nfft_doppler * range_count * num_rx, Complex(0.0f, 0.0f));
    std::vector<float> rd_power(nfft_doppler * range_count, 0.0f);
    std::vector<Complex> doppler_input(nfft_doppler, Complex(0.0f, 0.0f));
    std::vector<Complex> doppler_output;

    for (std::size_t r = 0; r < range_count; ++r) {
        for (std::size_t rx = 0; rx < num_rx; ++rx) {
            for (std::size_t chirp = 0; chirp < n_chirps; ++chirp) {
                doppler_input[chirp] =
                    spec[cubeIndex(chirp, r, rx, range_count, num_rx)] *
                    doppler_window_[chirp];
            }
            // Remaining doppler_input entries are zero (zero-padding).

            fft.fwd(doppler_output, doppler_input);
            fftShiftInPlace(doppler_output);

            for (std::size_t dbin = 0; dbin < nfft_doppler; ++dbin) {
                const Complex value = doppler_output[dbin];
                rd_cube[cubeIndex(dbin, r, rx, range_count, num_rx)] = value;
                rd_power[dbin * range_count + r] += std::norm(value);
            }
        }
    }

    for (float &power : rd_power) {
        power /= static_cast<float>(num_rx);
    }

    // Candidate scoring over the full RD plane.
    struct CandidateScore {
        std::size_t doppler_bin = 0;
        std::size_t range_bin = 0;
        float range_m = 0.0f;
        float doppler_hz = 0.0f;
        float score = std::numeric_limits<float>::lowest();
    };

    constexpr float kScoreEpsilon = 1.0e-12f;
    constexpr std::size_t kMaxRangeDopplerCandidates = 6;

    const std::size_t zero_doppler_bin = nfft_doppler / 2;
    float predicted_range_m = 0.0f;
    float predicted_doppler_hz = 0.0f;
    Vec3 predicted_direction = Vec3::UnitX();
    const float range_bin_spacing_m =
        range_axis_sliced_m_.size() > 1
            ? std::abs(range_axis_sliced_m_[1] - range_axis_sliced_m_[0])
            : 0.0f;
    const float doppler_bin_spacing_hz =
        doppler_axis_hz_.size() > 1 ? std::abs(doppler_axis_hz_[1] - doppler_axis_hz_[0])
                                    : 0.0f;

    if (tracking_state_.initialized) {
        const float dt = static_cast<float>(detection_config_.hop_chirps) *
                         radar_config_.chirp_duration_s;
        predicted_range_m =
            tracking_state_.range_m + tracking_state_.radial_velocity_mps * dt;
        predicted_doppler_hz = tracking_state_.doppler_hz;
        predicted_direction = tracking_state_.direction;
    }

    // Gate the search to a region around the prediction when initialized.
    const std::size_t range_gate =
        tracking_state_.initialized
            ? static_cast<std::size_t>(std::max(
                  static_cast<float>(detection_config_.range_gate_bins),
                  std::ceil(3.0f * detection_config_.range_association_sigma_m /
                            std::max(range_bin_spacing_m, 1.0e-6f))))
            : range_count;
    const std::size_t doppler_gate =
        tracking_state_.initialized
            ? static_cast<std::size_t>(std::max(
                  static_cast<float>(detection_config_.doppler_gate_bins),
                  std::ceil(3.0f * detection_config_.doppler_association_sigma_hz /
                            std::max(doppler_bin_spacing_hz, 1.0e-6f))))
            : nfft_doppler;

    std::size_t center_rbin = range_count / 2;
    std::size_t center_dbin = nfft_doppler / 2;
    if (tracking_state_.initialized) {
        auto it = std::lower_bound(range_axis_sliced_m_.begin(),
                                   range_axis_sliced_m_.end(),
                                   predicted_range_m);
        center_rbin = static_cast<std::size_t>(
            std::distance(range_axis_sliced_m_.begin(), it));
        center_rbin = std::min(center_rbin, range_count - 1);

        for (std::size_t db = 0; db < nfft_doppler; ++db) {
            if (doppler_axis_hz_[db] >= predicted_doppler_hz) {
                center_dbin = db;
                break;
            }
        }
    }

    std::vector<CandidateScore> rd_candidates;
    rd_candidates.reserve(
        std::min(doppler_gate, nfft_doppler) * std::min(range_gate, range_count));

    const std::size_t r_start = (center_rbin > range_gate / 2) ? center_rbin - range_gate / 2 : 0;
    const std::size_t r_end = std::min(range_count, center_rbin + range_gate / 2 + 1);
    const std::size_t d_start = (center_dbin > doppler_gate / 2) ? center_dbin - doppler_gate / 2 : 0;
    const std::size_t d_end = std::min(nfft_doppler, center_dbin + doppler_gate / 2 + 1);

    for (std::size_t dbin = d_start; dbin < d_end; ++dbin) {
        if (detection_config_.zero_doppler_guard_bins > 0) {
            const std::size_t low =
                zero_doppler_bin > detection_config_.zero_doppler_guard_bins
                    ? zero_doppler_bin - detection_config_.zero_doppler_guard_bins
                    : 0;
            const std::size_t high =
                std::min(nfft_doppler - 1,
                         zero_doppler_bin + detection_config_.zero_doppler_guard_bins);
            if (dbin >= low && dbin <= high) {
                continue;
            }
        }

        const float candidate_doppler_hz = doppler_axis_hz_[dbin];
        const float range_correction =
            (candidate_doppler_hz / radar_config_.chirpSlopeHzPerS()) *
            (radar_config_.speed_of_light_mps / 2.0f);

        for (std::size_t rbin = r_start; rbin < r_end; ++rbin) {
            const float power = rd_power[dbin * range_count + rbin];
            const float candidate_range_m = range_axis_sliced_m_[rbin] - range_correction;
            float score = std::log(power + kScoreEpsilon);
            if (tracking_state_.initialized) {
                score += associationPenalty(candidate_range_m - predicted_range_m,
                                            detection_config_.range_association_sigma_m);
                score += associationPenalty(candidate_doppler_hz - predicted_doppler_hz,
                                            detection_config_.doppler_association_sigma_hz);
            }

            CandidateScore candidate;
            candidate.doppler_bin = dbin;
            candidate.range_bin = rbin;
            candidate.range_m = candidate_range_m;
            candidate.doppler_hz = candidate_doppler_hz;
            candidate.score = score;
            rd_candidates.push_back(candidate);
        }
    }

    if (rd_candidates.empty()) {
        throw std::runtime_error("tracker failed to find a valid range-doppler peak");
    }

    const std::size_t candidate_count =
        std::min(kMaxRangeDopplerCandidates, rd_candidates.size());
    std::partial_sort(rd_candidates.begin(),
                      rd_candidates.begin() + candidate_count,
                      rd_candidates.end(),
                      [](const CandidateScore &lhs, const CandidateScore &rhs) {
                          return lhs.score > rhs.score;
                      });

    // Select best candidate and run AoA beamforming.
    float best_total_score = std::numeric_limits<float>::lowest();
    std::size_t best_doppler_bin = rd_candidates.front().doppler_bin;
    std::size_t best_range_bin = rd_candidates.front().range_bin;
    std::size_t best_direction_index = 0;
    float best_range_m = rd_candidates.front().range_m;
    float best_doppler_hz = rd_candidates.front().doppler_hz;
    std::vector<Complex> best_snapshot(num_rx, Complex(0.0f, 0.0f));
    Vec3 best_direction = tracking_state_.initialized ? tracking_state_.direction : Vec3::UnitX();

    const float predicted_azimuth_deg = azimuthDeg(predicted_direction);
    const float predicted_elevation_deg = elevationDeg(predicted_direction);

    for (std::size_t candidate_index = 0; candidate_index < candidate_count;
         ++candidate_index) {
        const CandidateScore &candidate = rd_candidates[candidate_index];
        std::vector<Complex> snapshot(num_rx, Complex(0.0f, 0.0f));
        for (std::size_t rx = 0; rx < num_rx; ++rx) {
            snapshot[rx] = rd_cube[cubeIndex(candidate.doppler_bin,
                                             candidate.range_bin,
                                             rx,
                                             range_count,
                                             num_rx)];
        }

        Vec3 candidate_direction = Vec3::UnitX();
        std::size_t candidate_direction_index = 0;
        float best_direction_score = detection_config_.aoa_enable
                                         ? std::numeric_limits<float>::lowest()
                                         : 0.0f;

        if (detection_config_.aoa_enable) {
            const std::size_t azimuth_count = detection_config_.azimuth_count;
            const std::size_t elevation_count = detection_config_.elevation_count;
            const std::size_t coarse_elevation_stride =
                std::max<std::size_t>(1U, elevation_count / 91U);

            auto evaluate_direction = [&](std::size_t direction_index) {
                Complex response(0.0f, 0.0f);
                const std::size_t base = direction_index * num_rx;
                for (std::size_t rx = 0; rx < num_rx; ++rx) {
                    response += steering_conj_[base + rx] * snapshot[rx];
                }

                float direction_score =
                    std::log(std::norm(response) + kScoreEpsilon);
                if (tracking_state_.initialized) {
                    direction_score +=
                        associationPenalty(angleDifferenceDeg(
                                             azimuthDeg(directions_[direction_index]),
                                             predicted_azimuth_deg),
                                           detection_config_.azimuth_association_sigma_deg);
                    direction_score +=
                        associationPenalty(elevationDeg(directions_[direction_index]) -
                                               predicted_elevation_deg,
                                           detection_config_.elevation_association_sigma_deg);
                }

                if (direction_score > best_direction_score) {
                    best_direction_score = direction_score;
                    candidate_direction = directions_[direction_index];
                    candidate_direction_index = direction_index;
                }
            };

            for (std::size_t azimuth_index = 0; azimuth_index < azimuth_count; ++azimuth_index) {
                std::size_t best_coarse_elevation_index = 0U;
                float best_coarse_score = std::numeric_limits<float>::lowest();

                for (std::size_t elevation_index = 0;
                     elevation_index < elevation_count;
                     elevation_index += coarse_elevation_stride) {
                    const std::size_t direction_index =
                        azimuth_index * elevation_count + elevation_index;

                    Complex response(0.0f, 0.0f);
                    const std::size_t base = direction_index * num_rx;
                    for (std::size_t rx = 0; rx < num_rx; ++rx) {
                        response += steering_conj_[base + rx] * snapshot[rx];
                    }

                    float direction_score =
                        std::log(std::norm(response) + kScoreEpsilon);
                    if (tracking_state_.initialized) {
                        direction_score +=
                            associationPenalty(angleDifferenceDeg(
                                                 azimuthDeg(directions_[direction_index]),
                                                 predicted_azimuth_deg),
                                               detection_config_.azimuth_association_sigma_deg);
                        direction_score +=
                            associationPenalty(elevationDeg(directions_[direction_index]) -
                                                   predicted_elevation_deg,
                                               detection_config_.elevation_association_sigma_deg);
                    }

                    if (direction_score > best_coarse_score) {
                        best_coarse_score = direction_score;
                        best_coarse_elevation_index = elevation_index;
                    }
                }

                const std::size_t refine_start =
                    best_coarse_elevation_index > coarse_elevation_stride
                        ? best_coarse_elevation_index - coarse_elevation_stride
                        : 0U;
                const std::size_t refine_stop = std::min(
                    elevation_count,
                    best_coarse_elevation_index + coarse_elevation_stride + 1U);

                for (std::size_t elevation_index = refine_start;
                     elevation_index < refine_stop;
                     ++elevation_index) {
                    const std::size_t direction_index =
                        azimuth_index * elevation_count + elevation_index;
                    evaluate_direction(direction_index);
                }
            }
        }

        const float total_score = candidate.score + best_direction_score;
        if (total_score > best_total_score) {
            best_total_score = total_score;
            best_doppler_bin = candidate.doppler_bin;
            best_range_bin = candidate.range_bin;
            best_direction_index = candidate_direction_index;
            best_range_m = candidate.range_m;
            best_doppler_hz = candidate.doppler_hz;
            best_direction = candidate_direction;
            best_snapshot = snapshot;
        }
    }

    // Sub-bin interpolation for Doppler.
    float doppler_bin_offset = 0.0f;
    if (best_doppler_bin > 0 && best_doppler_bin + 1 < nfft_doppler) {
        auto d_interp = quadraticPeakOffsetChecked(
            rd_power[(best_doppler_bin - 1) * range_count + best_range_bin],
            rd_power[best_doppler_bin * range_count + best_range_bin],
            rd_power[(best_doppler_bin + 1) * range_count + best_range_bin]);

        float doppler_delta_raw = d_interp.valid ? d_interp.delta : 0.0f;
        float doppler_hz_raw =
            interpolateAxis(doppler_axis_hz_, best_doppler_bin, doppler_delta_raw);

        bool bad_doppler_interp = false;
        if (tracking_state_.initialized) {
            float doppler_jump_hz =
                std::abs(doppler_hz_raw - predicted_doppler_hz);
            bad_doppler_interp =
                doppler_jump_hz > detection_config_.doppler_interp_gate_hz;
        }

        doppler_bin_offset = bad_doppler_interp ? 0.0f : doppler_delta_raw;
    }

    // Sub-bin interpolation for range.
    float range_bin_offset = 0.0f;
    if (best_range_bin > 0 && best_range_bin + 1 < range_count) {
        auto r_interp = quadraticPeakOffsetChecked(
            rd_power[best_doppler_bin * range_count + (best_range_bin - 1)],
            rd_power[best_doppler_bin * range_count + best_range_bin],
            rd_power[best_doppler_bin * range_count + (best_range_bin + 1)]);

        float range_offset_raw = r_interp.valid ? r_interp.delta : 0.0f;

        bool bad_range_interp = false;
        if (tracking_state_.initialized) {
            float refined_range_candidate =
                interpolateAxis(range_axis_sliced_m_, best_range_bin, range_offset_raw);
            const float candidate_doppler_hz = doppler_axis_hz_[best_doppler_bin];
            const float range_correction =
                (candidate_doppler_hz / radar_config_.chirpSlopeHzPerS()) *
                (radar_config_.speed_of_light_mps / 2.0f);
            refined_range_candidate -= range_correction;
            float range_jump_m =
                std::abs(refined_range_candidate - predicted_range_m);
            bad_range_interp =
                range_jump_m > detection_config_.range_interp_gate_m;
        }

        range_bin_offset = bad_range_interp ? 0.0f : range_offset_raw;
    }

    best_doppler_hz = interpolateAxis(doppler_axis_hz_, best_doppler_bin, doppler_bin_offset);
    const float refined_raw_range_m =
        interpolateAxis(range_axis_sliced_m_, best_range_bin, range_bin_offset);
    const float refined_range_correction =
        (best_doppler_hz / radar_config_.chirpSlopeHzPerS()) *
        (radar_config_.speed_of_light_mps / 2.0f);
    best_range_m = refined_raw_range_m - refined_range_correction;

    // Measurement gating.
    const float max_range_jump_m =
        std::max(detection_config_.range_association_sigma_m,
                 (static_cast<float>(detection_config_.range_gate_bins) + 1.0f) *
                     range_bin_spacing_m);
    const float max_doppler_jump_hz =
        std::max(detection_config_.doppler_association_sigma_hz,
                 (static_cast<float>(detection_config_.doppler_gate_bins) + 1.5f) *
                     doppler_bin_spacing_hz);

    bool range_ok = !tracking_state_.initialized ||
                    std::abs(best_range_m - predicted_range_m) < max_range_jump_m;
    bool doppler_ok = !tracking_state_.initialized ||
                      std::abs(best_doppler_hz - predicted_doppler_hz) < max_doppler_jump_hz;

    BatchResult result;
    result.time_s =
        (static_cast<float>(start_chirp) + 0.5f * static_cast<float>(n_chirps)) *
        radar_config_.chirp_duration_s;
    result.predicted_range_m = predicted_range_m;
    result.predicted_doppler_hz = predicted_doppler_hz;
    result.predicted_direction = predicted_direction;

    if (!std::isfinite(best_range_m) || !std::isfinite(best_doppler_hz) ||
        !range_ok || !doppler_ok) {
        result.valid = false;
        return result;
    }

    result.valid = true;
    result.range_m = best_range_m;
    result.doppler_hz = best_doppler_hz;
    result.phase_rad = std::arg(best_snapshot[0]);
    result.range_bin_offset = range_bin_offset;
    result.doppler_bin_offset = doppler_bin_offset;
    result.direction = best_direction;
    result.range_bin = best_range_bin;
    result.doppler_bin = best_doppler_bin;
    result.azimuth_bin =
        best_direction_index / detection_config_.elevation_count;
    result.elevation_bin =
        best_direction_index % detection_config_.elevation_count;
    result.doppler_slice_power.resize(nfft_doppler, 0.0f);
    for (std::size_t dbin = 0; dbin < nfft_doppler; ++dbin) {
        result.doppler_slice_power[dbin] =
            rd_power[dbin * range_count + best_range_bin];
    }
    return result;
}

TrackSummary StreamingTracker::buildSummary() const {
    TrackSummary summary;
    summary.batch_results = batch_results_;
    summary.velocity_axis_mps = velocity_axis_mps_;

    if (batch_results_.empty()) {
        return summary;
    }

    const float wavelength_m = radar_config_.wavelengthM();

    summary.times_s.reserve(batch_results_.size());
    summary.raw_positions_m.reserve(batch_results_.size());
    summary.smoothed_positions_m.reserve(batch_results_.size());
    summary.ranges_m.reserve(batch_results_.size());
    summary.radial_velocity_mps.reserve(batch_results_.size());
    summary.unwrapped_phase_rad.reserve(batch_results_.size());
    summary.cartesian_velocity_mps.reserve(batch_results_.size());
    summary.truth_metrics.reserve(batch_results_.size());

    std::vector<float> phases;
    std::vector<bool> measurement_valid;
    phases.reserve(batch_results_.size());
    measurement_valid.reserve(batch_results_.size());

    std::vector<float> raw_x(batch_results_.size());
    std::vector<float> raw_y(batch_results_.size());
    std::vector<float> raw_z(batch_results_.size());

    for (std::size_t i = 0; i < batch_results_.size(); ++i) {
        const BatchResult &batch = batch_results_[i];
        summary.times_s.push_back(batch.time_s);
        const Vec3 raw_xyz = batch.range_m * batch.direction;
        summary.raw_positions_m.push_back(raw_xyz);
        summary.ranges_m.push_back(batch.range_m);
        summary.radial_velocity_mps.push_back(
            0.5f * batch.doppler_hz * wavelength_m);
        phases.push_back(batch.phase_rad);
        measurement_valid.push_back(batch.valid);
        raw_x[i] = raw_xyz.x();
        raw_y[i] = raw_xyz.y();
        raw_z[i] = raw_xyz.z();
        summary.truth_metrics.push_back(truthAtTime(description_, batch.time_s));
    }

    summary.unwrapped_phase_rad = unwrapPhases(phases);

    const std::size_t window_len =
        std::min<std::size_t>(5, batch_results_.size());
    const std::vector<float> boxcar(window_len,
                                    1.0f / static_cast<float>(window_len));

    const std::vector<float> smooth_x = convolveSame(raw_x, boxcar);
    const std::vector<float> smooth_y = convolveSame(raw_y, boxcar);
    const std::vector<float> smooth_z = convolveSame(raw_z, boxcar);

    for (std::size_t i = 0; i < batch_results_.size(); ++i) {
        summary.smoothed_positions_m.emplace_back(smooth_x[i],
                                                  smooth_y[i],
                                                  smooth_z[i]);
    }

    const std::vector<float> vx = gradient(smooth_x, summary.times_s);
    const std::vector<float> vy = gradient(smooth_y, summary.times_s);
    const std::vector<float> vz = gradient(smooth_z, summary.times_s);

    for (std::size_t i = 0; i < batch_results_.size(); ++i) {
        summary.cartesian_velocity_mps.emplace_back(vx[i], vy[i], vz[i]);
    }

    return summary;
}

problem::SimulationMetrics truthAtTime(const problem::ProblemDescription &description,
                                       float time_s) {
    if (description.cars.empty()) {
        throw std::runtime_error("truthAtTime requires at least one car");
    }

    const CarDynamics dynamics(description.cars.front());
    const radar::TargetObservation observation =
        radar::observeTarget(dynamics, description.radar, time_s);
    return radar::makeSimulationMetrics(time_s, observation);
}

} // namespace fmcw_tracker
