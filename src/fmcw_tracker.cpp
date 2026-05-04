#include "fmcw_tracker.h"

#include "dynamics.h"
#include "target_observation.h"

#include <unsupported/Eigen/FFT>

#include <algorithm>
#include <cmath>
#include <numeric>
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

    const std::vector<float> freqs =
        fftFreq(detection_config_.nfft_range_min, 1.0f / radar_config_.sample_rate_hz);
    range_indices_.reserve(freqs.size());
    range_axis_sliced_m_.reserve(freqs.size());
    for (std::size_t i = 0; i < freqs.size(); ++i) {
        const float range_m =
            radar_config_.speed_of_light_mps * std::abs(freqs[i]) /
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

    doppler_axis_hz_ = fftFreq(detection_config_.coherent_processing_interval_chirps,
                               radar_config_.chirp_duration_s);
    fftShiftInPlace(doppler_axis_hz_);

    velocity_axis_mps_.resize(doppler_axis_hz_.size(), 0.0f);
    for (std::size_t i = 0; i < doppler_axis_hz_.size(); ++i) {
        velocity_axis_mps_[i] = 0.5f * doppler_axis_hz_[i] * radar_config_.wavelengthM();
    }
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

    batch_results_.push_back(processCurrentWindow(start_chirp));
}

BatchResult StreamingTracker::processCurrentWindow(std::size_t start_chirp) const {
    const std::size_t n_chirps = detection_config_.coherent_processing_interval_chirps;
    const std::size_t num_rx = radar_config_.numRx();
    const std::size_t nfft = detection_config_.nfft_range_min;
    const std::size_t range_count = range_indices_.size();

    std::vector<Complex> spec(n_chirps * range_count * num_rx, Complex(0.0f, 0.0f));
    Eigen::FFT<float> fft;
    std::vector<Complex> fft_input(nfft, Complex(0.0f, 0.0f));
    std::vector<Complex> fft_output;

    for (std::size_t chirp = 0; chirp < n_chirps; ++chirp) {
        const std::vector<Complex> &rx_block = chirp_window_[chirp];
        for (std::size_t rx = 0; rx < num_rx; ++rx) {
            std::fill(fft_input.begin(), fft_input.end(), Complex(0.0f, 0.0f));
            for (std::size_t sample = 0; sample < radar_config_.block_size; ++sample) {
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

    std::vector<Complex> rd_cube(n_chirps * range_count * num_rx, Complex(0.0f, 0.0f));
    std::vector<float> rd_power(n_chirps * range_count, 0.0f);
    std::vector<Complex> doppler_input(n_chirps, Complex(0.0f, 0.0f));
    std::vector<Complex> doppler_output;

    for (std::size_t r = 0; r < range_count; ++r) {
        for (std::size_t rx = 0; rx < num_rx; ++rx) {
            for (std::size_t chirp = 0; chirp < n_chirps; ++chirp) {
                doppler_input[chirp] =
                    spec[cubeIndex(chirp, r, rx, range_count, num_rx)] *
                    doppler_window_[chirp];
            }

            fft.fwd(doppler_output, doppler_input);
            fftShiftInPlace(doppler_output);

            for (std::size_t dbin = 0; dbin < n_chirps; ++dbin) {
                const Complex value = doppler_output[dbin];
                rd_cube[cubeIndex(dbin, r, rx, range_count, num_rx)] = value;
                rd_power[dbin * range_count + r] += std::norm(value);
            }
        }
    }

    for (float &power : rd_power) {
        power /= static_cast<float>(num_rx);
    }

    const std::size_t zero_doppler_bin = n_chirps / 2;
    std::size_t best_doppler_bin = 0;
    std::size_t best_range_bin = 0;
    float best_power = -1.0f;
    for (std::size_t dbin = 0; dbin < n_chirps; ++dbin) {
        if (detection_config_.zero_doppler_guard_bins > 0) {
            const std::size_t low = zero_doppler_bin > detection_config_.zero_doppler_guard_bins
                                        ? zero_doppler_bin -
                                              detection_config_.zero_doppler_guard_bins
                                        : 0;
            const std::size_t high =
                std::min(n_chirps - 1,
                         zero_doppler_bin + detection_config_.zero_doppler_guard_bins);
            if (dbin >= low && dbin <= high) {
                continue;
            }
        }

        for (std::size_t rbin = 0; rbin < range_count; ++rbin) {
            const float power = rd_power[dbin * range_count + rbin];
            if (power > best_power) {
                best_power = power;
                best_doppler_bin = dbin;
                best_range_bin = rbin;
            }
        }
    }

    const float doppler_hz = doppler_axis_hz_[best_doppler_bin];
    const float range_correction =
        (doppler_hz / radar_config_.chirpSlopeHzPerS()) *
        (radar_config_.speed_of_light_mps / 2.0f);
    const float corrected_range_m =
        range_axis_sliced_m_[best_range_bin] - range_correction;

    std::vector<Complex> snapshot(num_rx, Complex(0.0f, 0.0f));
    for (std::size_t rx = 0; rx < num_rx; ++rx) {
        snapshot[rx] =
            rd_cube[cubeIndex(best_doppler_bin, best_range_bin, rx, range_count, num_rx)];
    }

    Vec3 best_direction = Vec3::UnitX();
    if (detection_config_.aoa_enable) {
        float best_response = -1.0f;
        for (std::size_t direction_index = 0; direction_index < directions_.size();
             ++direction_index) {
            Complex response(0.0f, 0.0f);
            const std::size_t base = direction_index * num_rx;
            for (std::size_t rx = 0; rx < num_rx; ++rx) {
                response += steering_conj_[base + rx] * snapshot[rx];
            }

            const float power = std::norm(response);
            if (power > best_response) {
                best_response = power;
                best_direction = directions_[direction_index];
            }
        }
    }

    BatchResult result;
    result.time_s = (static_cast<float>(start_chirp) + 0.5f * static_cast<float>(n_chirps)) *
                    radar_config_.chirp_duration_s;
    result.range_m = corrected_range_m;
    result.doppler_hz = doppler_hz;
    result.phase_rad = std::arg(snapshot[0]);
    result.direction = best_direction;
    result.doppler_slice_power.resize(n_chirps, 0.0f);
    for (std::size_t dbin = 0; dbin < n_chirps; ++dbin) {
        result.doppler_slice_power[dbin] = rd_power[dbin * range_count + best_range_bin];
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
    phases.reserve(batch_results_.size());
    std::vector<float> raw_x;
    std::vector<float> raw_y;
    std::vector<float> raw_z;
    raw_x.reserve(batch_results_.size());
    raw_y.reserve(batch_results_.size());
    raw_z.reserve(batch_results_.size());

    for (const BatchResult &batch : batch_results_) {
        summary.times_s.push_back(batch.time_s);
        const Vec3 raw_xyz = batch.range_m * batch.direction;
        summary.raw_positions_m.push_back(raw_xyz);
        summary.ranges_m.push_back(batch.range_m);
        summary.radial_velocity_mps.push_back(0.5f * batch.doppler_hz * wavelength_m);
        phases.push_back(batch.phase_rad);
        raw_x.push_back(raw_xyz.x());
        raw_y.push_back(raw_xyz.y());
        raw_z.push_back(raw_xyz.z());
        summary.truth_metrics.push_back(truthAtTime(description_, batch.time_s));
    }

    summary.unwrapped_phase_rad = unwrapPhases(phases);

    const std::size_t window_len = std::min<std::size_t>(11, batch_results_.size());
    const std::vector<float> boxcar(window_len, 1.0f / static_cast<float>(window_len));
    const std::vector<float> smooth_x = convolveSame(raw_x, boxcar);
    const std::vector<float> smooth_y = convolveSame(raw_y, boxcar);
    const std::vector<float> smooth_z = convolveSame(raw_z, boxcar);

    for (std::size_t i = 0; i < batch_results_.size(); ++i) {
        summary.smoothed_positions_m.emplace_back(smooth_x[i], smooth_y[i], smooth_z[i]);
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
