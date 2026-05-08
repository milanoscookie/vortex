#include "fmcw_tracker.h"

#include "dynamics.h"
#include "target_observation.h"

#ifndef EIGEN_FFTW_DEFAULT
#define EIGEN_FFTW_DEFAULT
#endif
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
    const double chirp_duration_s = static_cast<double>(problem::RadarSettings::kRadarBlockSize) /
                                    description.radar.sample_rate_hz;
    return static_cast<std::size_t>(
        std::llround(description.simulator.burst_duration_s / chirp_duration_s));
}

RadarConfig makeRadarConfig(const problem::ProblemDescription &description) {
    const double chirp_duration_s = static_cast<double>(problem::RadarSettings::kRadarBlockSize) /
                                    description.radar.sample_rate_hz;
    const double lambda_m = problem::Constants::kSpeedOfLightMps / description.radar.carrier_hz;

    RadarConfig config;
    config.sample_rate_hz = description.radar.sample_rate_hz;
    config.carrier_hz = description.radar.carrier_hz;
    config.bandwidth_hz = description.radar.bandwidth_hz;
    config.chirp_duration_s = chirp_duration_s;
    config.speed_of_light_mps = problem::Constants::kSpeedOfLightMps;
    config.block_size = problem::RadarSettings::kRadarBlockSize;
    config.chirp_count = computeChirpCount(description);
    config.probe_num_x = problem::RadarSettings::kProbeNumX;
    config.probe_num_y = problem::RadarSettings::kProbeNumY;
    config.probe_dx_m = description.probe.spacing_x_wavelengths * lambda_m;
    config.probe_dy_m = description.probe.spacing_y_wavelengths * lambda_m;
    return config;
}

std::vector<double> linspace(double start, double stop, std::size_t count) {
    std::vector<double> values(count);
    if (count == 0) {
        return values;
    }
    if (count == 1) {
        values[0] = start;
        return values;
    }

    const double step = (stop - start) / static_cast<double>(count - 1);
    for (std::size_t i = 0; i < count; ++i) {
        values[i] = start + static_cast<double>(i) * step;
    }
    return values;
}

std::vector<double> hannWindow(std::size_t length) {
    std::vector<double> window(length, 1.0f);
    if (length <= 1) {
        return window;
    }

    for (std::size_t n = 0; n < length; ++n) {
        window[n] = 0.5f - 0.5f * std::cos(2.0f * problem::Constants::kPi * static_cast<double>(n) /
                                           static_cast<double>(length - 1));
    }
    return window;
}

std::vector<double> fftFreq(std::size_t nfft, double sample_period_s) {
    std::vector<double> freqs(nfft, 0.0f);
    const double scale = 1.0f / (static_cast<double>(nfft) * sample_period_s);
    const std::size_t half = nfft / 2;
    for (std::size_t k = 0; k < nfft; ++k) {
        if (k < half) {
            freqs[k] = static_cast<double>(k) * scale;
        } else {
            freqs[k] =
                static_cast<double>(static_cast<long long>(k) - static_cast<long long>(nfft)) *
                scale;
        }
    }
    return freqs;
}

std::size_t centeredBinToFftIndex(std::size_t centered_bin, std::size_t nfft) {
    return (centered_bin + nfft / 2U) % nfft;
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
                (static_cast<double>(ix) - 0.5f * static_cast<double>(cfg.probe_num_x - 1)) *
                cfg.probe_dx_m;
            positions[flat].y() =
                (static_cast<double>(iy) - 0.5f * static_cast<double>(cfg.probe_num_y - 1)) *
                cfg.probe_dy_m;
        }
    }
    return positions;
}

struct SteeringGrid {
    std::vector<Complex> steering_conj;
    std::vector<Vec3> directions;
    std::vector<double> azimuth_deg;
    std::vector<double> elevation_deg;
};

SteeringGrid makeSteeringGrid(const RadarConfig &cfg, const DetectionConfig &det) {
    const std::vector<Vec3> positions_m = elementPositions(cfg);
    const std::vector<double> az_rad =
        linspace(det.azimuth_min_deg, det.azimuth_max_deg, det.azimuth_count);
    const std::vector<double> el_rad =
        linspace(det.elevation_min_deg, det.elevation_max_deg, det.elevation_count);

    SteeringGrid grid;
    grid.steering_conj.reserve(det.azimuth_count * det.elevation_count * cfg.numRx());
    grid.directions.reserve(det.azimuth_count * det.elevation_count);
    grid.azimuth_deg.reserve(det.azimuth_count * det.elevation_count);
    grid.elevation_deg.reserve(det.azimuth_count * det.elevation_count);

    for (double az_deg : az_rad) {
        const double az = az_deg * problem::Constants::kPi / 180.0f;
        for (double el_deg : el_rad) {
            const double el = el_deg * problem::Constants::kPi / 180.0f;
            Vec3 direction;
            direction.x() = std::cos(el) * std::cos(az);
            direction.y() = std::cos(el) * std::sin(az);
            direction.z() = std::sin(el);
            grid.directions.push_back(direction);
            grid.azimuth_deg.push_back(az_deg);
            grid.elevation_deg.push_back(el_deg);

            for (const Vec3 &position : positions_m) {
                const double phase =
                    (2.0f * problem::Constants::kPi / cfg.wavelengthM()) * direction.dot(position);
                grid.steering_conj.emplace_back(std::cos(-phase), std::sin(-phase));
            }
        }
    }

    return grid;
}

std::vector<Complex> makeComplexWindow(const std::vector<double> &real_window) {
    std::vector<Complex> window(real_window.size(), Complex(0.0f, 0.0f));
    const std::size_t count = real_window.size();
    for (std::size_t i = 0; i < count; ++i) {
        window[i] = Complex(real_window[i], 0.0f);
    }
    return window;
}

double degrees(double radians) {
    return radians * 180.0f / problem::Constants::kPi;
}

double azimuthDeg(const Vec3 &direction) {
    return degrees(std::atan2(direction.y(), direction.x()));
}

double elevationDeg(const Vec3 &direction) {
    return degrees(std::atan2(direction.z(), std::hypot(direction.x(), direction.y())));
}

double wrapAngleDeg(double angle_deg) {
    while (angle_deg > 180.0f) {
        angle_deg -= 360.0f;
    }
    while (angle_deg < -180.0f) {
        angle_deg += 360.0f;
    }
    return angle_deg;
}

double angleDifferenceDeg(double lhs_deg, double rhs_deg) {
    return wrapAngleDeg(lhs_deg - rhs_deg);
}

Vec3 normalizedOr(const Vec3 &value, const Vec3 &fallback) {
    const double norm = value.norm();
    if (!std::isfinite(norm) || norm <= 1.0e-6f) {
        return fallback;
    }
    return value / norm;
}

struct KinematicEstimate {
    Vec3 position_m = Vec3::Zero();
    Vec3 velocity_mps = Vec3::Zero();
    Vec3 direction = Vec3::UnitX();
    double range_m = 0.0f;
    double radial_velocity_mps = 0.0f;
    double doppler_hz = 0.0f;
};

KinematicEstimate makeKinematicEstimate(const RadarConfig &cfg,
                                        const Vec3 &position_m,
                                        const Vec3 &velocity_mps,
                                        const Vec3 &direction_fallback) {
    KinematicEstimate estimate;
    estimate.position_m = position_m;
    estimate.velocity_mps = velocity_mps;
    estimate.range_m = position_m.norm();
    estimate.direction = normalizedOr(position_m, normalizedOr(direction_fallback, Vec3::UnitX()));
    estimate.radial_velocity_mps = velocity_mps.dot(estimate.direction);
    estimate.doppler_hz = 2.0f * estimate.radial_velocity_mps / cfg.wavelengthM();
    return estimate;
}

double associationPenalty(double error, double sigma) {
    if (sigma <= 0.0f) {
        return 0.0f;
    }
    const double normalized_error = error / sigma;
    return -0.5f * normalized_error * normalized_error;
}

struct PeakInterpResult {
    double delta = 0.0f;
    bool valid = true;
};

PeakInterpResult
quadraticPeakOffsetChecked(double power_minus, double power_center, double power_plus) {
    constexpr double kLogEpsilon = 1.0e-12f;
    const double y_minus = std::log(power_minus + kLogEpsilon);
    const double y_center = std::log(power_center + kLogEpsilon);
    const double y_plus = std::log(power_plus + kLogEpsilon);
    const double denom = y_minus - 2.0f * y_center + y_plus;

    PeakInterpResult out{};

    if (!(denom < -1.0e-12f)) {
        out.delta = 0.0f;
        out.valid = false;
        return out;
    }

    const double raw_delta = 0.5f * (y_minus - y_plus) / denom;

    if (!std::isfinite(raw_delta) || std::abs(raw_delta) >= 0.45f) {
        out.delta = 0.0f;
        out.valid = false;
        return out;
    }

    out.delta = raw_delta;
    out.valid = true;
    return out;
}

double interpolateAxis(const std::vector<double> &axis, std::size_t bin, double offset) {
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

std::size_t nearestAxisBin(const std::vector<double> &axis, double value) {
    if (axis.empty()) {
        return 0;
    }

    std::size_t best_bin = 0;
    double best_error = std::abs(axis[0] - value);
    for (std::size_t i = 1; i < axis.size(); ++i) {
        const double error = std::abs(axis[i] - value);
        if (error < best_error) {
            best_error = error;
            best_bin = i;
        }
    }
    return best_bin;
}

std::vector<double> unwrapPhases(const std::vector<double> &phases) {
    if (phases.empty()) {
        return {};
    }

    std::vector<double> unwrapped(phases.size(), 0.0f);
    unwrapped[0] = phases[0];
    double offset = 0.0f;
    for (std::size_t i = 1; i < phases.size(); ++i) {
        const double delta = phases[i] - phases[i - 1];
        if (delta > problem::Constants::kPi) {
            offset -= 2.0f * problem::Constants::kPi;
        } else if (delta < -problem::Constants::kPi) {
            offset += 2.0f * problem::Constants::kPi;
        }
        unwrapped[i] = phases[i] + offset;
    }
    return unwrapped;
}

std::vector<double> convolveSame(const std::vector<double> &signal,
                                 const std::vector<double> &kernel) {
    if (signal.empty() || kernel.empty()) {
        return {};
    }

    std::vector<double> full(signal.size() + kernel.size() - 1, 0.0f);
    for (std::size_t i = 0; i < signal.size(); ++i) {
        for (std::size_t k = 0; k < kernel.size(); ++k) {
            full[i + k] += signal[i] * kernel[k];
        }
    }

    const std::size_t start = (kernel.size() - 1) / 2;
    return std::vector<double>(full.begin() + static_cast<std::ptrdiff_t>(start),
                               full.begin() + static_cast<std::ptrdiff_t>(start + signal.size()));
}

std::vector<double> movingAverageSame(const std::vector<double> &signal, std::size_t window_size) {
    if (signal.empty()) {
        return {};
    }

    window_size = std::max<std::size_t>(1U, window_size);
    std::vector<double> prefix_sum(signal.size() + 1U, 0.0f);
    for (std::size_t i = 0; i < signal.size(); ++i) {
        prefix_sum[i + 1U] = prefix_sum[i] + signal[i];
    }

    std::vector<double> averaged(signal.size(), 0.0f);
    const std::size_t half_window = window_size / 2U;
    for (std::size_t i = 0; i < signal.size(); ++i) {
        const std::size_t begin = (i > half_window) ? (i - half_window) : 0U;
        const std::size_t end = std::min(signal.size(), i + half_window + 1U);
        const double sum = prefix_sum[end] - prefix_sum[begin];
        averaged[i] = sum / static_cast<double>(end - begin);
    }
    return averaged;
}

std::vector<double> gradient(const std::vector<double> &values,
                             const std::vector<double> &times_s) {
    const std::size_t count = values.size();
    std::vector<double> grad(count, 0.0f);
    if (count < 2) {
        return grad;
    }

    const double first_dt = times_s[1] - times_s[0];
    const double last_dt = times_s[count - 1] - times_s[count - 2];
    grad[0] = (values[1] - values[0]) / first_dt;
    grad[count - 1] = (values[count - 1] - values[count - 2]) / last_dt;

    for (std::size_t i = 1; i + 1 < count; ++i) {
        const double dt = times_s[i + 1] - times_s[i - 1];
        grad[i] = (values[i + 1] - values[i - 1]) / dt;
    }

    return grad;
}

std::vector<double> detrendPhase(const std::vector<double> &times_s,
                                 const std::vector<double> &phase_rad) {
    if (times_s.size() != phase_rad.size() || phase_rad.empty()) {
        return {};
    }
    if (phase_rad.size() == 1U) {
        return std::vector<double>(1U, 0.0f);
    }

    double sum_t = 0.0f;
    double sum_p = 0.0f;
    double sum_tt = 0.0f;
    double sum_tp = 0.0f;
    for (std::size_t i = 0; i < phase_rad.size(); ++i) {
        const double t = times_s[i];
        const double p = phase_rad[i];
        sum_t += t;
        sum_p += p;
        sum_tt += t * t;
        sum_tp += t * p;
    }

    const double count = static_cast<double>(phase_rad.size());
    const double denom = count * sum_tt - sum_t * sum_t;
    double slope = 0.0f;
    double intercept = phase_rad.front();
    if (std::abs(denom) > 1.0e-9f) {
        slope = (count * sum_tp - sum_t * sum_p) / denom;
        intercept = (sum_p - slope * sum_t) / count;
    }

    std::vector<double> residual(phase_rad.size(), 0.0f);
    for (std::size_t i = 0; i < phase_rad.size(); ++i) {
        residual[i] = phase_rad[i] - (intercept + slope * times_s[i]);
    }
    return residual;
}

double estimateDominantCpiResidualFrequencyHz(const std::vector<BatchResult> &batch_results,
                                              double chirp_duration_s,
                                              std::vector<double> *times_s_out = nullptr,
                                              std::vector<double> *unwrapped_phase_out = nullptr,
                                              std::vector<double> *residual_phase_out = nullptr) {
    constexpr double kMinSearchFrequencyHz = 10.0f;
    constexpr double kMaxSearchFrequencyHz = 100.0f;
    constexpr double kHighPassCutoffHz = 15.0f;
    constexpr std::size_t kBoundaryFitSamples = 8U;

    if (chirp_duration_s <= 0.0f) {
        return 0.0f;
    }

    std::size_t chirps_per_cpi = 0U;
    for (const BatchResult &batch : batch_results) {
        if (!batch.slow_time_phase_rad.empty()) {
            chirps_per_cpi = batch.slow_time_phase_rad.size();
            break;
        }
    }
    if (chirps_per_cpi == 0U) {
        return 0.0f;
    }
    const double chirp_rate_hz = 1.0f / chirp_duration_s;
    const std::size_t high_pass_window = std::max<std::size_t>(
        3U, static_cast<std::size_t>(std::llround(chirp_rate_hz / kHighPassCutoffHz)));

    struct SegmentBounds {
        std::size_t begin = 0U;
        std::size_t end = 0U;
    };

    std::vector<double> times_s;
    std::vector<double> wrapped_phase_rad;
    std::vector<SegmentBounds> continuous_segments;
    std::vector<SegmentBounds> batch_segments;
    times_s.reserve(batch_results.size() * chirps_per_cpi);
    wrapped_phase_rad.reserve(batch_results.size() * chirps_per_cpi);

    long long next_expected_chirp = -1;
    std::size_t valid_batch_count = 0U;
    for (const BatchResult &batch : batch_results) {
        if (!batch.valid) {
            continue;
        }
        ++valid_batch_count;
    }

    if (valid_batch_count == 0U) {
        return 0.0f;
    }

    for (const BatchResult &batch : batch_results) {
        if (!batch.valid) {
            next_expected_chirp = -1;
            continue;
        }

        const long long cpi_start_chirp = static_cast<long long>(std::llround(
            batch.time_s / chirp_duration_s - 0.5f * static_cast<double>(chirps_per_cpi)));
        const long long first_new_chirp = (next_expected_chirp < 0)
                                              ? cpi_start_chirp
                                              : std::max(cpi_start_chirp, next_expected_chirp);
        const long long append_count =
            static_cast<long long>(chirps_per_cpi) - (first_new_chirp - cpi_start_chirp);
        if (append_count <= 0) {
            continue;
        }

        if (continuous_segments.empty() || first_new_chirp != next_expected_chirp) {
            continuous_segments.push_back(SegmentBounds{times_s.size(), times_s.size()});
        }

        const std::size_t batch_begin = times_s.size();
        const std::size_t first_local_chirp =
            static_cast<std::size_t>(first_new_chirp - cpi_start_chirp);
        for (std::size_t chirp = first_local_chirp; chirp < chirps_per_cpi; ++chirp) {
            const long long global_chirp = cpi_start_chirp + static_cast<long long>(chirp);
            const double chirp_time_s = static_cast<double>(global_chirp) * chirp_duration_s;
            times_s.push_back(chirp_time_s);

            const double raw_phase = batch.slow_time_phase_rad[chirp];
            const double local_time_s = static_cast<double>(chirp) * chirp_duration_s;
            const double coarse_phase =
                2.0 * problem::Constants::kPi * batch.doppler_hz * local_time_s;
            const Complex baseband = std::polar(1.0, raw_phase) * std::polar(1.0, -coarse_phase);
            wrapped_phase_rad.push_back(std::arg(baseband));
        }

        const std::size_t batch_end = times_s.size();
        if (batch_end > batch_begin) {
            batch_segments.push_back(SegmentBounds{batch_begin, batch_end});
        }

        continuous_segments.back().end = times_s.size();
        next_expected_chirp = cpi_start_chirp + static_cast<long long>(chirps_per_cpi);
    }

    if (wrapped_phase_rad.size() < 4U || continuous_segments.empty()) {
        if (times_s_out != nullptr) {
            *times_s_out = std::move(times_s);
        }
        if (unwrapped_phase_out != nullptr) {
            *unwrapped_phase_out = {};
        }
        if (residual_phase_out != nullptr) {
            *residual_phase_out = {};
        }
        return 0.0f;
    }

    std::vector<double> filtered_phase_all;
    std::vector<double> unwrapped_phase_all;
    filtered_phase_all.reserve(wrapped_phase_rad.size());
    unwrapped_phase_all.reserve(wrapped_phase_rad.size());

    std::size_t max_segment_length = 0U;
    for (const SegmentBounds &segment : continuous_segments) {
        max_segment_length = std::max(max_segment_length, segment.end - segment.begin);
    }

    if (max_segment_length < 4U) {
        if (times_s_out != nullptr) {
            *times_s_out = std::move(times_s);
        }
        if (unwrapped_phase_out != nullptr) {
            *unwrapped_phase_out = std::move(unwrapped_phase_all);
        }
        if (residual_phase_out != nullptr) {
            *residual_phase_out = std::move(filtered_phase_all);
        }
        return 0.0f;
    }

    std::size_t nfft = 1U;
    while (nfft < max_segment_length) {
        nfft <<= 1U;
    }
    nfft <<= 2U;

    std::vector<double> accumulated_power(nfft / 2U, 0.0f);
    Eigen::FFT<double> fft;
    std::vector<Complex> fft_output;
    std::vector<Complex> fft_input(nfft, Complex(0.0f, 0.0f));

    std::size_t batch_segment_index = 0U;
    for (const SegmentBounds &continuous_segment : continuous_segments) {
        const std::size_t segment_length = continuous_segment.end - continuous_segment.begin;
        if (segment_length < 4U) {
            while (batch_segment_index < batch_segments.size() &&
                   batch_segments[batch_segment_index].end <= continuous_segment.end) {
                ++batch_segment_index;
            }
            continue;
        }

        std::vector<double> aligned_unwrapped_segment;
        aligned_unwrapped_segment.reserve(segment_length);

        while (batch_segment_index < batch_segments.size() &&
               batch_segments[batch_segment_index].begin < continuous_segment.end) {
            const SegmentBounds batch_segment = batch_segments[batch_segment_index++];
            if (batch_segment.end <= continuous_segment.begin) {
                continue;
            }

            const std::vector<double> wrapped_batch_segment(
                wrapped_phase_rad.begin() + static_cast<std::ptrdiff_t>(batch_segment.begin),
                wrapped_phase_rad.begin() + static_cast<std::ptrdiff_t>(batch_segment.end));
            std::vector<double> unwrapped_batch_segment = unwrapPhases(wrapped_batch_segment);
            if (unwrapped_batch_segment.empty()) {
                continue;
            }

            if (!aligned_unwrapped_segment.empty()) {
                const std::size_t prev_count =
                    std::min<std::size_t>(kBoundaryFitSamples, aligned_unwrapped_segment.size());
                double predicted_next_phase = aligned_unwrapped_segment.back();
                if (aligned_unwrapped_segment.size() >= 2U) {
                    double step_sum = 0.0f;
                    std::size_t step_count = 0U;
                    const std::size_t start = aligned_unwrapped_segment.size() - prev_count;
                    for (std::size_t i = start + 1U; i < aligned_unwrapped_segment.size(); ++i) {
                        step_sum +=
                            aligned_unwrapped_segment[i] - aligned_unwrapped_segment[i - 1U];
                        ++step_count;
                    }
                    if (step_count > 0U) {
                        predicted_next_phase += step_sum / static_cast<double>(step_count);
                    }
                }

                const double phase_offset = predicted_next_phase - unwrapped_batch_segment.front();
                for (double &sample : unwrapped_batch_segment) {
                    sample += phase_offset;
                }
            }

            aligned_unwrapped_segment.insert(aligned_unwrapped_segment.end(),
                                             unwrapped_batch_segment.begin(),
                                             unwrapped_batch_segment.end());
        }

        if (aligned_unwrapped_segment.size() != segment_length) {
            continue;
        }

        const std::vector<double> low_pass_segment =
            movingAverageSame(aligned_unwrapped_segment,
                              std::min(high_pass_window, aligned_unwrapped_segment.size()));

        std::vector<double> filtered_segment(aligned_unwrapped_segment.size(), 0.0f);
        for (std::size_t i = 0; i < aligned_unwrapped_segment.size(); ++i) {
            filtered_segment[i] = aligned_unwrapped_segment[i] - low_pass_segment[i];
        }

        unwrapped_phase_all.insert(unwrapped_phase_all.end(),
                                   aligned_unwrapped_segment.begin(),
                                   aligned_unwrapped_segment.end());
        filtered_phase_all.insert(
            filtered_phase_all.end(), filtered_segment.begin(), filtered_segment.end());

        const std::vector<double> window = hannWindow(filtered_segment.size());
        std::fill(fft_input.begin(), fft_input.end(), Complex(0.0f, 0.0f));
        for (std::size_t i = 0; i < filtered_segment.size(); ++i) {
            fft_input[i] = Complex(filtered_segment[i] * window[i], 0.0f);
        }

        fft.fwd(fft_output, fft_input);
        for (std::size_t bin = 1U; bin < accumulated_power.size(); ++bin) {
            accumulated_power[bin] += std::norm(fft_output[bin]);
        }
    }

    if (times_s_out != nullptr) {
        *times_s_out = times_s;
    }
    if (unwrapped_phase_out != nullptr) {
        *unwrapped_phase_out = unwrapped_phase_all;
    }
    if (residual_phase_out != nullptr) {
        *residual_phase_out = filtered_phase_all;
    }

    double best_power = 0.0f;
    double best_frequency_hz = 0.0f;
    const std::size_t half = nfft / 2U;
    for (std::size_t bin = 1U; bin < half; ++bin) {
        const double frequency_hz =
            static_cast<double>(bin) / (static_cast<double>(nfft) * chirp_duration_s);
        if (frequency_hz < kMinSearchFrequencyHz || frequency_hz > kMaxSearchFrequencyHz) {
            continue;
        }
        const double power = accumulated_power[bin];
        if (power > best_power) {
            best_power = power;
            best_frequency_hz = frequency_hz;
        }
    }

    return best_frequency_hz;
}

double
estimateDominantBatchDopplerFrequencyHz(const std::vector<BatchResult> &batch_results,
                                        double batch_period_s,
                                        std::vector<double> *candidate_frequency_hz_out = nullptr,
                                        std::vector<double> *candidate_power_out = nullptr,
                                        double *peak_power_out = nullptr,
                                        std::size_t *valid_batch_count_out = nullptr) {
    constexpr double kMinSearchFrequencyHz = 10.0f;
    constexpr double kMaxSearchFrequencyHz = 100.0f;
    constexpr double kHighPassCutoffHz = 10.0f;

    if (batch_period_s <= 0.0f) {
        return 0.0f;
    }

    struct SegmentBounds {
        std::size_t begin = 0U;
        std::size_t end = 0U;
    };

    std::vector<double> doppler_trace_hz;
    std::vector<SegmentBounds> segments;
    doppler_trace_hz.reserve(batch_results.size());

    std::size_t valid_batch_count = 0U;
    for (const BatchResult &batch : batch_results) {
        if (!batch.valid) {
            continue;
        }
        ++valid_batch_count;
    }

    if (valid_batch_count_out != nullptr) {
        *valid_batch_count_out = valid_batch_count;
    }

    if (valid_batch_count < 4U) {
        return 0.0f;
    }

    bool previous_valid = false;
    for (const BatchResult &batch : batch_results) {
        if (!batch.valid) {
            previous_valid = false;
            continue;
        }

        if (!previous_valid) {
            segments.push_back(SegmentBounds{doppler_trace_hz.size(), doppler_trace_hz.size()});
        }

        doppler_trace_hz.push_back(batch.doppler_hz - batch.predicted_doppler_hz);
        segments.back().end = doppler_trace_hz.size();
        previous_valid = true;
    }

    std::size_t max_segment_length = 0U;
    for (const SegmentBounds &segment : segments) {
        max_segment_length = std::max(max_segment_length, segment.end - segment.begin);
    }
    if (max_segment_length < 4U) {
        return 0.0f;
    }

    std::size_t nfft = 1U;
    while (nfft < max_segment_length) {
        nfft <<= 1U;
    }
    nfft <<= 2U;

    const double batch_rate_hz = 1.0f / batch_period_s;
    const std::size_t high_pass_window = std::max<std::size_t>(
        3U, static_cast<std::size_t>(std::llround(batch_rate_hz / kHighPassCutoffHz)));

    std::vector<double> accumulated_power(nfft / 2U, 0.0f);
    Eigen::FFT<double> fft;
    std::vector<Complex> fft_output;
    std::vector<Complex> fft_input(nfft, Complex(0.0f, 0.0f));

    for (const SegmentBounds &segment : segments) {
        const std::size_t segment_length = segment.end - segment.begin;
        if (segment_length < 4U) {
            continue;
        }
        const std::vector<double> filtered_segment(
            doppler_trace_hz.begin() + static_cast<std::ptrdiff_t>(segment.begin),
            doppler_trace_hz.begin() + static_cast<std::ptrdiff_t>(segment.end));
        // const std::vector<double> doppler_segment(
        //     doppler_trace_hz.begin() + static_cast<std::ptrdiff_t>(segment.begin),
        //     doppler_trace_hz.begin() + static_cast<std::ptrdiff_t>(segment.end));
        // const std::vector<double> low_pass_segment =
        //     movingAverageSame(doppler_segment, std::min(high_pass_window, segment_length));
        //
        // std::vector<double> filtered_segment(segment_length, 0.0f);
        // for (std::size_t i = 0; i < segment_length; ++i) {
        //     filtered_segment[i] = doppler_segment[i] - low_pass_segment[i];
        // }

        const std::vector<double> window = hannWindow(segment_length);
        std::fill(fft_input.begin(), fft_input.end(), Complex(0.0f, 0.0f));
        for (std::size_t i = 0; i < segment_length; ++i) {
            fft_input[i] = Complex(filtered_segment[i] * window[i], 0.0f);
        }

        fft.fwd(fft_output, fft_input);
        for (std::size_t bin = 1U; bin < accumulated_power.size(); ++bin) {
            accumulated_power[bin] += std::norm(fft_output[bin]);
        }
    }

    struct Candidate {
        double frequency_hz = 0.0f;
        double power = 0.0f;
    };
    std::vector<Candidate> candidates;
    double best_power = 0.0f;
    double best_frequency_hz = 0.0f;
    std::size_t best_bin = 0U;
    const std::size_t half = nfft / 2U;
    for (std::size_t bin = 1U; bin < half; ++bin) {
        const double frequency_hz =
            static_cast<double>(bin) / (static_cast<double>(nfft) * batch_period_s);
        if (frequency_hz < kMinSearchFrequencyHz || frequency_hz > kMaxSearchFrequencyHz) {
            continue;
        }

        const double power = accumulated_power[bin];
        candidates.push_back(Candidate{frequency_hz, power});
        if (power > best_power) {
            best_power = power;
            best_frequency_hz = frequency_hz;
            best_bin = bin;
        }
    }

    if (best_bin > 0U && best_bin + 1U < accumulated_power.size()) {
        const PeakInterpResult interp =
            quadraticPeakOffsetChecked(accumulated_power[best_bin - 1U],
                                       accumulated_power[best_bin],
                                       accumulated_power[best_bin + 1U]);
        if (interp.valid) {
            best_frequency_hz = (static_cast<double>(best_bin) + interp.delta) /
                                (static_cast<double>(nfft) * batch_period_s);
        }
    }

    std::partial_sort(
        candidates.begin(),
        candidates.begin() + std::min<std::size_t>(5U, candidates.size()),
        candidates.end(),
        [](const Candidate &lhs, const Candidate &rhs) { return lhs.power > rhs.power; });

    if (candidate_frequency_hz_out != nullptr) {
        candidate_frequency_hz_out->clear();
        for (std::size_t i = 0; i < std::min<std::size_t>(5U, candidates.size()); ++i) {
            candidate_frequency_hz_out->push_back(candidates[i].frequency_hz);
        }
    }
    if (candidate_power_out != nullptr) {
        candidate_power_out->clear();
        for (std::size_t i = 0; i < std::min<std::size_t>(5U, candidates.size()); ++i) {
            candidate_power_out->push_back(candidates[i].power);
        }
    }
    if (peak_power_out != nullptr) {
        *peak_power_out = best_power;
    }

    return best_frequency_hz;
}

} // namespace

StreamingTracker::StreamingTracker(const problem::ProblemDescription &description,
                                   DetectionConfig detection_config)
    : description_(description), radar_config_(makeRadarConfig(description)),
      detection_config_(detection_config),
      chirp_window_(std::make_unique<RingBuffer<ChirpBlock, kMaxCpiChirps>>()) {
    if (description_.cars.size() != 1) {
        throw std::runtime_error("StreamingTracker currently supports exactly one car");
    }
    if (detection_config_.coherent_processing_interval_chirps > kMaxCpiChirps) {
        throw std::runtime_error("tracker CPI exceeds RingBuffer capacity");
    }
    if (detection_config_.coherent_processing_interval_chirps == 0U) {
        throw std::runtime_error("tracker CPI must be non-zero");
    }
    if (detection_config_.nfft_range_min < radar_config_.block_size) {
        throw std::runtime_error("tracker range FFT must be at least block size");
    }
    if (detection_config_.azimuth_count == 0U || detection_config_.elevation_count == 0U) {
        throw std::runtime_error("tracker AoA grid must be non-empty");
    }

    // Range axis: use only negative-frequency FFT bins (down-chirp produces
    // negative beat frequencies for targets). Map to range via
    // R = -c * f_beat / (2 * chirp_slope).
    const std::vector<double> freqs =
        fftFreq(detection_config_.nfft_range_min, 1.0f / radar_config_.sample_rate_hz);
    range_bin_count_ = 0;
    range_indices_.clear();
    range_axis_sliced_m_.clear();
    range_indices_.reserve(freqs.size() / 2U + 1U);
    range_axis_sliced_m_.reserve(freqs.size() / 2U + 1U);
    for (std::size_t i = 0; i < freqs.size(); ++i) {
        const double beat_frequency_hz = freqs[i];
        if (beat_frequency_hz > 0.0f) {
            continue;
        }

        const double range_m = -radar_config_.speed_of_light_mps * beat_frequency_hz /
                               (2.0f * radar_config_.chirpSlopeHzPerS());
        if (range_m >= detection_config_.min_range_m && range_m <= detection_config_.max_range_m) {
            range_indices_.push_back(static_cast<int>(i));
            range_axis_sliced_m_.push_back(range_m);
            ++range_bin_count_;
        }
    }
    if (range_bin_count_ == 0) {
        throw std::runtime_error("tracker range gate produced no bins");
    }

    const std::vector<Complex> range_window =
        makeComplexWindow(hannWindow(radar_config_.block_size));
    doppler_window_ =
        makeComplexWindow(hannWindow(detection_config_.coherent_processing_interval_chirps));
    range_mix_coeff_.resize(radar_config_.block_size, Complex(0.0f, 0.0f));

    const SteeringGrid grid = makeSteeringGrid(radar_config_, detection_config_);
    steering_conj_ = grid.steering_conj;
    directions_ = grid.directions;
    direction_azimuth_deg_ = grid.azimuth_deg;
    direction_elevation_deg_ = grid.elevation_deg;

    // Doppler axis: zero-pad to 2x CPI for finer frequency resolution
    // (~305 Hz/bin instead of ~610 Hz/bin). The truth model uses
    // f_D = 2*v_r/lambda (positive = approaching). The FFT on the beat-signal
    // slow-time produces the opposite sign, so we negate the axis.
    const std::size_t nfft_doppler = 2U * detection_config_.coherent_processing_interval_chirps;
    const std::vector<double> doppler_axis_native_hz =
        fftFreq(nfft_doppler, radar_config_.chirp_duration_s);
    doppler_axis_hz_.resize(nfft_doppler, 0.0f);
    for (std::size_t centered_bin = 0; centered_bin < nfft_doppler; ++centered_bin) {
        doppler_axis_hz_[centered_bin] =
            -doppler_axis_native_hz[centeredBinToFftIndex(centered_bin, nfft_doppler)];
    }

    velocity_axis_mps_.resize(doppler_axis_hz_.size(), 0.0f);
    for (std::size_t i = 0; i < velocity_axis_mps_.size(); ++i) {
        velocity_axis_mps_[i] = 0.5f * doppler_axis_hz_[i] * radar_config_.wavelengthM();
    }

    const std::size_t n_chirps = detection_config_.coherent_processing_interval_chirps;
    const std::size_t num_rx = radar_config_.numRx();
    const std::size_t nfft_range = detection_config_.nfft_range_min;
    range_fft_input_.resize(nfft_range, Complex(0.0f, 0.0f));
    range_fft_output_.reserve(nfft_range);
    doppler_fft_input_.resize(nfft_doppler, Complex(0.0f, 0.0f));
    doppler_fft_output_.reserve(nfft_doppler);
    spec_scratch_.resize(n_chirps * range_bin_count_ * num_rx);
    rd_cube_scratch_.resize(nfft_doppler * range_bin_count_ * num_rx);
    clutter_mean_scratch_.resize(range_bin_count_ * num_rx, Complex(0.0f, 0.0f));
    rd_power_scratch_.resize(nfft_doppler * range_bin_count_);
    rd_candidates_scratch_.reserve(range_bin_count_ * nfft_doppler);

    if (!range_mix_coeff_initialized_) {
        for (std::size_t i = 0; i < radar_config_.block_size; ++i) {
            range_mix_coeff_[i] = range_window[i];
        }
    }

    // The simulator keeps a continuous TX history, so the first fast-time
    // samples of a chirp can include delayed energy from the previous chirp.
    // Blanking that prefix avoids wrap-around contamination. Cap the guard
    // to a fraction of the block so extreme-range configs don't starve the
    // range FFT.
    const double max_tracked_range_m =
        std::min(description_.radar.max_range_m, detection_config_.max_range_m);
    const double max_delay_samples =
        (2.0f * max_tracked_range_m / radar_config_.speed_of_light_mps) *
        radar_config_.sample_rate_hz;
    range_wrap_guard_samples_ = std::min(
        radar_config_.block_size / 8U, static_cast<std::size_t>(std::ceil(max_delay_samples)) + 2U);
}

void StreamingTracker::pushChirp(std::size_t chirp_index,
                                 std::span<const Complex> tx_chirp,
                                 std::span<const Complex> rx_block) {
    if (!range_mix_coeff_initialized_) {
        for (std::size_t i = 0; i < tx_chirp.size(); ++i) {
            range_mix_coeff_[i] *= std::conj(tx_chirp[i]);
        }
        range_mix_coeff_initialized_ = true;
    }

    if (tx_chirp.size() != radar_config_.block_size) {
        throw std::runtime_error("unexpected TX chirp size");
    }
    if (rx_block.size() != radar_config_.block_size * radar_config_.numRx()) {
        throw std::runtime_error("unexpected RX block size");
    }

    ChirpBlock chirp_block{};
    std::copy(rx_block.begin(), rx_block.end(), chirp_block.begin());
    chirp_window_->push_back(chirp_block);
    while (chirp_window_->size() > detection_config_.coherent_processing_interval_chirps) {
        chirp_window_->pop_front();
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
            const double dt = std::max(0.0, result.time_s - tracking_state_.time_s);
            tracking_state_.position_m += tracking_state_.velocity_mps * dt;
            const KinematicEstimate predicted = makeKinematicEstimate(radar_config_,
                                                                      tracking_state_.position_m,
                                                                      tracking_state_.velocity_mps,
                                                                      tracking_state_.direction);
            tracking_state_.time_s = result.time_s;
            tracking_state_.range_m = predicted.range_m;
            tracking_state_.radial_velocity_mps = predicted.radial_velocity_mps;
            tracking_state_.doppler_hz = predicted.doppler_hz;
            tracking_state_.direction = predicted.direction;

            result.range_m = predicted.range_m;
            result.doppler_hz = predicted.doppler_hz;
            result.direction = predicted.direction;
            result.predicted_range_m = predicted.range_m;
            result.predicted_doppler_hz = predicted.doppler_hz;
            result.predicted_direction = predicted.direction;
        }
        batch_results_.push_back(result);
        return;
    }
    const Vec3 measured_position = result.range_m * result.direction;
    const double measured_radial_velocity_mps =
        0.5 * result.doppler_hz * radar_config_.wavelengthM();

    if (!tracking_state_.initialized) {
        tracking_state_.position_m = measured_position;
        tracking_state_.velocity_mps = measured_radial_velocity_mps * result.direction;
        tracking_state_.initialized = true;
    } else {
        const double dt = std::max(0.0, result.time_s - tracking_state_.time_s);
        const Vec3 predicted_position =
            tracking_state_.position_m + tracking_state_.velocity_mps * dt;

        // Glide through position snaps, and kill velocity integration from position entirely.
        constexpr double kPositionGain = 0.005;
        constexpr double kVelocityGain = 0.0;         // Completely decoupled!
        constexpr double kRadialVelocityGain = 0.002; // ~0.4 second time constant

        const Vec3 residual = measured_position - predicted_position;
        tracking_state_.position_m = predicted_position + kPositionGain * residual;

        // Anchor the velocity vector ONLY using the precise Doppler measurement
        const double radial_velocity_error_mps =
            measured_radial_velocity_mps - tracking_state_.velocity_mps.dot(result.direction);
        tracking_state_.velocity_mps +=
            kRadialVelocityGain * radial_velocity_error_mps * result.direction;
    }
    // } else {
    //     const double dt = std::max(0.001f, result.time_s - tracking_state_.time_s);
    //     const Vec3 predicted_position =
    //         tracking_state_.position_m + tracking_state_.velocity_mps * dt;
    //
    //     // Ultra-low gains to glide smoothly through AoA grid-snapping noise
    //     constexpr double kPositionGain = 0.01f;
    //     constexpr double kVelocityGain = 0.0001f;
    //     constexpr double kRadialVelocityGain = 0.002f;
    //
    //     const Vec3 residual = measured_position - predicted_position;
    //     tracking_state_.position_m = predicted_position + kPositionGain * residual;
    //
    //     // This is now safe. A 1-meter grid snap only alters velocity by ~0.5 m/s instead of 40
    //     m/s tracking_state_.velocity_mps += (kVelocityGain / dt) * residual;
    //
    //     // Anchor the velocity vector using the highly accurate Doppler measurement
    //     const double radial_velocity_error_mps =
    //         measured_radial_velocity_mps - tracking_state_.velocity_mps.dot(result.direction);
    //     tracking_state_.velocity_mps +=
    //         kRadialVelocityGain * radial_velocity_error_mps * result.direction;
    // }
    // } else {
    //     const double dt = std::max(0.001f, result.time_s - tracking_state_.time_s);
    //     const Vec3 predicted_position = tracking_state_.position_m + tracking_state_.velocity_mps
    //     * dt;
    //
    //     // Lower, stable gains for a high-rate radar tracker
    //     constexpr double kPositionGain = 0.30f;
    //     constexpr double kVelocityGain = 0.08f;
    //     constexpr double kRadialVelocityGain = 0.45f;
    //
    //     const Vec3 residual = measured_position - predicted_position;
    //     tracking_state_.position_m = predicted_position + kPositionGain * residual;
    //     tracking_state_.velocity_mps = tracking_state_.velocity_mps + (kVelocityGain / dt) *
    //     residual;
    //
    //     // Anchor the velocity vector using the highly accurate Doppler measurement
    //     const double radial_velocity_error_mps =
    //         measured_radial_velocity_mps - tracking_state_.velocity_mps.dot(result.direction);
    //     tracking_state_.velocity_mps +=
    //         kRadialVelocityGain * radial_velocity_error_mps * result.direction;
    // }

    const KinematicEstimate estimate = makeKinematicEstimate(
        radar_config_, tracking_state_.position_m, tracking_state_.velocity_mps, result.direction);
    tracking_state_.time_s = result.time_s;
    tracking_state_.range_m = estimate.range_m;
    tracking_state_.radial_velocity_mps = estimate.radial_velocity_mps;
    tracking_state_.doppler_hz = estimate.doppler_hz;
    tracking_state_.direction = estimate.direction;

    batch_results_.push_back(result);
}

BatchResult StreamingTracker::processCurrentWindow(std::size_t start_chirp) {
    const std::size_t n_chirps = detection_config_.coherent_processing_interval_chirps;
    const std::size_t nfft_doppler = n_chirps * 2U;
    const std::size_t num_rx = radar_config_.numRx();
    const std::size_t range_count = range_bin_count_;

    // Range FFT: dechirp, window, and transform each chirp.
    std::vector<Complex> &spec = spec_scratch_;
    std::vector<Complex> &fft_input = range_fft_input_;
    std::vector<Complex> &fft_output = range_fft_output_;
    std::vector<Complex> &clutter_mean = clutter_mean_scratch_;
    const std::size_t chirp_stride = range_count * num_rx;
    std::fill(clutter_mean.begin(), clutter_mean.end(), Complex(0.0f, 0.0f));

    for (std::size_t chirp = 0; chirp < n_chirps; ++chirp) {
        const ChirpBlock &rx_block = (*chirp_window_)[chirp];
        for (std::size_t rx = 0; rx < num_rx; ++rx) {
            std::fill_n(fft_input.begin(), range_wrap_guard_samples_, Complex(0.0f, 0.0f));
            for (std::size_t sample = range_wrap_guard_samples_; sample < radar_config_.block_size;
                 ++sample) {
                fft_input[sample] = rx_block[sample * num_rx + rx] * range_mix_coeff_[sample];
            }

            std::fill(fft_input.begin() + static_cast<std::ptrdiff_t>(radar_config_.block_size),
                      fft_input.end(),
                      Complex(0.0f, 0.0f));

            fft_.fwd(fft_output, fft_input);
            const std::size_t chirp_base = chirp * chirp_stride;
            for (std::size_t r = 0; r < range_count; ++r) {
                const Complex value = fft_output[static_cast<std::size_t>(range_indices_[r])];
                const std::size_t spec_index = chirp_base + r * num_rx + rx;
                spec[spec_index] = value;
                clutter_mean[r * num_rx + rx] += value;
            }
        }
    }

    constexpr double kScoreEpsilon = 1.0e-12f;
    constexpr std::size_t kMaxRangeDopplerCandidates = 6;

    // Static-clutter suppression: remove DC (mean) across slow-time.
    if (detection_config_.static_clutter_suppression_enable) {
        const double inv_n_chirps = 1.0 / static_cast<double>(n_chirps);
        for (Complex &mean : clutter_mean) {
            mean *= inv_n_chirps;
        }
        for (std::size_t chirp = 0; chirp < n_chirps; ++chirp) {
            const std::size_t chirp_base = chirp * chirp_stride;
            for (std::size_t idx = 0; idx < chirp_stride; ++idx) {
                spec[chirp_base + idx] -= clutter_mean[idx];
            }
        }
    }

    auto insertCandidate =
        [](std::array<CandidateScoreScratch, kMaxRangeDopplerCandidates> &top_candidates,
           std::size_t &top_candidate_count,
           const CandidateScoreScratch &candidate) {
            std::size_t insert_pos = top_candidate_count;
            if (top_candidate_count == kMaxRangeDopplerCandidates) {
                if (candidate.score <= top_candidates[top_candidate_count - 1U].score) {
                    return;
                }
                insert_pos = top_candidate_count - 1U;
            } else {
                ++top_candidate_count;
            }

            while (insert_pos > 0U && candidate.score > top_candidates[insert_pos - 1U].score) {
                top_candidates[insert_pos] = top_candidates[insert_pos - 1U];
                --insert_pos;
            }
            top_candidates[insert_pos] = candidate;
        };

    std::array<CandidateScoreScratch, kMaxRangeDopplerCandidates> top_candidates{};
    std::size_t top_candidate_count = 0U;

    // Doppler FFT: zero-padded to 2x CPI for finer Doppler resolution.
    std::vector<Complex> &rd_cube = rd_cube_scratch_;
    std::vector<double> &rd_power = rd_power_scratch_;
    std::vector<Complex> &doppler_input = doppler_fft_input_;
    std::vector<Complex> &doppler_output = doppler_fft_output_;
    std::fill(rd_power.begin(), rd_power.end(), 0.0f);

    for (std::size_t r = 0; r < range_count; ++r) {
        for (std::size_t rx = 0; rx < num_rx; ++rx) {
            for (std::size_t chirp = 0; chirp < n_chirps; ++chirp) {
                doppler_input[chirp] =
                    spec[cubeIndex(chirp, r, rx, range_count, num_rx)] * doppler_window_[chirp];
            }
            std::fill(doppler_input.begin() + static_cast<std::ptrdiff_t>(n_chirps),
                      doppler_input.end(),
                      Complex(0.0f, 0.0f));

            fft_.fwd(doppler_output, doppler_input);

            for (std::size_t dbin = 0; dbin < nfft_doppler; ++dbin) {
                const Complex value = doppler_output[centeredBinToFftIndex(dbin, nfft_doppler)];
                rd_cube[cubeIndex(dbin, r, rx, range_count, num_rx)] = value;
                rd_power[dbin * range_count + r] += std::norm(value);
            }
        }
    }

    for (double &power : rd_power) {
        power /= static_cast<double>(num_rx);
    }

    const std::size_t zero_doppler_bin = nfft_doppler / 2;
    double predicted_range_m = 0.0f;
    double predicted_doppler_hz = 0.0f;
    Vec3 predicted_direction = Vec3::UnitX();
    const double batch_time_s =
        (static_cast<double>(start_chirp) + 0.5 * static_cast<double>(n_chirps)) *
        radar_config_.chirp_duration_s;
    const double range_bin_spacing_m =
        range_count > 1 ? std::abs(range_axis_sliced_m_[1] - range_axis_sliced_m_[0]) : 0.0f;
    const double doppler_bin_spacing_hz =
        nfft_doppler > 1 ? std::abs(doppler_axis_hz_[1] - doppler_axis_hz_[0]) : 0.0f;

    if (tracking_state_.initialized) {
        const double dt = std::max(0.0, batch_time_s - tracking_state_.time_s);
        const Vec3 predicted_position_m =
            tracking_state_.position_m + tracking_state_.velocity_mps * dt;
        const KinematicEstimate predicted = makeKinematicEstimate(radar_config_,
                                                                  predicted_position_m,
                                                                  tracking_state_.velocity_mps,
                                                                  tracking_state_.direction);
        predicted_range_m = predicted.range_m;
        predicted_doppler_hz = predicted.doppler_hz;
        predicted_direction = predicted.direction;
    }

    // Gate the search to a region around the prediction when initialized.
    const std::size_t range_gate =
        tracking_state_.initialized
            ? static_cast<std::size_t>(
                  std::max(static_cast<double>(detection_config_.range_gate_bins),
                           std::ceil(3.0 * detection_config_.range_association_sigma_m /
                                     std::max(range_bin_spacing_m, 1.0e-6))))
            : range_count;
    const std::size_t doppler_gate =
        tracking_state_.initialized
            ? static_cast<std::size_t>(
                  std::max(static_cast<double>(detection_config_.doppler_gate_bins),
                           std::ceil(3.0 * detection_config_.doppler_association_sigma_hz /
                                     std::max(doppler_bin_spacing_hz, 1.0e-6))))
            : nfft_doppler;

    std::size_t center_rbin = range_count / 2;
    std::size_t center_dbin = nfft_doppler / 2;
    if (tracking_state_.initialized) {
        center_rbin = nearestAxisBin(range_axis_sliced_m_, predicted_range_m);
        center_dbin = nearestAxisBin(doppler_axis_hz_, predicted_doppler_hz);
    }

    const std::size_t r_start = (center_rbin > range_gate / 2) ? center_rbin - range_gate / 2 : 0;
    const std::size_t r_end = std::min(range_count, center_rbin + range_gate / 2 + 1);
    const std::size_t d_start =
        (center_dbin > doppler_gate / 2) ? center_dbin - doppler_gate / 2 : 0;
    const std::size_t d_end = std::min(nfft_doppler, center_dbin + doppler_gate / 2 + 1);

    for (std::size_t dbin = d_start; dbin < d_end; ++dbin) {
        if (detection_config_.zero_doppler_guard_bins > 0) {
            const std::size_t low =
                zero_doppler_bin > detection_config_.zero_doppler_guard_bins
                    ? zero_doppler_bin - detection_config_.zero_doppler_guard_bins
                    : 0;
            const std::size_t high = std::min(
                nfft_doppler - 1, zero_doppler_bin + detection_config_.zero_doppler_guard_bins);
            if (dbin >= low && dbin <= high) {
                continue;
            }
        }

        const double candidate_doppler_hz = doppler_axis_hz_[dbin];
        const double range_correction = (candidate_doppler_hz / radar_config_.chirpSlopeHzPerS()) *
                                        (radar_config_.speed_of_light_mps / 2.0f);

        for (std::size_t rbin = r_start; rbin < r_end; ++rbin) {
            const double power = rd_power[dbin * range_count + rbin];
            const double candidate_range_m =
                range_axis_sliced_m_[static_cast<Eigen::Index>(rbin)] - range_correction;
            double score = std::log(power + kScoreEpsilon);
            if (tracking_state_.initialized) {
                score += associationPenalty(candidate_range_m - predicted_range_m,
                                            detection_config_.range_association_sigma_m);
                score += associationPenalty(candidate_doppler_hz - predicted_doppler_hz,
                                            detection_config_.doppler_association_sigma_hz);
            }

            CandidateScoreScratch candidate;
            candidate.doppler_bin = dbin;
            candidate.range_bin = rbin;
            candidate.range_m = candidate_range_m;
            candidate.doppler_hz = candidate_doppler_hz;
            candidate.score = score;
            insertCandidate(top_candidates, top_candidate_count, candidate);
        }
    }

    if (top_candidate_count == 0U) {
        throw std::runtime_error("tracker failed to find a valid range-doppler peak");
    }

    const std::size_t candidate_count = top_candidate_count;

    // Select best candidate and run AoA beamforming.
    double best_total_score = std::numeric_limits<double>::lowest();
    std::size_t best_doppler_bin = top_candidates.front().doppler_bin;
    std::size_t best_range_bin = top_candidates.front().range_bin;
    std::size_t best_direction_index = 0;
    double best_range_m = top_candidates.front().range_m;
    double best_doppler_hz = top_candidates.front().doppler_hz;
    std::array<Complex, kNumRx> best_snapshot{};
    Vec3 best_direction = tracking_state_.initialized ? tracking_state_.direction : Vec3::UnitX();

    const double predicted_azimuth_deg = azimuthDeg(predicted_direction);
    const double predicted_elevation_deg = elevationDeg(predicted_direction);

    for (std::size_t candidate_index = 0; candidate_index < candidate_count; ++candidate_index) {
        const CandidateScoreScratch &candidate = top_candidates[candidate_index];
        std::array<Complex, kNumRx> snapshot{};
        for (std::size_t rx = 0; rx < num_rx; ++rx) {
            snapshot[rx] = rd_cube[cubeIndex(
                candidate.doppler_bin, candidate.range_bin, rx, range_count, num_rx)];
        }

        Vec3 candidate_direction = Vec3::UnitX();
        std::size_t candidate_direction_index = 0;
        double best_direction_score =
            detection_config_.aoa_enable ? std::numeric_limits<double>::lowest() : 0.0f;

        if (detection_config_.aoa_enable) {
            const std::size_t azimuth_count = detection_config_.azimuth_count;
            const std::size_t elevation_count = detection_config_.elevation_count;
            constexpr std::size_t kTargetCoarseElevationSamples = 96U;
            const std::size_t coarse_elevation_stride =
                (elevation_count <= 1U)
                    ? 1U
                    : std::max<std::size_t>(1U,
                                            (elevation_count + kTargetCoarseElevationSamples - 1U) /
                                                kTargetCoarseElevationSamples);

            auto evaluate_direction = [&](std::size_t direction_index) {
                Complex response(0.0f, 0.0f);
                const std::size_t base = direction_index * num_rx;
                for (std::size_t rx = 0; rx < num_rx; ++rx) {
                    response += steering_conj_[base + rx] * snapshot[rx];
                }

                double direction_score = std::log(std::norm(response) + kScoreEpsilon);
                if (tracking_state_.initialized) {
                    direction_score += associationPenalty(
                        angleDifferenceDeg(direction_azimuth_deg_[direction_index],
                                           predicted_azimuth_deg),
                        detection_config_.azimuth_association_sigma_deg);
                    direction_score += associationPenalty(
                        direction_elevation_deg_[direction_index] - predicted_elevation_deg,
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
                double best_coarse_score = std::numeric_limits<double>::lowest();

                for (std::size_t elevation_index = 0; elevation_index < elevation_count;
                     elevation_index += coarse_elevation_stride) {
                    const std::size_t direction_index =
                        azimuth_index * elevation_count + elevation_index;

                    Complex response(0.0f, 0.0f);
                    const std::size_t base = direction_index * num_rx;
                    for (std::size_t rx = 0; rx < num_rx; ++rx) {
                        response += steering_conj_[base + rx] * snapshot[rx];
                    }

                    double direction_score = std::log(std::norm(response) + kScoreEpsilon);
                    if (tracking_state_.initialized) {
                        direction_score += associationPenalty(
                            angleDifferenceDeg(direction_azimuth_deg_[direction_index],
                                               predicted_azimuth_deg),
                            detection_config_.azimuth_association_sigma_deg);
                        direction_score += associationPenalty(
                            direction_elevation_deg_[direction_index] - predicted_elevation_deg,
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
                    elevation_count, best_coarse_elevation_index + coarse_elevation_stride + 1U);

                for (std::size_t elevation_index = refine_start; elevation_index < refine_stop;
                     ++elevation_index) {
                    const std::size_t direction_index =
                        azimuth_index * elevation_count + elevation_index;
                    evaluate_direction(direction_index);
                }
            }
        }

        const double total_score = candidate.score + best_direction_score;
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
    double doppler_bin_offset = 0.0f;
    if (best_doppler_bin > 0 && best_doppler_bin + 1 < nfft_doppler) {
        auto d_interp = quadraticPeakOffsetChecked(
            rd_power[(best_doppler_bin - 1) * range_count + best_range_bin],
            rd_power[best_doppler_bin * range_count + best_range_bin],
            rd_power[(best_doppler_bin + 1) * range_count + best_range_bin]);

        doppler_bin_offset = d_interp.valid ? d_interp.delta : 0.0f; // No gating!
    }

    // Sub-bin interpolation for range.
    double range_bin_offset = 0.0f;
    if (best_range_bin > 0 && best_range_bin + 1 < range_count) {
        auto r_interp = quadraticPeakOffsetChecked(
            rd_power[best_doppler_bin * range_count + (best_range_bin - 1)],
            rd_power[best_doppler_bin * range_count + best_range_bin],
            rd_power[best_doppler_bin * range_count + (best_range_bin + 1)]);

        range_bin_offset = r_interp.valid ? r_interp.delta : 0.0f; // No gating!
    }
    // double doppler_bin_offset = 0.0f;
    // if (best_doppler_bin > 0 && best_doppler_bin + 1 < nfft_doppler) {
    //     auto d_interp = quadraticPeakOffsetChecked(
    //         rd_power[(best_doppler_bin - 1) * range_count + best_range_bin],
    //         rd_power[best_doppler_bin * range_count + best_range_bin],
    //         rd_power[(best_doppler_bin + 1) * range_count + best_range_bin]);
    //
    //     double doppler_delta_raw = d_interp.valid ? d_interp.delta : 0.0f;
    //     double doppler_hz_raw =
    //         interpolateAxis(doppler_axis_hz_, nfft_doppler, best_doppler_bin, doppler_delta_raw);
    //
    //     bool bad_doppler_interp = false;
    //     if (tracking_state_.initialized) {
    //         double doppler_jump_hz = std::abs(doppler_hz_raw - predicted_doppler_hz);
    //         bad_doppler_interp = doppler_jump_hz > detection_config_.doppler_interp_gate_hz;
    //     }
    //
    //     doppler_bin_offset = bad_doppler_interp ? 0.0f : doppler_delta_raw;
    // }
    //
    // // Sub-bin interpolation for range.
    // double range_bin_offset = 0.0f;
    // if (best_range_bin > 0 && best_range_bin + 1 < range_count) {
    //     auto r_interp = quadraticPeakOffsetChecked(
    //         rd_power[best_doppler_bin * range_count + (best_range_bin - 1)],
    //         rd_power[best_doppler_bin * range_count + best_range_bin],
    //         rd_power[best_doppler_bin * range_count + (best_range_bin + 1)]);
    //
    //     double range_offset_raw = r_interp.valid ? r_interp.delta : 0.0f;
    //
    //     bool bad_range_interp = false;
    //     if (tracking_state_.initialized) {
    //         double refined_range_candidate = interpolateAxis(
    //             range_axis_sliced_m_, range_count, best_range_bin, range_offset_raw);
    //         const double candidate_doppler_hz = doppler_axis_hz_[best_doppler_bin];
    //         const double range_correction =
    //             (candidate_doppler_hz / radar_config_.chirpSlopeHzPerS()) *
    //             (radar_config_.speed_of_light_mps / 2.0f);
    //         refined_range_candidate -= range_correction;
    //         double range_jump_m = std::abs(refined_range_candidate - predicted_range_m);
    //         bad_range_interp = range_jump_m > detection_config_.range_interp_gate_m;
    //     }
    //
    //     range_bin_offset = bad_range_interp ? 0.0f : range_offset_raw;
    // }
    //
    best_doppler_hz = interpolateAxis(doppler_axis_hz_, best_doppler_bin, doppler_bin_offset);
    const double refined_raw_range_m =
        interpolateAxis(range_axis_sliced_m_, best_range_bin, range_bin_offset);
    const double refined_range_correction = (best_doppler_hz / radar_config_.chirpSlopeHzPerS()) *
                                            (radar_config_.speed_of_light_mps / 2.0f);
    best_range_m = refined_raw_range_m - refined_range_correction;

    // Measurement gating.
    const double max_range_jump_m = std::max(
        detection_config_.range_association_sigma_m,
        (static_cast<double>(detection_config_.range_gate_bins) + 1.0f) * range_bin_spacing_m);
    const double max_doppler_jump_hz = std::max(
        detection_config_.doppler_association_sigma_hz,
        (static_cast<double>(detection_config_.doppler_gate_bins) + 1.5f) * doppler_bin_spacing_hz);

    bool range_ok = !tracking_state_.initialized ||
                    std::abs(best_range_m - predicted_range_m) < max_range_jump_m;
    bool doppler_ok = !tracking_state_.initialized ||
                      std::abs(best_doppler_hz - predicted_doppler_hz) < max_doppler_jump_hz;

    BatchResult result;
    result.time_s = batch_time_s;
    result.predicted_range_m = predicted_range_m;
    result.predicted_doppler_hz = predicted_doppler_hz;
    result.predicted_direction = predicted_direction;
    result.slow_time_phase_rad.assign(n_chirps, 0.0f);
    result.doppler_slice_power.assign(nfft_doppler, 0.0f);

    if (!std::isfinite(best_range_m) || !std::isfinite(best_doppler_hz) || !range_ok ||
        !doppler_ok) {
        result.valid = false;
        return result;
    }

    result.valid = true;
    result.range_m = best_range_m;
    result.doppler_hz = best_doppler_hz;
    Complex beamformed_response(0.0f, 0.0f);
    if (best_direction_index < directions_.size()) {
        const std::size_t base = best_direction_index * num_rx;
        for (std::size_t rx = 0; rx < num_rx; ++rx) {
            beamformed_response += steering_conj_[base + rx] * best_snapshot[rx];
        }
    } else {
        beamformed_response = best_snapshot[0];
    }
    result.phase_rad = std::arg(beamformed_response);
    result.range_bin_offset = range_bin_offset;
    result.doppler_bin_offset = doppler_bin_offset;
    result.direction = best_direction;
    result.range_bin = best_range_bin;
    result.doppler_bin = best_doppler_bin;
    result.azimuth_bin = best_direction_index / detection_config_.elevation_count;
    result.elevation_bin = best_direction_index % detection_config_.elevation_count;
    if (best_direction_index < directions_.size()) {
        const std::size_t base = best_direction_index * num_rx;
        for (std::size_t chirp = 0; chirp < n_chirps; ++chirp) {
            Complex slow_time_response(0.0f, 0.0f);
            for (std::size_t rx = 0; rx < num_rx; ++rx) {
                slow_time_response +=
                    steering_conj_[base + rx] *
                    spec[cubeIndex(chirp, best_range_bin, rx, range_count, num_rx)];
            }
            result.slow_time_phase_rad[chirp] = std::arg(slow_time_response);
        }
    }
    for (std::size_t dbin = 0; dbin < nfft_doppler; ++dbin) {
        result.doppler_slice_power[dbin] = rd_power[dbin * range_count + best_range_bin];
    }
    return result;
}

TrackSummary StreamingTracker::buildSummary() const {
    TrackSummary summary;
    summary.batch_results = batch_results_;
    summary.microdoppler_truth_frequency_hz =
        description_.cars.empty() ? 0.0f : description_.cars.front().bounce_frequency_hz;
    summary.velocity_axis_mps = velocity_axis_mps_;

    if (batch_results_.empty()) {
        return summary;
    }

    const double wavelength_m = radar_config_.wavelengthM();

    summary.times_s.reserve(batch_results_.size());
    summary.raw_positions_m.reserve(batch_results_.size());
    summary.smoothed_positions_m.reserve(batch_results_.size());
    summary.ranges_m.reserve(batch_results_.size());
    summary.radial_velocity_mps.reserve(batch_results_.size());
    summary.unwrapped_phase_rad.reserve(batch_results_.size());
    summary.detrended_phase_rad.reserve(batch_results_.size());
    summary.cartesian_velocity_mps.reserve(batch_results_.size());
    summary.truth_metrics.reserve(batch_results_.size());

    std::vector<double> raw_x(batch_results_.size());
    std::vector<double> raw_y(batch_results_.size());
    std::vector<double> raw_z(batch_results_.size());

    for (std::size_t i = 0; i < batch_results_.size(); ++i) {
        const BatchResult &batch = batch_results_[i];
        summary.times_s.push_back(batch.time_s);
        const Vec3 raw_xyz = batch.range_m * batch.direction;
        summary.raw_positions_m.push_back(raw_xyz);
        summary.ranges_m.push_back(batch.range_m);
        summary.radial_velocity_mps.push_back(0.5f * batch.doppler_hz * wavelength_m);
        raw_x[i] = raw_xyz.x();
        raw_y[i] = raw_xyz.y();
        raw_z[i] = raw_xyz.z();
        summary.truth_metrics.push_back(truthAtTime(description_, batch.time_s));
    }

    std::vector<double> microdoppler_times_s;
    std::vector<double> microdoppler_unwrapped_phase;
    std::vector<double> residual_phase_example;
    std::vector<double> microdoppler_candidate_frequency_hz;
    std::vector<double> microdoppler_candidate_power;
    const double phase_microdoppler_frequency_hz =
        estimateDominantCpiResidualFrequencyHz(batch_results_,
                                               radar_config_.chirp_duration_s,
                                               &microdoppler_times_s,
                                               &microdoppler_unwrapped_phase,
                                               &residual_phase_example);
    summary.microdoppler_phase_frequency_hz = estimateDominantBatchDopplerFrequencyHz(
        batch_results_,
        static_cast<double>(detection_config_.hop_chirps) * radar_config_.chirp_duration_s,
        &microdoppler_candidate_frequency_hz,
        &microdoppler_candidate_power,
        &summary.microdoppler_peak_power,
        &summary.microdoppler_valid_cpi_count);
    if (summary.microdoppler_phase_frequency_hz <= 0.0f) {
        summary.microdoppler_phase_frequency_hz = phase_microdoppler_frequency_hz;
    }
    summary.unwrapped_phase_rad = microdoppler_unwrapped_phase;
    summary.detrended_phase_rad = residual_phase_example;
    summary.microdoppler_candidate_frequency_hz = microdoppler_candidate_frequency_hz;
    summary.microdoppler_candidate_power = microdoppler_candidate_power;
    const double microdoppler_error_hz =
        summary.microdoppler_phase_frequency_hz - summary.microdoppler_truth_frequency_hz;
    summary.microdoppler_frequency_rmse_hz =
        std::sqrt(microdoppler_error_hz * microdoppler_error_hz);
    if (!summary.detrended_phase_rad.empty()) {
        double sum = 0.0f;
        double squared_sum = 0.0f;
        for (double sample : summary.detrended_phase_rad) {
            sum += sample;
            squared_sum += sample * sample;
        }
        summary.microdoppler_residual_phase_mean_rad =
            sum / static_cast<double>(summary.detrended_phase_rad.size());
        summary.microdoppler_residual_phase_rms_rad =
            std::sqrt(squared_sum / static_cast<double>(summary.detrended_phase_rad.size()));
        double variance_sum = 0.0f;
        for (double sample : summary.detrended_phase_rad) {
            const double centered = sample - summary.microdoppler_residual_phase_mean_rad;
            variance_sum += centered * centered;
        }
        summary.microdoppler_residual_phase_stddev_rad =
            std::sqrt(variance_sum / static_cast<double>(summary.detrended_phase_rad.size()));
    }

    const std::vector<double> &smooth_x = raw_x;
    const std::vector<double> &smooth_y = raw_y;
    const std::vector<double> &smooth_z = raw_z;

    for (std::size_t i = 0; i < batch_results_.size(); ++i) {
        summary.smoothed_positions_m.emplace_back(smooth_x[i], smooth_y[i], smooth_z[i]);
    }

    const std::vector<double> vx = gradient(smooth_x, summary.times_s);
    const std::vector<double> vy = gradient(smooth_y, summary.times_s);
    const std::vector<double> vz = gradient(smooth_z, summary.times_s);

    for (std::size_t i = 0; i < batch_results_.size(); ++i) {
        summary.cartesian_velocity_mps.emplace_back(vx[i], vy[i], vz[i]);
    }

    return summary;
}

problem::SimulationMetrics truthAtTime(const problem::ProblemDescription &description,
                                       double time_s) {
    if (description.cars.empty()) {
        throw std::runtime_error("truthAtTime requires at least one car");
    }

    const CarDynamics dynamics(description.cars.front());
    const radar::TargetObservation observation =
        radar::observeTarget(dynamics, description.radar, time_s);
    return radar::makeSimulationMetrics(time_s, observation);
}

} // namespace fmcw_tracker
