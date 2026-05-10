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
namespace detail {

using Real = problem::Real;
using Vec3 = problem::Vec3;

constexpr Real kRangeDopplerCouplingSign = 1.0f;

std::size_t computeChirpCount(const problem::ProblemDescription &description) {
    const Real chirp_duration_s = static_cast<Real>(problem::RadarSettings::kRadarBlockSize) /
                                  description.radar.sample_rate_hz;
    return static_cast<std::size_t>(
        std::llround(description.simulator.burst_duration_s / chirp_duration_s));
}

RadarConfig makeRadarConfig(const problem::ProblemDescription &description) {
    const Real chirp_duration_s = static_cast<Real>(problem::RadarSettings::kRadarBlockSize) /
                                  description.radar.sample_rate_hz;
    const Real lambda_m = problem::Constants::kSpeedOfLightMps / description.radar.carrier_hz;

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

std::vector<Real> linspace(Real start, Real stop, std::size_t count) {
    std::vector<Real> values(count);
    if (count == 0) {
        return values;
    }
    if (count == 1) {
        values[0] = start;
        return values;
    }

    const Real step = (stop - start) / static_cast<Real>(count - 1);
    for (std::size_t i = 0; i < count; ++i) {
        values[i] = start + static_cast<Real>(i) * step;
    }
    return values;
}

std::vector<Real> hannWindow(std::size_t length) {
    std::vector<Real> window(length, 1.0f);
    if (length <= 1) {
        return window;
    }

    for (std::size_t n = 0; n < length; ++n) {
        window[n] = 0.5f - 0.5f * std::cos(2.0f * problem::Constants::kPi * static_cast<double>(n) /
                                           static_cast<double>(length - 1));
    }
    return window;
}

std::vector<Real> fftFreq(std::size_t nfft, Real sample_period_s) {
    std::vector<Real> freqs(nfft, 0.0f);
    const Real scale = 1.0f / (static_cast<Real>(nfft) * sample_period_s);
    const std::size_t half = nfft / 2;
    for (std::size_t k = 0; k < nfft; ++k) {
        if (k < half) {
            freqs[k] = static_cast<Real>(k) * scale;
        } else {
            freqs[k] =
                static_cast<Real>(static_cast<long long>(k) - static_cast<long long>(nfft)) * scale;
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
            // Match lib/probe.h exactly: ix -> X, iy -> Y, Z=0
            positions[flat].x() =
                (static_cast<Real>(ix) - 0.5f * static_cast<Real>(cfg.probe_num_x - 1)) *
                cfg.probe_dx_m;
            positions[flat].y() =
                (static_cast<Real>(iy) - 0.5f * static_cast<Real>(cfg.probe_num_y - 1)) *
                cfg.probe_dy_m;
            positions[flat].z() = 0.0f;
        }
    }
    return positions;
}

SteeringGrid makeSteeringGrid(const RadarConfig &cfg, const DetectionConfig &det) {
    const std::vector<Vec3> positions_m = elementPositions(cfg);
    const std::vector<Real> az_deg_vec =
        linspace(det.azimuth_min_deg, det.azimuth_max_deg, det.azimuth_count);
    const std::vector<Real> el_deg_vec =
        linspace(det.elevation_min_deg, det.elevation_max_deg, det.elevation_count);

    SteeringGrid grid;
    grid.steering_conj.reserve(det.azimuth_count * det.elevation_count * cfg.numRx());
    grid.directions.reserve(det.azimuth_count * det.elevation_count);
    grid.azimuth_deg.reserve(det.azimuth_count * det.elevation_count);
    grid.elevation_deg.reserve(det.azimuth_count * det.elevation_count);

    for (Real az_deg : az_deg_vec) {
        const Real az = az_deg * problem::Constants::kPi / 180.0f;
        for (Real el_deg : el_deg_vec) {
            const Real el = el_deg * problem::Constants::kPi / 180.0f;
            Vec3 direction;
            // X-forward, Y-left, Z-up coordinate system
            direction.x() = std::cos(el) * std::cos(az);
            direction.y() = std::cos(el) * std::sin(az);
            direction.z() = std::sin(el);
            grid.directions.push_back(direction);
            grid.azimuth_deg.push_back(az_deg);
            grid.elevation_deg.push_back(el_deg);

            for (const Vec3 &position : positions_m) {
                const Real phase =
                    (2.0f * problem::Constants::kPi / cfg.wavelengthM()) * direction.dot(position);
                grid.steering_conj.emplace_back(std::cos(-phase), std::sin(-phase));
            }
        }
    }

    return grid;
}

std::vector<Complex> makeComplexWindow(const std::vector<Real> &real_window) {
    std::vector<Complex> window(real_window.size(), Complex(0.0f, 0.0f));
    const std::size_t count = real_window.size();
    for (std::size_t i = 0; i < count; ++i) {
        window[i] = Complex(real_window[i], 0.0f);
    }
    return window;
}

PeakInterpResult quadraticPeakOffsetChecked(Real ym1, Real y0, Real yp1) {
    const Real denom = ym1 - 2.0f * y0 + yp1;
    if (std::abs(denom) < 1.0e-12f) {
        return {0.0f, false};
    }
    const Real delta = 0.5f * (ym1 - yp1) / denom;
    return {delta, std::abs(delta) <= 1.0f};
}

Real interpolateAxis(const std::vector<Real> &axis, std::size_t bin, Real offset) {
    if (axis.empty()) {
        return 0.0f;
    }
    if (axis.size() == 1U) {
        return axis[0];
    }
    const std::size_t n = axis.size();
    if (bin == 0U) {
        const Real step = axis[1] - axis[0];
        return axis[0] + offset * step;
    }
    if (bin + 1U >= n) {
        const Real step = axis[n - 1] - axis[n - 2];
        return axis[n - 1] + offset * step;
    }
    if (offset >= 0.0f) {
        const Real step = axis[bin + 1] - axis[bin];
        return axis[bin] + offset * step;
    }
    const Real step = axis[bin] - axis[bin - 1];
    return axis[bin] + offset * step;
}

std::vector<double> gradient(const std::vector<double> &y, const std::vector<double> &x) {
    if (y.size() < 2U || y.size() != x.size()) {
        return std::vector<double>(y.size(), 0.0);
    }
    const std::size_t n = y.size();
    std::vector<double> dy_dx(n);
    dy_dx[0] = (y[1] - y[0]) / std::max(1.0e-9, x[1] - x[0]);
    for (std::size_t i = 1; i + 1 < n; ++i) {
        dy_dx[i] = (y[i + 1] - y[i - 1]) / std::max(1.0e-9, x[i + 1] - x[i - 1]);
    }
    dy_dx[n - 1] = (y[n - 1] - y[n - 2]) / std::max(1.0e-9, x[n - 1] - x[n - 2]);
    return dy_dx;
}

Real degrees(Real radians) {
    return radians * 180.0f / problem::Constants::kPi;
}

Real azimuthDeg(const Vec3 &direction) {
    return degrees(std::atan2(direction.y(), direction.x()));
}

Real elevationDeg(const Vec3 &direction) {
    const Real r = direction.norm();
    if (r < 1.0e-6f) {
        return 0.0f;
    }
    return degrees(std::asin(std::clamp(direction.z() / r, -1.0f, 1.0f)));
}

Real wrapAngleDeg(Real angle_deg) {
    while (angle_deg > 180.0f) {
        angle_deg -= 360.0f;
    }
    while (angle_deg <= -180.0f) {
        angle_deg += 360.0f;
    }
    return angle_deg;
}

Real angleDifferenceDeg(Real lhs_deg, Real rhs_deg) {
    return wrapAngleDeg(lhs_deg - rhs_deg);
}

Vec3 normalizedOr(const Vec3 &value, const Vec3 &fallback) {
    const Real norm = value.norm();
    if (norm < 1.0e-6f) {
        return fallback;
    }
    return value / norm;
}

KinematicEstimate makeKinematicEstimate(const RadarConfig &cfg,
                                        const Vec3 &position_m,
                                        const Vec3 &velocity_mps,
                                        const Vec3 &direction_fallback) {
    KinematicEstimate estimate;
    estimate.position_m = position_m;
    estimate.velocity_mps = velocity_mps;
    estimate.range_m = position_m.norm();
    estimate.direction = normalizedOr(position_m, direction_fallback);
    estimate.radial_velocity_mps = velocity_mps.dot(estimate.direction);
    estimate.doppler_hz = 2.0f * estimate.radial_velocity_mps / cfg.wavelengthM();
    return estimate;
}

std::size_t nearestAxisBin(const std::vector<Real> &axis, Real value) {
    if (axis.empty()) {
        return 0U;
    }
    auto it = std::lower_bound(axis.begin(), axis.end(), value);
    if (it == axis.begin()) {
        return 0U;
    }
    if (it == axis.end()) {
        return axis.size() - 1U;
    }
    const std::size_t idx = static_cast<std::size_t>(std::distance(axis.begin(), it));
    if (std::abs(*it - value) < std::abs(*(it - 1) - value)) {
        return idx;
    }
    return idx - 1U;
}

std::vector<double> unwrapPhases(const std::vector<double> &phases) {
    if (phases.empty()) {
        return {};
    }
    std::vector<double> unwrapped(phases.size());
    double cumulative_jump = 0.0;
    unwrapped[0] = phases[0];
    for (std::size_t i = 1; i < phases.size(); ++i) {
        double diff = phases[i] - phases[i - 1];
        if (diff > problem::Constants::kPi) {
            cumulative_jump -= 2.0 * problem::Constants::kPi;
        } else if (diff < -problem::Constants::kPi) {
            cumulative_jump += 2.0 * problem::Constants::kPi;
        }
        unwrapped[i] = phases[i] + cumulative_jump;
    }
    return unwrapped;
}

std::vector<double> movingAverageSame(const std::vector<double> &signal, std::size_t window_size) {
    if (window_size <= 1U || signal.empty()) {
        return signal;
    }
    const std::size_t n = signal.size();
    std::vector<double> result(n);
    double sum = 0.0;
    const std::size_t half = window_size / 2U;
    for (std::size_t i = 0; i < n; ++i) {
        sum = 0.0;
        std::size_t count = 0U;
        const std::size_t start = (i > half) ? i - half : 0U;
        const std::size_t end = std::min(n, i + half + 1U);
        for (std::size_t j = start; j < end; ++j) {
            sum += signal[j];
            ++count;
        }
        result[i] = sum / static_cast<double>(count);
    }
    return result;
}

struct Candidate {
    double frequency_hz;
    double power;
};

struct SegmentBounds {
    std::size_t begin;
    std::size_t end;
};

double estimateDominantCpiResidualFrequencyHz(const std::vector<BatchResult> &batch_results,
                                              double chirp_duration_s,
                                              std::vector<double> *times_s_out = nullptr,
                                              std::vector<double> *unwrapped_phase_out = nullptr,
                                              std::vector<double> *residual_phase_out = nullptr) {
    constexpr double kMinSearchFrequencyHz = 10.0f;
    constexpr double kMaxSearchFrequencyHz = 100.0f;
    constexpr std::size_t kBoundaryFitSamples = 8U;

    if (chirp_duration_s <= 0.0f) {
        return 0.0f;
    }

    std::vector<double> times_s;
    std::vector<double> wrapped_phase_rad;
    const std::size_t chirps_per_cpi =
        batch_results.empty() ? 0U : batch_results.front().slow_time_phase_rad.size();
    if (chirps_per_cpi == 0U) {
        return 0.0f;
    }

    std::vector<SegmentBounds> continuous_segments;
    std::vector<SegmentBounds> batch_segments;
    long long next_expected_chirp = -1;

    std::size_t valid_batch_count = 0;
    for (const BatchResult &batch : batch_results) {
        if (batch.valid) {
            ++valid_batch_count;
        }
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
            batch.time_s / chirp_duration_s - 0.5f * static_cast<Real>(chirps_per_cpi)));
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
            const std::complex<double> baseband =
                std::polar(1.0, raw_phase) * std::polar(1.0, -coarse_phase);
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
            times_s_out->assign(times_s.begin(), times_s.end());
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
            times_s_out->assign(times_s.begin(), times_s.end());
        }
        if (unwrapped_phase_out != nullptr) {
            unwrapped_phase_out->assign(unwrapped_phase_all.begin(), unwrapped_phase_all.end());
        }
        if (residual_phase_out != nullptr) {
            residual_phase_out->assign(filtered_phase_all.begin(), filtered_phase_all.end());
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
    std::vector<std::complex<double>> fft_output;
    std::vector<std::complex<double>> fft_input(nfft, std::complex<double>(0.0, 0.0));

    for (const SegmentBounds &segment : continuous_segments) {
        const std::size_t length = segment.end - segment.begin;
        std::vector<double> segment_wrapped(wrapped_phase_rad.begin() + segment.begin,
                                            wrapped_phase_rad.begin() + segment.end);
        std::vector<double> segment_unwrapped(length);
        double cumulative_jump = 0.0;
        segment_unwrapped[0] = segment_wrapped[0];
        for (std::size_t i = 1; i < length; ++i) {
            double diff = segment_wrapped[i] - segment_wrapped[i - 1];
            if (diff > problem::Constants::kPi) {
                cumulative_jump -= 2.0 * problem::Constants::kPi;
            } else if (diff < -problem::Constants::kPi) {
                cumulative_jump += 2.0 * problem::Constants::kPi;
            }
            segment_unwrapped[i] = segment_wrapped[i] + cumulative_jump;
        }

        std::vector<double> segment_filtered = segment_unwrapped;
        if (length > kBoundaryFitSamples * 2U) {
            double sum_x = 0.0, sum_y = 0.0, sum_xx = 0.0, sum_xy = 0.0;
            for (std::size_t i = 0; i < length; ++i) {
                const double t = times_s[segment.begin + i] - times_s[segment.begin];
                sum_x += t;
                sum_y += segment_unwrapped[i];
                sum_xx += t * t;
                sum_xy += t * segment_unwrapped[i];
            }
            const double det = static_cast<double>(length) * sum_xx - sum_x * sum_x;
            const double slope = (static_cast<double>(length) * sum_xy - sum_x * sum_y) / det;
            const double intercept = (sum_y - slope * sum_x) / static_cast<double>(length);
            for (std::size_t i = 0; i < length; ++i) {
                const double t = times_s[segment.begin + i] - times_s[segment.begin];
                segment_filtered[i] -= (intercept + slope * t);
            }
        }

        unwrapped_phase_all.insert(unwrapped_phase_all.end(),
                                   segment_unwrapped.begin(),
                                   segment_unwrapped.end());
        filtered_phase_all.insert(filtered_phase_all.end(),
                                  segment_filtered.begin(),
                                  segment_filtered.end());

        std::fill(fft_input.begin(), fft_input.end(), std::complex<double>(0.0, 0.0));
        const std::vector<Real> win = hannWindow(length);
        for (std::size_t i = 0; i < length; ++i) {
            fft_input[i] = segment_filtered[i] * win[i];
        }
        fft.fwd(fft_output, fft_input);
        for (std::size_t i = 0; i < nfft / 2U; ++i) {
            accumulated_power[i] += std::norm(fft_output[i]);
        }
    }

    if (times_s_out != nullptr) {
        times_s_out->assign(times_s.begin(), times_s.end());
    }
    if (unwrapped_phase_out != nullptr) {
        *unwrapped_phase_out = unwrapped_phase_all;
    }
    if (residual_phase_out != nullptr) {
        *residual_phase_out = filtered_phase_all;
    }

    double best_power = -1.0;
    std::size_t best_bin = 0U;
    const double min_bin = kMinSearchFrequencyHz * (static_cast<double>(nfft) * chirp_duration_s);
    const double max_bin = kMaxSearchFrequencyHz * (static_cast<double>(nfft) * chirp_duration_s);

    for (std::size_t i = static_cast<std::size_t>(std::max(1.0, std::floor(min_bin)));
         i < std::min<std::size_t>(nfft / 2U, static_cast<std::size_t>(std::ceil(max_bin)));
         ++i) {
        if (accumulated_power[i] > best_power) {
            best_power = accumulated_power[i];
            best_bin = i;
        }
    }

    double best_frequency_hz = 0.0;
    if (best_power > 0.0) {
        best_frequency_hz = static_cast<double>(best_bin) / (static_cast<double>(nfft) * chirp_duration_s);
        if (best_bin > 0U && best_bin + 1U < nfft / 2U) {
            const PeakInterpResult interp = quadraticPeakOffsetChecked(
                static_cast<Real>(accumulated_power[best_bin - 1]),
                static_cast<Real>(accumulated_power[best_bin]),
                static_cast<Real>(accumulated_power[best_bin + 1]));
            if (interp.valid) {
                best_frequency_hz = (static_cast<double>(best_bin) + interp.delta) /
                                    (static_cast<double>(nfft) * chirp_duration_s);
            }
        }
    }

    return best_frequency_hz;
}

double estimateDominantBatchDopplerFrequencyHz(const std::vector<BatchResult> &batch_results,
                                               double batch_period_s,
                                               std::vector<double> *candidate_frequency_hz_out = nullptr,
                                               std::vector<double> *candidate_power_out = nullptr,
                                               double *peak_power_out = nullptr,
                                               std::size_t *valid_cpi_count_out = nullptr) {
    if (batch_results.empty() || batch_period_s <= 0.0) {
        return 0.0;
    }

    std::vector<std::complex<double>> slow_time;
    slow_time.reserve(batch_results.size());
    std::size_t valid_count = 0;
    for (const auto &batch : batch_results) {
        if (batch.valid) {
            slow_time.push_back(std::polar(1.0, static_cast<double>(batch.phase_rad)));
            ++valid_count;
        } else {
            slow_time.push_back(std::complex<double>(0.0, 0.0));
        }
    }

    if (valid_cpi_count_out != nullptr) {
        *valid_cpi_count_out = valid_count;
    }

    if (valid_count < 4U) {
        return 0.0;
    }

    std::size_t nfft = 1U;
    while (nfft < slow_time.size()) {
        nfft <<= 1U;
    }
    nfft <<= 3U;

    std::vector<std::complex<double>> fft_input(nfft, std::complex<double>(0.0, 0.0));
    const std::vector<Real> win = hannWindow(slow_time.size());
    for (std::size_t i = 0; i < slow_time.size(); ++i) {
        fft_input[i] = slow_time[i] * static_cast<double>(win[i]);
    }

    Eigen::FFT<double> fft;
    std::vector<std::complex<double>> fft_output;
    fft.fwd(fft_output, fft_input);

    std::vector<double> accumulated_power(nfft / 2U);
    for (std::size_t i = 0; i < nfft / 2U; ++i) {
        accumulated_power[i] = std::norm(fft_output[i]);
    }

    constexpr double kMinFreqHz = 5.0;
    constexpr double kMaxFreqHz = 100.0;
    const std::size_t min_bin =
        static_cast<std::size_t>(std::floor(kMinFreqHz * static_cast<double>(nfft) * batch_period_s));
    const std::size_t max_bin =
        static_cast<std::size_t>(std::ceil(kMaxFreqHz * static_cast<double>(nfft) * batch_period_s));

    double best_power = -1.0;
    std::size_t best_bin = 0U;
    std::vector<Candidate> candidates;
    for (std::size_t i = std::max<std::size_t>(1U, min_bin);
         i < std::min<std::size_t>(nfft / 2U, max_bin);
         ++i) {
        const double p = accumulated_power[i];
        candidates.push_back({static_cast<double>(i) / (static_cast<double>(nfft) * batch_period_s), p});
        if (p > best_power) {
            best_power = p;
            best_bin = i;
        }
    }

    double best_frequency_hz = 0.0;
    if (best_bin > 0U && best_bin + 1U < accumulated_power.size()) {
        const PeakInterpResult interp =
            quadraticPeakOffsetChecked(static_cast<Real>(accumulated_power[best_bin - 1U]),
                                       static_cast<Real>(accumulated_power[best_bin]),
                                       static_cast<Real>(accumulated_power[best_bin + 1U]));
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

Vec3 measurementPosition(const MeasurementCandidate &measurement) {
    return measurement.range_m * measurement.direction;
}

Vec3 measurementVelocity(const MeasurementCandidate &measurement) {
    return measurement.radial_velocity_mps * measurement.direction;
}

Real measurementAzimuthDeg(const MeasurementCandidate &measurement) {
    return azimuthDeg(measurement.direction);
}

Real measurementElevationDeg(const MeasurementCandidate &measurement) {
    return elevationDeg(measurement.direction);
}

} // namespace detail

using detail::AssignmentSolution;
using detail::KinematicEstimate;
using detail::PeakInterpResult;
using detail::SceneObservation;
using detail::SteeringGrid;
using detail::Vec3;
using namespace detail;

StreamingTracker::StreamingTracker(const problem::ProblemDescription &description,
                                   DetectionConfig detection_config)
    : description_(description), radar_config_(makeRadarConfig(description)),
      detection_config_(detection_config),
      chirp_window_(std::make_unique<RingBuffer<ChirpBlock, kMaxCpiChirps>>()) {
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
    const std::vector<Real> freqs =
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

        const Real range_m = -radar_config_.speed_of_light_mps * beat_frequency_hz /
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

    range_window_ = makeComplexWindow(hannWindow(radar_config_.block_size));
    doppler_window_ =
        makeComplexWindow(hannWindow(detection_config_.coherent_processing_interval_chirps));

    const SteeringGrid grid = makeSteeringGrid(radar_config_, detection_config_);
    steering_conj_ = grid.steering_conj;
    directions_ = grid.directions;
    direction_azimuth_deg_ = grid.azimuth_deg;
    direction_elevation_deg_ = grid.elevation_deg;

    // Doppler axis: zero-pad to 2x CPI for finer frequency resolution
    // (~305 Hz/bin instead of ~610 Hz/bin). The truth model uses
    // f_D = 2*v_r/lambda (positive = receding). The FFT on the beat-signal
    // slow-time produces the opposite sign, so we negate the axis.
    const std::size_t nfft_doppler = 2U * detection_config_.coherent_processing_interval_chirps;
    const std::vector<Real> doppler_axis_native_hz =
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
    if (tx_chirp.size() != radar_config_.block_size) {
        throw std::runtime_error("unexpected TX chirp size");
    }
    if (rx_block.size() != radar_config_.block_size * radar_config_.numRx()) {
        throw std::runtime_error("unexpected RX block size");
    }

    const std::size_t num_rx = radar_config_.numRx();
    ChirpBlock chirp_block{};
    for (std::size_t sample = 0; sample < radar_config_.block_size; ++sample) {
        const Complex dechirp = std::conj(tx_chirp[sample]);
        for (std::size_t rx = 0; rx < num_rx; ++rx) {
            chirp_block[sample * num_rx + rx] = rx_block[sample * num_rx + rx] * dechirp;
        }
    }
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

    const SceneObservation observation = processCurrentWindow(start_chirp);
    updateTracks(observation);
}

detail::SceneObservation StreamingTracker::processCurrentWindow(std::size_t start_chirp) {
    const std::size_t n_chirps = detection_config_.coherent_processing_interval_chirps;
    const std::size_t nfft_doppler = n_chirps * 2U;
    const std::size_t num_rx = radar_config_.numRx();
    const std::size_t range_count = range_bin_count_;

    // Range FFT: window already dechirped data and transform each chirp.
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
                fft_input[sample] = rx_block[sample * num_rx + rx] * range_window_[sample];
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

    // Doppler FFT: zero-padded to 2x CPI for finer Doppler resolution.
    std::vector<Complex> &rd_cube = rd_cube_scratch_;
    std::vector<Real> &rd_power = rd_power_scratch_;
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

    for (Real &power : rd_power) {
        power /= static_cast<Real>(num_rx);
    }

    const Real batch_time_s =
        (static_cast<Real>(start_chirp) + 0.5f * static_cast<Real>(n_chirps)) *
        radar_config_.chirp_duration_s;
    SceneObservation observation;
    observation.time_s = batch_time_s;
    observation.n_chirps = n_chirps;
    observation.nfft_doppler = nfft_doppler;
    observation.range_count = range_count;
    observation.num_rx = num_rx;
    observation.spec_cube = spec;
    observation.rd_cube = rd_cube;

    const std::vector<DetectionCell> detections =
        detectLocalPeakCells(rd_power, range_count, nfft_doppler, detection_config_);

    const std::vector<std::vector<DetectionCell>> clusters =
        clusterNearbyPeaks(detections, detection_config_);

    constexpr std::size_t kMaxClustersPerCpi = 16U;
    const std::size_t cluster_count = std::min<std::size_t>(clusters.size(), kMaxClustersPerCpi);
    observation.measurements.reserve(cluster_count);

    if (cluster_count == 0) {
        return observation;
    }

    // 1. Extract snapshots for all clusters at their peak RD cell.
    Eigen::Matrix<Complex, kNumRx, Eigen::Dynamic> snapshots(kNumRx, cluster_count);
    for (std::size_t i = 0; i < cluster_count; ++i) {
        const auto &cluster = clusters[i];
        const auto peak_it = std::max_element(
            cluster.begin(), cluster.end(), [](const DetectionCell &lhs, const DetectionCell &rhs) {
                return lhs.power < rhs.power;
            });
        for (std::size_t rx = 0; rx < num_rx; ++rx) {
            snapshots(rx, i) = rd_cube[cubeIndex(peak_it->doppler_bin, peak_it->range_bin, rx, range_count, num_rx)];
        }
    }

    // 2. Batched beamforming across all directions and all clusters.
    Eigen::Map<const Eigen::Matrix<Complex, Eigen::Dynamic, kNumRx, Eigen::RowMajor>>
        steering_matrix(steering_conj_.data(), directions_.size(), kNumRx);
    
    Eigen::Matrix<Complex, Eigen::Dynamic, Eigen::Dynamic> responses = 
        steering_matrix * snapshots;

    // 3. Process each cluster to form measurement candidates.
    for (std::size_t cluster_index = 0; cluster_index < cluster_count; ++cluster_index) {
        const auto &cluster = clusters[cluster_index];
        const auto peak_it = std::max_element(
            cluster.begin(), cluster.end(), [](const DetectionCell &lhs, const DetectionCell &rhs) {
                return lhs.power < rhs.power;
            });
        const std::size_t best_doppler_bin = peak_it->doppler_bin;
        const std::size_t best_range_bin = peak_it->range_bin;

        int best_direction_index_int = 0;
        responses.col(cluster_index).array().abs2().maxCoeff(&best_direction_index_int);
        const std::size_t best_direction_index = static_cast<std::size_t>(best_direction_index_int);
        const Vec3 best_direction = directions_[best_direction_index];

        Real doppler_bin_offset = 0.0f;
        if (best_doppler_bin > 0U && best_doppler_bin + 1U < nfft_doppler) {
            const PeakInterpResult interp = quadraticPeakOffsetChecked(
                rd_power[(best_doppler_bin - 1U) * range_count + best_range_bin],
                rd_power[best_doppler_bin * range_count + best_range_bin],
                rd_power[(best_doppler_bin + 1U) * range_count + best_range_bin]);
            doppler_bin_offset = interp.valid ? interp.delta : 0.0f;
        }

        Real range_bin_offset = 0.0f;
        if (best_range_bin > 0U && best_range_bin + 1U < range_count) {
            const PeakInterpResult interp = quadraticPeakOffsetChecked(
                rd_power[best_doppler_bin * range_count + (best_range_bin - 1U)],
                rd_power[best_doppler_bin * range_count + best_range_bin],
                rd_power[best_doppler_bin * range_count + (best_range_bin + 1U)]);
            range_bin_offset = interp.valid ? interp.delta : 0.0f;
        }

        const Real doppler_hz =
            interpolateAxis(doppler_axis_hz_, best_doppler_bin, doppler_bin_offset);
        const Real raw_range_m =
            interpolateAxis(range_axis_sliced_m_, best_range_bin, range_bin_offset);
        const Real range_correction = kRangeDopplerCouplingSign *
                                      (doppler_hz / radar_config_.chirpSlopeHzPerS()) *
                                      (radar_config_.speed_of_light_mps / 2.0f);
        const Real range_m = raw_range_m + range_correction;
        if (!std::isfinite(range_m) || !std::isfinite(doppler_hz) || range_m <= 0.0f) {
            continue;
        }

        MeasurementCandidate measurement;
        measurement.time_s = batch_time_s;
        measurement.range_m = range_m;
        measurement.doppler_hz = doppler_hz;
        measurement.radial_velocity_mps = 0.5f * doppler_hz * radar_config_.wavelengthM();
        measurement.range_bin_offset = range_bin_offset;
        measurement.doppler_bin_offset = doppler_bin_offset;
        measurement.direction = best_direction;
        measurement.range_bin = best_range_bin;
        measurement.doppler_bin = best_doppler_bin;
        measurement.azimuth_bin = best_direction_index / detection_config_.elevation_count;
        measurement.elevation_bin = best_direction_index % detection_config_.elevation_count;
        measurement.cluster_size = cluster.size();
        measurement.cells = cluster;
        measurement.doppler_slice_power.assign(nfft_doppler, 0.0f);
        measurement.slow_time_phase_rad.assign(n_chirps, 0.0f);

        double integrated_power = 0.0;
        double peak_power = 0.0;
        for (const DetectionCell &cell : cluster) {
            integrated_power += cell.power;
            peak_power = std::max<double>(peak_power, cell.power);
        }
        measurement.integrated_power = static_cast<Real>(integrated_power);
        measurement.peak_power = static_cast<Real>(peak_power);

        const Complex beamformed_response = responses(best_direction_index, cluster_index);
        measurement.phase_rad = std::arg(beamformed_response);

        const std::size_t base = best_direction_index * num_rx;
        for (std::size_t chirp = 0; chirp < n_chirps; ++chirp) {
            Complex slow_time_response(0.0f, 0.0f);
            for (std::size_t rx = 0; rx < num_rx; ++rx) {
                slow_time_response +=
                    steering_conj_[base + rx] *
                    spec[cubeIndex(chirp, best_range_bin, rx, range_count, num_rx)];
            }
            measurement.slow_time_phase_rad[chirp] = std::arg(slow_time_response);
        }

        for (std::size_t dbin = 0; dbin < nfft_doppler; ++dbin) {
            measurement.doppler_slice_power[dbin] = rd_power[dbin * range_count + best_range_bin];
        }
        observation.measurements.push_back(std::move(measurement));
    }

    observation.measurements = mergeNearbyMeasurements(observation.measurements, detection_config_);
    observation.measurements = selectTopMeasurements(std::move(observation.measurements),
                                                     detection_config_.max_measurements_per_cpi);

    return observation;
}

std::vector<StreamingTracker::AoAPeak>
StreamingTracker::estimateTopAoAs(std::span<const Complex> snapshot, std::size_t n) const {
    if (snapshot.size() != radar_config_.numRx()) {
        return {};
    }

    const std::size_t num_rx = radar_config_.numRx();
    const std::size_t grid_size = directions_.size();
    std::vector<AoAPeak> peaks;

    for (std::size_t i = 0; i < grid_size; ++i) {
        Complex response(0.0f, 0.0f);
        const std::size_t base = i * num_rx;
        for (std::size_t rx = 0; rx < num_rx; ++rx) {
            response += steering_conj_[base + rx] * snapshot[rx];
        }
        const Real score = std::norm(response);
        peaks.push_back({direction_azimuth_deg_[i], direction_elevation_deg_[i], score, directions_[i]});
    }

    std::sort(peaks.begin(), peaks.end(), [](const AoAPeak &a, const AoAPeak &b) {
        return a.score > b.score;
    });

    if (peaks.size() > n) {
        peaks.resize(n);
    }
    return peaks;
}

std::size_t StreamingTracker::getRangeBin(Real range_m) const {
    return detail::nearestAxisBin(range_axis_sliced_m_, range_m);
}

Complex StreamingTracker::computeRangeFftBin(std::span<const Complex> dechirped_chirp,
                                             std::size_t rx,
                                             std::size_t range_bin) const {
    if (range_bin >= range_indices_.size()) {
        return {0.0f, 0.0f};
    }

    const std::size_t num_rx = radar_config_.numRx();
    const std::size_t nfft = range_fft_input_.size();
    
    // This is a bit inefficient for a single bin, but ensures consistency with processCurrentWindow
    std::vector<Complex> fft_input(nfft, Complex(0.0f, 0.0f));
    for (std::size_t sample = range_wrap_guard_samples_; sample < radar_config_.block_size; ++sample) {
        fft_input[sample] = dechirped_chirp[sample * num_rx + rx] * range_window_[sample];
    }
    
    // We use a local FFT to avoid mutating member scratchpads in a const method if possible,
    // but StreamingTracker has an Eigen::FFT member. Since this is for offline analysis, 
    // we'll just do it simply.
    Eigen::FFT<Real> fft;
    std::vector<Complex> fft_output;
    fft.fwd(fft_output, fft_input);
    
    return fft_output[static_cast<std::size_t>(range_indices_[range_bin])];
}

} // namespace fmcw_tracker
