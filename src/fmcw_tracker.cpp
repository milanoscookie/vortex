// #include "fmcw_tracker.h"
//
// #include "dynamics.h"
// #include "target_observation.h"
//
// #include <unsupported/Eigen/FFT>
//
// #include <algorithm>
// #include <cmath>
// #include <limits>
// #include <stdexcept>
// #include <vector>
//
// namespace fmcw_tracker {
// namespace {
//
// using Vec3 = problem::Vec3;
//
// std::size_t computeChirpCount(const problem::ProblemDescription &description) {
//     const float chirp_duration_s =
//         static_cast<float>(problem::Constants::kRadarBlockSize) /
//         description.radar.sample_rate_hz;
//     return static_cast<std::size_t>(
//         std::llround(description.simulator.burst_duration_s / chirp_duration_s));
// }
//
// RadarConfig makeRadarConfig(const problem::ProblemDescription &description) {
//     const float chirp_duration_s =
//         static_cast<float>(problem::Constants::kRadarBlockSize) /
//         description.radar.sample_rate_hz;
//     const float lambda_m = problem::Constants::kSpeedOfLightMps / description.radar.carrier_hz;
//
//     RadarConfig config;
//     config.sample_rate_hz = description.radar.sample_rate_hz;
//     config.carrier_hz = description.radar.carrier_hz;
//     config.bandwidth_hz = description.radar.bandwidth_hz;
//     config.chirp_duration_s = chirp_duration_s;
//     config.speed_of_light_mps = problem::Constants::kSpeedOfLightMps;
//     config.block_size = problem::Constants::kRadarBlockSize;
//     config.chirp_count = computeChirpCount(description);
//     config.probe_num_x = problem::RadarSettings::kProbeNumX;
//     config.probe_num_y = problem::RadarSettings::kProbeNumY;
//     config.probe_dx_m = description.probe.spacing_x_wavelengths * lambda_m;
//     config.probe_dy_m = description.probe.spacing_y_wavelengths * lambda_m;
//     return config;
// }
//
// std::vector<float> linspace(float start, float stop, std::size_t count) {
//     std::vector<float> values(count);
//     if (count == 0) {
//         return values;
//     }
//     if (count == 1) {
//         values[0] = start;
//         return values;
//     }
//
//     const float step = (stop - start) / static_cast<float>(count - 1);
//     for (std::size_t i = 0; i < count; ++i) {
//         values[i] = start + static_cast<float>(i) * step;
//     }
//     return values;
// }
//
// std::vector<float> hannWindow(std::size_t length) {
//     std::vector<float> window(length, 1.0f);
//     if (length <= 1) {
//         return window;
//     }
//
//     for (std::size_t n = 0; n < length; ++n) {
//         window[n] = 0.5f - 0.5f * std::cos(2.0f * problem::Constants::kPi * static_cast<float>(n)
//         /
//                                            static_cast<float>(length - 1));
//     }
//     return window;
// }
//
// std::vector<float> fftFreq(std::size_t nfft, float sample_period_s) {
//     std::vector<float> freqs(nfft, 0.0f);
//     const float scale = 1.0f / (static_cast<float>(nfft) * sample_period_s);
//     const std::size_t half = nfft / 2;
//     for (std::size_t k = 0; k < nfft; ++k) {
//         if (k < half) {
//             freqs[k] = static_cast<float>(k) * scale;
//         } else {
//             freqs[k] =
//                 static_cast<float>(static_cast<long long>(k) - static_cast<long long>(nfft)) *
//                 scale;
//         }
//     }
//     return freqs;
// }
//
// template <typename T> void fftShiftInPlace(std::vector<T> &values) {
//     if (values.empty()) {
//         return;
//     }
//     std::rotate(values.begin(),
//                 values.begin() + static_cast<std::ptrdiff_t>(values.size() / 2),
//                 values.end());
// }
//
// template <typename T, int Size> void fftShiftInPlace(Eigen::Matrix<T, Size, 1> &values) {
//     constexpr int kHalf = Size / 2;
//     Eigen::Matrix<T, Size, 1> shifted;
//     shifted.template head<Size - kHalf>() = values.template tail<Size - kHalf>();
//     shifted.template tail<kHalf>() = values.template head<kHalf>();
//     values = shifted;
// }
//
// std::size_t cubeIndex(std::size_t outer,
//                       std::size_t middle,
//                       std::size_t inner,
//                       std::size_t middle_size,
//                       std::size_t inner_size) {
//     return (outer * middle_size + middle) * inner_size + inner;
// }
//
// std::vector<Vec3> elementPositions(const RadarConfig &cfg) {
//     std::vector<Vec3> positions(cfg.numRx(), Vec3::Zero());
//     for (std::size_t ix = 0; ix < cfg.probe_num_x; ++ix) {
//         for (std::size_t iy = 0; iy < cfg.probe_num_y; ++iy) {
//             const std::size_t flat = ix * cfg.probe_num_y + iy;
//             positions[flat].x() =
//                 (static_cast<float>(ix) - 0.5f * static_cast<float>(cfg.probe_num_x - 1)) *
//                 cfg.probe_dx_m;
//             positions[flat].y() =
//                 (static_cast<float>(iy) - 0.5f * static_cast<float>(cfg.probe_num_y - 1)) *
//                 cfg.probe_dy_m;
//         }
//     }
//     return positions;
// }
//
// struct SteeringGrid {
//     std::vector<Complex> steering_conj;
//     std::vector<Vec3> directions;
// };
//
// SteeringGrid makeSteeringGrid(const RadarConfig &cfg, const DetectionConfig &det) {
//     const std::vector<Vec3> positions_m = elementPositions(cfg);
//     const std::vector<float> az_rad =
//         linspace(det.azimuth_min_deg, det.azimuth_max_deg, det.azimuth_count);
//     const std::vector<float> el_rad =
//         linspace(det.elevation_min_deg, det.elevation_max_deg, det.elevation_count);
//
//     SteeringGrid grid;
//     grid.steering_conj.reserve(det.azimuth_count * det.elevation_count * cfg.numRx());
//     grid.directions.reserve(det.azimuth_count * det.elevation_count);
//
//     for (float az_deg : az_rad) {
//         const float az = az_deg * problem::Constants::kPi / 180.0f;
//         for (float el_deg : el_rad) {
//             const float el = el_deg * problem::Constants::kPi / 180.0f;
//             Vec3 direction;
//             direction.x() = std::cos(el) * std::cos(az);
//             direction.y() = std::cos(el) * std::sin(az);
//             direction.z() = std::sin(el);
//             grid.directions.push_back(direction);
//
//             for (const Vec3 &position : positions_m) {
//                 const float phase =
//                     (2.0f * problem::Constants::kPi / cfg.wavelengthM()) *
//                     direction.dot(position);
//                 grid.steering_conj.emplace_back(std::cos(-phase), std::sin(-phase));
//             }
//         }
//     }
//
//     return grid;
// }
//
// std::vector<Complex> makeComplexWindow(const std::vector<float> &real_window) {
//     std::vector<Complex> window(real_window.size(), Complex(0.0f, 0.0f));
//     for (std::size_t i = 0; i < real_window.size(); ++i) {
//         window[i] = Complex(real_window[i], 0.0f);
//     }
//     return window;
// }
//
// template <int Size>
// Eigen::Matrix<Complex, Size, 1> makeComplexWindowEigen(const std::vector<float> &real_window) {
//     Eigen::Matrix<Complex, Size, 1> window = Eigen::Matrix<Complex, Size, 1>::Zero();
//     const std::size_t count = std::min<std::size_t>(Size, real_window.size());
//     for (std::size_t i = 0; i < count; ++i) {
//         window[static_cast<Eigen::Index>(i)] = Complex(real_window[i], 0.0f);
//     }
//     return window;
// }
//
// StreamingTracker::ChirpReference makeComplexWindowArray(const std::vector<float> &real_window) {
//     StreamingTracker::ChirpReference window{};
//     const std::size_t count = std::min(window.size(),
//     static_cast<std::size_t>(real_window.size())); for (std::size_t i = 0; i < count; ++i) {
//         window[i] = Complex(real_window[i], 0.0f);
//     }
//     return window;
// }
//
// float degrees(float radians) {
//     return radians * 180.0f / problem::Constants::kPi;
// }
//
// float azimuthDeg(const Vec3 &direction) {
//     return degrees(std::atan2(direction.y(), direction.x()));
// }
//
// float elevationDeg(const Vec3 &direction) {
//     return degrees(std::atan2(direction.z(), std::hypot(direction.x(), direction.y())));
// }
//
// float wrapAngleDeg(float angle_deg) {
//     while (angle_deg > 180.0f) {
//         angle_deg -= 360.0f;
//     }
//     while (angle_deg < -180.0f) {
//         angle_deg += 360.0f;
//     }
//     return angle_deg;
// }
//
// float angleDifferenceDeg(float lhs_deg, float rhs_deg) {
//     return wrapAngleDeg(lhs_deg - rhs_deg);
// }
//
// Vec3 normalizedOr(const Vec3 &value, const Vec3 &fallback) {
//     const float norm = value.norm();
//     if (!std::isfinite(norm) || norm <= 1.0e-6f) {
//         return fallback;
//     }
//     return value / norm;
// }
//
// struct KinematicEstimate {
//     Vec3 position_m = Vec3::Zero();
//     Vec3 velocity_mps = Vec3::Zero();
//     Vec3 direction = Vec3::UnitX();
//     float range_m = 0.0f;
//     float radial_velocity_mps = 0.0f;
//     float doppler_hz = 0.0f;
// };
//
// KinematicEstimate makeKinematicEstimate(const RadarConfig &cfg,
//                                         const Vec3 &position_m,
//                                         const Vec3 &velocity_mps,
//                                         const Vec3 &direction_fallback) {
//     KinematicEstimate estimate;
//     estimate.position_m = position_m;
//     estimate.velocity_mps = velocity_mps;
//     estimate.range_m = position_m.norm();
//     estimate.direction = normalizedOr(position_m, normalizedOr(direction_fallback,
//     Vec3::UnitX())); estimate.radial_velocity_mps = velocity_mps.dot(estimate.direction);
//     estimate.doppler_hz = 2.0f * estimate.radial_velocity_mps / cfg.wavelengthM();
//     return estimate;
// }
//
// float associationPenalty(float error, float sigma) {
//     if (sigma <= 0.0f) {
//         return 0.0f;
//     }
//     const float normalized_error = error / sigma;
//     return -0.5f * normalized_error * normalized_error;
// }
//
// struct PeakInterpResult {
//     float delta = 0.0f;
//     bool valid = true;
// };
//
// PeakInterpResult
// quadraticPeakOffsetChecked(float power_minus, float power_center, float power_plus) {
//     constexpr float kLogEpsilon = 1.0e-12f;
//     const float y_minus = std::log(power_minus + kLogEpsilon);
//     const float y_center = std::log(power_center + kLogEpsilon);
//     const float y_plus = std::log(power_plus + kLogEpsilon);
//     const float denom = y_minus - 2.0f * y_center + y_plus;
//
//     PeakInterpResult out{};
//
//     if (!(denom < -1.0e-12f)) {
//         out.delta = 0.0f;
//         out.valid = false;
//         return out;
//     }
//
//     const float raw_delta = 0.5f * (y_minus - y_plus) / denom;
//
//     if (!std::isfinite(raw_delta) || std::abs(raw_delta) >= 0.45f) {
//         out.delta = 0.0f;
//         out.valid = false;
//         return out;
//     }
//
//     out.delta = raw_delta;
//     out.valid = true;
//     return out;
// }
//
// float interpolateAxis(const std::vector<float> &axis, std::size_t bin, float offset) {
//     if (axis.empty() || bin >= axis.size()) {
//         return 0.0f;
//     }
//     if (offset > 0.0f && bin + 1 < axis.size()) {
//         return axis[bin] + offset * (axis[bin + 1] - axis[bin]);
//     }
//     if (offset < 0.0f && bin > 0) {
//         return axis[bin] + offset * (axis[bin] - axis[bin - 1]);
//     }
//     return axis[bin];
// }
//
// template <int Size>
// float interpolateAxis(const Eigen::Matrix<float, Size, 1> &axis,
//                       std::size_t count,
//                       std::size_t bin,
//                       float offset) {
//     if (count == 0 || bin >= count) {
//         return 0.0f;
//     }
//     if (offset > 0.0f && bin + 1 < count) {
//         return axis[static_cast<Eigen::Index>(bin)] +
//                offset * (axis[static_cast<Eigen::Index>(bin + 1)] -
//                          axis[static_cast<Eigen::Index>(bin)]);
//     }
//     if (offset < 0.0f && bin > 0) {
//         return axis[static_cast<Eigen::Index>(bin)] +
//                offset * (axis[static_cast<Eigen::Index>(bin)] -
//                          axis[static_cast<Eigen::Index>(bin - 1)]);
//     }
//     return axis[static_cast<Eigen::Index>(bin)];
// }
//
// template <int Size>
// std::size_t
// nearestAxisBin(const Eigen::Matrix<float, Size, 1> &axis, std::size_t count, float value) {
//     if (count == 0) {
//         return 0;
//     }
//
//     std::size_t best_bin = 0;
//     float best_error = std::abs(axis[0] - value);
//     for (std::size_t i = 1; i < count; ++i) {
//         const float error = std::abs(axis[static_cast<Eigen::Index>(i)] - value);
//         if (error < best_error) {
//             best_error = error;
//             best_bin = i;
//         }
//     }
//     return best_bin;
// }
//
// std::vector<float> unwrapPhases(const std::vector<float> &phases) {
//     if (phases.empty()) {
//         return {};
//     }
//
//     std::vector<float> unwrapped(phases.size(), 0.0f);
//     unwrapped[0] = phases[0];
//     float offset = 0.0f;
//     for (std::size_t i = 1; i < phases.size(); ++i) {
//         const float delta = phases[i] - phases[i - 1];
//         if (delta > problem::Constants::kPi) {
//             offset -= 2.0f * problem::Constants::kPi;
//         } else if (delta < -problem::Constants::kPi) {
//             offset += 2.0f * problem::Constants::kPi;
//         }
//         unwrapped[i] = phases[i] + offset;
//     }
//     return unwrapped;
// }
//
// std::vector<float> convolveSame(const std::vector<float> &signal,
//                                 const std::vector<float> &kernel) {
//     if (signal.empty() || kernel.empty()) {
//         return {};
//     }
//
//     std::vector<float> full(signal.size() + kernel.size() - 1, 0.0f);
//     for (std::size_t i = 0; i < signal.size(); ++i) {
//         for (std::size_t k = 0; k < kernel.size(); ++k) {
//             full[i + k] += signal[i] * kernel[k];
//         }
//     }
//
//     const std::size_t start = (kernel.size() - 1) / 2;
//     return std::vector<float>(full.begin() + static_cast<std::ptrdiff_t>(start),
//                               full.begin() + static_cast<std::ptrdiff_t>(start + signal.size()));
// }
//
// std::vector<float> movingAverageSame(const std::vector<float> &signal, std::size_t window_size) {
//     if (signal.empty()) {
//         return {};
//     }
//
//     window_size = std::max<std::size_t>(1U, window_size);
//     std::vector<float> prefix_sum(signal.size() + 1U, 0.0f);
//     for (std::size_t i = 0; i < signal.size(); ++i) {
//         prefix_sum[i + 1U] = prefix_sum[i] + signal[i];
//     }
//
//     std::vector<float> averaged(signal.size(), 0.0f);
//     const std::size_t half_window = window_size / 2U;
//     for (std::size_t i = 0; i < signal.size(); ++i) {
//         const std::size_t begin = (i > half_window) ? (i - half_window) : 0U;
//         const std::size_t end = std::min(signal.size(), i + half_window + 1U);
//         const float sum = prefix_sum[end] - prefix_sum[begin];
//         averaged[i] = sum / static_cast<float>(end - begin);
//     }
//     return averaged;
// }
//
// std::vector<float> gradient(const std::vector<float> &values, const std::vector<float> &times_s)
// {
//     const std::size_t count = values.size();
//     std::vector<float> grad(count, 0.0f);
//     if (count < 2) {
//         return grad;
//     }
//
//     const float first_dt = times_s[1] - times_s[0];
//     const float last_dt = times_s[count - 1] - times_s[count - 2];
//     grad[0] = (values[1] - values[0]) / first_dt;
//     grad[count - 1] = (values[count - 1] - values[count - 2]) / last_dt;
//
//     for (std::size_t i = 1; i + 1 < count; ++i) {
//         const float dt = times_s[i + 1] - times_s[i - 1];
//         grad[i] = (values[i + 1] - values[i - 1]) / dt;
//     }
//
//     return grad;
// }
//
// std::vector<float> detrendPhase(const std::vector<float> &times_s,
//                                 const std::vector<float> &phase_rad) {
//     if (times_s.size() != phase_rad.size() || phase_rad.empty()) {
//         return {};
//     }
//     if (phase_rad.size() == 1U) {
//         return std::vector<float>(1U, 0.0f);
//     }
//
//     float sum_t = 0.0f;
//     float sum_p = 0.0f;
//     float sum_tt = 0.0f;
//     float sum_tp = 0.0f;
//     for (std::size_t i = 0; i < phase_rad.size(); ++i) {
//         const float t = times_s[i];
//         const float p = phase_rad[i];
//         sum_t += t;
//         sum_p += p;
//         sum_tt += t * t;
//         sum_tp += t * p;
//     }
//
//     const float count = static_cast<float>(phase_rad.size());
//     const float denom = count * sum_tt - sum_t * sum_t;
//     float slope = 0.0f;
//     float intercept = phase_rad.front();
//     if (std::abs(denom) > 1.0e-9f) {
//         slope = (count * sum_tp - sum_t * sum_p) / denom;
//         intercept = (sum_p - slope * sum_t) / count;
//     }
//
//     std::vector<float> residual(phase_rad.size(), 0.0f);
//     for (std::size_t i = 0; i < phase_rad.size(); ++i) {
//         residual[i] = phase_rad[i] - (intercept + slope * times_s[i]);
//     }
//     return residual;
// }
//
// float estimateDominantPhaseFrequencyHz(const std::vector<float> &times_s,
//                                        const std::vector<float> &detrended_phase_rad) {
//     if (times_s.size() != detrended_phase_rad.size() || detrended_phase_rad.size() < 4U) {
//         return 0.0f;
//     }
//     constexpr float kMinSearchFrequencyHz = 10.0f;
//     constexpr float kMaxSearchFrequencyHz = 100.0f;
//
//     const float dt = std::max((times_s.back() - times_s.front()) /
//                                   static_cast<float>(detrended_phase_rad.size() - 1U),
//                               1.0e-6f);
//
//     const std::size_t n = detrended_phase_rad.size();
//     std::size_t nfft = 1U;
//     while (nfft < n) {
//         nfft <<= 1U;
//     }
//     nfft <<= 1U;
//
//     const std::vector<float> window = hannWindow(n);
//     std::vector<Complex> fft_input(nfft, Complex(0.0f, 0.0f));
//     for (std::size_t i = 0; i < n; ++i) {
//         fft_input[i] = Complex(detrended_phase_rad[i] * window[i], 0.0f);
//     }
//
//     Eigen::FFT<float> fft;
//     std::vector<Complex> fft_output;
//     fft.fwd(fft_output, fft_input);
//
//     float best_power = 0.0f;
//     float best_frequency_hz = 0.0f;
//     const std::size_t half = nfft / 2U;
//     for (std::size_t bin = 1U; bin < half; ++bin) {
//         const float frequency_hz = static_cast<float>(bin) / (static_cast<float>(nfft) * dt);
//         if (frequency_hz < kMinSearchFrequencyHz) {
//             continue;
//         }
//         if (frequency_hz > kMaxSearchFrequencyHz) {
//             continue;
//         }
//         const float power = std::norm(fft_output[bin]);
//         if (power > best_power) {
//             best_power = power;
//             best_frequency_hz = frequency_hz;
//         }
//     }
//
//     return best_frequency_hz;
// }
//
// float estimateDominantResidualSlowTimeFrequencyHz(
//     const std::vector<BatchResult> &batch_results,
//     float chirp_duration_s,
//     std::vector<float> *residual_phase_example = nullptr) {
//     constexpr float kMinSearchFrequencyHz = 10.0f;
//     constexpr float kMaxSearchFrequencyHz = 100.0f;
//
//     const std::size_t n = BatchResult::kSlowTimeSize;
//     if (batch_results.empty() || chirp_duration_s <= 0.0f) {
//         return 0.0f;
//     }
//
//     std::vector<float> cpi_times_s(n, 0.0f);
//     for (std::size_t i = 0; i < n; ++i) {
//         cpi_times_s[i] = static_cast<float>(i) * chirp_duration_s;
//     }
//
//     std::size_t nfft = 1U;
//     while (nfft < n) {
//         nfft <<= 1U;
//     }
//     nfft <<= 1U;
//
//     const std::vector<float> window = hannWindow(n);
//     std::vector<float> accumulated_power(nfft / 2U, 0.0f);
//     std::size_t valid_cpi_count = 0;
//
//     Eigen::FFT<float> fft;
//     std::vector<Complex> fft_input(nfft, Complex(0.0f, 0.0f));
//     std::vector<Complex> fft_output;
//
//     for (const BatchResult &batch : batch_results) {
//         if (!batch.valid) {
//             continue;
//         }
//
//         std::vector<float> phase_trace(n, 0.0f);
//         for (std::size_t chirp = 0; chirp < n; ++chirp) {
//             phase_trace[chirp] = batch.slow_time_phase_rad[static_cast<Eigen::Index>(chirp)];
//         }
//
//         const std::vector<float> unwrapped_phase = unwrapPhases(phase_trace);
//         const std::vector<float> residual_phase = detrendPhase(cpi_times_s, unwrapped_phase);
//         if (residual_phase.size() != n) {
//             continue;
//         }
//
//         if (residual_phase_example != nullptr && residual_phase_example->empty()) {
//             *residual_phase_example = residual_phase;
//         }
//
//         std::fill(fft_input.begin(), fft_input.end(), Complex(0.0f, 0.0f));
//         for (std::size_t i = 0; i < n; ++i) {
//             fft_input[i] = Complex(residual_phase[i] * window[i], 0.0f);
//         }
//
//         fft.fwd(fft_output, fft_input);
//         for (std::size_t bin = 1U; bin < accumulated_power.size(); ++bin) {
//             accumulated_power[bin] += std::norm(fft_output[bin]);
//         }
//         ++valid_cpi_count;
//     }
//
//     if (valid_cpi_count == 0) {
//         return 0.0f;
//     }
//
//     float best_power = 0.0f;
//     float best_frequency_hz = 0.0f;
//     for (std::size_t bin = 1U; bin < accumulated_power.size(); ++bin) {
//         const float frequency_hz =
//             static_cast<float>(bin) / (static_cast<float>(nfft) * chirp_duration_s);
//         if (frequency_hz < kMinSearchFrequencyHz || frequency_hz > kMaxSearchFrequencyHz) {
//             continue;
//         }
//         if (accumulated_power[bin] > best_power) {
//             best_power = accumulated_power[bin];
//             best_frequency_hz = frequency_hz;
//         }
//     }
//
//     return best_frequency_hz;
// }
//
// float estimateDominantCpiResidualFrequencyHz(
//     const std::vector<BatchResult> &batch_results,
//     float chirp_duration_s,
//     std::vector<float> *times_s_out = nullptr,
//     std::vector<float> *unwrapped_phase_out = nullptr,
//     std::vector<float> *residual_phase_out = nullptr,
//     std::vector<float> *candidate_frequency_hz_out = nullptr,
//     std::vector<float> *candidate_power_out = nullptr,
//     float *peak_power_out = nullptr,
//     std::size_t *valid_cpi_count_out = nullptr) {
//     constexpr float kMinSearchFrequencyHz = 10.0f;
//     constexpr float kMaxSearchFrequencyHz = 100.0f;
//     constexpr float kHighPassCutoffHz = 15.0f;
//     constexpr std::size_t kBoundaryFitSamples = 8U;
//
//     if (chirp_duration_s <= 0.0f) {
//         return 0.0f;
//     }
//
//     const std::size_t chirps_per_cpi = BatchResult::kSlowTimeSize;
//     const float chirp_rate_hz = 1.0f / chirp_duration_s;
//     const std::size_t high_pass_window = std::max<std::size_t>(
//         3U, static_cast<std::size_t>(std::llround(chirp_rate_hz / kHighPassCutoffHz)));
//
//     struct SegmentBounds {
//         std::size_t begin = 0U;
//         std::size_t end = 0U;
//     };
//
//     std::vector<float> times_s;
//     std::vector<float> wrapped_phase_rad;
//     std::vector<SegmentBounds> continuous_segments;
//     std::vector<SegmentBounds> batch_segments;
//     times_s.reserve(batch_results.size() * chirps_per_cpi);
//     wrapped_phase_rad.reserve(batch_results.size() * chirps_per_cpi);
//
//     long long next_expected_chirp = -1;
//     std::size_t valid_batch_count = 0U;
//     for (const BatchResult &batch : batch_results) {
//         if (!batch.valid) {
//             continue;
//         }
//         ++valid_batch_count;
//     }
//
//     if (valid_cpi_count_out != nullptr) {
//         *valid_cpi_count_out = valid_batch_count;
//     }
//
//     if (valid_batch_count == 0U) {
//         return 0.0f;
//     }
//
//     for (const BatchResult &batch : batch_results) {
//         if (!batch.valid) {
//             next_expected_chirp = -1;
//             continue;
//         }
//
//         const long long cpi_start_chirp = static_cast<long long>(std::llround(
//             batch.time_s / chirp_duration_s - 0.5f * static_cast<float>(chirps_per_cpi)));
//         const long long first_new_chirp = (next_expected_chirp < 0)
//                                               ? cpi_start_chirp
//                                               : std::max(cpi_start_chirp, next_expected_chirp);
//         const long long append_count =
//             static_cast<long long>(chirps_per_cpi) - (first_new_chirp - cpi_start_chirp);
//         if (append_count <= 0) {
//             continue;
//         }
//
//         if (continuous_segments.empty() || first_new_chirp != next_expected_chirp) {
//             continuous_segments.push_back(SegmentBounds{times_s.size(), times_s.size()});
//         }
//
//         const std::size_t batch_begin = times_s.size();
//         const std::size_t first_local_chirp =
//             static_cast<std::size_t>(first_new_chirp - cpi_start_chirp);
//         for (std::size_t chirp = first_local_chirp; chirp < chirps_per_cpi; ++chirp) {
//             const long long global_chirp = cpi_start_chirp + static_cast<long long>(chirp);
//             const float chirp_time_s = static_cast<float>(global_chirp) * chirp_duration_s;
//             times_s.push_back(chirp_time_s);
//
//             const float raw_phase = batch.slow_time_phase_rad[static_cast<Eigen::Index>(chirp)];
//             const float local_time_s = static_cast<float>(chirp) * chirp_duration_s;
//             const float coarse_phase =
//                 2.0f * problem::Constants::kPi * batch.doppler_hz * local_time_s;
//             const Complex baseband = std::polar(1.0f, raw_phase) * std::polar(1.0f,
//             -coarse_phase); wrapped_phase_rad.push_back(std::arg(baseband));
//         }
//
//         const std::size_t batch_end = times_s.size();
//         if (batch_end > batch_begin) {
//             batch_segments.push_back(SegmentBounds{batch_begin, batch_end});
//         }
//
//         continuous_segments.back().end = times_s.size();
//         next_expected_chirp = cpi_start_chirp + static_cast<long long>(chirps_per_cpi);
//     }
//
//     if (wrapped_phase_rad.size() < 4U || continuous_segments.empty()) {
//         if (times_s_out != nullptr) {
//             *times_s_out = std::move(times_s);
//         }
//         if (unwrapped_phase_out != nullptr) {
//             *unwrapped_phase_out = {};
//         }
//         if (residual_phase_out != nullptr) {
//             *residual_phase_out = {};
//         }
//         return 0.0f;
//     }
//
//     std::vector<float> filtered_phase_all;
//     std::vector<float> unwrapped_phase_all;
//     filtered_phase_all.reserve(wrapped_phase_rad.size());
//     unwrapped_phase_all.reserve(wrapped_phase_rad.size());
//
//     std::size_t max_segment_length = 0U;
//     for (const SegmentBounds &segment : continuous_segments) {
//         max_segment_length = std::max(max_segment_length, segment.end - segment.begin);
//     }
//
//     if (max_segment_length < 4U) {
//         if (times_s_out != nullptr) {
//             *times_s_out = std::move(times_s);
//         }
//         if (unwrapped_phase_out != nullptr) {
//             *unwrapped_phase_out = std::move(unwrapped_phase_all);
//         }
//         if (residual_phase_out != nullptr) {
//             *residual_phase_out = std::move(filtered_phase_all);
//         }
//         return 0.0f;
//     }
//
//     std::size_t nfft = 1U;
//     while (nfft < max_segment_length) {
//         nfft <<= 1U;
//     }
//     nfft <<= 2U;
//
//     std::vector<float> accumulated_power(nfft / 2U, 0.0f);
//     Eigen::FFT<float> fft;
//     std::vector<Complex> fft_output;
//     std::vector<Complex> fft_input(nfft, Complex(0.0f, 0.0f));
//
//     std::size_t batch_segment_index = 0U;
//     for (const SegmentBounds &continuous_segment : continuous_segments) {
//         const std::size_t segment_length = continuous_segment.end - continuous_segment.begin;
//         if (segment_length < 4U) {
//             while (batch_segment_index < batch_segments.size() &&
//                    batch_segments[batch_segment_index].end <= continuous_segment.end) {
//                 ++batch_segment_index;
//             }
//             continue;
//         }
//
//         std::vector<float> aligned_unwrapped_segment;
//         aligned_unwrapped_segment.reserve(segment_length);
//
//         while (batch_segment_index < batch_segments.size() &&
//                batch_segments[batch_segment_index].begin < continuous_segment.end) {
//             const SegmentBounds batch_segment = batch_segments[batch_segment_index++];
//             if (batch_segment.end <= continuous_segment.begin) {
//                 continue;
//             }
//
//             const std::vector<float> wrapped_batch_segment(
//                 wrapped_phase_rad.begin() + static_cast<std::ptrdiff_t>(batch_segment.begin),
//                 wrapped_phase_rad.begin() + static_cast<std::ptrdiff_t>(batch_segment.end));
//             std::vector<float> unwrapped_batch_segment = unwrapPhases(wrapped_batch_segment);
//             if (unwrapped_batch_segment.empty()) {
//                 continue;
//             }
//
//             if (!aligned_unwrapped_segment.empty()) {
//                 const std::size_t prev_count =
//                     std::min<std::size_t>(kBoundaryFitSamples, aligned_unwrapped_segment.size());
//                 float predicted_next_phase = aligned_unwrapped_segment.back();
//                 if (aligned_unwrapped_segment.size() >= 2U) {
//                     float step_sum = 0.0f;
//                     std::size_t step_count = 0U;
//                     const std::size_t start = aligned_unwrapped_segment.size() - prev_count;
//                     for (std::size_t i = start + 1U; i < aligned_unwrapped_segment.size(); ++i) {
//                         step_sum +=
//                             aligned_unwrapped_segment[i] - aligned_unwrapped_segment[i - 1U];
//                         ++step_count;
//                     }
//                     if (step_count > 0U) {
//                         predicted_next_phase += step_sum / static_cast<float>(step_count);
//                     }
//                 }
//
//                 const float phase_offset = predicted_next_phase -
//                 unwrapped_batch_segment.front(); for (float &sample : unwrapped_batch_segment) {
//                     sample += phase_offset;
//                 }
//             }
//
//             aligned_unwrapped_segment.insert(aligned_unwrapped_segment.end(),
//                                              unwrapped_batch_segment.begin(),
//                                              unwrapped_batch_segment.end());
//         }
//
//         if (aligned_unwrapped_segment.size() != segment_length) {
//             continue;
//         }
//
//         const std::vector<float> low_pass_segment =
//             movingAverageSame(aligned_unwrapped_segment,
//                               std::min(high_pass_window, aligned_unwrapped_segment.size()));
//
//         std::vector<float> filtered_segment(aligned_unwrapped_segment.size(), 0.0f);
//         for (std::size_t i = 0; i < aligned_unwrapped_segment.size(); ++i) {
//             filtered_segment[i] = aligned_unwrapped_segment[i] - low_pass_segment[i];
//         }
//
//         unwrapped_phase_all.insert(unwrapped_phase_all.end(),
//                                    aligned_unwrapped_segment.begin(),
//                                    aligned_unwrapped_segment.end());
//         filtered_phase_all.insert(
//             filtered_phase_all.end(), filtered_segment.begin(), filtered_segment.end());
//
//         const std::vector<float> window = hannWindow(filtered_segment.size());
//         std::fill(fft_input.begin(), fft_input.end(), Complex(0.0f, 0.0f));
//         for (std::size_t i = 0; i < filtered_segment.size(); ++i) {
//             fft_input[i] = Complex(filtered_segment[i] * window[i], 0.0f);
//         }
//
//         fft.fwd(fft_output, fft_input);
//         for (std::size_t bin = 1U; bin < accumulated_power.size(); ++bin) {
//             accumulated_power[bin] += std::norm(fft_output[bin]);
//         }
//     }
//
//     if (times_s_out != nullptr) {
//         *times_s_out = times_s;
//     }
//     if (unwrapped_phase_out != nullptr) {
//         *unwrapped_phase_out = unwrapped_phase_all;
//     }
//     if (residual_phase_out != nullptr) {
//         *residual_phase_out = filtered_phase_all;
//     }
//
//     struct Candidate {
//         float frequency_hz = 0.0f;
//         float power = 0.0f;
//     };
//     std::vector<Candidate> candidates;
//     float best_power = 0.0f;
//     float best_frequency_hz = 0.0f;
//     const std::size_t half = nfft / 2U;
//     for (std::size_t bin = 1U; bin < half; ++bin) {
//         const float frequency_hz =
//             static_cast<float>(bin) / (static_cast<float>(nfft) * chirp_duration_s);
//         if (frequency_hz < kMinSearchFrequencyHz || frequency_hz > kMaxSearchFrequencyHz) {
//             continue;
//         }
//         const float power = accumulated_power[bin];
//         candidates.push_back(Candidate{frequency_hz, power});
//         if (power > best_power) {
//             best_power = power;
//             best_frequency_hz = frequency_hz;
//         }
//     }
//
//     std::partial_sort(
//         candidates.begin(),
//         candidates.begin() + std::min<std::size_t>(5U, candidates.size()),
//         candidates.end(),
//         [](const Candidate &lhs, const Candidate &rhs) { return lhs.power > rhs.power; });
//
//     if (candidate_frequency_hz_out != nullptr) {
//         candidate_frequency_hz_out->clear();
//         for (std::size_t i = 0; i < std::min<std::size_t>(5U, candidates.size()); ++i) {
//             candidate_frequency_hz_out->push_back(candidates[i].frequency_hz);
//         }
//     }
//     if (candidate_power_out != nullptr) {
//         candidate_power_out->clear();
//         for (std::size_t i = 0; i < std::min<std::size_t>(5U, candidates.size()); ++i) {
//             candidate_power_out->push_back(candidates[i].power);
//         }
//     }
//     if (peak_power_out != nullptr) {
//         *peak_power_out = best_power;
//     }
//
//     return best_frequency_hz;
// }
//
// float estimateDominantBatchDopplerFrequencyHz(
//     const std::vector<BatchResult> &batch_results,
//     float batch_period_s,
//     std::vector<float> *candidate_frequency_hz_out = nullptr,
//     std::vector<float> *candidate_power_out = nullptr,
//     float *peak_power_out = nullptr,
//     std::size_t *valid_batch_count_out = nullptr) {
//     constexpr float kMinSearchFrequencyHz = 10.0f;
//     constexpr float kMaxSearchFrequencyHz = 100.0f;
//     constexpr float kHighPassCutoffHz = 10.0f;
//
//     if (batch_period_s <= 0.0f) {
//         return 0.0f;
//     }
//
//     struct SegmentBounds {
//         std::size_t begin = 0U;
//         std::size_t end = 0U;
//     };
//
//     std::vector<float> doppler_trace_hz;
//     std::vector<SegmentBounds> segments;
//     doppler_trace_hz.reserve(batch_results.size());
//
//     std::size_t valid_batch_count = 0U;
//     for (const BatchResult &batch : batch_results) {
//         if (!batch.valid) {
//             continue;
//         }
//         ++valid_batch_count;
//     }
//
//     if (valid_batch_count_out != nullptr) {
//         *valid_batch_count_out = valid_batch_count;
//     }
//
//     if (valid_batch_count < 4U) {
//         return 0.0f;
//     }
//
//     bool previous_valid = false;
//     for (const BatchResult &batch : batch_results) {
//         if (!batch.valid) {
//             previous_valid = false;
//             continue;
//         }
//
//         if (!previous_valid) {
//             segments.push_back(SegmentBounds{doppler_trace_hz.size(), doppler_trace_hz.size()});
//         }
//
//         // doppler_trace_hz.push_back(batch.doppler_hz);
//         doppler_trace_hz.push_back(batch.doppler_hz - batch.predicted_doppler_hz);
//         segments.back().end = doppler_trace_hz.size();
//         previous_valid = true;
//     }
//
//     std::size_t max_segment_length = 0U;
//     for (const SegmentBounds &segment : segments) {
//         max_segment_length = std::max(max_segment_length, segment.end - segment.begin);
//     }
//     if (max_segment_length < 4U) {
//         return 0.0f;
//     }
//
//     std::size_t nfft = 1U;
//     while (nfft < max_segment_length) {
//         nfft <<= 1U;
//     }
//     nfft <<= 2U;
//
//     const float batch_rate_hz = 1.0f / batch_period_s;
//     const std::size_t high_pass_window = std::max<std::size_t>(
//         3U, static_cast<std::size_t>(std::llround(batch_rate_hz / kHighPassCutoffHz)));
//
//     std::vector<float> accumulated_power(nfft / 2U, 0.0f);
//     Eigen::FFT<float> fft;
//     std::vector<Complex> fft_output;
//     std::vector<Complex> fft_input(nfft, Complex(0.0f, 0.0f));
//
//     for (const SegmentBounds &segment : segments) {
//         const std::size_t segment_length = segment.end - segment.begin;
//         if (segment_length < 4U) {
//             continue;
//         }
//
//         const std::vector<float> doppler_segment(
//             doppler_trace_hz.begin() + static_cast<std::ptrdiff_t>(segment.begin),
//             doppler_trace_hz.begin() + static_cast<std::ptrdiff_t>(segment.end));
//         const std::vector<float> low_pass_segment =
//             movingAverageSame(doppler_segment, std::min(high_pass_window, segment_length));
//
//         std::vector<float> filtered_segment(segment_length, 0.0f);
//         for (std::size_t i = 0; i < segment_length; ++i) {
//             filtered_segment[i] = doppler_segment[i] - low_pass_segment[i];
//         }
//
//         const std::vector<float> window = hannWindow(segment_length);
//         std::fill(fft_input.begin(), fft_input.end(), Complex(0.0f, 0.0f));
//         for (std::size_t i = 0; i < segment_length; ++i) {
//             fft_input[i] = Complex(filtered_segment[i] * window[i], 0.0f);
//         }
//
//         fft.fwd(fft_output, fft_input);
//         for (std::size_t bin = 1U; bin < accumulated_power.size(); ++bin) {
//             accumulated_power[bin] += std::norm(fft_output[bin]);
//         }
//     }
//
//     struct Candidate {
//         float frequency_hz = 0.0f;
//         float power = 0.0f;
//     };
//     std::vector<Candidate> candidates;
//     float best_power = 0.0f;
//     float best_frequency_hz = 0.0f;
//     std::size_t best_bin = 0U;
//     const std::size_t half = nfft / 2U;
//     for (std::size_t bin = 1U; bin < half; ++bin) {
//         const float frequency_hz =
//             static_cast<float>(bin) / (static_cast<float>(nfft) * batch_period_s);
//         if (frequency_hz < kMinSearchFrequencyHz || frequency_hz > kMaxSearchFrequencyHz) {
//             continue;
//         }
//
//         const float power = accumulated_power[bin];
//         candidates.push_back(Candidate{frequency_hz, power});
//         if (power > best_power) {
//             best_power = power;
//             best_frequency_hz = frequency_hz;
//             best_bin = bin;
//         }
//     }
//
//     if (best_bin > 0U && best_bin + 1U < accumulated_power.size()) {
//         const PeakInterpResult interp =
//             quadraticPeakOffsetChecked(accumulated_power[best_bin - 1U],
//                                        accumulated_power[best_bin],
//                                        accumulated_power[best_bin + 1U]);
//         if (interp.valid) {
//             best_frequency_hz = (static_cast<float>(best_bin) + interp.delta) /
//                                 (static_cast<float>(nfft) * batch_period_s);
//         }
//     }
//
//     std::partial_sort(
//         candidates.begin(),
//         candidates.begin() + std::min<std::size_t>(5U, candidates.size()),
//         candidates.end(),
//         [](const Candidate &lhs, const Candidate &rhs) { return lhs.power > rhs.power; });
//
//     if (candidate_frequency_hz_out != nullptr) {
//         candidate_frequency_hz_out->clear();
//         for (std::size_t i = 0; i < std::min<std::size_t>(5U, candidates.size()); ++i) {
//             candidate_frequency_hz_out->push_back(candidates[i].frequency_hz);
//         }
//     }
//     if (candidate_power_out != nullptr) {
//         candidate_power_out->clear();
//         for (std::size_t i = 0; i < std::min<std::size_t>(5U, candidates.size()); ++i) {
//             candidate_power_out->push_back(candidates[i].power);
//         }
//     }
//     if (peak_power_out != nullptr) {
//         *peak_power_out = best_power;
//     }
//
//     return best_frequency_hz;
// }
//
// } // namespace
//
// StreamingTracker::StreamingTracker(const problem::ProblemDescription &description,
//                                    DetectionConfig detection_config)
//     : description_(description), radar_config_(makeRadarConfig(description)),
//       detection_config_(detection_config),
//       chirp_window_(std::make_unique<RingBuffer<ChirpBlock, kMaxCpiChirps>>()) {
//     if (description_.cars.size() != 1) {
//         throw std::runtime_error("StreamingTracker currently supports exactly one car");
//     }
//     if (detection_config_.coherent_processing_interval_chirps > kMaxCpiChirps) {
//         throw std::runtime_error("tracker CPI exceeds RingBuffer capacity");
//     }
//     if (detection_config_.coherent_processing_interval_chirps != kFixedCpiChirps) {
//         throw std::runtime_error("tracker currently requires 64-chirp CPI");
//     }
//     if (detection_config_.nfft_range_min != kFixedRangeNfft) {
//         throw std::runtime_error("tracker currently requires 4096-point range FFT");
//     }
//     if (detection_config_.azimuth_count != kFixedAzimuthCount ||
//         detection_config_.elevation_count != kFixedElevationCount) {
//         throw std::runtime_error("tracker AoA grid must match fixed compile-time sizes");
//     }
//
//     // Range axis: use only negative-frequency FFT bins (down-chirp produces
//     // negative beat frequencies for targets). Map to range via
//     // R = -c * f_beat / (2 * chirp_slope).
//     const std::vector<float> freqs =
//         fftFreq(detection_config_.nfft_range_min, 1.0f / radar_config_.sample_rate_hz);
//     range_bin_count_ = 0;
//     for (std::size_t i = 0; i < freqs.size(); ++i) {
//         const float beat_frequency_hz = freqs[i];
//         if (beat_frequency_hz > 0.0f) {
//             continue;
//         }
//
//         const float range_m = -radar_config_.speed_of_light_mps * beat_frequency_hz /
//                               (2.0f * radar_config_.chirpSlopeHzPerS());
//         if (range_m >= detection_config_.min_range_m && range_m <= detection_config_.max_range_m)
//         {
//             if (range_bin_count_ >= kFixedRangeBinCount) {
//                 throw std::runtime_error("range gate exceeds fixed bin capacity");
//             }
//             range_indices_[range_bin_count_] = static_cast<int>(i);
//             range_axis_sliced_m_[static_cast<Eigen::Index>(range_bin_count_)] = range_m;
//             ++range_bin_count_;
//         }
//     }
//     if (range_bin_count_ == 0) {
//         throw std::runtime_error("tracker range gate produced no bins");
//     }
//
//     range_window_ = makeComplexWindowArray(hannWindow(radar_config_.block_size));
//     doppler_window_ = makeComplexWindowEigen<static_cast<int>(kFixedCpiChirps)>(
//         hannWindow(detection_config_.coherent_processing_interval_chirps));
//
//     const SteeringGrid grid = makeSteeringGrid(radar_config_, detection_config_);
//     steering_conj_ = grid.steering_conj;
//     directions_ = grid.directions;
//
//     // Doppler axis: zero-pad to 2x CPI for finer frequency resolution
//     // (~305 Hz/bin instead of ~610 Hz/bin). The truth model uses
//     // f_D = 2*v_r/lambda (positive = approaching). The FFT on the beat-signal
//     // slow-time produces the opposite sign, so we negate the axis.
//     const std::size_t nfft_doppler = 2U * detection_config_.coherent_processing_interval_chirps;
//     const std::vector<float> doppler_freqs = fftFreq(nfft_doppler,
//     radar_config_.chirp_duration_s); for (std::size_t i = 0; i < nfft_doppler; ++i) {
//         doppler_axis_hz_[static_cast<Eigen::Index>(i)] = doppler_freqs[i];
//     }
//     fftShiftInPlace(doppler_axis_hz_);
//     for (Eigen::Index i = 0; i < doppler_axis_hz_.size(); ++i) {
//         doppler_axis_hz_[i] = -doppler_axis_hz_[i];
//     }
//
//     for (Eigen::Index i = 0; i < velocity_axis_mps_.size(); ++i) {
//         velocity_axis_mps_[i] = 0.5f * doppler_axis_hz_[i] * radar_config_.wavelengthM();
//     }
//
//     // The simulator keeps a continuous TX history, so the first fast-time
//     // samples of a chirp can include delayed energy from the previous chirp.
//     // Blanking that prefix avoids wrap-around contamination. Cap the guard
//     // to a fraction of the block so extreme-range configs don't starve the
//     // range FFT.
//     const float max_tracked_range_m =
//         std::min(description_.radar.max_range_m, detection_config_.max_range_m);
//     const float max_delay_samples =
//         (2.0f * max_tracked_range_m / radar_config_.speed_of_light_mps) *
//         radar_config_.sample_rate_hz;
//     range_wrap_guard_samples_ = std::min(
//         radar_config_.block_size / 8U, static_cast<std::size_t>(std::ceil(max_delay_samples)) +
//         2U);
// }
//
// void StreamingTracker::pushChirp(std::size_t chirp_index,
//                                  std::span<const Complex> tx_chirp,
//                                  std::span<const Complex> rx_block) {
//     if (!tx_conj_initialized_) {
//         for (std::size_t i = 0; i < tx_chirp.size(); ++i) {
//             tx_conj_[i] = std::conj(tx_chirp[i]);
//         }
//         tx_conj_initialized_ = true;
//     }
//
//     if (tx_chirp.size() != radar_config_.block_size) {
//         throw std::runtime_error("unexpected TX chirp size");
//     }
//     if (rx_block.size() != radar_config_.block_size * radar_config_.numRx()) {
//         throw std::runtime_error("unexpected RX block size");
//     }
//
//     ChirpBlock chirp_block{};
//     std::copy(rx_block.begin(), rx_block.end(), chirp_block.begin());
//     chirp_window_->push_back(chirp_block);
//     while (chirp_window_->size() > detection_config_.coherent_processing_interval_chirps) {
//         chirp_window_->pop_front();
//     }
//
//     const std::size_t received_chirps = chirp_index + 1;
//     const std::size_t cpi = detection_config_.coherent_processing_interval_chirps;
//     if (received_chirps < cpi) {
//         return;
//     }
//
//     const std::size_t start_chirp = received_chirps - cpi;
//     if (start_chirp % detection_config_.hop_chirps != 0) {
//         return;
//     }
//
//     BatchResult result = processCurrentWindow(start_chirp);
//
//     if (!result.valid) {
//         if (tracking_state_.initialized) {
//             const float dt = std::max(0.0f, result.time_s - tracking_state_.time_s);
//             tracking_state_.position_m += tracking_state_.velocity_mps * dt;
//             const KinematicEstimate predicted = makeKinematicEstimate(radar_config_,
//                                                                       tracking_state_.position_m,
//                                                                       tracking_state_.velocity_mps,
//                                                                       tracking_state_.direction);
//             tracking_state_.time_s = result.time_s;
//             tracking_state_.range_m = predicted.range_m;
//             tracking_state_.radial_velocity_mps = predicted.radial_velocity_mps;
//             tracking_state_.doppler_hz = predicted.doppler_hz;
//             tracking_state_.direction = predicted.direction;
//
//             result.range_m = predicted.range_m;
//             result.doppler_hz = predicted.doppler_hz;
//             result.direction = predicted.direction;
//             result.predicted_range_m = predicted.range_m;
//             result.predicted_doppler_hz = predicted.doppler_hz;
//             result.predicted_direction = predicted.direction;
//         }
//         batch_results_.push_back(result);
//         return;
//     }
//     const Vec3 measured_position = result.range_m * result.direction;
//     const float measured_radial_velocity_mps =
//         0.5f * result.doppler_hz * radar_config_.wavelengthM();
//
//     if (!tracking_state_.initialized) {
//         tracking_state_.position_m = measured_position;
//         tracking_state_.velocity_mps = measured_radial_velocity_mps * result.direction;
//         tracking_state_.initialized = true;
//     } else {
//         const float dt = std::max(0.001f, result.time_s - tracking_state_.time_s);
//         const Vec3 predicted_position = tracking_state_.position_m + tracking_state_.velocity_mps
//         * dt;
//
//         // Lower, stable gains for a high-rate radar tracker
//         constexpr float kPositionGain = 0.30f;
//         constexpr float kVelocityGain = 0.08f;
//         constexpr float kRadialVelocityGain = 0.45f;
//
//         const Vec3 residual = measured_position - predicted_position;
//         tracking_state_.position_m = predicted_position + kPositionGain * residual;
//         tracking_state_.velocity_mps = tracking_state_.velocity_mps + (kVelocityGain / dt) *
//         residual;
//
//         // Anchor the velocity vector using the highly accurate Doppler measurement
//         const float radial_velocity_error_mps =
//             measured_radial_velocity_mps - tracking_state_.velocity_mps.dot(result.direction);
//         tracking_state_.velocity_mps +=
//             kRadialVelocityGain * radial_velocity_error_mps * result.direction;
//         // const float dt = std::max(0.001f, result.time_s - tracking_state_.time_s);
//         // const Vec3 predicted_position = tracking_state_.position_m +
//         tracking_state_.velocity_mps * dt;
//         //
//         // constexpr float kAlpha = 0.75f;
//         // constexpr float kBeta = 0.25f;
//         //
//         // const Vec3 residual = measured_position - predicted_position;
//         // tracking_state_.position_m = predicted_position + kAlpha * residual;
//         // tracking_state_.velocity_mps = tracking_state_.velocity_mps + (kBeta / dt) * residual;
//     }
//
//     const KinematicEstimate estimate =
//         makeKinematicEstimate(radar_config_,
//                               tracking_state_.position_m,
//                               tracking_state_.velocity_mps,
//                               result.direction);
//     tracking_state_.time_s = result.time_s;
//     tracking_state_.range_m = estimate.range_m;
//     tracking_state_.radial_velocity_mps = estimate.radial_velocity_mps;
//     tracking_state_.doppler_hz = estimate.doppler_hz;
//     tracking_state_.direction = estimate.direction;
//
//     batch_results_.push_back(result);
//
// }
// BatchResult StreamingTracker::processCurrentWindow(std::size_t start_chirp) {
//     const std::size_t n_chirps = detection_config_.coherent_processing_interval_chirps;
//     const std::size_t nfft_doppler = n_chirps * 2U;
//     const std::size_t num_rx = radar_config_.numRx();
//     const std::size_t nfft = detection_config_.nfft_range_min;
//     const std::size_t range_count = range_bin_count_;
//
//     // Range FFT: dechirp, window, and transform each chirp.
//     std::vector<Complex> spec(n_chirps * range_count * num_rx, Complex(0.0f, 0.0f));
//     Eigen::FFT<float> fft;
//     std::vector<Complex> fft_input(nfft, Complex(0.0f, 0.0f));
//     std::vector<Complex> fft_output;
//
//     for (std::size_t chirp = 0; chirp < n_chirps; ++chirp) {
//         const ChirpBlock &rx_block = (*chirp_window_)[chirp];
//         for (std::size_t rx = 0; rx < num_rx; ++rx) {
//             std::fill(fft_input.begin(), fft_input.end(), Complex(0.0f, 0.0f));
//             for (std::size_t sample = range_wrap_guard_samples_; sample <
//             radar_config_.block_size;
//                  ++sample) {
//                 const Complex beat = rx_block[sample * num_rx + rx] * tx_conj_[sample];
//                 fft_input[sample] = beat * range_window_[sample];
//             }
//
//             fft.fwd(fft_output, fft_input);
//             for (std::size_t r = 0; r < range_count; ++r) {
//                 spec[cubeIndex(chirp, r, rx, range_count, num_rx)] =
//                     fft_output[static_cast<std::size_t>(range_indices_[r])];
//             }
//         }
//     }
//
//     // Static-clutter suppression: remove DC (mean) across slow-time.
//     if (detection_config_.static_clutter_suppression_enable) {
//         for (std::size_t r = 0; r < range_count; ++r) {
//             for (std::size_t rx = 0; rx < num_rx; ++rx) {
//                 Complex mean(0.0f, 0.0f);
//                 for (std::size_t chirp = 0; chirp < n_chirps; ++chirp) {
//                     mean += spec[cubeIndex(chirp, r, rx, range_count, num_rx)];
//                 }
//                 mean /= static_cast<float>(n_chirps);
//                 for (std::size_t chirp = 0; chirp < n_chirps; ++chirp) {
//                     spec[cubeIndex(chirp, r, rx, range_count, num_rx)] -= mean;
//                 }
//             }
//         }
//     }
//
//     // Doppler FFT: zero-padded to 2x CPI for finer Doppler resolution.
//     std::vector<Complex> rd_cube(nfft_doppler * range_count * num_rx, Complex(0.0f, 0.0f));
//     std::vector<float> rd_power(nfft_doppler * range_count, 0.0f);
//     std::vector<Complex> doppler_input(nfft_doppler, Complex(0.0f, 0.0f));
//     std::vector<Complex> doppler_output;
//
//     for (std::size_t r = 0; r < range_count; ++r) {
//         for (std::size_t rx = 0; rx < num_rx; ++rx) {
//             for (std::size_t chirp = 0; chirp < n_chirps; ++chirp) {
//                 doppler_input[chirp] =
//                     spec[cubeIndex(chirp, r, rx, range_count, num_rx)] * doppler_window_[chirp];
//             }
//             // Remaining doppler_input entries are zero (zero-padding).
//
//             fft.fwd(doppler_output, doppler_input);
//             fftShiftInPlace(doppler_output);
//
//             for (std::size_t dbin = 0; dbin < nfft_doppler; ++dbin) {
//                 const Complex value = doppler_output[dbin];
//                 rd_cube[cubeIndex(dbin, r, rx, range_count, num_rx)] = value;
//                 rd_power[dbin * range_count + r] += std::norm(value);
//             }
//         }
//     }
//
//     for (float &power : rd_power) {
//         power /= static_cast<float>(num_rx);
//     }
//
//     // Candidate scoring over the full RD plane.
//     struct CandidateScore {
//         std::size_t doppler_bin = 0;
//         std::size_t range_bin = 0;
//         float range_m = 0.0f;
//         float doppler_hz = 0.0f;
//         float score = std::numeric_limits<float>::lowest();
//     };
//
//     constexpr float kScoreEpsilon = 1.0e-12f;
//     constexpr std::size_t kMaxRangeDopplerCandidates = 6;
//
//     const std::size_t zero_doppler_bin = nfft_doppler / 2;
//     float predicted_range_m = 0.0f;
//     float predicted_doppler_hz = 0.0f;
//     Vec3 predicted_direction = Vec3::UnitX();
//     const float batch_time_s =
//         (static_cast<float>(start_chirp) + 0.5f * static_cast<float>(n_chirps)) *
//         radar_config_.chirp_duration_s;
//     const float range_bin_spacing_m =
//         range_count > 1 ? std::abs(range_axis_sliced_m_[1] - range_axis_sliced_m_[0]) : 0.0f;
//     const float doppler_bin_spacing_hz =
//         nfft_doppler > 1 ? std::abs(doppler_axis_hz_[1] - doppler_axis_hz_[0]) : 0.0f;
//
//     if (tracking_state_.initialized) {
//         const float dt = std::max(0.0f, batch_time_s - tracking_state_.time_s);
//         const Vec3 predicted_position_m =
//             tracking_state_.position_m + tracking_state_.velocity_mps * dt;
//         const KinematicEstimate predicted = makeKinematicEstimate(radar_config_,
//                                                                   predicted_position_m,
//                                                                   tracking_state_.velocity_mps,
//                                                                   tracking_state_.direction);
//         predicted_range_m = predicted.range_m;
//         predicted_doppler_hz = predicted.doppler_hz;
//         predicted_direction = predicted.direction;
//     }
//
//     // Gate the search to a region around the prediction when initialized.
//     const std::size_t range_gate =
//         tracking_state_.initialized
//             ? static_cast<std::size_t>(
//                   std::max(static_cast<float>(detection_config_.range_gate_bins),
//                            std::ceil(3.0f * detection_config_.range_association_sigma_m /
//                                      std::max(range_bin_spacing_m, 1.0e-6f))))
//             : range_count;
//     const std::size_t doppler_gate =
//         tracking_state_.initialized
//             ? static_cast<std::size_t>(
//                   std::max(static_cast<float>(detection_config_.doppler_gate_bins),
//                            std::ceil(3.0f * detection_config_.doppler_association_sigma_hz /
//                                      std::max(doppler_bin_spacing_hz, 1.0e-6f))))
//             : nfft_doppler;
//
//     std::size_t center_rbin = range_count / 2;
//     std::size_t center_dbin = nfft_doppler / 2;
//     if (tracking_state_.initialized) {
//         center_rbin = nearestAxisBin(range_axis_sliced_m_, range_count, predicted_range_m);
//         center_dbin = nearestAxisBin(doppler_axis_hz_, nfft_doppler, predicted_doppler_hz);
//     }
//
//     std::vector<CandidateScore> rd_candidates;
//     rd_candidates.reserve(std::min(doppler_gate, nfft_doppler) * std::min(range_gate,
//     range_count));
//
//     const std::size_t r_start = (center_rbin > range_gate / 2) ? center_rbin - range_gate / 2 :
//     0; const std::size_t r_end = std::min(range_count, center_rbin + range_gate / 2 + 1); const
//     std::size_t d_start =
//         (center_dbin > doppler_gate / 2) ? center_dbin - doppler_gate / 2 : 0;
//     const std::size_t d_end = std::min(nfft_doppler, center_dbin + doppler_gate / 2 + 1);
//
//     for (std::size_t dbin = d_start; dbin < d_end; ++dbin) {
//         if (detection_config_.zero_doppler_guard_bins > 0) {
//             const std::size_t low =
//                 zero_doppler_bin > detection_config_.zero_doppler_guard_bins
//                     ? zero_doppler_bin - detection_config_.zero_doppler_guard_bins
//                     : 0;
//             const std::size_t high = std::min(
//                 nfft_doppler - 1, zero_doppler_bin + detection_config_.zero_doppler_guard_bins);
//             if (dbin >= low && dbin <= high) {
//                 continue;
//             }
//         }
//
//         const float candidate_doppler_hz = doppler_axis_hz_[dbin];
//         const float range_correction = (candidate_doppler_hz / radar_config_.chirpSlopeHzPerS())
//         *
//                                        (radar_config_.speed_of_light_mps / 2.0f);
//
//         for (std::size_t rbin = r_start; rbin < r_end; ++rbin) {
//             const float power = rd_power[dbin * range_count + rbin];
//             const float candidate_range_m =
//                 range_axis_sliced_m_[static_cast<Eigen::Index>(rbin)] - range_correction;
//             float score = std::log(power + kScoreEpsilon);
//             if (tracking_state_.initialized) {
//                 score += associationPenalty(candidate_range_m - predicted_range_m,
//                                             detection_config_.range_association_sigma_m);
//                 score += associationPenalty(candidate_doppler_hz - predicted_doppler_hz,
//                                             detection_config_.doppler_association_sigma_hz);
//             }
//
//             CandidateScore candidate;
//             candidate.doppler_bin = dbin;
//             candidate.range_bin = rbin;
//             candidate.range_m = candidate_range_m;
//             candidate.doppler_hz = candidate_doppler_hz;
//             candidate.score = score;
//             rd_candidates.push_back(candidate);
//         }
//     }
//
//     if (rd_candidates.empty()) {
//         throw std::runtime_error("tracker failed to find a valid range-doppler peak");
//     }
//
//     const std::size_t candidate_count = std::min(kMaxRangeDopplerCandidates,
//     rd_candidates.size()); std::partial_sort(
//         rd_candidates.begin(),
//         rd_candidates.begin() + candidate_count,
//         rd_candidates.end(),
//         [](const CandidateScore &lhs, const CandidateScore &rhs) { return lhs.score > rhs.score;
//         });
//
//     // Select best candidate and run AoA beamforming.
//     float best_total_score = std::numeric_limits<float>::lowest();
//     std::size_t best_doppler_bin = rd_candidates.front().doppler_bin;
//     std::size_t best_range_bin = rd_candidates.front().range_bin;
//     std::size_t best_direction_index = 0;
//     float best_range_m = rd_candidates.front().range_m;
//     float best_doppler_hz = rd_candidates.front().doppler_hz;
//     std::array<Complex, kNumRx> best_snapshot{};
//     Vec3 best_direction = tracking_state_.initialized ? tracking_state_.direction :
//     Vec3::UnitX();
//
//     const float predicted_azimuth_deg = azimuthDeg(predicted_direction);
//     const float predicted_elevation_deg = elevationDeg(predicted_direction);
//
//     for (std::size_t candidate_index = 0; candidate_index < candidate_count; ++candidate_index) {
//         const CandidateScore &candidate = rd_candidates[candidate_index];
//         std::array<Complex, kNumRx> snapshot{};
//         for (std::size_t rx = 0; rx < num_rx; ++rx) {
//             snapshot[rx] = rd_cube[cubeIndex(
//                 candidate.doppler_bin, candidate.range_bin, rx, range_count, num_rx)];
//         }
//
//         Vec3 candidate_direction = Vec3::UnitX();
//         std::size_t candidate_direction_index = 0;
//         float best_direction_score =
//             detection_config_.aoa_enable ? std::numeric_limits<float>::lowest() : 0.0f;
//
//         if (detection_config_.aoa_enable) {
//             const std::size_t azimuth_count = detection_config_.azimuth_count;
//             const std::size_t elevation_count = detection_config_.elevation_count;
//             const std::size_t coarse_elevation_stride =
//                 std::max<std::size_t>(1U, elevation_count / 91U);
//
//             auto evaluate_direction = [&](std::size_t direction_index) {
//                 Complex response(0.0f, 0.0f);
//                 const std::size_t base = direction_index * num_rx;
//                 for (std::size_t rx = 0; rx < num_rx; ++rx) {
//                     response += steering_conj_[base + rx] * snapshot[rx];
//                 }
//
//                 float direction_score = std::log(std::norm(response) + kScoreEpsilon);
//                 if (tracking_state_.initialized) {
//                     direction_score += associationPenalty(
//                         angleDifferenceDeg(azimuthDeg(directions_[direction_index]),
//                                            predicted_azimuth_deg),
//                         detection_config_.azimuth_association_sigma_deg);
//                     direction_score += associationPenalty(
//                         elevationDeg(directions_[direction_index]) - predicted_elevation_deg,
//                         detection_config_.elevation_association_sigma_deg);
//                 }
//
//                 if (direction_score > best_direction_score) {
//                     best_direction_score = direction_score;
//                     candidate_direction = directions_[direction_index];
//                     candidate_direction_index = direction_index;
//                 }
//             };
//
//             for (std::size_t azimuth_index = 0; azimuth_index < azimuth_count; ++azimuth_index) {
//                 std::size_t best_coarse_elevation_index = 0U;
//                 float best_coarse_score = std::numeric_limits<float>::lowest();
//
//                 for (std::size_t elevation_index = 0; elevation_index < elevation_count;
//                      elevation_index += coarse_elevation_stride) {
//                     const std::size_t direction_index =
//                         azimuth_index * elevation_count + elevation_index;
//
//                     Complex response(0.0f, 0.0f);
//                     const std::size_t base = direction_index * num_rx;
//                     for (std::size_t rx = 0; rx < num_rx; ++rx) {
//                         response += steering_conj_[base + rx] * snapshot[rx];
//                     }
//
//                     float direction_score = std::log(std::norm(response) + kScoreEpsilon);
//                     if (tracking_state_.initialized) {
//                         direction_score += associationPenalty(
//                             angleDifferenceDeg(azimuthDeg(directions_[direction_index]),
//                                                predicted_azimuth_deg),
//                             detection_config_.azimuth_association_sigma_deg);
//                         direction_score += associationPenalty(
//                             elevationDeg(directions_[direction_index]) - predicted_elevation_deg,
//                             detection_config_.elevation_association_sigma_deg);
//                     }
//
//                     if (direction_score > best_coarse_score) {
//                         best_coarse_score = direction_score;
//                         best_coarse_elevation_index = elevation_index;
//                     }
//                 }
//
//                 const std::size_t refine_start =
//                     best_coarse_elevation_index > coarse_elevation_stride
//                         ? best_coarse_elevation_index - coarse_elevation_stride
//                         : 0U;
//                 const std::size_t refine_stop = std::min(
//                     elevation_count, best_coarse_elevation_index + coarse_elevation_stride + 1U);
//
//                 for (std::size_t elevation_index = refine_start; elevation_index < refine_stop;
//                      ++elevation_index) {
//                     const std::size_t direction_index =
//                         azimuth_index * elevation_count + elevation_index;
//                     evaluate_direction(direction_index);
//                 }
//             }
//         }
//
//         const float total_score = candidate.score + best_direction_score;
//         if (total_score > best_total_score) {
//             best_total_score = total_score;
//             best_doppler_bin = candidate.doppler_bin;
//             best_range_bin = candidate.range_bin;
//             best_direction_index = candidate_direction_index;
//             best_range_m = candidate.range_m;
//             best_doppler_hz = candidate.doppler_hz;
//             best_direction = candidate_direction;
//             best_snapshot = snapshot;
//         }
//     }
//
//     // Sub-bin interpolation for Doppler.
//     float doppler_bin_offset = 0.0f;
//     if (best_doppler_bin > 0 && best_doppler_bin + 1 < nfft_doppler) {
//         auto d_interp = quadraticPeakOffsetChecked(
//             rd_power[(best_doppler_bin - 1) * range_count + best_range_bin],
//             rd_power[best_doppler_bin * range_count + best_range_bin],
//             rd_power[(best_doppler_bin + 1) * range_count + best_range_bin]);
//
//         float doppler_delta_raw = d_interp.valid ? d_interp.delta : 0.0f;
//         float doppler_hz_raw =
//             interpolateAxis(doppler_axis_hz_, nfft_doppler, best_doppler_bin, doppler_delta_raw);
//
//         bool bad_doppler_interp = false;
//         if (tracking_state_.initialized) {
//             float doppler_jump_hz = std::abs(doppler_hz_raw - predicted_doppler_hz);
//             bad_doppler_interp = doppler_jump_hz > detection_config_.doppler_interp_gate_hz;
//         }
//
//         doppler_bin_offset = bad_doppler_interp ? 0.0f : doppler_delta_raw;
//     }
//
//     // Sub-bin interpolation for range.
//     float range_bin_offset = 0.0f;
//     if (best_range_bin > 0 && best_range_bin + 1 < range_count) {
//         auto r_interp = quadraticPeakOffsetChecked(
//             rd_power[best_doppler_bin * range_count + (best_range_bin - 1)],
//             rd_power[best_doppler_bin * range_count + best_range_bin],
//             rd_power[best_doppler_bin * range_count + (best_range_bin + 1)]);
//
//         float range_offset_raw = r_interp.valid ? r_interp.delta : 0.0f;
//
//         bool bad_range_interp = false;
//         if (tracking_state_.initialized) {
//             float refined_range_candidate = interpolateAxis(
//                 range_axis_sliced_m_, range_count, best_range_bin, range_offset_raw);
//             const float candidate_doppler_hz = doppler_axis_hz_[best_doppler_bin];
//             const float range_correction =
//                 (candidate_doppler_hz / radar_config_.chirpSlopeHzPerS()) *
//                 (radar_config_.speed_of_light_mps / 2.0f);
//             refined_range_candidate -= range_correction;
//             float range_jump_m = std::abs(refined_range_candidate - predicted_range_m);
//             bad_range_interp = range_jump_m > detection_config_.range_interp_gate_m;
//         }
//
//         range_bin_offset = bad_range_interp ? 0.0f : range_offset_raw;
//     }
//
//     best_doppler_hz =
//         interpolateAxis(doppler_axis_hz_, nfft_doppler, best_doppler_bin, doppler_bin_offset);
//     const float refined_raw_range_m =
//         interpolateAxis(range_axis_sliced_m_, range_count, best_range_bin, range_bin_offset);
//     const float refined_range_correction = (best_doppler_hz / radar_config_.chirpSlopeHzPerS()) *
//                                            (radar_config_.speed_of_light_mps / 2.0f);
//     best_range_m = refined_raw_range_m - refined_range_correction;
//
//     // Measurement gating.
//     const float max_range_jump_m = std::max(
//         detection_config_.range_association_sigma_m,
//         (static_cast<float>(detection_config_.range_gate_bins) + 1.0f) * range_bin_spacing_m);
//     const float max_doppler_jump_hz = std::max(
//         detection_config_.doppler_association_sigma_hz,
//         (static_cast<float>(detection_config_.doppler_gate_bins) + 1.5f) *
//         doppler_bin_spacing_hz);
//
//     bool range_ok = !tracking_state_.initialized ||
//                     std::abs(best_range_m - predicted_range_m) < max_range_jump_m;
//     bool doppler_ok = !tracking_state_.initialized ||
//                       std::abs(best_doppler_hz - predicted_doppler_hz) < max_doppler_jump_hz;
//
//     BatchResult result;
//     result.time_s = batch_time_s;
//     result.predicted_range_m = predicted_range_m;
//     result.predicted_doppler_hz = predicted_doppler_hz;
//     result.predicted_direction = predicted_direction;
//
//     if (!std::isfinite(best_range_m) || !std::isfinite(best_doppler_hz) || !range_ok ||
//         !doppler_ok) {
//         result.valid = false;
//         return result;
//     }
//
//     result.valid = true;
//     result.range_m = best_range_m;
//     result.doppler_hz = best_doppler_hz;
//     Complex beamformed_response(0.0f, 0.0f);
//     if (best_direction_index < directions_.size()) {
//         const std::size_t base = best_direction_index * num_rx;
//         for (std::size_t rx = 0; rx < num_rx; ++rx) {
//             beamformed_response += steering_conj_[base + rx] * best_snapshot[rx];
//         }
//     } else {
//         beamformed_response = best_snapshot[0];
//     }
//     result.phase_rad = std::arg(beamformed_response);
//     result.range_bin_offset = range_bin_offset;
//     result.doppler_bin_offset = doppler_bin_offset;
//     result.direction = best_direction;
//     result.range_bin = best_range_bin;
//     result.doppler_bin = best_doppler_bin;
//     result.azimuth_bin = best_direction_index / detection_config_.elevation_count;
//     result.elevation_bin = best_direction_index % detection_config_.elevation_count;
//     if (best_direction_index < directions_.size()) {
//         const std::size_t base = best_direction_index * num_rx;
//         for (std::size_t chirp = 0; chirp < n_chirps; ++chirp) {
//             Complex slow_time_response(0.0f, 0.0f);
//             for (std::size_t rx = 0; rx < num_rx; ++rx) {
//                 slow_time_response +=
//                     steering_conj_[base + rx] *
//                     spec[cubeIndex(chirp, best_range_bin, rx, range_count, num_rx)];
//             }
//             result.slow_time_phase_rad[static_cast<Eigen::Index>(chirp)] =
//                 std::arg(slow_time_response);
//         }
//     }
//     for (std::size_t dbin = 0; dbin < nfft_doppler; ++dbin) {
//         result.doppler_slice_power[static_cast<Eigen::Index>(dbin)] =
//             rd_power[dbin * range_count + best_range_bin];
//     }
//     return result;
// }
//
// TrackSummary StreamingTracker::buildSummary() const {
//     TrackSummary summary;
//     summary.batch_results = batch_results_;
//     summary.microdoppler_truth_frequency_hz =
//         description_.cars.empty() ? 0.0f : description_.cars.front().bounce_frequency_hz;
//     summary.velocity_axis_mps.resize(kFixedDopplerFftSize);
//     for (std::size_t i = 0; i < kFixedDopplerFftSize; ++i) {
//         summary.velocity_axis_mps[i] = velocity_axis_mps_[static_cast<Eigen::Index>(i)];
//     }
//
//     if (batch_results_.empty()) {
//         return summary;
//     }
//
//     const float wavelength_m = radar_config_.wavelengthM();
//
//     summary.times_s.reserve(batch_results_.size());
//     summary.raw_positions_m.reserve(batch_results_.size());
//     summary.smoothed_positions_m.reserve(batch_results_.size());
//     summary.ranges_m.reserve(batch_results_.size());
//     summary.radial_velocity_mps.reserve(batch_results_.size());
//     summary.unwrapped_phase_rad.reserve(batch_results_.size());
//     summary.detrended_phase_rad.reserve(batch_results_.size());
//     summary.cartesian_velocity_mps.reserve(batch_results_.size());
//     summary.truth_metrics.reserve(batch_results_.size());
//
//     std::vector<float> raw_x(batch_results_.size());
//     std::vector<float> raw_y(batch_results_.size());
//     std::vector<float> raw_z(batch_results_.size());
//
//     for (std::size_t i = 0; i < batch_results_.size(); ++i) {
//         const BatchResult &batch = batch_results_[i];
//         summary.times_s.push_back(batch.time_s);
//         const Vec3 raw_xyz = batch.range_m * batch.direction;
//         summary.raw_positions_m.push_back(raw_xyz);
//         summary.ranges_m.push_back(batch.range_m);
//         summary.radial_velocity_mps.push_back(0.5f * batch.doppler_hz * wavelength_m);
//         raw_x[i] = raw_xyz.x();
//         raw_y[i] = raw_xyz.y();
//         raw_z[i] = raw_xyz.z();
//         summary.truth_metrics.push_back(truthAtTime(description_, batch.time_s));
//     }
//
//     std::vector<float> microdoppler_times_s;
//     std::vector<float> microdoppler_unwrapped_phase;
//     std::vector<float> residual_phase_example;
//     std::vector<float> microdoppler_candidate_frequency_hz;
//     std::vector<float> microdoppler_candidate_power;
//     const float phase_microdoppler_frequency_hz =
//         estimateDominantCpiResidualFrequencyHz(batch_results_,
//                                                radar_config_.chirp_duration_s,
//                                                &microdoppler_times_s,
//                                                &microdoppler_unwrapped_phase,
//                                                &residual_phase_example,
//                                                nullptr,
//                                                nullptr,
//                                                nullptr,
//                                                nullptr);
//     summary.microdoppler_phase_frequency_hz = estimateDominantBatchDopplerFrequencyHz(
//         batch_results_,
//         static_cast<float>(detection_config_.hop_chirps) * radar_config_.chirp_duration_s,
//         &microdoppler_candidate_frequency_hz,
//         &microdoppler_candidate_power,
//         &summary.microdoppler_peak_power,
//         &summary.microdoppler_valid_cpi_count);
//     if (summary.microdoppler_phase_frequency_hz <= 0.0f) {
//         summary.microdoppler_phase_frequency_hz = phase_microdoppler_frequency_hz;
//     }
//     summary.unwrapped_phase_rad = microdoppler_unwrapped_phase;
//     summary.detrended_phase_rad = residual_phase_example;
//     summary.microdoppler_candidate_frequency_hz = microdoppler_candidate_frequency_hz;
//     summary.microdoppler_candidate_power = microdoppler_candidate_power;
//     const float microdoppler_error_hz =
//         summary.microdoppler_phase_frequency_hz - summary.microdoppler_truth_frequency_hz;
//     summary.microdoppler_frequency_rmse_hz =
//         std::sqrt(microdoppler_error_hz * microdoppler_error_hz);
//     if (!summary.detrended_phase_rad.empty()) {
//         float sum = 0.0f;
//         float squared_sum = 0.0f;
//         for (float sample : summary.detrended_phase_rad) {
//             sum += sample;
//             squared_sum += sample * sample;
//         }
//         summary.microdoppler_residual_phase_mean_rad =
//             sum / static_cast<float>(summary.detrended_phase_rad.size());
//         summary.microdoppler_residual_phase_rms_rad =
//             std::sqrt(squared_sum / static_cast<float>(summary.detrended_phase_rad.size()));
//         float variance_sum = 0.0f;
//         for (float sample : summary.detrended_phase_rad) {
//             const float centered = sample - summary.microdoppler_residual_phase_mean_rad;
//             variance_sum += centered * centered;
//         }
//         summary.microdoppler_residual_phase_stddev_rad =
//             std::sqrt(variance_sum / static_cast<float>(summary.detrended_phase_rad.size()));
//     }
//
//     const std::size_t window_len = std::min<std::size_t>(5, batch_results_.size());
//     const std::vector<float> boxcar(window_len, 1.0f / static_cast<float>(window_len));
//
//     const std::vector<float> smooth_x = convolveSame(raw_x, boxcar);
//     const std::vector<float> smooth_y = convolveSame(raw_y, boxcar);
//     const std::vector<float> smooth_z = convolveSame(raw_z, boxcar);
//
//     for (std::size_t i = 0; i < batch_results_.size(); ++i) {
//         summary.smoothed_positions_m.emplace_back(smooth_x[i], smooth_y[i], smooth_z[i]);
//     }
//
//     const std::vector<float> vx = gradient(smooth_x, summary.times_s);
//     const std::vector<float> vy = gradient(smooth_y, summary.times_s);
//     const std::vector<float> vz = gradient(smooth_z, summary.times_s);
//
//     for (std::size_t i = 0; i < batch_results_.size(); ++i) {
//         summary.cartesian_velocity_mps.emplace_back(vx[i], vy[i], vz[i]);
//     }
//
//     return summary;
// }
//
// problem::SimulationMetrics truthAtTime(const problem::ProblemDescription &description,
//                                        float time_s) {
//     if (description.cars.empty()) {
//         throw std::runtime_error("truthAtTime requires at least one car");
//     }
//
//     const CarDynamics dynamics(description.cars.front());
//     const radar::TargetObservation observation =
//         radar::observeTarget(dynamics, description.radar, time_s);
//     return radar::makeSimulationMetrics(time_s, observation);
// }
//
// } // namespace fmcw_tracker
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
        static_cast<float>(problem::Constants::kRadarBlockSize) / description.radar.sample_rate_hz;
    return static_cast<std::size_t>(
        std::llround(description.simulator.burst_duration_s / chirp_duration_s));
}

RadarConfig makeRadarConfig(const problem::ProblemDescription &description) {
    const float chirp_duration_s =
        static_cast<float>(problem::Constants::kRadarBlockSize) / description.radar.sample_rate_hz;
    const float lambda_m = problem::Constants::kSpeedOfLightMps / description.radar.carrier_hz;

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
        window[n] = 0.5f - 0.5f * std::cos(2.0f * problem::Constants::kPi * static_cast<float>(n) /
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
                static_cast<float>(static_cast<long long>(k) - static_cast<long long>(nfft)) *
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

template <typename T, int Size> void fftShiftInPlace(Eigen::Matrix<T, Size, 1> &values) {
    constexpr int kHalf = Size / 2;
    Eigen::Matrix<T, Size, 1> shifted;
    shifted.template head<Size - kHalf>() = values.template tail<Size - kHalf>();
    shifted.template tail<kHalf>() = values.template head<kHalf>();
    values = shifted;
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
                (static_cast<float>(ix) - 0.5f * static_cast<float>(cfg.probe_num_x - 1)) *
                cfg.probe_dx_m;
            positions[flat].y() =
                (static_cast<float>(iy) - 0.5f * static_cast<float>(cfg.probe_num_y - 1)) *
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
                    (2.0f * problem::Constants::kPi / cfg.wavelengthM()) * direction.dot(position);
                grid.steering_conj.emplace_back(std::cos(-phase), std::sin(-phase));
            }
        }
    }

    return grid;
}

template <int Size>
Eigen::Matrix<Complex, Size, 1> makeComplexWindowEigen(const std::vector<float> &real_window) {
    Eigen::Matrix<Complex, Size, 1> window = Eigen::Matrix<Complex, Size, 1>::Zero();
    const std::size_t count = std::min<std::size_t>(Size, real_window.size());
    for (std::size_t i = 0; i < count; ++i) {
        window[static_cast<Eigen::Index>(i)] = Complex(real_window[i], 0.0f);
    }
    return window;
}

StreamingTracker::ChirpReference makeComplexWindowArray(const std::vector<float> &real_window) {
    StreamingTracker::ChirpReference window{};
    const std::size_t count = std::min(window.size(), static_cast<std::size_t>(real_window.size()));
    for (std::size_t i = 0; i < count; ++i) {
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

Vec3 normalizedOr(const Vec3 &value, const Vec3 &fallback) {
    const float norm = value.norm();
    if (!std::isfinite(norm) || norm <= 1.0e-6f) {
        return fallback;
    }
    return value / norm;
}

struct KinematicEstimate {
    Vec3 position_m = Vec3::Zero();
    Vec3 velocity_mps = Vec3::Zero();
    Vec3 direction = Vec3::UnitX();
    float range_m = 0.0f;
    float radial_velocity_mps = 0.0f;
    float doppler_hz = 0.0f;
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

PeakInterpResult
quadraticPeakOffsetChecked(float power_minus, float power_center, float power_plus) {
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

float interpolateAxis(const std::vector<float> &axis, std::size_t bin, float offset) {
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

template <int Size>
float interpolateAxis(const Eigen::Matrix<float, Size, 1> &axis,
                      std::size_t count,
                      std::size_t bin,
                      float offset) {
    if (count == 0 || bin >= count) {
        return 0.0f;
    }
    if (offset > 0.0f && bin + 1 < count) {
        return axis[static_cast<Eigen::Index>(bin)] +
               offset * (axis[static_cast<Eigen::Index>(bin + 1)] -
                         axis[static_cast<Eigen::Index>(bin)]);
    }
    if (offset < 0.0f && bin > 0) {
        return axis[static_cast<Eigen::Index>(bin)] +
               offset * (axis[static_cast<Eigen::Index>(bin)] -
                         axis[static_cast<Eigen::Index>(bin - 1)]);
    }
    return axis[static_cast<Eigen::Index>(bin)];
}

template <int Size>
std::size_t
nearestAxisBin(const Eigen::Matrix<float, Size, 1> &axis, std::size_t count, float value) {
    if (count == 0) {
        return 0;
    }

    std::size_t best_bin = 0;
    float best_error = std::abs(axis[0] - value);
    for (std::size_t i = 1; i < count; ++i) {
        const float error = std::abs(axis[static_cast<Eigen::Index>(i)] - value);
        if (error < best_error) {
            best_error = error;
            best_bin = i;
        }
    }
    return best_bin;
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

std::vector<float> movingAverageSame(const std::vector<float> &signal, std::size_t window_size) {
    if (signal.empty()) {
        return {};
    }

    window_size = std::max<std::size_t>(1U, window_size);
    std::vector<float> prefix_sum(signal.size() + 1U, 0.0f);
    for (std::size_t i = 0; i < signal.size(); ++i) {
        prefix_sum[i + 1U] = prefix_sum[i] + signal[i];
    }

    std::vector<float> averaged(signal.size(), 0.0f);
    const std::size_t half_window = window_size / 2U;
    for (std::size_t i = 0; i < signal.size(); ++i) {
        const std::size_t begin = (i > half_window) ? (i - half_window) : 0U;
        const std::size_t end = std::min(signal.size(), i + half_window + 1U);
        const float sum = prefix_sum[end] - prefix_sum[begin];
        averaged[i] = sum / static_cast<float>(end - begin);
    }
    return averaged;
}

std::vector<float> gradient(const std::vector<float> &values, const std::vector<float> &times_s) {
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

std::vector<float> detrendPhase(const std::vector<float> &times_s,
                                const std::vector<float> &phase_rad) {
    if (times_s.size() != phase_rad.size() || phase_rad.empty()) {
        return {};
    }
    if (phase_rad.size() == 1U) {
        return std::vector<float>(1U, 0.0f);
    }

    float sum_t = 0.0f;
    float sum_p = 0.0f;
    float sum_tt = 0.0f;
    float sum_tp = 0.0f;
    for (std::size_t i = 0; i < phase_rad.size(); ++i) {
        const float t = times_s[i];
        const float p = phase_rad[i];
        sum_t += t;
        sum_p += p;
        sum_tt += t * t;
        sum_tp += t * p;
    }

    const float count = static_cast<float>(phase_rad.size());
    const float denom = count * sum_tt - sum_t * sum_t;
    float slope = 0.0f;
    float intercept = phase_rad.front();
    if (std::abs(denom) > 1.0e-9f) {
        slope = (count * sum_tp - sum_t * sum_p) / denom;
        intercept = (sum_p - slope * sum_t) / count;
    }

    std::vector<float> residual(phase_rad.size(), 0.0f);
    for (std::size_t i = 0; i < phase_rad.size(); ++i) {
        residual[i] = phase_rad[i] - (intercept + slope * times_s[i]);
    }
    return residual;
}

float estimateDominantCpiResidualFrequencyHz(const std::vector<BatchResult> &batch_results,
                                             float chirp_duration_s,
                                             std::vector<float> *times_s_out = nullptr,
                                             std::vector<float> *unwrapped_phase_out = nullptr,
                                             std::vector<float> *residual_phase_out = nullptr) {
    constexpr float kMinSearchFrequencyHz = 10.0f;
    constexpr float kMaxSearchFrequencyHz = 100.0f;
    constexpr float kHighPassCutoffHz = 15.0f;
    constexpr std::size_t kBoundaryFitSamples = 8U;

    if (chirp_duration_s <= 0.0f) {
        return 0.0f;
    }

    const std::size_t chirps_per_cpi = BatchResult::kSlowTimeSize;
    const float chirp_rate_hz = 1.0f / chirp_duration_s;
    const std::size_t high_pass_window = std::max<std::size_t>(
        3U, static_cast<std::size_t>(std::llround(chirp_rate_hz / kHighPassCutoffHz)));

    struct SegmentBounds {
        std::size_t begin = 0U;
        std::size_t end = 0U;
    };

    std::vector<float> times_s;
    std::vector<float> wrapped_phase_rad;
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
            batch.time_s / chirp_duration_s - 0.5f * static_cast<float>(chirps_per_cpi)));
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
            const float chirp_time_s = static_cast<float>(global_chirp) * chirp_duration_s;
            times_s.push_back(chirp_time_s);

            const float raw_phase = batch.slow_time_phase_rad[static_cast<Eigen::Index>(chirp)];
            const float local_time_s = static_cast<float>(chirp) * chirp_duration_s;
            const float coarse_phase =
                2.0f * problem::Constants::kPi * batch.doppler_hz * local_time_s;
            const Complex baseband = std::polar(1.0f, raw_phase) * std::polar(1.0f, -coarse_phase);
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

    std::vector<float> filtered_phase_all;
    std::vector<float> unwrapped_phase_all;
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

    std::vector<float> accumulated_power(nfft / 2U, 0.0f);
    Eigen::FFT<float> fft;
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

        std::vector<float> aligned_unwrapped_segment;
        aligned_unwrapped_segment.reserve(segment_length);

        while (batch_segment_index < batch_segments.size() &&
               batch_segments[batch_segment_index].begin < continuous_segment.end) {
            const SegmentBounds batch_segment = batch_segments[batch_segment_index++];
            if (batch_segment.end <= continuous_segment.begin) {
                continue;
            }

            const std::vector<float> wrapped_batch_segment(
                wrapped_phase_rad.begin() + static_cast<std::ptrdiff_t>(batch_segment.begin),
                wrapped_phase_rad.begin() + static_cast<std::ptrdiff_t>(batch_segment.end));
            std::vector<float> unwrapped_batch_segment = unwrapPhases(wrapped_batch_segment);
            if (unwrapped_batch_segment.empty()) {
                continue;
            }

            if (!aligned_unwrapped_segment.empty()) {
                const std::size_t prev_count =
                    std::min<std::size_t>(kBoundaryFitSamples, aligned_unwrapped_segment.size());
                float predicted_next_phase = aligned_unwrapped_segment.back();
                if (aligned_unwrapped_segment.size() >= 2U) {
                    float step_sum = 0.0f;
                    std::size_t step_count = 0U;
                    const std::size_t start = aligned_unwrapped_segment.size() - prev_count;
                    for (std::size_t i = start + 1U; i < aligned_unwrapped_segment.size(); ++i) {
                        step_sum +=
                            aligned_unwrapped_segment[i] - aligned_unwrapped_segment[i - 1U];
                        ++step_count;
                    }
                    if (step_count > 0U) {
                        predicted_next_phase += step_sum / static_cast<float>(step_count);
                    }
                }

                const float phase_offset = predicted_next_phase - unwrapped_batch_segment.front();
                for (float &sample : unwrapped_batch_segment) {
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

        const std::vector<float> low_pass_segment =
            movingAverageSame(aligned_unwrapped_segment,
                              std::min(high_pass_window, aligned_unwrapped_segment.size()));

        std::vector<float> filtered_segment(aligned_unwrapped_segment.size(), 0.0f);
        for (std::size_t i = 0; i < aligned_unwrapped_segment.size(); ++i) {
            filtered_segment[i] = aligned_unwrapped_segment[i] - low_pass_segment[i];
        }

        unwrapped_phase_all.insert(unwrapped_phase_all.end(),
                                   aligned_unwrapped_segment.begin(),
                                   aligned_unwrapped_segment.end());
        filtered_phase_all.insert(
            filtered_phase_all.end(), filtered_segment.begin(), filtered_segment.end());

        const std::vector<float> window = hannWindow(filtered_segment.size());
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

    float best_power = 0.0f;
    float best_frequency_hz = 0.0f;
    const std::size_t half = nfft / 2U;
    for (std::size_t bin = 1U; bin < half; ++bin) {
        const float frequency_hz =
            static_cast<float>(bin) / (static_cast<float>(nfft) * chirp_duration_s);
        if (frequency_hz < kMinSearchFrequencyHz || frequency_hz > kMaxSearchFrequencyHz) {
            continue;
        }
        const float power = accumulated_power[bin];
        if (power > best_power) {
            best_power = power;
            best_frequency_hz = frequency_hz;
        }
    }

    return best_frequency_hz;
}

float estimateDominantBatchDopplerFrequencyHz(
    const std::vector<BatchResult> &batch_results,
    float batch_period_s,
    std::vector<float> *candidate_frequency_hz_out = nullptr,
    std::vector<float> *candidate_power_out = nullptr,
    float *peak_power_out = nullptr,
    std::size_t *valid_batch_count_out = nullptr) {
    constexpr float kMinSearchFrequencyHz = 10.0f;
    constexpr float kMaxSearchFrequencyHz = 100.0f;
    constexpr float kHighPassCutoffHz = 10.0f;

    if (batch_period_s <= 0.0f) {
        return 0.0f;
    }

    struct SegmentBounds {
        std::size_t begin = 0U;
        std::size_t end = 0U;
    };

    std::vector<float> doppler_trace_hz;
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

    const float batch_rate_hz = 1.0f / batch_period_s;
    const std::size_t high_pass_window = std::max<std::size_t>(
        3U, static_cast<std::size_t>(std::llround(batch_rate_hz / kHighPassCutoffHz)));

    std::vector<float> accumulated_power(nfft / 2U, 0.0f);
    Eigen::FFT<float> fft;
    std::vector<Complex> fft_output;
    std::vector<Complex> fft_input(nfft, Complex(0.0f, 0.0f));

    for (const SegmentBounds &segment : segments) {
        const std::size_t segment_length = segment.end - segment.begin;
        if (segment_length < 4U) {
            continue;
        }

        const std::vector<float> doppler_segment(
            doppler_trace_hz.begin() + static_cast<std::ptrdiff_t>(segment.begin),
            doppler_trace_hz.begin() + static_cast<std::ptrdiff_t>(segment.end));
        const std::vector<float> low_pass_segment =
            movingAverageSame(doppler_segment, std::min(high_pass_window, segment_length));

        std::vector<float> filtered_segment(segment_length, 0.0f);
        for (std::size_t i = 0; i < segment_length; ++i) {
            filtered_segment[i] = doppler_segment[i] - low_pass_segment[i];
        }

        const std::vector<float> window = hannWindow(segment_length);
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
        float frequency_hz = 0.0f;
        float power = 0.0f;
    };
    std::vector<Candidate> candidates;
    float best_power = 0.0f;
    float best_frequency_hz = 0.0f;
    std::size_t best_bin = 0U;
    const std::size_t half = nfft / 2U;
    for (std::size_t bin = 1U; bin < half; ++bin) {
        const float frequency_hz =
            static_cast<float>(bin) / (static_cast<float>(nfft) * batch_period_s);
        if (frequency_hz < kMinSearchFrequencyHz || frequency_hz > kMaxSearchFrequencyHz) {
            continue;
        }

        const float power = accumulated_power[bin];
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
            best_frequency_hz = (static_cast<float>(best_bin) + interp.delta) /
                                (static_cast<float>(nfft) * batch_period_s);
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
    if (detection_config_.coherent_processing_interval_chirps != kFixedCpiChirps) {
        throw std::runtime_error("tracker currently requires 64-chirp CPI");
    }
    if (detection_config_.nfft_range_min != kFixedRangeNfft) {
        throw std::runtime_error("tracker currently requires 4096-point range FFT");
    }
    if (detection_config_.azimuth_count != kFixedAzimuthCount ||
        detection_config_.elevation_count != kFixedElevationCount) {
        throw std::runtime_error("tracker AoA grid must match fixed compile-time sizes");
    }

    // Range axis: use only negative-frequency FFT bins (down-chirp produces
    // negative beat frequencies for targets). Map to range via
    // R = -c * f_beat / (2 * chirp_slope).
    const std::vector<float> freqs =
        fftFreq(detection_config_.nfft_range_min, 1.0f / radar_config_.sample_rate_hz);
    range_bin_count_ = 0;
    for (std::size_t i = 0; i < freqs.size(); ++i) {
        const float beat_frequency_hz = freqs[i];
        if (beat_frequency_hz > 0.0f) {
            continue;
        }

        const float range_m = -radar_config_.speed_of_light_mps * beat_frequency_hz /
                              (2.0f * radar_config_.chirpSlopeHzPerS());
        if (range_m >= detection_config_.min_range_m && range_m <= detection_config_.max_range_m) {
            if (range_bin_count_ >= kFixedRangeBinCount) {
                throw std::runtime_error("range gate exceeds fixed bin capacity");
            }
            range_indices_[range_bin_count_] = static_cast<int>(i);
            range_axis_sliced_m_[static_cast<Eigen::Index>(range_bin_count_)] = range_m;
            ++range_bin_count_;
        }
    }
    if (range_bin_count_ == 0) {
        throw std::runtime_error("tracker range gate produced no bins");
    }

    range_window_ = makeComplexWindowArray(hannWindow(radar_config_.block_size));
    doppler_window_ = makeComplexWindowEigen<static_cast<int>(kFixedCpiChirps)>(
        hannWindow(detection_config_.coherent_processing_interval_chirps));

    const SteeringGrid grid = makeSteeringGrid(radar_config_, detection_config_);
    steering_conj_ = grid.steering_conj;
    directions_ = grid.directions;

    // Doppler axis: zero-pad to 2x CPI for finer frequency resolution
    // (~305 Hz/bin instead of ~610 Hz/bin). The truth model uses
    // f_D = 2*v_r/lambda (positive = approaching). The FFT on the beat-signal
    // slow-time produces the opposite sign, so we negate the axis.
    const std::size_t nfft_doppler = 2U * detection_config_.coherent_processing_interval_chirps;
    const std::vector<float> doppler_freqs = fftFreq(nfft_doppler, radar_config_.chirp_duration_s);
    for (std::size_t i = 0; i < nfft_doppler; ++i) {
        doppler_axis_hz_[static_cast<Eigen::Index>(i)] = doppler_freqs[i];
    }
    fftShiftInPlace(doppler_axis_hz_);
    for (Eigen::Index i = 0; i < doppler_axis_hz_.size(); ++i) {
        doppler_axis_hz_[i] = -doppler_axis_hz_[i];
    }

    for (Eigen::Index i = 0; i < velocity_axis_mps_.size(); ++i) {
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
        radar_config_.block_size / 8U, static_cast<std::size_t>(std::ceil(max_delay_samples)) + 2U);
}

void StreamingTracker::pushChirp(std::size_t chirp_index,
                                 std::span<const Complex> tx_chirp,
                                 std::span<const Complex> rx_block) {
    if (!tx_conj_initialized_) {
        for (std::size_t i = 0; i < tx_chirp.size(); ++i) {
            tx_conj_[i] = std::conj(tx_chirp[i]);
        }
        tx_conj_initialized_ = true;
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
            const float dt = std::max(0.0f, result.time_s - tracking_state_.time_s);
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
    const float measured_radial_velocity_mps =
        0.5f * result.doppler_hz * radar_config_.wavelengthM();

    if (!tracking_state_.initialized) {
        tracking_state_.position_m = measured_position;
        tracking_state_.velocity_mps = measured_radial_velocity_mps * result.direction;
        tracking_state_.initialized = true;
    } else {
        const float dt = std::max(0.001f, result.time_s - tracking_state_.time_s);
        const Vec3 predicted_position =
            tracking_state_.position_m + tracking_state_.velocity_mps * dt;

        // Ultra-low gains to glide smoothly through AoA grid-snapping noise
        constexpr float kPositionGain = 0.10f;
        constexpr float kVelocityGain = 0.001f;
        constexpr float kRadialVelocityGain = 0.20f;

        const Vec3 residual = measured_position - predicted_position;
        tracking_state_.position_m = predicted_position + kPositionGain * residual;

        // This is now safe. A 1-meter grid snap only alters velocity by ~0.5 m/s instead of 40 m/s
        tracking_state_.velocity_mps += (kVelocityGain / dt) * residual;

        // Anchor the velocity vector using the highly accurate Doppler measurement
        const float radial_velocity_error_mps =
            measured_radial_velocity_mps - tracking_state_.velocity_mps.dot(result.direction);
        tracking_state_.velocity_mps +=
            kRadialVelocityGain * radial_velocity_error_mps * result.direction;
    }
    // } else {
    //     const float dt = std::max(0.001f, result.time_s - tracking_state_.time_s);
    //     const Vec3 predicted_position = tracking_state_.position_m + tracking_state_.velocity_mps
    //     * dt;
    //
    //     // Lower, stable gains for a high-rate radar tracker
    //     constexpr float kPositionGain = 0.30f;
    //     constexpr float kVelocityGain = 0.08f;
    //     constexpr float kRadialVelocityGain = 0.45f;
    //
    //     const Vec3 residual = measured_position - predicted_position;
    //     tracking_state_.position_m = predicted_position + kPositionGain * residual;
    //     tracking_state_.velocity_mps = tracking_state_.velocity_mps + (kVelocityGain / dt) *
    //     residual;
    //
    //     // Anchor the velocity vector using the highly accurate Doppler measurement
    //     const float radial_velocity_error_mps =
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
    const std::size_t nfft = detection_config_.nfft_range_min;
    const std::size_t range_count = range_bin_count_;

    // Range FFT: dechirp, window, and transform each chirp.
    std::vector<Complex> spec(n_chirps * range_count * num_rx, Complex(0.0f, 0.0f));
    Eigen::FFT<float> fft;
    std::vector<Complex> fft_input(nfft, Complex(0.0f, 0.0f));
    std::vector<Complex> fft_output;

    for (std::size_t chirp = 0; chirp < n_chirps; ++chirp) {
        const ChirpBlock &rx_block = (*chirp_window_)[chirp];
        for (std::size_t rx = 0; rx < num_rx; ++rx) {
            std::fill(fft_input.begin(), fft_input.end(), Complex(0.0f, 0.0f));
            for (std::size_t sample = range_wrap_guard_samples_; sample < radar_config_.block_size;
                 ++sample) {
                const Complex beat = rx_block[sample * num_rx + rx] * tx_conj_[sample];
                fft_input[sample] = beat * range_window_[sample];
            }

            fft.fwd(fft_output, fft_input);
            for (std::size_t r = 0; r < range_count; ++r) {
                spec[cubeIndex(chirp, r, rx, range_count, num_rx)] =
                    fft_output[static_cast<std::size_t>(range_indices_[r])];
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
                    spec[cubeIndex(chirp, r, rx, range_count, num_rx)] * doppler_window_[chirp];
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
    const float batch_time_s =
        (static_cast<float>(start_chirp) + 0.5f * static_cast<float>(n_chirps)) *
        radar_config_.chirp_duration_s;
    const float range_bin_spacing_m =
        range_count > 1 ? std::abs(range_axis_sliced_m_[1] - range_axis_sliced_m_[0]) : 0.0f;
    const float doppler_bin_spacing_hz =
        nfft_doppler > 1 ? std::abs(doppler_axis_hz_[1] - doppler_axis_hz_[0]) : 0.0f;

    if (tracking_state_.initialized) {
        const float dt = std::max(0.0f, batch_time_s - tracking_state_.time_s);
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
                  std::max(static_cast<float>(detection_config_.range_gate_bins),
                           std::ceil(3.0f * detection_config_.range_association_sigma_m /
                                     std::max(range_bin_spacing_m, 1.0e-6f))))
            : range_count;
    const std::size_t doppler_gate =
        tracking_state_.initialized
            ? static_cast<std::size_t>(
                  std::max(static_cast<float>(detection_config_.doppler_gate_bins),
                           std::ceil(3.0f * detection_config_.doppler_association_sigma_hz /
                                     std::max(doppler_bin_spacing_hz, 1.0e-6f))))
            : nfft_doppler;

    std::size_t center_rbin = range_count / 2;
    std::size_t center_dbin = nfft_doppler / 2;
    if (tracking_state_.initialized) {
        center_rbin = nearestAxisBin(range_axis_sliced_m_, range_count, predicted_range_m);
        center_dbin = nearestAxisBin(doppler_axis_hz_, nfft_doppler, predicted_doppler_hz);
    }

    std::vector<CandidateScore> rd_candidates;
    rd_candidates.reserve(std::min(doppler_gate, nfft_doppler) * std::min(range_gate, range_count));

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

        const float candidate_doppler_hz = doppler_axis_hz_[dbin];
        const float range_correction = (candidate_doppler_hz / radar_config_.chirpSlopeHzPerS()) *
                                       (radar_config_.speed_of_light_mps / 2.0f);

        for (std::size_t rbin = r_start; rbin < r_end; ++rbin) {
            const float power = rd_power[dbin * range_count + rbin];
            const float candidate_range_m =
                range_axis_sliced_m_[static_cast<Eigen::Index>(rbin)] - range_correction;
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

    const std::size_t candidate_count = std::min(kMaxRangeDopplerCandidates, rd_candidates.size());
    std::partial_sort(
        rd_candidates.begin(),
        rd_candidates.begin() + candidate_count,
        rd_candidates.end(),
        [](const CandidateScore &lhs, const CandidateScore &rhs) { return lhs.score > rhs.score; });

    // Select best candidate and run AoA beamforming.
    float best_total_score = std::numeric_limits<float>::lowest();
    std::size_t best_doppler_bin = rd_candidates.front().doppler_bin;
    std::size_t best_range_bin = rd_candidates.front().range_bin;
    std::size_t best_direction_index = 0;
    float best_range_m = rd_candidates.front().range_m;
    float best_doppler_hz = rd_candidates.front().doppler_hz;
    std::array<Complex, kNumRx> best_snapshot{};
    Vec3 best_direction = tracking_state_.initialized ? tracking_state_.direction : Vec3::UnitX();

    const float predicted_azimuth_deg = azimuthDeg(predicted_direction);
    const float predicted_elevation_deg = elevationDeg(predicted_direction);

    for (std::size_t candidate_index = 0; candidate_index < candidate_count; ++candidate_index) {
        const CandidateScore &candidate = rd_candidates[candidate_index];
        std::array<Complex, kNumRx> snapshot{};
        for (std::size_t rx = 0; rx < num_rx; ++rx) {
            snapshot[rx] = rd_cube[cubeIndex(
                candidate.doppler_bin, candidate.range_bin, rx, range_count, num_rx)];
        }

        Vec3 candidate_direction = Vec3::UnitX();
        std::size_t candidate_direction_index = 0;
        float best_direction_score =
            detection_config_.aoa_enable ? std::numeric_limits<float>::lowest() : 0.0f;

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

                float direction_score = std::log(std::norm(response) + kScoreEpsilon);
                if (tracking_state_.initialized) {
                    direction_score += associationPenalty(
                        angleDifferenceDeg(azimuthDeg(directions_[direction_index]),
                                           predicted_azimuth_deg),
                        detection_config_.azimuth_association_sigma_deg);
                    direction_score += associationPenalty(
                        elevationDeg(directions_[direction_index]) - predicted_elevation_deg,
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

                for (std::size_t elevation_index = 0; elevation_index < elevation_count;
                     elevation_index += coarse_elevation_stride) {
                    const std::size_t direction_index =
                        azimuth_index * elevation_count + elevation_index;

                    Complex response(0.0f, 0.0f);
                    const std::size_t base = direction_index * num_rx;
                    for (std::size_t rx = 0; rx < num_rx; ++rx) {
                        response += steering_conj_[base + rx] * snapshot[rx];
                    }

                    float direction_score = std::log(std::norm(response) + kScoreEpsilon);
                    if (tracking_state_.initialized) {
                        direction_score += associationPenalty(
                            angleDifferenceDeg(azimuthDeg(directions_[direction_index]),
                                               predicted_azimuth_deg),
                            detection_config_.azimuth_association_sigma_deg);
                        direction_score += associationPenalty(
                            elevationDeg(directions_[direction_index]) - predicted_elevation_deg,
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
            interpolateAxis(doppler_axis_hz_, nfft_doppler, best_doppler_bin, doppler_delta_raw);

        bool bad_doppler_interp = false;
        if (tracking_state_.initialized) {
            float doppler_jump_hz = std::abs(doppler_hz_raw - predicted_doppler_hz);
            bad_doppler_interp = doppler_jump_hz > detection_config_.doppler_interp_gate_hz;
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
            float refined_range_candidate = interpolateAxis(
                range_axis_sliced_m_, range_count, best_range_bin, range_offset_raw);
            const float candidate_doppler_hz = doppler_axis_hz_[best_doppler_bin];
            const float range_correction =
                (candidate_doppler_hz / radar_config_.chirpSlopeHzPerS()) *
                (radar_config_.speed_of_light_mps / 2.0f);
            refined_range_candidate -= range_correction;
            float range_jump_m = std::abs(refined_range_candidate - predicted_range_m);
            bad_range_interp = range_jump_m > detection_config_.range_interp_gate_m;
        }

        range_bin_offset = bad_range_interp ? 0.0f : range_offset_raw;
    }

    best_doppler_hz =
        interpolateAxis(doppler_axis_hz_, nfft_doppler, best_doppler_bin, doppler_bin_offset);
    const float refined_raw_range_m =
        interpolateAxis(range_axis_sliced_m_, range_count, best_range_bin, range_bin_offset);
    const float refined_range_correction = (best_doppler_hz / radar_config_.chirpSlopeHzPerS()) *
                                           (radar_config_.speed_of_light_mps / 2.0f);
    best_range_m = refined_raw_range_m - refined_range_correction;

    // Measurement gating.
    const float max_range_jump_m = std::max(
        detection_config_.range_association_sigma_m,
        (static_cast<float>(detection_config_.range_gate_bins) + 1.0f) * range_bin_spacing_m);
    const float max_doppler_jump_hz = std::max(
        detection_config_.doppler_association_sigma_hz,
        (static_cast<float>(detection_config_.doppler_gate_bins) + 1.5f) * doppler_bin_spacing_hz);

    bool range_ok = !tracking_state_.initialized ||
                    std::abs(best_range_m - predicted_range_m) < max_range_jump_m;
    bool doppler_ok = !tracking_state_.initialized ||
                      std::abs(best_doppler_hz - predicted_doppler_hz) < max_doppler_jump_hz;

    BatchResult result;
    result.time_s = batch_time_s;
    result.predicted_range_m = predicted_range_m;
    result.predicted_doppler_hz = predicted_doppler_hz;
    result.predicted_direction = predicted_direction;

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
            result.slow_time_phase_rad[static_cast<Eigen::Index>(chirp)] =
                std::arg(slow_time_response);
        }
    }
    for (std::size_t dbin = 0; dbin < nfft_doppler; ++dbin) {
        result.doppler_slice_power[static_cast<Eigen::Index>(dbin)] =
            rd_power[dbin * range_count + best_range_bin];
    }
    return result;
}

TrackSummary StreamingTracker::buildSummary() const {
    TrackSummary summary;
    summary.batch_results = batch_results_;
    summary.microdoppler_truth_frequency_hz =
        description_.cars.empty() ? 0.0f : description_.cars.front().bounce_frequency_hz;
    summary.velocity_axis_mps.resize(kFixedDopplerFftSize);
    for (std::size_t i = 0; i < kFixedDopplerFftSize; ++i) {
        summary.velocity_axis_mps[i] = velocity_axis_mps_[static_cast<Eigen::Index>(i)];
    }

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
    summary.detrended_phase_rad.reserve(batch_results_.size());
    summary.cartesian_velocity_mps.reserve(batch_results_.size());
    summary.truth_metrics.reserve(batch_results_.size());

    std::vector<float> raw_x(batch_results_.size());
    std::vector<float> raw_y(batch_results_.size());
    std::vector<float> raw_z(batch_results_.size());

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

    std::vector<float> microdoppler_times_s;
    std::vector<float> microdoppler_unwrapped_phase;
    std::vector<float> residual_phase_example;
    std::vector<float> microdoppler_candidate_frequency_hz;
    std::vector<float> microdoppler_candidate_power;
    const float phase_microdoppler_frequency_hz =
        estimateDominantCpiResidualFrequencyHz(batch_results_,
                                               radar_config_.chirp_duration_s,
                                               &microdoppler_times_s,
                                               &microdoppler_unwrapped_phase,
                                               &residual_phase_example);
    summary.microdoppler_phase_frequency_hz = estimateDominantBatchDopplerFrequencyHz(
        batch_results_,
        static_cast<float>(detection_config_.hop_chirps) * radar_config_.chirp_duration_s,
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
    const float microdoppler_error_hz =
        summary.microdoppler_phase_frequency_hz - summary.microdoppler_truth_frequency_hz;
    summary.microdoppler_frequency_rmse_hz =
        std::sqrt(microdoppler_error_hz * microdoppler_error_hz);
    if (!summary.detrended_phase_rad.empty()) {
        float sum = 0.0f;
        float squared_sum = 0.0f;
        for (float sample : summary.detrended_phase_rad) {
            sum += sample;
            squared_sum += sample * sample;
        }
        summary.microdoppler_residual_phase_mean_rad =
            sum / static_cast<float>(summary.detrended_phase_rad.size());
        summary.microdoppler_residual_phase_rms_rad =
            std::sqrt(squared_sum / static_cast<float>(summary.detrended_phase_rad.size()));
        float variance_sum = 0.0f;
        for (float sample : summary.detrended_phase_rad) {
            const float centered = sample - summary.microdoppler_residual_phase_mean_rad;
            variance_sum += centered * centered;
        }
        summary.microdoppler_residual_phase_stddev_rad =
            std::sqrt(variance_sum / static_cast<float>(summary.detrended_phase_rad.size()));
    }

    const std::size_t window_len = std::min<std::size_t>(5, batch_results_.size());
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
