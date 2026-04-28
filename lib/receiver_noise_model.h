#pragma once

#include "problem_description.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <complex>
#include <cstdint>

namespace radar {

class ReceiverNoiseModel {
public:
  using Complex = std::complex<float>;
  static constexpr std::size_t kZigguratTableSize = 256U;

  ReceiverNoiseModel(float receiver_noiselevel_stddev,
                     float receiver_noiselevel_mean,
                     float receiver_noise_distribution_stddev,
                     std::uint32_t random_seed)
      : receiver_noiselevel_stddev_(receiver_noiselevel_stddev),
        receiver_noise_distribution_mean_(receiver_noiselevel_mean),
        receiver_noise_distribution_stddev_(
            receiver_noise_distribution_stddev) {
    seedRng(random_seed);
    initializeZigguratTables();
  }

  [[nodiscard]] bool enabled() const noexcept {
    return receiver_noiselevel_stddev_ > 0.0f;
  }

  [[nodiscard]] Complex sample() {
    if (!enabled()) {
      return Complex(0.0f, 0.0f);
    }

    const float quadrature_sigma =
        receiver_noiselevel_stddev_ * problem::Constants::kInvSqrt2;
    const float effective_sigma =
        quadrature_sigma * receiver_noise_distribution_stddev_;
    return Complex(sampleStandardNormal() * effective_sigma +
                       receiver_noise_distribution_mean_,
                   sampleStandardNormal() * effective_sigma +
                       receiver_noise_distribution_mean_);
  }

  template <typename Buffer> void fill(Buffer &buffer) {
    if (!enabled()) {
      std::fill(buffer.begin(), buffer.end(), Complex(0.0f, 0.0f));
      return;
    }

    for (auto &sample_value : buffer) {
      sample_value = sample();
    }
  }

private:
  static constexpr float kZigguratTailStart = 3.442619855899f;
  static constexpr float kZigguratArea = 9.91256303526217e-3f;
  static constexpr float kInt32Scale = 2147483648.0f;
  static constexpr float kOpenUnitScale = 1.0f / 4294967296.0f;

  static constexpr std::uint32_t rotateLeft(std::uint32_t value,
                                            int shift) noexcept {
    return (value << shift) | (value >> (32 - shift));
  }

  static std::uint32_t splitMix32(std::uint32_t &state) noexcept {
    std::uint32_t z = (state += 0x9e3779b9U);
    z = (z ^ (z >> 16)) * 0x85ebca6bU;
    z = (z ^ (z >> 13)) * 0xc2b2ae35U;
    return z ^ (z >> 16);
  }

  void seedRng(std::uint32_t seed) noexcept {
    std::uint32_t state = seed;
    for (auto &word : rng_state_) {
      word = splitMix32(state);
    }
    if ((rng_state_[0] | rng_state_[1] | rng_state_[2] | rng_state_[3]) ==
        0U) {
      rng_state_[0] = 1U;
    }
  }

  [[nodiscard]] static float gaussianPdf(float x) noexcept {
    return std::exp(-0.5f * x * x);
  }

  void initializeZigguratTables() noexcept {
    float dn = kZigguratTailStart;
    float tn = dn;
    const float q = kZigguratArea / gaussianPdf(dn);

    kn_[0] = static_cast<std::uint32_t>((dn / q) * kInt32Scale);
    kn_[1] = 0U;

    wn_[0] = q / kInt32Scale;
    wn_[kZigguratTableSize - 1U] = dn / kInt32Scale;

    fn_[0] = 1.0f;
    fn_[kZigguratTableSize - 1U] = gaussianPdf(dn);

    for (int i = static_cast<int>(kZigguratTableSize) - 2; i >= 1; --i) {
      dn = std::sqrt(-2.0f * std::log(kZigguratArea / dn + gaussianPdf(dn)));
      kn_[static_cast<std::size_t>(i) + 1U] =
          static_cast<std::uint32_t>((dn / tn) * kInt32Scale);
      tn = dn;
      fn_[static_cast<std::size_t>(i)] = gaussianPdf(dn);
      wn_[static_cast<std::size_t>(i)] = dn / kInt32Scale;
    }
  }

  [[nodiscard]] std::uint32_t sampleUint32() noexcept {
    const std::uint32_t result =
        rotateLeft(rng_state_[0] + rng_state_[3], 7) + rng_state_[0];

    const std::uint32_t t = rng_state_[1] << 9;

    rng_state_[2] ^= rng_state_[0];
    rng_state_[3] ^= rng_state_[1];
    rng_state_[1] ^= rng_state_[2];
    rng_state_[0] ^= rng_state_[3];

    rng_state_[2] ^= t;
    rng_state_[3] = rotateLeft(rng_state_[3], 11);

    return result;
  }

  [[nodiscard]] float sampleOpenUnitFloat() noexcept {
    return (static_cast<float>(sampleUint32()) + 1.0f) * kOpenUnitScale;
  }

  [[nodiscard]] static std::uint32_t magnitude(std::int32_t value) noexcept {
    return value < 0 ? static_cast<std::uint32_t>(-static_cast<std::int64_t>(value))
                     : static_cast<std::uint32_t>(value);
  }

  [[nodiscard]] float sampleTail(std::int32_t sample_value) noexcept {
    for (;;) {
      const float x = -0.2904764f * std::log(sampleOpenUnitFloat());
      const float y = -std::log(sampleOpenUnitFloat());
      if (y + y >= x * x) {
        return sample_value > 0 ? kZigguratTailStart + x
                                : -kZigguratTailStart - x;
      }
    }
  }

  [[nodiscard]] float sampleStandardNormal() noexcept {
    std::uint32_t bits = sampleUint32();
    std::int32_t sample_value = static_cast<std::int32_t>(bits);
    std::size_t index = bits & (kZigguratTableSize - 1U);

    if (magnitude(sample_value) < kn_[index]) {
      return static_cast<float>(sample_value) * wn_[index];
    }

    for (;;) {
      if (index == 0U) {
        return sampleTail(sample_value);
      }

      const float x = static_cast<float>(sample_value) * wn_[index];
      const float threshold =
          fn_[index] + sampleOpenUnitFloat() * (fn_[index - 1U] - fn_[index]);
      if (threshold < gaussianPdf(x)) {
        return x;
      }

      bits = sampleUint32();
      sample_value = static_cast<std::int32_t>(bits);
      index = bits & (kZigguratTableSize - 1U);
      if (magnitude(sample_value) < kn_[index]) {
        return static_cast<float>(sample_value) * wn_[index];
      }
    }
  }

  float receiver_noiselevel_stddev_ = 0.0f;
  float receiver_noise_distribution_mean_ = 0.0f;
  float receiver_noise_distribution_stddev_ = 1.0f;
  std::array<std::uint32_t, 4> rng_state_{};
  std::array<std::uint32_t, kZigguratTableSize> kn_{};
  std::array<float, kZigguratTableSize> fn_{};
  std::array<float, kZigguratTableSize> wn_{};
};

} // namespace radar
