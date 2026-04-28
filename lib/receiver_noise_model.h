#pragma once

#include "problem_description.h"

#include <algorithm>
#include <array>
#include <bit>
#include <cmath>
#include <complex>
#include <cstdint>

namespace radar {

class ReceiverNoiseModel {
  public:
    using Complex = std::complex<float>;

    ReceiverNoiseModel(float receiver_noiselevel_stddev,
                       float receiver_noiselevel_mean,
                       float receiver_noise_distribution_stddev,
                       std::uint32_t random_seed)
        : receiver_noiselevel_stddev_(receiver_noiselevel_stddev),
          receiver_noise_distribution_mean_(receiver_noiselevel_mean),
          receiver_noise_distribution_stddev_(receiver_noise_distribution_stddev) {
        updateDerivedConstants();
        seedRng(random_seed);
    }

    bool enabled() const noexcept {
        return noise_enabled_;
    }

    Complex sample() {
        if (!enabled()) {
            return Complex(0.0f, 0.0f);
        }

        return Complex(
            sampleStandardNormal() * effective_sigma_ + receiver_noise_distribution_mean_,
            sampleStandardNormal() * effective_sigma_ + receiver_noise_distribution_mean_);
    }

    template <typename Buffer> void fill(Buffer &buffer) {
        if (!enabled()) {
            std::fill(buffer.begin(), buffer.end(), Complex(0.0f, 0.0f));
            return;
        }

        for (auto &sample_value : buffer) {
            sample_value = Complex(
                sampleStandardNormal() * effective_sigma_ + receiver_noise_distribution_mean_,
                sampleStandardNormal() * effective_sigma_ + receiver_noise_distribution_mean_);
        }
    }

  private:
    static constexpr float kPopcountNormalMean = 48.0f;
    static constexpr float kPopcountNormalScale = 0.2041241452319315f;

    static constexpr std::uint32_t rotateLeft(std::uint32_t value, int shift) noexcept {
        return (value << shift) | (value >> (32 - shift));
    }

    static std::uint32_t splitMix32(std::uint32_t &state) noexcept {
        std::uint32_t z = (state += 0x9e3779b9U);
        z = (z ^ (z >> 16)) * 0x85ebca6bU;
        z = (z ^ (z >> 13)) * 0xc2b2ae35U;
        return z ^ (z >> 16);
    }

    void updateDerivedConstants() noexcept {
        noise_enabled_ = receiver_noiselevel_stddev_ > 0.0f;
        effective_sigma_ = receiver_noiselevel_stddev_ * problem::Constants::kInvSqrt2 *
                           receiver_noise_distribution_stddev_;
    }

    void seedRng(std::uint32_t seed) noexcept {
        std::uint32_t state = seed;
        for (auto &word : rng_state_) {
            word = splitMix32(state);
        }
        if ((rng_state_[0] | rng_state_[1] | rng_state_[2] | rng_state_[3]) == 0U) {
            rng_state_[0] = 1U;
        }
    }

    std::uint32_t sampleUint32() noexcept {
        const std::uint32_t result = rotateLeft(rng_state_[0] + rng_state_[3], 7) + rng_state_[0];

        const std::uint32_t t = rng_state_[1] << 9;

        rng_state_[2] ^= rng_state_[0];
        rng_state_[3] ^= rng_state_[1];
        rng_state_[1] ^= rng_state_[2];
        rng_state_[0] ^= rng_state_[3];

        rng_state_[2] ^= t;
        rng_state_[3] = rotateLeft(rng_state_[3], 11);

        return result;
    }

    float sampleStandardNormal() noexcept {
        const unsigned bit_count = std::popcount(sampleUint32()) + std::popcount(sampleUint32()) +
                                   std::popcount(sampleUint32());
        return (static_cast<float>(bit_count) - kPopcountNormalMean) * kPopcountNormalScale;
    }

    float receiver_noiselevel_stddev_ = 0.0f;
    float receiver_noise_distribution_mean_ = 0.0f;
    float receiver_noise_distribution_stddev_ = 1.0f;
    float effective_sigma_ = 0.0f;
    bool noise_enabled_ = false;
    std::array<std::uint32_t, 4> rng_state_{};
};

} // namespace radar
