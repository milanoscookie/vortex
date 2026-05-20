#pragma once

#include <array>
#include <cmath>
#include <cstddef>

namespace radar {

template <typename Complex, std::size_t HistorySize> class TxHistoryBuffer {
  public:
    static_assert((HistorySize & (HistorySize - 1U)) == 0U,
                  "TX history size must be a power of two");

    static constexpr std::size_t kMask = HistorySize - 1U;

    TxHistoryBuffer() {
        clear();
    }

    // RT-safe.
    // Clears the fixed-capacity history buffer.
    void clear() {
        samples_.fill(Complex(0.0, 0.0));
    }

    // RT-safe.
    // Stores one transmit sample into the circular history.
    void store(std::size_t sampleIndex, Complex sample) {
        samples_[sampleIndex & kMask] = sample;
    }

    // RT-safe.
    // Stores a contiguous block of transmit samples into the circular history.
    void storeBlock(std::size_t startIndex, const Complex *samples, std::size_t sampleCount) {
        for (std::size_t index = 0; index < sampleCount; ++index) {
            store(startIndex + index, samples[index]);
        }
    }

    // RT-safe.
    // Returns a linearly interpolated delayed sample from the fixed-capacity history.
    [[nodiscard]] Complex delayedSample(std::size_t sampleIndex, double delaySamples) const {
        const double delayedIndex =
            static_cast<double>(sampleIndex) - static_cast<double>(delaySamples);
        if (delayedIndex < 0.0) {
            return Complex(0.0, 0.0);
        }

        const std::size_t lowerIndex = static_cast<std::size_t>(std::floor(delayedIndex));
        const std::size_t upperIndex = lowerIndex + 1U;
        if (sampleIndex < lowerIndex || sampleIndex - lowerIndex >= HistorySize) {
            return Complex(0.0, 0.0);
        }

        const double fraction = delayedIndex - static_cast<double>(lowerIndex);
        const Complex lowerSample = samples_[lowerIndex & kMask];
        Complex upperSample = lowerSample;
        if (upperIndex <= sampleIndex && sampleIndex - upperIndex < HistorySize) {
            upperSample = samples_[upperIndex & kMask];
        }

        return ((1.0 - fraction) * lowerSample) + (fraction * upperSample);
    }

  private:
    std::array<Complex, HistorySize> samples_{};
};

} // namespace radar
