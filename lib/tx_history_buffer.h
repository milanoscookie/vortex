#pragma once

#include <array>
#include <cmath>
#include <cstddef>

namespace radar {

template <typename Complex, std::size_t HistorySize> class TxHistoryBuffer {
public:
  static_assert((HistorySize & (HistorySize - 1U)) == 0U,
                "TX history size must be a power of two");

  using size_t = std::size_t;
  static constexpr size_t kMask = HistorySize - 1U;

  TxHistoryBuffer() { clear(); }

  void clear() noexcept { samples_.fill(Complex(0.0f, 0.0f)); }

  void store(size_t sample_index, Complex sample) noexcept {
    samples_[sample_index & kMask] = sample;
  }

  void storeBlock(size_t start_index, const Complex *samples,
                  size_t sample_count) noexcept {
    for (size_t i = 0; i < sample_count; ++i) {
      store(start_index + i, samples[i]);
    }
  }

  [[nodiscard]] Complex delayedSample(size_t sample_index, float delay_s,
                                      float sample_rate_hz) const noexcept {
    const float delayed_index =
        static_cast<float>(sample_index) - delay_s * sample_rate_hz;
    if (delayed_index < 0.0f) {
      return Complex(0.0f, 0.0f);
    }

    const size_t lower_index = static_cast<size_t>(std::floor(delayed_index));
    const size_t upper_index = lower_index + 1U;
    if (sample_index < lower_index ||
        sample_index - lower_index >= HistorySize) {
      return Complex(0.0f, 0.0f);
    }

    const float fraction = delayed_index - static_cast<float>(lower_index);
    const Complex lower_sample = samples_[lower_index & kMask];
    Complex upper_sample = lower_sample;
    if (upper_index <= sample_index &&
        sample_index - upper_index < HistorySize) {
      upper_sample = samples_[upper_index & kMask];
    }

    return (1.0f - fraction) * lower_sample + fraction * upper_sample;
  }

private:
  std::array<Complex, HistorySize> samples_{};
};

} // namespace radar
