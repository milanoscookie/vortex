#pragma once

#include "problem_description.h"

namespace dsp {

inline constexpr int SAMPLE_RATE = static_cast<int>(kDefaultSampleRateHz);
inline constexpr int BLOCK_SIZE = static_cast<int>(kRadarBlockSize);
inline constexpr int FIR_SIZE = BLOCK_SIZE;
inline constexpr int IR_SIZE = FIR_SIZE;
inline constexpr int CONTEXT_BLOCKS = (FIR_SIZE + BLOCK_SIZE - 1) / BLOCK_SIZE;
inline constexpr int BLOCK_LATENCY_US =
    static_cast<int>((static_cast<long long>(BLOCK_SIZE) * 1'000'000LL) / SAMPLE_RATE);

} // namespace dsp
