#pragma once

namespace dsp {

inline constexpr int SAMPLE_RATE = 10'000'000;
inline constexpr int BLOCK_SIZE = 4096;
inline constexpr int FIR_SIZE = 4096;
inline constexpr int IR_SIZE = FIR_SIZE;
inline constexpr int CONTEXT_BLOCKS =
    (FIR_SIZE + BLOCK_SIZE - 1) / BLOCK_SIZE;
inline constexpr int BLOCK_LATENCY_US =
    static_cast<int>((static_cast<long long>(BLOCK_SIZE) * 1'000'000LL) /
                     SAMPLE_RATE);

} // namespace dsp
