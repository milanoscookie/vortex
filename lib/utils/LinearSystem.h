#pragma once

#include "RingBuffer.h"
#include "dsp_config.h"
#include <Eigen/Dense>

template <int IR_SIZE> class LinearSystem {
  public:
    using Block = Eigen::Matrix<float, dsp::BLOCK_SIZE, 1>;
    using IRBlock = Eigen::Matrix<float, IR_SIZE, 1>;

    static constexpr int kNumBlocks = (IR_SIZE + dsp::BLOCK_SIZE - 1) / dsp::BLOCK_SIZE + 1;

    using InputHistoryBuffer = RingBuffer<Block, kNumBlocks>;

    LinearSystem() = default;
    LinearSystem(const IRBlock &impulseResponse) : impulseResponse_(impulseResponse) {}

    // RT-unsafe.
    // Updates filter coefficients/state outside the processing path.
    void setImpulseResponse(const IRBlock &impulseResponse) {
        impulseResponse_ = impulseResponse;
    }

    // RT-safe.
    const IRBlock &getImpulseResponse() const {
        return impulseResponse_;
    }

    // RT-safe.
    // Processes one fixed-size block using preallocated history storage.
    // Implements output[n] = sum_k h[k] * input[n - k] across block boundaries.
    void step(const Block &input, Block &output) {
        inputHistory_.pushBack(input);
        output.setZero();

        // For each output sample within the block
        for (int n = 0; n < dsp::BLOCK_SIZE; ++n) {
            float y = 0.0f;

            // Convolve with h[0..IR_SIZE-1]
            for (int k = 0; k < IR_SIZE; ++k) {
                const int x_index = n - k; // relative to current block start

                if (x_index >= 0) {
                    // current block
                    y += impulseResponse_(k) * input(x_index);
                } else {
                    // previous blocks
                    const int past =
                        (-x_index - 1) / dsp::BLOCK_SIZE + 1; // 1 => immediately previous block
                    const int idx_in_block =
                        x_index + past * dsp::BLOCK_SIZE; // bring into [0, BLOCK_SIZE)

                    if (static_cast<std::size_t>(past) <= inputHistory_.size()) {
                        const Block *b = inputHistory_.fromBack(static_cast<std::size_t>(past));
                        if (b) {
                            y += impulseResponse_(k) * (*b)(idx_in_block);
                        }
                    } else {
                        // this is older than history buffer, treat as zero
                    }
                }
            }

            output(n) = y;
        }
    }

  private:
    IRBlock impulseResponse_ = IRBlock::Zero();
    InputHistoryBuffer inputHistory_;
};
