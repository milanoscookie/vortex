#pragma once

#include <atomic>
#include <cstddef>
#include <cstdint>

template <typename T> class DoubleBufferSPSC {
  public:
    DoubleBufferSPSC() = default;

    // RT-safe.
    // Publishes one fully written value through fixed-capacity double-buffer storage.
    void publish(const T &value) {
        const std::uint64_t nextSequence = sequence_.load(std::memory_order_relaxed) + 1U;
        const std::size_t writeIndex = indexFromSequence(nextSequence);
        buffers_[writeIndex] = value;
        sequence_.store(nextSequence, std::memory_order_release);
    }

    // RT-safe.
    // Returns a writable staging slot for the next publish on the producer side.
    T &beginWrite() noexcept {
        pendingSequence_ = sequence_.load(std::memory_order_relaxed) + 1U;
        pendingIndex_ = indexFromSequence(pendingSequence_);
        return buffers_[pendingIndex_];
    }

    // RT-safe.
    // Publishes the value previously written through beginWrite().
    void commit() noexcept {
        sequence_.store(pendingSequence_, std::memory_order_release);
    }

    // RT-safe.
    // Reads the latest value only when it has not been consumed yet.
    [[nodiscard]] bool tryRead(T &out) const {
        const std::uint64_t currentSequence = sequence_.load(std::memory_order_acquire);
        if (currentSequence == lastReadSequence_) {
            return false;
        }

        out = buffers_[indexFromSequence(currentSequence)];
        lastReadSequence_ = currentSequence;
        return true;
    }

    // RT-safe.
    // Reads the latest published value, including repeats.
    void readLatest(T &out) const {
        const std::uint64_t currentSequence = sequence_.load(std::memory_order_acquire);
        out = buffers_[indexFromSequence(currentSequence)];
        lastReadSequence_ = currentSequence;
    }

    // RT-safe.
    [[nodiscard]] bool hasNew() const noexcept {
        const std::uint64_t currentSequence = sequence_.load(std::memory_order_acquire);
        return currentSequence != lastReadSequence_;
    }

    // RT-safe.
    [[nodiscard]] std::uint64_t sequence() const noexcept {
        return sequence_.load(std::memory_order_acquire);
    }

  private:
    [[nodiscard]] static std::size_t indexFromSequence(std::uint64_t sequence) noexcept {
        return static_cast<std::size_t>(sequence & 1ULL);
    }

    alignas(64) mutable T buffers_[2] = {T{}, T{}};
    alignas(64) std::atomic<std::uint64_t> sequence_{0U};
    mutable std::uint64_t lastReadSequence_ = 0U;
    std::uint64_t pendingSequence_ = 0U;
    std::size_t pendingIndex_ = 0U;
};
