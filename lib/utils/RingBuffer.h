#pragma once

#include <array>
#include <cstddef>

template <typename T, std::size_t Capacity> class RingBuffer {
  public:
    // RT-safe.
    // Writes one item into fixed-capacity storage and overwrites the oldest item when full.
    void pushBack(const T &value) {
        buffer_[tailIndex_] = value;
        tailIndex_ = (tailIndex_ + 1U) % Capacity;

        if (size_ < Capacity) {
            ++size_;
        } else {
            headIndex_ = (headIndex_ + 1U) % Capacity;
        }
    }

    // RT-safe.
    // Removes the oldest item when one is available.
    void popFront() noexcept {
        if (size_ == 0U) {
            return;
        }

        headIndex_ = (headIndex_ + 1U) % Capacity;
        --size_;
    }

    // RT-safe.
    // Returns the oldest item pointer or null when the buffer is empty.
    [[nodiscard]] const T *front() const noexcept {
        if (size_ == 0U) {
            return nullptr;
        }

        return &buffer_[headIndex_];
    }

    // RT-safe.
    // Returns the newest item pointer or null when the buffer is empty.
    [[nodiscard]] const T *back() const noexcept {
        if (size_ == 0U) {
            return nullptr;
        }

        return &buffer_[(tailIndex_ + Capacity - 1U) % Capacity];
    }

    // RT-safe.
    // Returns the kth item from the newest end, where k = 0 selects the newest item.
    [[nodiscard]] const T *fromBack(std::size_t index) const noexcept {
        if (index >= size_) {
            return nullptr;
        }

        return &buffer_[(tailIndex_ + Capacity - 1U - index) % Capacity];
    }

    // RT-safe.
    // Returns the kth item from the oldest end, where k = 0 selects the oldest item.
    [[nodiscard]] const T *fromFront(std::size_t index) const noexcept {
        if (index >= size_) {
            return nullptr;
        }

        return &buffer_[(headIndex_ + index) % Capacity];
    }

    // RT-safe.
    // Returns the kth item from the oldest end, where k = 0 selects the oldest item.
    [[nodiscard]] const T &operator[](std::size_t index) const noexcept {
        return buffer_[(headIndex_ + index) % Capacity];
    }

    // RT-safe.
    [[nodiscard]] std::size_t size() const noexcept {
        return size_;
    }

    // RT-safe.
    [[nodiscard]] static constexpr std::size_t capacity() noexcept {
        return Capacity;
    }

    // RT-safe.
    // Resets logical indices without touching stored elements.
    void clear() noexcept {
        headIndex_ = 0U;
        tailIndex_ = 0U;
        size_ = 0U;
    }

  private:
    std::array<T, Capacity> buffer_{};
    std::size_t headIndex_ = 0U;
    std::size_t tailIndex_ = 0U;
    std::size_t size_ = 0U;
};
