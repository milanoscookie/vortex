#pragma once

#include <array>
#include <cstddef>
#include <optional>
#include <utility>

template <typename T, std::size_t Capacity> class BoundedQueue {
  public:
    static_assert(Capacity > 0U, "BoundedQueue capacity must be greater than zero");

    // RT-safe.
    // Uses fixed-capacity std::array storage and overwrites the oldest item when full.
    BoundedQueue() = default;
    ~BoundedQueue() = default;

    // RT-safe.
    // Appends one item and discards the oldest item when the queue is full.
    void push(const T &item) {
        items_[tailIndex_] = item;
        tailIndex_ = nextIndex(tailIndex_);

        if (size_ < Capacity) {
            ++size_;
        } else {
            headIndex_ = nextIndex(headIndex_);
        }
    }

    // RT-safe.
    // Removes and returns the oldest item when available.
    [[nodiscard]] std::optional<T> pop() {
        if (size_ == 0U) {
            return std::nullopt;
        }

        std::optional<T> item = std::move(items_[headIndex_]);
        items_[headIndex_].reset();
        headIndex_ = nextIndex(headIndex_);
        --size_;
        return item;
    }

    // RT-safe.
    [[nodiscard]] bool empty() const noexcept {
        return size_ == 0U;
    }

    // RT-safe.
    [[nodiscard]] std::size_t size() const noexcept {
        return size_;
    }

    // RT-safe.
    [[nodiscard]] static constexpr std::size_t capacity() noexcept {
        return Capacity;
    }

  private:
    [[nodiscard]] static constexpr std::size_t nextIndex(std::size_t index) noexcept {
        return (index + 1U) % Capacity;
    }

    std::array<std::optional<T>, Capacity> items_{};
    std::size_t headIndex_ = 0U;
    std::size_t tailIndex_ = 0U;
    std::size_t size_ = 0U;
};
