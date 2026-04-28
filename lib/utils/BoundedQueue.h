#pragma once

#include <cstddef>
#include <deque>
#include <optional>

template <typename T> class BoundedQueue {
  public:
    explicit BoundedQueue(std::size_t max_capacity) : max_capacity_(max_capacity) {}
    ~BoundedQueue() = default;

    void push(const T &item) {
        if (max_capacity_ == 0) {
            return;
        }
        if (items_.size() == max_capacity_) {
            items_.pop_front();
        }
        items_.push_back(item);
    }

    std::optional<T> pop() {
        if (items_.empty()) {
            return std::nullopt;
        }

        T item = items_.front();
        items_.pop_front();
        return item;
    }

    bool empty() const noexcept {
        return items_.empty();
    }
    std::size_t size() const noexcept {
        return items_.size();
    }
    std::size_t capacity() const noexcept {
        return max_capacity_;
    }

  private:
    std::size_t max_capacity_ = 0;
    std::deque<T> items_;
};
