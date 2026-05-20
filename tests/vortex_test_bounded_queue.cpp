// Tests for BoundedQueue<T, Capacity>
#include "test_harness.h"
#include "utils/BoundedQueue.h"

TEST(empty_on_construction) {
    BoundedQueue<int, 4> queue;
    ASSERT_TRUE(queue.empty());
    ASSERT_EQ(queue.size(), 0u);
    ASSERT_EQ(queue.capacity(), 4u);
    ASSERT_TRUE(!queue.pop().has_value());
}

TEST(push_and_pop) {
    BoundedQueue<int, 4> queue;
    queue.push(10);
    queue.push(20);

    ASSERT_EQ(queue.size(), 2u);
    ASSERT_EQ(queue.pop().value(), 10);
    ASSERT_EQ(queue.pop().value(), 20);
    ASSERT_TRUE(queue.empty());
}

TEST(overwrite_oldest_when_full) {
    BoundedQueue<int, 3> queue;
    queue.push(1);
    queue.push(2);
    queue.push(3);
    queue.push(4);

    ASSERT_EQ(queue.size(), 3u);
    ASSERT_EQ(queue.pop().value(), 2);
    ASSERT_EQ(queue.pop().value(), 3);
    ASSERT_EQ(queue.pop().value(), 4);
    ASSERT_TRUE(!queue.pop().has_value());
}

TEST(wraparound_after_pop) {
    BoundedQueue<int, 3> queue;
    queue.push(1);
    queue.push(2);
    ASSERT_EQ(queue.pop().value(), 1);

    queue.push(3);
    queue.push(4);

    ASSERT_EQ(queue.size(), 3u);
    ASSERT_EQ(queue.pop().value(), 2);
    ASSERT_EQ(queue.pop().value(), 3);
    ASSERT_EQ(queue.pop().value(), 4);
}

int main() {
    RUN_TEST(empty_on_construction);
    RUN_TEST(push_and_pop);
    RUN_TEST(overwrite_oldest_when_full);
    RUN_TEST(wraparound_after_pop);
    PRINT_RESULTS();
    return g_fails > 0 ? 1 : 0;
}
