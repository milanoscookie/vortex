// Tests for RingBuffer<T, Capacity>
#include "test_harness.h"
#include "utils/RingBuffer.h"

TEST(empty_on_construction) {
    RingBuffer<int, 4> rb;
    ASSERT_EQ(rb.size(), 0u);
    ASSERT_TRUE(rb.front() == nullptr);
    ASSERT_TRUE(rb.back() == nullptr);
}

TEST(push_and_front_back) {
    RingBuffer<int, 4> rb;
    rb.pushBack(10);
    ASSERT_EQ(rb.size(), 1u);
    ASSERT_EQ(*rb.front(), 10);
    ASSERT_EQ(*rb.back(), 10);

    rb.pushBack(20);
    ASSERT_EQ(rb.size(), 2u);
    ASSERT_EQ(*rb.front(), 10);
    ASSERT_EQ(*rb.back(), 20);
}

TEST(pop_front) {
    RingBuffer<int, 4> rb;
    rb.pushBack(1);
    rb.pushBack(2);
    rb.pushBack(3);
    rb.popFront();
    ASSERT_EQ(rb.size(), 2u);
    ASSERT_EQ(*rb.front(), 2);
    ASSERT_EQ(*rb.back(), 3);
}

TEST(overwrite_oldest_when_full) {
    RingBuffer<int, 3> rb;
    rb.pushBack(1);
    rb.pushBack(2);
    rb.pushBack(3);
    ASSERT_EQ(rb.size(), 3u);
    // Push a 4th — should overwrite 1
    rb.pushBack(4);
    ASSERT_EQ(rb.size(), 3u);
    ASSERT_EQ(*rb.front(), 2);
    ASSERT_EQ(*rb.back(), 4);
}

TEST(from_back) {
    RingBuffer<int, 8> rb;
    rb.pushBack(10);
    rb.pushBack(20);
    rb.pushBack(30);
    ASSERT_EQ(*rb.fromBack(0), 30); // newest
    ASSERT_EQ(*rb.fromBack(1), 20);
    ASSERT_EQ(*rb.fromBack(2), 10); // oldest
    ASSERT_TRUE(rb.fromBack(3) == nullptr);
}

TEST(from_front) {
    RingBuffer<int, 8> rb;
    rb.pushBack(10);
    rb.pushBack(20);
    rb.pushBack(30);
    ASSERT_EQ(*rb.fromFront(0), 10); // oldest
    ASSERT_EQ(*rb.fromFront(1), 20);
    ASSERT_EQ(*rb.fromFront(2), 30); // newest
    ASSERT_TRUE(rb.fromFront(3) == nullptr);
}

TEST(bracket_operator) {
    RingBuffer<int, 4> rb;
    rb.pushBack(100);
    rb.pushBack(200);
    rb.pushBack(300);
    ASSERT_EQ(rb[0], 100);
    ASSERT_EQ(rb[1], 200);
    ASSERT_EQ(rb[2], 300);
}

TEST(clear) {
    RingBuffer<int, 4> rb;
    rb.pushBack(1);
    rb.pushBack(2);
    rb.clear();
    ASSERT_EQ(rb.size(), 0u);
    ASSERT_TRUE(rb.front() == nullptr);
}

TEST(wraparound_stress) {
    RingBuffer<int, 4> rb;
    for (std::size_t i = 0; i < 100; ++i)
        rb.pushBack(static_cast<int>(i));
    ASSERT_EQ(rb.size(), 4u);
    ASSERT_EQ(*rb.front(), 96);
    ASSERT_EQ(*rb.back(), 99);
}

int main() {
    RUN_TEST(empty_on_construction);
    RUN_TEST(push_and_front_back);
    RUN_TEST(pop_front);
    RUN_TEST(overwrite_oldest_when_full);
    RUN_TEST(from_back);
    RUN_TEST(from_front);
    RUN_TEST(bracket_operator);
    RUN_TEST(clear);
    RUN_TEST(wraparound_stress);
    PRINT_RESULTS();
    return g_fails > 0 ? 1 : 0;
}
