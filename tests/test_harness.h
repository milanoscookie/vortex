// Minimal test harness — no external dependencies
#pragma once
#include <cmath>
#include <iostream>
#include <string>
#include <vector>

struct TestResult {
  std::string name;
  bool passed;
  std::string msg;
};

static std::vector<TestResult> g_results;
static std::size_t g_fails = 0;
static std::string g_current_test_name;

inline void fail_current_test(const std::string &msg) {
  for (auto &result : g_results) {
    if (result.name == g_current_test_name) {
      result.passed = false;
      result.msg = msg;
      return;
    }
  }
}

#define TEST(name)                                                             \
  static void test_##name();                                                   \
  struct Register_##name {                                                     \
    Register_##name() { g_results.push_back({#name, true, ""}); }             \
  } reg_##name;                                                               \
  static void test_##name()

#define RUN_TEST(test_name)                                                    \
  do {                                                                         \
    g_current_test_name = #test_name;                                          \
    try {                                                                      \
      test_##test_name();                                                      \
    } catch (const std::exception &e) {                                        \
      fail_current_test(std::string("EXCEPTION: ") + e.what());                \
      g_fails++;                                                               \
    }                                                                          \
  } while (0)

#define ASSERT_TRUE(cond)                                                      \
  do {                                                                         \
    if (!(cond)) {                                                             \
      fail_current_test(#cond);                                                \
      std::cerr << "  FAIL: " << __FILE__ << ":" << __LINE__ << "  "           \
                << #cond << std::endl;                                         \
      g_fails++;                                                               \
      return;                                                                  \
    }                                                                          \
  } while (0)

#define ASSERT_EQ(a, b)                                                        \
  do {                                                                         \
    if ((a) != (b)) {                                                          \
      fail_current_test(std::string(#a " == " #b));                            \
      std::cerr << "  FAIL: " << __FILE__ << ":" << __LINE__ << "  "           \
                << #a << " == " << #b << "  (" << (a) << " vs " << (b)        \
                << ")" << std::endl;                                           \
      g_fails++;                                                               \
      return;                                                                  \
    }                                                                          \
  } while (0)

#define ASSERT_NEAR(a, b, tol)                                                 \
  do {                                                                         \
    if (std::abs((a) - (b)) > (tol)) {                                         \
      fail_current_test(std::string(#a " ~= " #b));                            \
      std::cerr << "  FAIL: " << __FILE__ << ":" << __LINE__ << "  "           \
                << #a << " ≈ " << #b << "  (" << (a) << " vs " << (b)        \
                << ", tol=" << (tol) << ")" << std::endl;                      \
      g_fails++;                                                               \
      return;                                                                  \
    }                                                                          \
  } while (0)

#define PRINT_RESULTS()                                                        \
  do {                                                                         \
    std::cout << "\n--- Results ---" << std::endl;                             \
    for (auto &r : g_results)                                                  \
      std::cout << (r.passed ? "  ✓ " : "  ✗ ") << r.name                    \
                << (r.msg.empty() ? "" : "  " + r.msg) << std::endl;          \
    std::cout << "\n" << g_results.size() << " tests, " << g_fails            \
              << " failures" << std::endl;                                     \
  } while (0)
