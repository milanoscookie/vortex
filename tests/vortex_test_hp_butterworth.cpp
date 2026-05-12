// Tests for HPButterworthCoeff
#include "test_harness.h"
#include "utils/HPButterworthCoeff.h"

#include <cmath>

TEST(coefficients_valid) {
    HPButterworthCoeff bw(1000.0f, 48000.0f);
    auto c = bw.getCoefficients();
    for (std::size_t i = 0; i < 5; ++i) {
        ASSERT_TRUE(std::isfinite(c(static_cast<Eigen::Index>(i))));
    }
}

TEST(dc_gain_is_zero) {
    HPButterworthCoeff bw(1000.0f, 48000.0f);
    auto c = bw.getCoefficients();
    double dc = (c(0) + c(1) + c(2)) / (1.0 + c(3) + c(4));
    ASSERT_TRUE(std::abs(dc) < 1e-5);
}

TEST(nyquist_gain_is_unity) {
    HPButterworthCoeff bw(1000.0f, 48000.0f);
    auto c = bw.getCoefficients();
    double nyquist = (c(0) - c(1) + c(2)) / (1.0 - c(3) + c(4));
    ASSERT_NEAR(nyquist, 1.0, 1e-5);
}

TEST(filter_rejects_dc) {
    HPButterworthCoeff bw(500.0f, 48000.0f);
    IIRFilter f(bw.getCoefficients());
    double y = 0.0;
    for (std::size_t i = 0; i < 5000; ++i) {
        y = f.filterSample(1.0f);
    }
    ASSERT_TRUE(std::abs(y) < 1e-3);
}

int main() {
    RUN_TEST(coefficients_valid);
    RUN_TEST(dc_gain_is_zero);
    RUN_TEST(nyquist_gain_is_unity);
    RUN_TEST(filter_rejects_dc);
    PRINT_RESULTS();
    return g_fails > 0 ? 1 : 0;
}
