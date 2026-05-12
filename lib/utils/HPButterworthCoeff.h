#pragma once

#include "IIRFilter.h"

#include <cmath>

class HPButterworthCoeff {
  public:
    using FilterCoeff = IIRFilter::FilterCoeff;

    HPButterworthCoeff() = default;
    ~HPButterworthCoeff() = default;

    HPButterworthCoeff(double cutoffFrequency, double samplingRate)
        : cutoffFrequency_(cutoffFrequency), samplingRate_(samplingRate) {
        calculateCoefficients();
    }

    void setCutoffFrequency(double cutoffFrequency) {
        cutoffFrequency_ = cutoffFrequency;
        calculateCoefficients();
    }

    double getCutoffFrequency() const {
        return cutoffFrequency_;
    }

    void setSamplingRate(double samplingRate) {
        samplingRate_ = samplingRate;
        calculateCoefficients();
    }

    double getSamplingRate() const {
        return samplingRate_;
    }

    void calculateCoefficients() {
        constexpr double kPi = 3.14159265358979323846;

        const double w0 = 2.0 * kPi * cutoffFrequency_ / samplingRate_;
        const double cosw0 = std::cos(w0);
        const double sinw0 = std::sin(w0);

        constexpr double q = 0.7071067811865476;
        const double alpha = sinw0 / (2.0 * q);

        double b0 = (1.0 + cosw0) * 0.5;
        double b1 = -(1.0 + cosw0);
        double b2 = (1.0 + cosw0) * 0.5;
        const double a0 = 1.0 + alpha;
        double a1 = -2.0 * cosw0;
        double a2 = 1.0 - alpha;

        b0 /= a0;
        b1 /= a0;
        b2 /= a0;
        a1 /= a0;
        a2 /= a0;

        coeffs_ << b0, b1, b2, a1, a2;
    }

    const FilterCoeff &getCoefficients() const {
        return coeffs_;
    }

  private:
    FilterCoeff coeffs_ = IIRFilter::identityCoeffs();
    double cutoffFrequency_ = 1.0;
    double samplingRate_ = 1.0;
};
