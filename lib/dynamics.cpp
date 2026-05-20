#include "dynamics.h"

#include <cmath>
#include <stdexcept>

namespace {

double bouncePhaseRadians(const dsp::CarSettings &car, double t_s) noexcept {
    return 2.0 * dsp::Constants::kPi * car.bounce_frequency_hz * t_s + car.bounce_phase_rad;
}

void validateCarSettings(const dsp::CarSettings &car_settings) {
    if (!car_settings.initial_position_m.allFinite() ||
        !car_settings.base_velocity_mps.allFinite()) {
        throw std::invalid_argument("car position and velocity must be finite");
    }
    if (!std::isfinite(car_settings.yaw_rad) || !std::isfinite(car_settings.length_m) ||
        !std::isfinite(car_settings.width_m) || !std::isfinite(car_settings.height_m) ||
        !std::isfinite(car_settings.bounce_amplitude_m) ||
        !std::isfinite(car_settings.bounce_frequency_hz) ||
        !std::isfinite(car_settings.bounce_phase_rad) ||
        !std::isfinite(car_settings.reflectivity.real()) ||
        !std::isfinite(car_settings.reflectivity.imag())) {
        throw std::invalid_argument("car settings must be finite");
    }
    if (car_settings.length_m <= 0.0 || car_settings.width_m <= 0.0 ||
        car_settings.height_m <= 0.0) {
        throw std::invalid_argument("car dimensions must be positive");
    }
}

} // namespace

CarDynamics::CarDynamics(const CarSettings &car_settings) : car_(car_settings) {
    validateCarSettings(car_);
}

CarDynamics::Vec3 CarDynamics::positionAt(double t_s) const noexcept {
    Vec3 position = car_.initial_position_m + car_.base_velocity_mps * t_s;
    position.z() += car_.bounce_amplitude_m * std::sin(bouncePhaseRadians(car_, t_s));
    return position;
}

CarDynamics::Vec3 CarDynamics::velocityAt(double t_s) const noexcept {
    Vec3 velocity = car_.base_velocity_mps;
    velocity.z() += 2.0 * dsp::Constants::kPi * car_.bounce_frequency_hz * car_.bounce_amplitude_m *
                    std::cos(bouncePhaseRadians(car_, t_s));
    return velocity;
}
