#include "test_harness.h"

#include "simulation.h"

#include <cmath>

namespace {

problem::ProblemDescription makeDescription() {
    problem::ProblemDescription description = problem::kDefaultProblemDescription;
    description.simulator.prediction_duration_s = 10.0f;
    description.cars = {
        problem::CarSettings{.initial_position_m = problem::Vec3(10.0f, 20.0f, 5.0f),
                             .base_velocity_mps = problem::Vec3(3.0f, -1.0f, 0.0f),
                             .length_m = 4.0f,
                             .width_m = 2.0f,
                             .bounce_amplitude_m = 0.0f}};
    return description;
}

problem::Vec3 offsetFromLocalFrame(const problem::Vec3 &center,
                                   const problem::Vec3 &velocity,
                                   problem::Real forward_m,
                                   problem::Real lateral_m) {
    const problem::Real yaw_rad = std::atan2(velocity.y(), velocity.x());
    const problem::Real cos_yaw = std::cos(yaw_rad);
    const problem::Real sin_yaw = std::sin(yaw_rad);
    return center + problem::Vec3(cos_yaw * forward_m - sin_yaw * lateral_m,
                                  sin_yaw * forward_m + cos_yaw * lateral_m,
                                  0.0f);
}

} // namespace

TEST(guess_inside_predicted_car_returns_true) {
    const problem::ProblemDescription description = makeDescription();
    RadarSimulator simulator(description);

    const problem::Vec3 predicted_center(40.0f, 10.0f, 5.0f);
    const problem::Vec3 inside_guess =
        offsetFromLocalFrame(predicted_center, description.cars[0].base_velocity_mps, 1.0f, 0.5f);
    ASSERT_TRUE(simulator.isGuessLocationWithinPredictedCar(inside_guess));
}

TEST(guess_outside_predicted_car_returns_false) {
    const problem::ProblemDescription description = makeDescription();
    RadarSimulator simulator(description);

    const problem::Vec3 predicted_center(40.0f, 10.0f, 5.0f);
    const problem::Vec3 outside_guess =
        offsetFromLocalFrame(predicted_center, description.cars[0].base_velocity_mps, 2.1f, 0.0f);
    ASSERT_TRUE(!simulator.isGuessLocationWithinPredictedCar(outside_guess));
}

TEST(rotated_car_uses_local_length_and_width) {
    problem::ProblemDescription description = makeDescription();
    description.simulator.prediction_duration_s = 0.0f;
    description.cars[0].initial_position_m = problem::Vec3(0.0f, 0.0f, 0.0f);
    description.cars[0].base_velocity_mps = problem::Vec3(0.0f, 5.0f, 0.0f);
    RadarSimulator simulator(description);

    ASSERT_TRUE(simulator.isGuessLocationWithinPredictedCar(problem::Vec3(0.9f, 0.0f, 0.0f)));
    ASSERT_TRUE(!simulator.isGuessLocationWithinPredictedCar(problem::Vec3(1.1f, 0.0f, 0.0f)));
}

int main() {
    RUN_TEST(guess_inside_predicted_car_returns_true);
    RUN_TEST(guess_outside_predicted_car_returns_false);
    RUN_TEST(rotated_car_uses_local_length_and_width);
    PRINT_RESULTS();
    return g_fails > 0 ? 1 : 0;
}
