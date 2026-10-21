#include "flygripper_flight/takeoff.hpp"

#include <cstdlib>
#include <iostream>
#include <limits>
#include <stdexcept>

using flygripper_flight::TakeoffPlan;
using Status = TakeoffPlan::Status;

void require(bool condition, const char* message) {
  if (!condition) throw std::runtime_error(message);
}
void near(double actual, double expected, double tolerance, const char* message) {
  require(std::abs(actual - expected) <= tolerance, message);
}

int main() {
  try {
    TakeoffPlan plan;
    TakeoffPlan::Settings settings;
    require(plan.start({2.0, -3.0, 0.6}, 0.7, 100.0, settings), "valid start");
    const double duration = plan.duration();
    const double target_height = 0.6 + settings.height;
    near(plan.sample(100.0).position[2], 0.6, 1e-12, "starts at current height");
    near(plan.sample(100.0 + duration).position[2], target_height, 1e-12, "relative target height");
    near(plan.sample(100.0 + duration).velocity, 0, 1e-12, "zero final velocity");
    double last_z = 0.6;
    for (int i = 1; i < 10000; ++i) {
      const auto s = plan.sample(100.0 + duration * i / 10000.0);
      require(s.position[2] >= last_z, "monotone climb");
      near(s.position[0], 2.0, 1e-12, "fixed X");
      near(s.position[1], -3.0, 1e-12, "fixed Y");
      near(s.yaw, 0.7, 1e-12, "fixed heading");
      require(std::abs(s.velocity) <= settings.velocity + 1e-10, "velocity bound");
      require(std::abs(s.acceleration) <= settings.acceleration + 1e-10, "acceleration bound");
      require(std::abs(s.jerk) <= settings.jerk + 1e-10, "jerk bound");
      const double time = 100.0 + duration * i / 10000.0;
      const double eps = 1e-4;
      const double numerical_velocity = (plan.sample(time + eps).position[2] -
                                         plan.sample(time - eps).position[2]) / (2 * eps);
      near(s.velocity, numerical_velocity, 1e-7, "profile derivative");
      last_z = s.position[2];
    }
    // Follow the complete reference; completion requires a continuous dwell.
    for (double t = 100.0; t < 100.0 + duration; t += 0.01) {
      const auto s = plan.sample(t);
      require(plan.update(t, s.position, {0.0, 0.0, s.velocity}) == Status::RUNNING,
              "no early completion");
    }
    const double finish = 100.0 + duration;
    require(plan.update(finish, {2.0, -3.0, target_height}, {0, 0, 0}) == Status::RUNNING,
            "settling starts");
    require(plan.update(finish + 0.8, {2.0, -3.0, target_height}, {0, 0, 0.3}) == Status::RUNNING,
            "high speed resets dwell");
    require(plan.update(finish + 1.0, {2.0, -3.0, target_height}, {0, 0, 0}) == Status::RUNNING,
            "dwell restarts");
    require(plan.update(finish + 2.01, {2.0, -3.0, target_height}, {0, 0, 0}) == Status::COMPLETE,
            "stable arrival completes");
    // A second sortie must latch its own origin, heading and settings.
    require(plan.start({-4, 5, 2.0}, -0.4, 200, settings), "second sortie");
    settings.height = 4.0;
    near(plan.sample(300).position[2], 2.0 + target_height - 0.6, 1e-12, "settings latched per flight");
    near(plan.sample(300).position[0], -4, 1e-12, "new origin");
    near(plan.sample(300).yaw, -0.4, 1e-12, "new heading");
    require(plan.update(199, {-4, 5, 2.0}, {0, 0, 0}) == Status::INVALID_STATE,
            "clock rollback rejected");
    settings = TakeoffPlan::Settings{};
    require(plan.start({0, 0, 0}, 0, 0, settings), "restart");
    require(plan.update(6.01, {0, 0, 0}, {0, 0, 0}) == Status::NO_LIFTOFF, "no liftoff timeout");
    require(plan.start({0, 0, 0}, 0, 0, settings), "restart");
    require(plan.update(1.0, {1, 0, 0.2}, {0, 0, 0}) == Status::TRACKING_ERROR, "drift abort");
    require(plan.update(21, {0, 0, 1}, {0, 0, 0}) == Status::TIMEOUT, "total timeout");
    require(plan.start({0, 0, 0}, 0, 0, settings), "restart");
    require(plan.update(1, {0, 0, std::numeric_limits<double>::quiet_NaN()}, {0, 0, 0}) ==
            Status::INVALID_STATE, "invalid measurement rejected");
    settings.velocity = 0;
    require(!plan.start({0, 0, 0}, 0, 0, settings), "zero limit rejected");
    settings = TakeoffPlan::Settings{};
    settings.timeout = 2;
    require(!plan.start({0, 0, 0}, 0, 0, settings), "impossible timeout rejected");
    std::cout << "Takeoff profile, limits, settling, repeat flight and abort checks passed.\n";
  } catch (const std::exception& error) {
    std::cerr << error.what() << '\n';
    return EXIT_FAILURE;
  }
}
