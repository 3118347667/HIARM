#include "flygripper_controller/thrust_safety.hpp"

#include <iostream>
#include <limits>
#include <stdexcept>

using flygripper_controller::limitUpwardThrust;
using flygripper_controller::uprightTiltWithinLimit;

void require(bool condition, const char* message) {
  if (!condition) throw std::runtime_error(message);
}

int main() {
  try {
    constexpr double tilt = 0.5235987755982988;
    constexpr double gravity = 9.7936;
    require(uprightTiltWithinLimit(std::cos(tilt), tilt),
            "aligned FCU target at configured tilt limit is valid");
    require(uprightTiltWithinLimit(1.0, tilt), "upright hover target is valid");
    require(!uprightTiltWithinLimit(std::cos(tilt + 0.1), tilt),
            "frame alignment must not exceed the configured tilt limit");
    for (double z : {-1.0, -0.01, 0.0}) {
      require(!uprightTiltWithinLimit(z, tilt),
              "horizontal or inverted FCU target must be rejected");
      require(!uprightTiltWithinLimit(z, 0.0),
              "disabled configured tilt limit must still prevent inversion");
    }
    require(!uprightTiltWithinLimit(std::numeric_limits<double>::quiet_NaN(), tilt),
            "non-finite FCU tilt must be rejected");
    require(!uprightTiltWithinLimit(1.0, std::numeric_limits<double>::infinity()),
            "non-finite tilt configuration must be rejected");
    // Regression: 2026-10-26 17:15:04.098, compensated vertical acceleration
    // crossed zero and the previous limiter generated an inverted attitude.
    for (double z : {-0.11850724377037869, -4.15, -9.81, 0.0}) {
      const auto result = limitUpwardThrust({0.0, -1.02, z}, 1.9204 / 0.62, tilt, gravity);
      require(result.fault, "non-upward request must signal takeover");
      require(result.value == std::array<double, 3>{0.0, 0.0, gravity},
              "fault must give upright hover, not minimum thrust or inversion");
    }
    for (double bad : {std::numeric_limits<double>::quiet_NaN(),
                       std::numeric_limits<double>::infinity()}) {
      for (int axis = 0; axis < 3; ++axis) {
        std::array<double, 3> value{0.0, 0.0, gravity};
        value[axis] = bad;
        require(limitUpwardThrust(value, 3.0, tilt, gravity).fault,
                "non-finite acceleration must signal takeover");
      }
    }
    // Check the geometric contract across quadrants and near zero thrust.
    for (double x : {-100.0, -1.0, 0.0, 1.0, 100.0})
      for (double y : {-100.0, -1.0, 0.0, 1.0, 100.0})
        for (double z : {1e-12, 1.0, gravity, 100.0}) {
          const auto result = limitUpwardThrust({x, y, z}, 3.0, tilt, gravity);
          const auto& a = result.value;
          require(!result.fault, "feasible upward requests use saturation");
          require(a[2] >= 3.0, "positive vertical reserve");
          require(std::atan2(std::hypot(a[0], a[1]), a[2]) <= tilt + 1e-12,
                  "actual output must satisfy tilt limit");
          require(a[0] * x >= 0.0 && a[1] * y >= 0.0,
                  "horizontal direction must not reverse");
        }
    const auto nominal = limitUpwardThrust({0.3, -0.4, gravity}, 3.0, tilt, gravity);
    require(nominal.value == std::array<double, 3>{0.3, -0.4, gravity},
            "nominal hover command is unchanged");
    for (double unrestricted : {0.0, 3.14}) {
      const auto result = limitUpwardThrust({100.0, 100.0, 0.001}, 0.0,
                                            unrestricted, gravity);
      require(result.value[2] > 0.0, "disabled tilt setting must not allow inversion");
    }
    std::cout << "thrust safety regression passed\n";
  } catch (const std::exception& error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
