#pragma once

#include <algorithm>
#include <array>
#include <cmath>

namespace flygripper_controller {

struct SafeThrustAcceleration {
  std::array<double, 3> value;
  bool fault;
};

inline double uprightTiltLimit(double configured_limit) {
  constexpr double kAlmostHorizontal = 1.5533430342749532;  // 89 degrees
  return configured_limit > 0.0
      ? std::min(configured_limit, kAlmostHorizontal) : kAlmostHorizontal;
}

// Also apply this contract after aligning a target with the FCU frame; frame
// alignment can add roll/pitch to a target that was bounded in the odom frame.
inline bool uprightTiltWithinLimit(double body_z_vertical, double tilt_limit) {
  return std::isfinite(body_z_vertical) && std::isfinite(tilt_limit) &&
      body_z_vertical > 0.0 &&
      body_z_vertical >= std::cos(uprightTiltLimit(tilt_limit)) - 1e-12;
}

// A conventional multirotor can reduce upward thrust, but must never turn a
// downward force request into an inverted attitude. Keep a positive vertical
// component and bound the horizontal component directly (no rotation-axis
// normalization or antiparallel-vector singularity).
inline SafeThrustAcceleration limitUpwardThrust(
    std::array<double, 3> acceleration, double minimum_vertical,
    double tilt_limit, double hover_acceleration) {
  const SafeThrustAcceleration fallback{{0.0, 0.0, hover_acceleration}, true};
  if (!std::isfinite(minimum_vertical) || minimum_vertical < 0.0 ||
      !std::isfinite(tilt_limit) || !std::isfinite(acceleration[0]) ||
      !std::isfinite(acceleration[1]) || !std::isfinite(acceleration[2]) ||
      acceleration[2] <= 0.0)
    return fallback;

  // Historically <= 0 disabled the tilt limit, and simulation allows > pi/2.
  // Both still require an upward thrust direction.
  const double angle = uprightTiltLimit(tilt_limit);
  acceleration[2] = std::max(acceleration[2], std::max(minimum_vertical, 1e-3));
  const double horizontal = std::hypot(acceleration[0], acceleration[1]);
  const double maximum_horizontal = acceleration[2] * std::tan(angle);
  if (horizontal > maximum_horizontal) {
    const double scale = maximum_horizontal / horizontal;
    acceleration[0] *= scale;
    acceleration[1] *= scale;
  }
  return {acceleration, false};
}

}  // namespace flygripper_controller
