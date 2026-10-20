#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>

namespace flygripper_flight {

// A per-flight relative-height reference, independent of ROS and wall clocks.
class TakeoffPlan {
 public:
  struct Settings {
    double height = 1.0;
    double velocity = 0.25;
    double acceleration = 0.3;
    double jerk = 1.0;
    double position_tolerance = 0.08;
    double velocity_tolerance = 0.1;
    double settle_time = 1.0;
    double liftoff_timeout = 6.0;
    double timeout = 20.0;
    double max_tracking_error = 0.5;
  };
  struct Sample {
    std::array<double, 3> position;
    double yaw;
    double velocity = 0.0;
    double acceleration = 0.0;
    double jerk = 0.0;
    double snap = 0.0;
  };
  enum class Status { RUNNING, COMPLETE, NO_LIFTOFF, TIMEOUT, TRACKING_ERROR,
                      INVALID_STATE };
  static const char* statusName(Status status) {
    switch (status) {
      case Status::RUNNING: return "running";
      case Status::COMPLETE: return "complete";
      case Status::NO_LIFTOFF: return "no measured liftoff";
      case Status::TIMEOUT: return "takeoff timeout";
      case Status::TRACKING_ERROR: return "excessive position error";
      default: return "invalid state or clock rollback";
    }
  }

  bool start(const std::array<double, 3>& position, double yaw, double now,
             const Settings& settings) {
    active_ = false;
    const double positive[] = {
        settings.height, settings.velocity, settings.acceleration, settings.jerk,
        settings.position_tolerance, settings.velocity_tolerance,
        settings.settle_time, settings.liftoff_timeout, settings.timeout,
        settings.max_tracking_error};
    for (double value : positive)
      if (!std::isfinite(value) || value <= 0.0) return false;
    for (double value : position)
      if (!std::isfinite(value)) return false;
    if (!std::isfinite(yaw) || !std::isfinite(now)) return false;

    // Extrema of s(u)=35u^4-84u^5+70u^6-20u^7 on [0,1].
    const double duration = std::max({
        2.1875 * settings.height / settings.velocity,
        std::sqrt(7.514 * settings.height / settings.acceleration),
        std::cbrt(52.5 * settings.height / settings.jerk)});
    if (!std::isfinite(duration) ||
        settings.timeout <= duration + settings.settle_time ||
        settings.liftoff_timeout >= settings.timeout) return false;

    settings_ = settings;
    origin_ = position;
    yaw_ = yaw;
    start_time_ = last_time_ = now;
    duration_ = duration;
    settled_since_ = std::numeric_limits<double>::quiet_NaN();
    lifted_ = false;
    active_ = true;
    return true;
  }

  Sample sample(double now) const {
    Sample result{origin_, yaw_};
    const double u = std::clamp((now - start_time_) / duration_, 0.0, 1.0);
    const double u2 = u * u, u3 = u2 * u, u4 = u3 * u;
    const double s = u4 * (35.0 + u * (-84.0 + u * (70.0 - 20.0 * u)));
    result.position[2] += settings_.height * s;
    if (now <= start_time_ || now >= start_time_ + duration_) return result;
    const double scale = settings_.height / duration_;
    result.velocity = scale * 140.0 * u3 * std::pow(1.0 - u, 3);
    result.acceleration = scale / duration_ *
        (420.0 * u2 - 1680.0 * u3 + 2100.0 * u4 - 840.0 * u4 * u);
    result.jerk = scale / (duration_ * duration_) *
        (840.0 * u - 5040.0 * u2 + 8400.0 * u3 - 4200.0 * u4);
    result.snap = scale / std::pow(duration_, 3) *
        (840.0 - 10080.0 * u + 25200.0 * u2 - 16800.0 * u3);
    return result;
  }

  Status update(double now, const std::array<double, 3>& position,
                const std::array<double, 3>& velocity) {
    if (!active_ || !std::isfinite(now) || now < last_time_)
      return Status::INVALID_STATE;
    for (double value : position)
      if (!std::isfinite(value)) return Status::INVALID_STATE;
    for (double value : velocity)
      if (!std::isfinite(value)) return Status::INVALID_STATE;
    last_time_ = now;
    const double elapsed = now - start_time_;
    if (elapsed > settings_.timeout) return Status::TIMEOUT;
    lifted_ = lifted_ || position[2] - origin_[2] > std::min(0.1, settings_.height * 0.5);
    if (!lifted_ && elapsed > settings_.liftoff_timeout)
      return Status::NO_LIFTOFF;
    const Sample reference = sample(now);
    double error2 = 0.0, speed2 = 0.0;
    for (int i = 0; i < 3; ++i) {
      error2 += std::pow(position[i] - reference.position[i], 2);
      speed2 += velocity[i] * velocity[i];
    }
    if (std::sqrt(error2) > settings_.max_tracking_error)
      return Status::TRACKING_ERROR;
    if (elapsed >= duration_ &&
        std::sqrt(error2) <= settings_.position_tolerance &&
        std::sqrt(speed2) <= settings_.velocity_tolerance) {
      if (!std::isfinite(settled_since_)) settled_since_ = now;
      if (now - settled_since_ >= settings_.settle_time) return Status::COMPLETE;
    } else {
      settled_since_ = std::numeric_limits<double>::quiet_NaN();
    }
    return Status::RUNNING;
  }

  void reset() { active_ = false; }
  double duration() const { return duration_; }

 private:
  Settings settings_;
  std::array<double, 3> origin_{};
  double yaw_ = 0.0, start_time_ = 0.0, last_time_ = 0.0, duration_ = 1.0;
  double settled_since_ = std::numeric_limits<double>::quiet_NaN();
  bool active_ = false, lifted_ = false;
};
}  // namespace flygripper_flight
