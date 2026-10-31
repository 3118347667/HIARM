#pragma once

#include <Eigen/Core>
#include <algorithm>
#include <cmath>
#include <vector>

namespace planning {
namespace moving_grasp {

constexpr double kPi = 3.14159265358979323846;
inline double angleError(double target, double current) {
  return std::remainder(target - current, 2.0 * kPi);
}

// Both receipt age and measurement age matter: repeated/late packets are not
// fresh measurements. The caller supplies a monotonic receipt age.
inline bool fresh(double stamp, double now, double receipt_age, double timeout) {
  return std::isfinite(stamp) && std::isfinite(now) && stamp > 0.0 &&
         now - stamp >= -0.05 && now - stamp <= timeout &&
         receipt_age >= 0.0 && receipt_age <= timeout;
}

// First-order low-pass of world-frame linear velocity, once per measurement.
// Hz uses the conventional tau = 1/(2*pi*fc), independent of callback rate.
class VelocityFilter {
 public:
  void reset() { *this = VelocityFilter(); }
  bool update(const Eigen::Vector3d& raw, double stamp, double receipt,
              double cutoff_hz, double timeout) {
    if (!raw.allFinite() || !std::isfinite(stamp) || stamp <= 0.0 ||
        !std::isfinite(receipt) || !std::isfinite(cutoff_hz) || cutoff_hz < 0.0 ||
        !std::isfinite(timeout) || timeout <= 0.0) return false;
    const double dt = stamp - stamp_;
    const double receipt_dt = receipt - receipt_;
    // Replays/rollback must not advance either the filter or freshness clock.
    if (initialized_ && (dt <= 0.0 || receipt_dt < 0.0)) return false;
    if (!initialized_ || cutoff_hz == 0.0 || dt > timeout || receipt_dt > timeout) {
      // Seed from the measurement, not zero; stale history must not mask motion.
      value_ = raw;
    } else {
      const double alpha = -std::expm1(-2.0 * kPi * cutoff_hz * dt);
      value_ += alpha * (raw - value_);
    }
    stamp_ = stamp;
    receipt_ = receipt;
    initialized_ = true;
    return true;
  }
  const Eigen::Vector3d& value() const { return value_; }
 private:
  Eigen::Vector3d value_ = Eigen::Vector3d::Zero();
  double stamp_ = 0.0, receipt_ = 0.0;
  bool initialized_ = false;
};

enum class Phase { APPROACH, CLOSING, LIFTING, BRAKING, HOLD, ABORTED };
inline const char* phaseName(Phase phase) {
  switch (phase) {
    case Phase::APPROACH: return "approach";
    case Phase::CLOSING: return "closing_assumed";
    case Phase::LIFTING: return "lifting_assumed";
    case Phase::BRAKING: return "braking";
    case Phase::HOLD: return "hold_assumed";
    case Phase::ABORTED: return "aborted";
  }
  return "unknown";
}

// Time is monotonic seconds. There is intentionally no actuator-feedback gate:
// closing is assumed complete after close_duration, not verified as a grasp.
class Sequence {
 public:
  void reset() { *this = Sequence(); }
  Phase phase() const { return phase_; }
  bool ready(double now, bool gates, double dwell) {
    if (!gates) { ready_since_ = -1.0; return false; }
    if (ready_since_ < 0.0) ready_since_ = now;
    return now - ready_since_ >= dwell;
  }
  void close(double now) {
    if (phase_ != Phase::APPROACH) return;
    phase_ = Phase::CLOSING;
    close_start_ = now;
    close_sent_ = true;
  }
  bool update(double now, double close_duration, double lift_duration) {
    if (phase_ == Phase::CLOSING && now - close_start_ >= close_duration) {
      phase_ = Phase::LIFTING;
      // Start the lift only after a fresh-data control tick has checked timing.
      lift_start_ = now;
    }
    return phase_ == Phase::LIFTING && now - lift_start_ >= lift_duration;
  }
  double liftElapsed(double now) const {
    return phase_ == Phase::LIFTING ? std::max(0.0, now - lift_start_) : 0.0;
  }
  bool closeSent() const { return close_sent_; }
  void brake(bool abort) { phase_ = Phase::BRAKING; abort_ = abort; }
  void stopped() { phase_ = abort_ ? Phase::ABORTED : Phase::HOLD; }
 private:
  Phase phase_ = Phase::APPROACH;
  double ready_since_ = -1.0, close_start_ = 0.0, lift_start_ = 0.0;
  bool close_sent_ = false, abort_ = false;
};

struct State {
  Eigen::Vector3d p = Eigen::Vector3d::Zero();
  Eigen::Vector3d v = Eigen::Vector3d::Zero();
  Eigen::Vector3d a = Eigen::Vector3d::Zero();
};

// A missing or coincident target has no meaningful horizontal bearing.
inline double initialApproachYaw(const Eigen::Vector3d& aircraft,
                                 const Eigen::Vector3d& target, double fallback_yaw) {
  const Eigen::Vector2d delta = (target-aircraft).head<2>();
  return delta.allFinite() && delta.norm() > 1e-3 ? std::atan2(delta.y(), delta.x())
                                                 : fallback_yaw;
}

inline Eigen::Vector3d pregraspPoint(const Eigen::Vector3d& target, double direction,
                                     double distance) {
  return target - distance * Eigen::Vector3d(std::cos(direction), std::sin(direction), 0.0);
}
using Coefficients = Eigen::Matrix<double, 3, 6>;
inline Coefficients quintic(const State& from, const State& to, double t) {
  const Eigen::Vector3d dp = to.p - from.p - from.v*t - from.a*t*t/2.0;
  const Eigen::Vector3d dv = to.v - from.v - from.a*t;
  const Eigen::Vector3d da = to.a - from.a;
  Coefficients c;
  c.col(5) = from.p;
  c.col(4) = from.v;
  c.col(3) = from.a/2.0;
  c.col(2) = 10.0*dp/std::pow(t,3) - 4.0*dv/(t*t) + da/(2.0*t);
  c.col(1) = -15.0*dp/std::pow(t,4) + 7.0*dv/std::pow(t,3) - da/(t*t);
  c.col(0) = 6.0*dp/std::pow(t,5) - 3.0*dv/std::pow(t,4) + da/(2.0*std::pow(t,3));
  return c;
}

// Offset, vertical velocity and acceleration, with zero derivatives at both ends.
inline Eigen::Vector3d lift(double elapsed, double duration, double height) {
  const double s = std::clamp(elapsed/duration, 0.0, 1.0);
  return {height*(10*std::pow(s,3)-15*std::pow(s,4)+6*std::pow(s,5)),
          height*30*s*s*(1-s)*(1-s)/duration,
          height*60*s*(1-s)*(1-2*s)/(duration*duration)};
}

class Direction {
 public:
  void reset(double yaw) {
    yaw_ = yaw;
    confirmed_direction_ = false;
    resetEvidence();
  }
  // Preserve the last reliable yaw while requiring fresh confirmation.
  void resetEvidence() {
    moving_ = false;
    rate_ = 0.0;
    filtered_ = Eigen::Vector2d::Zero();
    last_sample_ = -1.0;
    candidate_since_ = -1.0;
    candidate_tier_ = 0;
    stationary_since_ = -1.0;
    stationary_confirmed_ = false;
    direction_consistent_ = false;
    rate_sample_valid_ = false;
  }
  // Called once per new velocity measurement, never once per planning tick.
  // Filter the vector rather than its angle so wraparound and opposite noisy
  // directions do not produce artificial 180-degree yaw commands.
  void update(double vx, double vy, double stop_speed,
              double start_speed, double offset, double sample_time,
              double cutoff_hz, double start_duration, double max_rate) {
    const Eigen::Vector2d raw(vx, vy);
    if (!raw.allFinite() || !std::isfinite(sample_time)) return;
    if (last_sample_ >= 0.0) {
      const double gap = sample_time - last_sample_;
      if (!std::isfinite(gap) || gap <= 0.0) return;
      // A gap invalidates motion readiness even if a new packet is fresh.
      if (gap > 0.20) resetEvidence();
    }
    const Eigen::Vector2d previous_filtered = filtered_;
    const bool previously_reliable = rate_sample_valid_;
    const double dt = last_sample_ < 0.0 ? 0.0 : sample_time - last_sample_;
    double alpha = 0.0;
    if (last_sample_ < 0.0) {
      // Do not seed the filter at zero: that would briefly classify a moving
      // car as stationary before the filter has had time to converge.
      filtered_ = raw;
    } else {
      if (!std::isfinite(dt) || dt <= 0.0) return;
      alpha = -std::expm1(-2.0 * kPi * cutoff_hz * dt);
      filtered_ += alpha * (raw - filtered_);
    }
    last_sample_ = sample_time;
    const double speed = filtered_.norm();
    const double raw_speed = raw.norm();
    // Ignore direction when the noisy raw vector is tiny or disagrees sharply
    // with its filtered direction, e.g. immediately after an abrupt reversal.
    constexpr double kCos45 = 0.7071067811865476;
    direction_consistent_ = raw_speed >= stop_speed && speed >= stop_speed &&
        raw.dot(filtered_) >= kCos45 * raw_speed * speed;
    if (speed < stop_speed) {
      moving_ = false;
      candidate_since_ = -1.0;
      candidate_tier_ = 0;
    } else if (!moving_) {
      // Slower motion needs twice as much evidence, because its direction is
      // closer to the noise floor. Higher speed keeps the original dwell.
      if (direction_consistent_) {
        const int tier = speed >= start_speed ? 2 : 1;
        if (candidate_tier_ != tier) {
          candidate_tier_ = tier;
          candidate_since_ = sample_time;
        }
        const double dwell = tier == 2 ? start_duration : 2.0 * start_duration;
        if (sample_time - candidate_since_ >= dwell) moving_ = true;
      } else {
        candidate_tier_ = 0;
        candidate_since_ = -1.0;
      }
    }
    if (!moving_ && raw_speed < stop_speed && speed < stop_speed) {
      if (stationary_since_ < 0.0) stationary_since_ = sample_time;
      stationary_confirmed_ = sample_time - stationary_since_ >= start_duration;
    } else {
      stationary_since_ = -1.0;
      stationary_confirmed_ = false;
    }
    // A single near-zero raw sample must not change yaw, even if the filtered
    // vector is still decaying from earlier motion.
    if (moving_ && direction_consistent_) {
      confirmed_direction_ = true;
      yaw_ += angleError(std::atan2(filtered_.y(), filtered_.x()) + offset, yaw_);
      // Feedforward is the angular rate of the filtered velocity vector, not
      // the car body's yaw rate. Disable it near the velocity noise floor or
      // across gaps/direction reversals; never differentiate an unlocked yaw.
      if (previously_reliable && dt > 0.0 && dt <= 0.20 &&
          previous_filtered.norm() >= start_speed && speed >= start_speed &&
          raw_speed >= start_speed) {
        const double delta = angleError(std::atan2(filtered_.y(), filtered_.x()),
                                        std::atan2(previous_filtered.y(), previous_filtered.x()));
        const double measured = std::clamp(delta / dt, -max_rate, max_rate);
        rate_ = std::clamp(rate_ + alpha * (measured - rate_), -max_rate, max_rate);
      } else rate_ = 0.0;
    } else rate_ = 0.0;
    rate_sample_valid_ = moving_ && direction_consistent_ &&
                         speed >= start_speed && raw_speed >= start_speed;
  }
  double yaw() const { return yaw_; }
  double rate() const { return rate_; }
  bool hasConfirmedDirection() const { return confirmed_direction_; }
  bool readyForGrasp(double raw_speed, double stop_speed, double start_speed) const {
    (void)start_speed;
    // A moving car must have a confirmed direction before it can be grasped.
    // A stationary car may use the last reliable (or initial) heading.
    return (moving_ && raw_speed >= stop_speed && direction_consistent_) ||
           (stationary_confirmed_ && raw_speed < stop_speed);
  }
 private:
  double yaw_ = 0.0, rate_ = 0.0;
  bool moving_ = false;
  Eigen::Vector2d filtered_ = Eigen::Vector2d::Zero();
  double last_sample_ = -1.0, candidate_since_ = -1.0;
  double stationary_since_ = -1.0;
  int candidate_tier_ = 0;
  bool stationary_confirmed_ = false, direction_consistent_ = false;
  bool rate_sample_valid_ = false;
  bool confirmed_direction_ = false;
};

struct YawPiece {
  double duration;
  // Descending powers, matching flygripper_msgs/Polynomial order=3.
  Eigen::Vector4d c;
  double pos(double t) const { return ((c[0]*t+c[1])*t+c[2])*t+c[3]; }
  double vel(double t) const { return (3*c[0]*t+2*c[1])*t+c[2]; }
  double acc(double t) const { return 6*c[0]*t+2*c[1]; }
};
inline bool yawWithinLimits(const YawPiece& p, double max_rate, double max_acc) {
  double rate = std::max(std::abs(p.vel(0)), std::abs(p.vel(p.duration)));
  if (std::abs(p.c[0]) > 1e-12) {
    const double vertex = -p.c[1]/(3*p.c[0]);
    if (vertex > 0 && vertex < p.duration) rate = std::max(rate, std::abs(p.vel(vertex)));
  }
  return rate <= max_rate+1e-8 &&
         std::max(std::abs(p.acc(0)), std::abs(p.acc(p.duration))) <= max_acc+1e-8;
}
inline std::vector<YawPiece> heading(double yaw, double rate, double target_now,
                                     double target_rate, double duration,
                                     double max_rate, double max_acc) {
  const double end = yaw + angleError(target_now, yaw) + target_rate*duration;
  const double dv = target_rate-rate, dp = end-yaw-rate*duration;
  YawPiece cubic{duration, {dv/(duration*duration)-2*dp/std::pow(duration,3),
                            3*dp/(duration*duration)-dv/duration, rate, yaw}};
  if (yawWithinLimits(cubic, max_rate, max_acc)) return {cubic};
  // A large direction change may be infeasible within this position horizon.
  // Continue rotating under limits; the actual-yaw gate prevents early closing.
  std::vector<YawPiece> result;
  for (double t = 0; t < duration-1e-9;) {
    const double dt = std::min(0.02, duration-t);
    const double error = angleError(target_now + target_rate*t, yaw);
    const double correction = std::copysign(std::min(2.0*std::abs(error),
                                         std::sqrt(2*max_acc*std::abs(error))), error);
    const double wanted = std::clamp(target_rate+correction, -max_rate, max_rate);
    const double a = std::clamp((wanted-rate)/dt, -max_acc, max_acc);
    YawPiece piece{dt, {0.0, a/2.0, rate, yaw}};
    yaw = piece.pos(dt); rate = piece.vel(dt);
    result.push_back(piece); t += dt;
  }
  return result;
}
inline Eigen::Vector2d sampleHeading(const std::vector<YawPiece>& pieces, double t) {
  for (const auto& p : pieces) {
    if (t <= p.duration) return {p.pos(std::max(0.0,t)), p.vel(std::max(0.0,t))};
    t -= p.duration;
  }
  if (pieces.empty()) return Eigen::Vector2d::Zero();
  const auto& p = pieces.back();
  return {p.pos(p.duration), p.vel(p.duration)};
}

}  // namespace moving_grasp
}  // namespace planning
