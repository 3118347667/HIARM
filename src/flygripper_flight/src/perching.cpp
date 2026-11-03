#include "flygripper_flight/fsm.hpp"
#include "flygripper_controller/thrust_safety.hpp"

namespace flygripper_flight {
namespace {
using Command = flygripper_msgs::PerchingCommand;
using Status = flygripper_msgs::PerchingStatus;

bool validSettings(const Command& c) {
  const auto positive = [](double v) { return std::isfinite(v) && v > 0.0; };
  return c.mission_id != 0 && std::isfinite(c.closed_position) &&
      std::isfinite(c.open_position) && c.closed_position != c.open_position &&
      positive(c.closed_tolerance) && positive(c.open_tolerance) &&
      c.closed_tolerance + c.open_tolerance < std::abs(c.closed_position - c.open_position) &&
      positive(c.motors_off_duration) && c.motors_off_duration <= 30.0 &&
      positive(c.spinup_duration) && c.spinup_duration >= 0.5 &&
      positive(c.recovery_duration) && positive(c.retreat_distance) &&
      c.retreat_distance <= 1.0 && positive(c.retreat_duration) &&
      positive(c.settle_duration) && positive(c.max_settle_speed) &&
      c.max_settle_speed <= 0.3 && positive(c.position_tolerance) &&
      positive(c.gripper_open_duration) && positive(c.unload_duration) &&
      positive(c.support_verify_duration) && positive(c.final_unload_duration) &&
      positive(c.test_throttle_ratio) && c.test_throttle_ratio <= 0.2 &&
      positive(c.max_height_drop) && c.max_height_drop <= 0.05 &&
      positive(c.max_downward_speed) && c.max_downward_speed <= 0.2 &&
      std::isfinite(c.min_vertical_acceleration) && c.min_vertical_acceleration < 0.0 &&
      positive(c.sensor_max_age) && c.sensor_max_age <= 0.1 &&
      positive(c.timeout) && c.timeout > c.motors_off_duration +
          c.spinup_duration + c.retreat_duration + c.settle_duration +
          c.unload_duration + c.support_verify_duration + c.final_unload_duration +
          c.gripper_open_duration;
}
}  // namespace

void Fsm::rosPerchingCallback(const ros::MessageEvent<const Command>& event) {
  perching_input_.receive(event);
}

bool Fsm::perchingActive() const {
  return (perching_status_.phase >= Status::MOTORS_OFF &&
          perching_status_.phase <= Status::RETREAT) ||
         (perching_status_.phase >= Status::UNLOADING &&
          perching_status_.phase <= Status::FINAL_UNLOAD);
}

bool Fsm::perchingActuatorFresh() const {
  return actuator_state_.valid() && actuator_state_.stamp > 0.0 &&
      actuator_state_.stamp <= now_ + fsm_param_->sensor_future_tolerance &&
      now_ - actuator_state_.stamp <= fsm_param_->actuator_state_timeout &&
      applied_.actuator.message &&
      steadySeconds() - applied_.actuator.received_steady <= fsm_param_->actuator_state_timeout;
}

void Fsm::clearPerchingTrajectory() {
  trajectory_ = Trajectory();
  active_trajectory_message_.reset();
  pending_trajectory_message_.reset();
  trajectory_message_decoded_ = false;
  awaiting_first_trajectory_ = false;
  setReferenceType(ReferenceType::TYPE_HOVER);
}

void Fsm::consumePerchingCommand() {
  const auto input = perching_input_.trySnapshot();
  if (!input || !input->message || input->count == perching_input_count_) return;
  perching_input_count_ = input->count;
  const auto& c = *input->message;
  // Never execute a delayed START after an input fault or a new flight session.
  if (c.header.stamp.isZero() || c.header.stamp.toSec() > now_ + 0.05 ||
      now_ - c.header.stamp.toSec() > 0.5 ||
      steadySeconds() - input->received_steady > 0.5) return;
  if (c.command == Command::CANCEL && c.mission_id == perching_status_.mission_id) {
    if (perchingActive()) beginHandover("Planner cancelled perching");
    else abortPerching("Planner cancelled perching");
    return;
  }
  if (c.command == Command::START) {
    if (c.mission_id == perching_status_.mission_id &&
        perching_status_.phase == Status::PREPARED)
      perching_start_requested_ = true;
    return;  // Retransmission cannot restart the motor-off timer.
  }
  if (c.command != Command::PREPARE || perchingActive()) return;
  if (c.mission_id == perching_status_.mission_id &&
      perching_status_.phase == Status::PREPARED) return;
  if (c.mission_id <= perching_last_mission_) return;
  if (perching_status_.phase == Status::PREPARED) return;
  if (!validSettings(c)) {
    perching_last_mission_ = c.mission_id;
    perching_status_.mission_id = c.mission_id;
    perching_status_.phase = Status::ABORTED;
    perching_status_.detail = "Invalid perching settings";
    return;
  }
  // Only a verified attitude/thrust GUIDED session can deliberately stop motors.
  // PREPARE is retried while the approach staging point is still settling.
  if (fsm_state_ != FsmState::STATE_TRAJ || !guided_entry_verified_ ||
      robot_command_.rotor_mode != RobotCommandMode::MODE_ATTITUDE ||
      !rc_input_.isTrajMode() || !fc_state_.armed || fc_state_.mode != "GUIDED_NOGPS" ||
      !std::isfinite(last_powered_throttle_) || last_powered_throttle_ <= 0.0 ||
      last_powered_throttle_ > 1.0 || steadySeconds() - last_powered_throttle_at_ > 0.2 ||
      robot_state_.vel.norm() > c.max_settle_speed) return;
  perching_config_ = c;
  perching_last_mission_ = c.mission_id;
  perching_status_.mission_id = c.mission_id;
  perching_status_.phase = Status::PREPARED;
  perching_status_.saved_throttle = last_powered_throttle_;
  perching_status_.detail = "Powered-flight throttle recorded before grasp approach";
  perching_prepared_at_ = steadySeconds();
  perching_start_requested_ = false;
  perching_settled_at_ = -1.0;
  perching_recovery_at_ = -1.0;
  perching_rescue_until_ = -1.0;
  perching_emergency_until_ = -1.0;
  ROS_INFO("Perching prepared: saved throttle %.4f", perching_status_.saved_throttle);
}

void Fsm::abortPerching(const std::string& reason) {
  if (!perchingActive() && perching_status_.phase != Status::PREPARED) return;
  const bool aborting_approach = perching_status_.phase == Status::PREPARED &&
      fsm_state_ == FsmState::STATE_TRAJ;
  perching_status_.phase = Status::ABORTED;
  perching_status_.detail = reason;
  perching_start_requested_ = false;
  perching_recovery_at_ = -1.0;
  if (aborting_approach) {
    // Cancelling the grasp must also revoke its still-running approach and any
    // queued successor. Hold the measured pose, not a future trajectory point.
    clearPerchingTrajectory();
    sampler_->getHoverReference(robot_state_, references_, now_, robot_command_.rotor_mode, true);
    rc_input_.resetTrajModeTrigger();
    setFsmState(FsmState::STATE_GUIDED);
  }
  ROS_ERROR_STREAM("Perching aborted: " << reason << "; gripper is not automatically opened");
}

void Fsm::publishPerchingThrust(double throttle) {
  // Keep sending attitude targets: silence would invoke the FCU's Guided timeout.
  // GUID_OPTIONS bit 3 was verified when this autonomous flight session began.
  robot_command_.stamp = now_;
  robot_command_.collective_thrust_command = throttle;
  robot_command_.collective_thrust = robot_param_->mass * kGravity *
      throttle / robot_param_->hover_percentage;
  robot_command_.attitude_command = Quaternion(Eigen::AngleAxisd(perching_fcu_yaw_, Vector<3>::UnitZ()));
  toRosMsg(robot_command_, thrust_att_command_msg_);
  thrust_att_command_pub_.publish(thrust_att_command_msg_);
  perching_status_.output_throttle = throttle;
  // Deliberately do not run the controller/L1 estimator against support forces.
  ext_wrench_est_->reset();
}

void Fsm::rejectPerchingSupport(const std::string& reason) {
  // A failed support test is a cancelled perch, not a completed motor-off dwell.
  // Restore the recorded throttle in this same control cycle, keep the gripper
  // command unchanged, and hold the pre-test altitude/heading under feedback.
  publishPerchingThrust(perching_status_.saved_throttle);
  abortPerching(reason);
  perching_rescue_until_ = steadySeconds() + perching_config_.recovery_duration;
  clearPerchingTrajectory();
  RobotState hold = robot_state_.getHoverState();
  hold.pos = perching_position_;
  hold.setQ(perching_yaw_);
  sampler_->getHoverReference(hold, references_, now_, robot_command_.rotor_mode, true);
  // Invalidate the trajectory session: a new CH6 edge must pass the normal
  // first-trajectory continuity checks before any planner can take control.
  rc_input_.resetTrajModeTrigger();
  setFsmState(FsmState::STATE_GUIDED);
  ROS_ERROR_STREAM("[PERCHING] Support NOT confirmed: " << reason
                   << ". Restored powered hover; motor-stop/reflight cancelled.");
}

bool Fsm::publishPerchingEmergencyRecovery() {
  if (steadySeconds() >= perching_emergency_until_ || !guided_entry_verified_ ||
      !fc_state_.armed || fc_state_.mode != "GUIDED_NOGPS" ||
      robot_command_.rotor_mode != RobotCommandMode::MODE_ATTITUDE ||
      !std::isfinite(perching_status_.saved_throttle) ||
      perching_status_.saved_throttle <= 0.0 || perching_status_.saved_throttle > 1.0)
    return false;
  // No stale pose/IMU feedback is fed into a controller here. The cached level
  // attitude and last powered-flight throttle only bridge the takeover request.
  publishPerchingThrust(perching_status_.saved_throttle);
  return true;
}

bool Fsm::updatePerching() {
  const double time = steadySeconds();
  if (perching_status_.phase == Status::PREPARED) {
    if (fsm_state_ != FsmState::STATE_TRAJ || rc_traj_release_seen_ || !rc_input_.isTrajMode() ||
        time - perching_prepared_at_ > perching_config_.timeout) {
      abortPerching("Prepared mission expired or trajectory mode exited");
      return false;
    }
    if (!perching_start_requested_) return false;
    // START follows either verified feedback or the planner's explicitly
    // configured command wait. Neither authorizes cutoff: unloading must pass.
    const bool closed = !perching_config_.use_actuator_feedback ||
        (perchingActuatorFresh() && (actuator_state_.overload ||
         std::abs(actuator_state_.position - perching_config_.closed_position) <
             perching_config_.closed_tolerance));
    const bool settled = closed && robot_state_.vel.norm() <= perching_config_.max_settle_speed &&
        robot_state_.omega.norm() <= 0.3;
    if (!settled) perching_settled_at_ = -1.0;
    else if (perching_settled_at_ < 0.0) perching_settled_at_ = time;
    if (perching_settled_at_ < 0.0 ||
        time - perching_settled_at_ < perching_config_.settle_duration) return false;
    perching_position_ = robot_state_.pos;
    perching_yaw_ = robot_state_.getYaw();
    perching_fcu_yaw_ = std::atan2((imu_.att * Vector<3>::UnitX()).y(),
                                 (imu_.att * Vector<3>::UnitX()).x());
    perching_started_at_ = perching_phase_at_ = time;
    perching_status_.phase = Status::UNLOADING;
    perching_status_.detail = "Gripper command settled; progressively testing support";
    clearPerchingTrajectory();
  }
  if (!perchingActive()) return false;
  if (fsm_state_ != FsmState::STATE_TRAJ || rc_traj_release_seen_ || !rc_input_.isTrajMode() ||
      (perching_config_.use_actuator_feedback && !perchingActuatorFresh()) ||
      time - perching_started_at_ > perching_config_.timeout) {
    beginHandover("Perching lost RC trajectory mode/actuator feedback or timed out");
    return false;
  }
  if (!flygripper_controller::uprightTiltWithinLimit(
          (imu_.att * Vector<3>::UnitZ()).z(), robot_param_->tilt_angle_max) ||
      !flygripper_controller::uprightTiltWithinLimit(
          (odom_.att * Vector<3>::UnitZ()).z(), robot_param_->tilt_angle_max)) {
    beginHandover("Perching attitude exceeds the flight tilt limit");
    return false;
  }
  clearPerchingTrajectory();  // Ignore any other planner's cached or new trajectory.
  const auto phase = perching_status_.phase;
  const bool testing_support = phase == Status::UNLOADING || phase == Status::SUPPORT_VERIFY ||
                               phase == Status::FINAL_UNLOAD || phase == Status::MOTORS_OFF;
  if (testing_support) {
    const double sensor_age = std::max(now_ - imu_.stamp, now_ - odom_.stamp);
    const double callback_age = std::max(imu_callback_gap_, odom_callback_gap_);
    std::string reason;
    if (sensor_age > perching_config_.sensor_max_age ||
        callback_age > perching_config_.sensor_max_age)
      reason = "IMU/EKF timing exceeds support-test limit";
    else if (perching_position_.z() - robot_state_.pos.z() > perching_config_.max_height_drop)
      reason = "Height dropped during unloading/perching";
    else if (robot_state_.vel.z() < -perching_config_.max_downward_speed)
      reason = "Downward velocity detected during unloading/perching";
    else if (robot_state_.acc.z() < perching_config_.min_vertical_acceleration)
      reason = "Downward acceleration detected during unloading/perching";
    else if ((robot_state_.pos - perching_position_).norm() > 0.05 ||
             robot_state_.vel.norm() > 0.2)
      reason = "Position/velocity changed during support test";
    if (!reason.empty()) {
      rejectPerchingSupport(reason);
      return true;
    }
  }
  if (testing_support || phase == Status::SPINUP) {
    const bool closed = !perching_config_.use_actuator_feedback || actuator_state_.overload ||
        std::abs(actuator_state_.position - perching_config_.closed_position) < perching_config_.closed_tolerance;
    if (!closed || (robot_state_.pos - perching_position_).norm() > 0.10 ||
        robot_state_.vel.norm() > 0.3 ||
        std::abs(std::remainder(robot_state_.getYaw() - perching_yaw_, 2 * M_PI)) > 0.2) {
      beginHandover("Perching support moved or closed feedback was lost");
      return false;
    }
  }
  const auto smooth = [](double u) { u = std::clamp(u, 0.0, 1.0); return u*u*u*(10 + u*(-15 + 6*u)); };
  if (perching_status_.phase == Status::UNLOADING) {
    const double u = (time - perching_phase_at_) / perching_config_.unload_duration;
    const double ratio = 1.0 - (1.0 - perching_config_.test_throttle_ratio) * smooth(u);
    publishPerchingThrust(perching_status_.saved_throttle * ratio);
    if (u >= 1.0) {
      perching_status_.phase = Status::SUPPORT_VERIFY;
      perching_status_.detail = "Low-throttle support verification; position alone is insufficient at hover thrust";
      perching_phase_at_ = time;
    }
    return true;
  }
  if (perching_status_.phase == Status::SUPPORT_VERIFY) {
    publishPerchingThrust(perching_status_.saved_throttle * perching_config_.test_throttle_ratio);
    if (time - perching_phase_at_ >= perching_config_.support_verify_duration) {
      perching_status_.phase = Status::FINAL_UNLOAD;
      perching_status_.detail = "Low-throttle stability passed; slowly reducing the remaining output";
      perching_phase_at_ = time;
    }
    return true;
  }
  if (perching_status_.phase == Status::FINAL_UNLOAD) {
    const double u = (time - perching_phase_at_) / perching_config_.final_unload_duration;
    publishPerchingThrust(perching_status_.saved_throttle * perching_config_.test_throttle_ratio * (1.0 - smooth(u)));
    if (u >= 1.0) {
      perching_status_.phase = Status::MOTORS_OFF;
      perching_status_.detail = "Software support test passed; zero-thrust dwell (not a load-sensor confirmation)";
      perching_phase_at_ = time;
    }
    return true;
  }
  if (perching_status_.phase == Status::MOTORS_OFF) {
    if (time - perching_phase_at_ < perching_config_.motors_off_duration) {
      publishPerchingThrust(0.0);
      return true;
    }
    perching_status_.phase = Status::SPINUP;
    perching_status_.detail = "Restoring recorded throttle with gripper still closed";
    perching_phase_at_ = time;
  }
  if (perching_status_.phase == Status::SPINUP) {
    publishPerchingThrust(perching_status_.saved_throttle);
    if (time - perching_phase_at_ >= perching_config_.spinup_duration) {
      perching_status_.phase = Status::OPENING;
      perching_status_.detail = "Spinup wait elapsed; commanding gripper open";
      perching_phase_at_ = time;
    }
    return true;
  }
  if (perching_status_.phase == Status::OPENING) {
    // Once opening starts, use altitude/yaw feedback immediately. Retain the
    // recorded-throttle floor until open feedback confirms release; otherwise
    // an opening servo delay would leave a released vehicle on open-loop thrust.
    sampler_->getPoseReference(perching_position_, perching_yaw_, references_, now_,
                              robot_command_.rotor_mode);
    const bool open = perching_config_.use_actuator_feedback
        ? std::abs(actuator_state_.position - perching_config_.open_position) < perching_config_.open_tolerance
        : time - perching_phase_at_ >= perching_config_.gripper_open_duration;
    // Repeated position commands are idempotent and keep the desired state explicit.
    flygripper_msgs::ActuatorCommand command;
    command.header.stamp = ros::Time::now();
    command.mode = flygripper_msgs::ActuatorCommand::MODE_POSITION;
    command.reference_position = perching_config_.open_position;
    actuator_command_pub_.publish(command);
    if (!open) return false;
    perching_status_.phase = Status::RETREAT;
    perching_status_.detail = perching_config_.use_actuator_feedback
        ? "Open feedback confirmed; retreating with fixed heading"
        : "Open command wait elapsed (physical opening unconfirmed); retreating with fixed heading";
    perching_phase_at_ = perching_recovery_at_ = time;
    perching_retreat_start_ = robot_state_.pos;
    perching_retreat_start_.z() = perching_position_.z();
    perching_settled_at_ = -1.0;
  }
  // Quintic zero-velocity/acceleration endpoints; backward is relative to the
  // heading captured at perching, not the world's negative X direction.
  const double duration = perching_config_.retreat_duration;
  const double u = std::clamp((time - perching_phase_at_) / duration, 0.0, 1.0);
  const double s = u*u*u*(10 + u*(-15 + 6*u));
  const double ds = 30*u*u*(1-u)*(1-u) / duration;
  const double dds = 60*u*(1-u)*(1-2*u) / (duration*duration);
  const Vector<3> displacement = -perching_config_.retreat_distance *
      Vector<3>(std::cos(perching_yaw_), std::sin(perching_yaw_), 0.0);
  sampler_->getPoseReference(perching_retreat_start_ + s * displacement, perching_yaw_,
                            references_, now_, robot_command_.rotor_mode);
  for (auto& reference : references_) {
    reference.state.vel = ds * displacement;
    reference.state.acc = dds * displacement;
  }
  const bool reached = u >= 1.0 &&
      (robot_state_.pos - perching_retreat_start_ - displacement).norm() <= perching_config_.position_tolerance &&
      robot_state_.vel.norm() <= perching_config_.max_settle_speed &&
      std::abs(std::remainder(robot_state_.getYaw() - perching_yaw_, 2*M_PI)) < 0.1;
  if (!reached) perching_settled_at_ = -1.0;
  else if (perching_settled_at_ < 0.0) perching_settled_at_ = time;
  if (perching_settled_at_ >= 0.0 && time - perching_settled_at_ >= perching_config_.settle_duration) {
    perching_status_.phase = Status::COMPLETE;
    perching_status_.detail = "Retreat complete; holding the retreat endpoint";
    RobotState hold = robot_state_.getHoverState();
    hold.pos = perching_retreat_start_ + displacement;
    hold.setQ(perching_yaw_);
    sampler_->getHoverReference(hold, references_, now_, robot_command_.rotor_mode, true);
    // Leave trajectory mode so newly received messages cannot resume a task.
    // GUIDED -> TRAJ clears old messages and checks the first new trajectory.
    rc_input_.resetTrajModeTrigger();
    setFsmState(FsmState::STATE_GUIDED);
  }
  return false;  // RETREAT uses the normal controller with the references above.
}

double Fsm::perchingRecoveryThrottle(double computed) const {
  if (!std::isfinite(computed)) return computed;
  if (steadySeconds() < perching_rescue_until_)
    return std::max(computed, perching_status_.saved_throttle);
  if (perching_status_.phase == Status::OPENING)
    return std::max(computed, perching_status_.saved_throttle);
  if (perching_recovery_at_ < 0.0) return computed;
  const double u = std::clamp((steadySeconds() - perching_recovery_at_) /
      perching_config_.recovery_duration, 0.0, 1.0);
  const double weight = 1.0 - u*u*(3.0 - 2.0*u);
  // Start at no less than recorded throttle; fade the floor into closed-loop
  // altitude control rather than dropping to a newly initialized low command.
  return std::max(computed, weight * perching_status_.saved_throttle + (1-weight)*computed);
}

void Fsm::publishPerchingStatus() {
  const double time = steadySeconds();
  if (time - perching_last_status_ < 0.05 &&
      perching_last_status_phase_ == perching_status_.phase &&
      perching_last_status_mission_ == perching_status_.mission_id) return;
  perching_last_status_ = time;
  perching_last_status_phase_ = perching_status_.phase;
  perching_last_status_mission_ = perching_status_.mission_id;
  perching_status_.header.stamp = ros::Time::now();
  perching_status_.output_throttle = std::isfinite(robot_command_.collective_thrust_command)
      ? robot_command_.collective_thrust_command : 0.0;
  perching_status_pub_.publish(perching_status_);
}
}  // namespace flygripper_flight
