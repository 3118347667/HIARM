#include "flygripper_flight/fsm.hpp"

#include "flygripper_common/type/const.hpp"
#include "flygripper_common/utils/realtime.hpp"
#include "flygripper_common/utils/runtime_profiler.hpp"
#include "flygripper_controller/thrust_safety.hpp"
#include "flygripper_msgs/GraspState.h"
#include "mavros_msgs/SetMode.h"
#include "mavros_msgs/ParamGet.h"

namespace flygripper_flight {
namespace {
template<class Message>
ros::SubscribeOptions inputOptions(
    const std::string& topic, uint32_t capacity,
    void (Fsm::*callback)(const ros::MessageEvent<const Message>&),
    Fsm* fsm, ros::CallbackQueue* queue) {
  ros::SubscribeOptions options;
  options.initByFullCallbackType<const ros::MessageEvent<const Message>&>(
      topic, capacity, [fsm, callback](const ros::MessageEvent<const Message>& event) {
        (fsm->*callback)(event);
      });
  options.callback_queue = queue;
  options.allow_concurrent_callbacks = false;
  options.transport_hints = ros::TransportHints().tcpNoDelay();
  return options;
}

bool validTrajectoryMessage(const flygripper_msgs::PolynomialTrajectory& message) {
  if (message.xyz_pieces.empty() || message.id > static_cast<uint32_t>(INT32_MAX)) return false;
  const auto order = message.xyz_pieces.front().order;
  if (order != 5 && order != 7) return false;
  const auto valid_piece = [](const auto& piece, uint32_t dim, uint32_t degree) {
    return piece.dim == dim && piece.order == degree &&
        piece.coeff.size() == dim * (degree + 1) &&
        std::isfinite(piece.duration) && piece.duration > 0.0 &&
        std::all_of(piece.coeff.begin(), piece.coeff.end(),
                    [](double value) { return std::isfinite(value); });
  };
  for (const auto& piece : message.xyz_pieces)
    if (!valid_piece(piece, 3, order)) return false;
  if (message.heading_mode > flygripper_msgs::PolynomialTrajectory::HEADING_POLY) return false;
  if (message.heading_mode == flygripper_msgs::PolynomialTrajectory::HEADING_POLY) {
    if (message.heading_pieces.empty()) return false;
    for (const auto& piece : message.heading_pieces)
      if (!valid_piece(piece, 1, 3)) return false;
  }
  return true;
}
}  // namespace

Fsm::Fsm(ros::NodeHandle &nh, ros::NodeHandle &pnh) {
  //! fsm parameter
  if (!fsm_param_config_.init(pnh)) {
    throw ros::Exception("Failed to initialize parameter config");
  }
  fsm_param_ = std::make_shared<FsmParameter>();
  fsm_param_config_.getParams(*fsm_param_);

  //! robot parameter
  ros::NodeHandle robot_param_nh(nh, "robot_parameter");
  if (!robot_param_config_.init(robot_param_nh)) {
    throw ros::Exception("Failed to initialize robot parameter config");
  }
  robot_param_ = std::make_shared<RobotParameter>();
  robot_param_config_.getParams(*robot_param_);

  //! rpm mapping
  rpm_maps_.resize(4);
  for (int i = 0; i < rpm_maps_.size(); ++i) {
    if (robot_param_->dshot_rpm_map_method == 0) {
      if (!rpm_maps_[i].init_simple_model(robot_param_->dshot_rpm_map_coeff)) {
        throw ros::Exception("Failed to initialize dshot_rpm_map_coeff config");
      }
    } else if (robot_param_->dshot_rpm_map_method == 1) {
      if (!rpm_maps_[i].init_precise_model(
              robot_param_->dshot_rpm_volt_map_coeff)) {
        throw ros::Exception(
            "Failed to initialize dshot_rpm_volt_map_coeff config");
      }
    } else if (robot_param_->dshot_rpm_map_method == 2) {
      std::filesystem::path rpm_map_file =
          std::filesystem::path(robot_param_->dshot_rpm_volt_map_file) /
          std::filesystem::path("rpm" + std::to_string(i) + "_map.csv");
      if (!rpm_maps_[i].init_precise_model(rpm_map_file)) {
        throw ros::Exception(
            "Failed to initialize dshot_rpm_volt_map_file config");
      }
    }
  }

  //! thrust acc mapping
  if (!thrust_acc_map_.init_simple_model(robot_param_->hover_percentage)) {
    throw ros::Exception("Failed to initialize thrust_acc_map_coeff config");
  }

  //! external wrench estimator
  ext_wrench_est_ = std::make_shared<ExternalWrenchEstimator<4>>(
      fsm_param_->external_wrench_buffer_size,
      fsm_param_->external_wrench_cutoff_frequency, fsm_param_->loop_frequency);
  external_wrench_.data = Eigen::Vector4d::Zero();

  //! default state and reference type
  fsm_state_ = FsmState::STATE_MANUAL;
  ref_type_ = ReferenceType::TYPE_HOVER;
  ROS_INFO("\033[32mInitial State: %s, Type: %s.\033[32m",
           FsmStateToString(fsm_state_).c_str(),
           ReferenceTypeToString(ref_type_).c_str());

  //! ros setup of subscriber, publisher, etc
  rosSetup(nh);

  //! initialize sampler and controller
  sampler_ = std::make_shared<Sampler>(robot_param_, fsm_param_);
  controller_ = std::make_shared<GeometricController>(
      robot_param_, fsm_param_->loop_frequency);
  mode_worker_ = std::make_unique<ModeWorker>(
      [this](const std::string& mode, bool verify_thrust, const ModeWorker::Current& current) {
        if (verify_thrust) {
          mavros_msgs::ParamGet parameter;
          parameter.request.param_id = "GUID_OPTIONS";
          if (!mavros_getparam_srv_.exists() || !mavros_getparam_srv_.call(parameter) ||
              !parameter.response.success || (parameter.response.value.integer & 8) == 0) {
            ROS_ERROR("Guided entry rejected: cannot verify GUID_OPTIONS bit 3.");
            return false;
          }
        }
        if (!current()) return false;
        mavros_msgs::SetMode service;
        service.request.custom_mode = mode;
        const bool success = mavros_setmode_srv_.exists() && current() &&
            mavros_setmode_srv_.call(service) && service.response.mode_sent;
        if (current()) {
          if (success) ROS_INFO_STREAM("Requested FCU mode " << mode << "; waiting for /mavros/state.");
          else ROS_ERROR_STREAM("Could not send FCU mode request: " << mode);
        }
        return success;
      });
  control_spinner_ = std::make_unique<ros::AsyncSpinner>(1, &control_queue_);
  input_queues_.start();
  ros::NodeHandle control_nh(nh);
  control_nh.setCallbackQueue(&control_queue_);
  loop_timer_ = control_nh.createTimer(ros::Duration(1.0 / fsm_param_->loop_frequency),
                               &Fsm::controlLoop, this);
  control_spinner_->start();
}

Fsm::~Fsm() {
  loop_timer_.stop();
  if (control_spinner_) control_spinner_->stop();
  input_queues_.stop();
  // Shutdown clients before joining; ROS shutdown also tears down service links.
  mavros_setmode_srv_.shutdown();
  mavros_getparam_srv_.shutdown();
  if (mode_worker_) mode_worker_->stop();
  publishZeroCommand();
};

void Fsm::publishZeroCommand() {
  // Zero thrust is a motor-stop command with GUID_OPTIONS bit 3 set.
  // A process exit is not a landing; let the FCU handle loss of setpoints.
  if (robot_command_.rotor_mode == RobotCommandMode::MODE_ATTITUDE) {
    ROS_WARN("Leaving attitude-output FCU to pilot/Guided timeout; no zero thrust sent.");
    return;
  }
  ROS_WARN("Start Shutting down");
  robot_command_.setZero();
  if (robot_command_.rotor_mode == RobotCommandMode::MODE_SINGLE_ROTOR) {
    for (int i = 0; i < rpm_maps_.size(); ++i) {
      robot_command_.rotor_velocities_command(i) = rpm_maps_[i].map(
          robot_command_.rotor_velocities(i), battery_.voltage);
    }
    toRosMsg(robot_command_, rotor_command_msg_);
    rotor_command_pub_.publish(rotor_command_msg_);
  }
  //  else if (robot_command_.rotor_mode ==
  //  RobotCommandMode::MODE_BODY_RATE_CUSTOMIZED){
  //    if (actuator_state_.isReceived(now_, fsm_param_->actuator_state_timeout)
  //    && actuator_state_.valid()){
  //      toRosMsg(robot_command_,
  //               -actuator_state_.position*0.005, //TODO: mapping from servo
  //               rotate rad to morphing delta length rotor_command_msg_);
  //    } else {
  //      toRosMsg(robot_command_,
  //               0.0,
  //               rotor_command_msg_);
  //    }
  //    rotor_command_pub_.publish(rotor_command_msg_);
  //  }
  else if (robot_command_.rotor_mode == RobotCommandMode::MODE_ATTITUDE) {
    robot_command_.collective_thrust_command = thrust_acc_map_.map(
        robot_command_.collective_thrust / robot_param_->mass);
    toRosMsg(robot_command_, thrust_att_command_msg_);
    thrust_att_command_pub_.publish(thrust_att_command_msg_);
  } else {
    ROS_WARN("other mode is not implemented.");
  }
  ROS_WARN("Shutting down completed");
}

void Fsm::rosSetup(ros::NodeHandle &nh) {
  using Channel = InputCallbackQueues::Channel;
  rc_options_ = inputOptions<mavros_msgs::RCIn>(
      "/mavros/rc/in", 10, &Fsm::rosRcCallback, this, input_queues_.get(Channel::Safety));
  rc_sub_ = nh.subscribe(rc_options_);
  fc_state_options_ = inputOptions<mavros_msgs::State>(
      "/mavros/state", 10, &Fsm::rosFcStateCallback, this, input_queues_.get(Channel::Safety));
  fc_state_sub_ = nh.subscribe(fc_state_options_);
  extended_state_options_ = inputOptions<mavros_msgs::ExtendedState>(
      "/mavros/extended_state", 10, &Fsm::rosExtendedStateCallback, this,
      input_queues_.get(Channel::Safety));
  extended_state_sub_ = nh.subscribe(extended_state_options_);
  odom_options_ = inputOptions<nav_msgs::Odometry>(
      "odom", 2, &Fsm::rosOdomCallback, this, input_queues_.get(Channel::Odom));
  odom_sub_ = nh.subscribe(odom_options_);
  imu_options_ = inputOptions<sensor_msgs::Imu>(
      "imu", 2, &Fsm::rosImuCallback, this, input_queues_.get(Channel::Imu));
  imu_sub_ = nh.subscribe(imu_options_);
  actuator_state_options_ = inputOptions<flygripper_msgs::ActuatorState>(
      "actuator_state", 10, &Fsm::rosActuatorStateCallback, this,
      input_queues_.get(Channel::Ordinary));
  actuator_state_sub_ = nh.subscribe(actuator_state_options_);
  battery_options_ = inputOptions<sensor_msgs::BatteryState>(
      "/mavros/battery", 10, &Fsm::rosBatteryCallback, this,
      input_queues_.get(Channel::Ordinary));
  battery_sub_ = nh.subscribe(battery_options_);
  rotor_speed_options_ = inputOptions<mavros_msgs::ESCTelemetry>(
      "rotor_speed", 10, &Fsm::rosRotorSpeedCallback, this,
      input_queues_.get(Channel::Ordinary));
  rotor_speed_sub_ = nh.subscribe(rotor_speed_options_);
  traj_options_ = inputOptions<flygripper_msgs::PolynomialTrajectory>(
      "trajectory", 10, &Fsm::rosTrajCallback, this,
      input_queues_.get(Channel::Ordinary));
  traj_sub_ = nh.subscribe(traj_options_);
  perching_options_ = inputOptions<flygripper_msgs::PerchingCommand>(
      "perching_command", 10, &Fsm::rosPerchingCallback, this,
      input_queues_.get(Channel::Ordinary));
  perching_sub_ = nh.subscribe(perching_options_);
  perching_status_pub_ = nh.advertise<flygripper_msgs::PerchingStatus>("perching_status", 10);

  runtime_diagnostic_pub_ = nh.advertise<diagnostic_msgs::DiagnosticArray>("runtime_diagnostics", 10);
  flight_trajectory_status_pub_ = nh.advertise<flygripper_msgs::FlightTrajectoryStatus>(
      "flight_trajectory_status", 1, false);

  // robot command
  rotor_command_pub_ =
      nh.advertise<mavros_msgs::ActuatorControl>("rotor_command", 10);
  actuator_command_pub_ =
      nh.advertise<flygripper_msgs::ActuatorCommand>("actuator_command", 10);
  thrust_att_command_pub_ = nh.advertise<mavros_msgs::AttitudeTarget>(
      "/mavros/setpoint_raw/attitude", 10);
  traj_trigger_pub_ = nh.advertise<geometry_msgs::PoseStamped>("trigger", 10);
  reference_pub_ = nh.advertise<nav_msgs::Path>("reference", 10);
  ext_indi_pub_ =
      nh.advertise<geometry_msgs::WrenchStamped>("external_wrench_indi", 10);
  ext_l1_pub_ =
      nh.advertise<geometry_msgs::WrenchStamped>("external_wrench_l1", 10);
  ext_tact_pub_ =
      nh.advertise<geometry_msgs::WrenchStamped>("external_wrench_tact", 10);
  interactive_wrench_pub_ =
      nh.advertise<geometry_msgs::WrenchStamped>("interactive_wrench", 10);
  robot_state_pub_ =
      nh.advertise<flygripper_msgs::RobotState>("robot_state", 10);
  ctrl_state_pub_ = nh.advertise<flygripper_msgs::GeometricControllerData>(
      "geometric_controller/data", 10);
  mavros_setmode_srv_ =
      nh.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");
  mavros_getparam_srv_ =
      nh.serviceClient<mavros_msgs::ParamGet>("/mavros/param/get");

}

void Fsm::rosOdomCallback(const ros::MessageEvent<const nav_msgs::Odometry>& event) {
  odom_input_.receive(event);
}

void Fsm::rosImuCallback(const ros::MessageEvent<const sensor_msgs::Imu>& event) {
  imu_input_.receive(event);
}

void Fsm::rosActuatorStateCallback(const ros::MessageEvent<const flygripper_msgs::ActuatorState>& event) {
  actuator_input_.receive(event);
}

void Fsm::rosBatteryCallback(const ros::MessageEvent<const sensor_msgs::BatteryState>& event) {
  battery_input_.receive(event);
}

void Fsm::rosRotorSpeedCallback(const ros::MessageEvent<const mavros_msgs::ESCTelemetry>& event) {
  rotor_input_.receive(event);
}

void Fsm::rosRcCallback(const ros::MessageEvent<const mavros_msgs::RCIn>& event) {
  rc_input_buffer_.receive(event);
}

void Fsm::rosFcStateCallback(const ros::MessageEvent<const mavros_msgs::State>& event) {
  fc_input_.receive(event);
}

void Fsm::rosExtendedStateCallback(const ros::MessageEvent<const mavros_msgs::ExtendedState>& event) {
  extended_input_.receive(event);
}

void Fsm::rosTrajCallback(const ros::MessageEvent<const flygripper_msgs::PolynomialTrajectory>& event) {
  trajectory_input_.receive(event);
}

void Fsm::consumeInputs() {
  const double started = steadySeconds();
  Inputs input = applied_;
  // Release each topic lock before taking another. Parsing, logging and old
  // message destruction are outside the buffers; no nested locks are needed.
  const auto rc_events = rc_input_buffer_.tryDrain();
  if (rc_events) input.rc = rc_events->latest;
  else ++input_snapshot_misses_;
  const auto copy = [this](auto& destination, const auto& source) {
    auto snapshot = source.trySnapshot();
    if (snapshot) destination = std::move(*snapshot);
    else ++input_snapshot_misses_;
  };
  copy(input.fc, fc_input_);
  copy(input.extended, extended_input_);
  copy(input.imu, imu_input_);
  copy(input.odom, odom_input_);
  copy(input.battery, battery_input_);
  copy(input.rotor, rotor_input_);
  copy(input.actuator, actuator_input_);
  copy(input.trajectory, trajectory_input_);
  const double copied = steadySeconds();
  input_lock_copy_duration_ = copied - started;
  rc_snapshot_overflow_ = rc_events && rc_events->overflow;
  rc_release_seen_ = rc_snapshot_overflow_;
  rc_traj_release_seen_ = rc_snapshot_overflow_;
  rc_invalid_seen_ = false;
  for (size_t i = 0; rc_events && i < rc_events->size; ++i) {
    const auto& event = rc_events->events[i];
    // Decode independently: a short/invalid frame must never refresh the last
    // valid channels or their timestamp. Preserve the fault across this batch,
    // even when a later valid frame has both autonomy switches high.
    Rc<CHANNEL_SIZE> decoded_rc;
    if (event.message->channels.size() >= CHANNEL_SIZE)
      fromRosMsg(event.message, decoded_rc, true);
    if (!decoded_rc.valid()) {
      rc_invalid_seen_ = rc_release_seen_ = rc_traj_release_seen_ = true;
      ROS_ERROR_THROTTLE(1.0, "Rejected malformed RC frame (missing or invalid channels)");
      continue;
    }
    rc_input_.channels = decoded_rc.channels;
    rc_input_.stamp = event.received_ros;
    rc_input_.update();
    if (!rc_input_.isGuidedMode()) rc_release_seen_ = true;
    // Preserve a CH6 exit even if a later event in this batch is already high.
    if (fsm_state_ == FsmState::STATE_TRAJ && rc_input_.valid() && !rc_input_.isTrajMode())
      rc_traj_release_seen_ = true;
  }
  if (input.imu.message && input.imu.count != applied_.imu.count) {
    fromRosMsg(input.imu.message, imu_);
    imu_.stamp = input.imu.message->header.stamp.toSec();
  }
  if (input.odom.message && input.odom.count != applied_.odom.count) {
    fromRosMsg(input.odom.message, odom_);
    odom_.stamp = input.odom.message->header.stamp.toSec();
  }
  if (input.fc.message && input.fc.count != applied_.fc.count) {
    fromRosMsg(input.fc.message, fc_state_);
    fc_state_.stamp = input.fc.received_ros;
  }
  if (input.battery.message && input.battery.count != applied_.battery.count) {
    fromRosMsg(input.battery.message, battery_);
    battery_.stamp = input.battery.received_ros;
  }
  if (input.actuator.message && input.actuator.count != applied_.actuator.count) {
    fromRosMsg(input.actuator.message, actuator_state_);
  }
  if (input.rotor.message && input.rotor.count != applied_.rotor.count) {
    fromRosMsg(input.rotor.message, rotor_speed_);
    rotor_speed_.stamp = input.rotor.received_ros;
  }
  if (input.trajectory.message && input.trajectory.count != applied_.trajectory.count) {
    // Decode into a fresh object: failed decoding must not acknowledge the
    // previous trajectory under a newly received message's identity.
    Trajectory decoded;
    if (validTrajectoryMessage(*input.trajectory.message))
      fromRosMsg(input.trajectory.message, decoded);
    else
      ROS_ERROR_THROTTLE(1.0, "Rejected malformed trajectory message");
    if (decoded.valid()) {
      pending_trajectory_ = decoded;
      pending_trajectory_message_ = input.trajectory.message;
    } else {
      trajectory_message_decoded_ = false;
      trajectory_ = Trajectory();
      active_trajectory_message_.reset();
      pending_trajectory_message_.reset();
    }
  }
  // A replanned trajectory can start slightly in the future. Continue the
  // current trajectory until its successor starts, rather than briefly
  // reporting a hover on every normal replan.
  if (pending_trajectory_message_ && now_ >= pending_trajectory_.stamp) {
    trajectory_ = pending_trajectory_;
    active_trajectory_message_ = pending_trajectory_message_;
    pending_trajectory_message_.reset();
    trajectory_message_decoded_ = true;
  }
  if (input.extended.message) {
    extended_state_ = *input.extended.message;
    extended_state_stamp_ = input.extended.received_ros;
  }
  applied_ = input;
  imu_receive_stamp_ = input.imu.received_ros;
  odom_receive_stamp_ = input.odom.received_ros;
  const double now = steadySeconds();
  imu_callback_gap_ = input.imu.count ? now - input.imu.received_steady : -1.0;
  odom_callback_gap_ = input.odom.count ? now - input.odom.received_steady : -1.0;
  input_copy_duration_ = now - started;
  input_decode_duration_ = now - copied;
}

void Fsm::controlLoop(const ros::TimerEvent &event) {
  trajectory_sampled_this_cycle_ = false;
  trajectory_control_published_this_cycle_ = false;
  compute_duration_ = 0.0;
  loop_started_ = steadySeconds();
  loop_period_ = last_loop_started_ > 0.0 ? loop_started_ - last_loop_started_ : 0.0;
  last_loop_started_ = loop_started_;
  max_loop_period_ = std::max(max_loop_period_, loop_period_);
  ScopeExit finish([this] {
    control_duration_ = steadySeconds() - loop_started_;
    max_control_duration_ = std::max(max_control_duration_, control_duration_);
    publishFlightTrajectoryStatus();
    publishPerchingStatus();
    publishRuntimeDiagnostics();
  });
  checkCycleTime(1.0 / fsm_param_->loop_frequency,
                 fsm_param_->loop_time_error_threshold, dt_);
  tic_toc_.tic();
  now_ = event.current_real.toSec();
  //! [ROS] Update all ros callback functions to update data and parameters.
  consumeInputs();
  now_ = ros::Time::now().toSec();
  const double parameter_started = steadySeconds();
  updateParameter();
  parameter_duration_ = steadySeconds() - parameter_started;
  now_ = ros::Time::now().toSec();
  const double health_time = steadySeconds();
  imu_callback_gap_ = applied_.imu.count ? health_time - applied_.imu.received_steady : -1.0;
  odom_callback_gap_ = applied_.odom.count ? health_time - applied_.odom.received_steady : -1.0;
  //    ROS_INFO_STREAM("Param: "<< *robot_param_);
  robot_command_.rotor_mode = fsm_state_ == FsmState::STATE_MANUAL
      ? fsm_param_->command_mode : session_command_mode_;
  const bool awaiting_late_guided = fsm_state_ == FsmState::STATE_HANDOVER &&
      handover_from_wait_ && fc_state_.mode == guided_entry_mode_ &&
      (mode_worker_->status().busy || now_ - mode_request_start_ < fsm_param_->mode_switch_timeout);
  // A pilot/FCU mode change always wins. Never switch back over a failsafe.
  if (fsm_state_ != FsmState::STATE_MANUAL &&
      fsm_state_ != FsmState::STATE_WAIT_GUIDED && !awaiting_late_guided && fcStateFresh() &&
      (!fc_state_.armed || fc_state_.mode != "GUIDED_NOGPS")) {
    stopAutonomy();
    return;
  }
  if (rc_release_seen_ && fsm_state_ != FsmState::STATE_MANUAL &&
      fsm_state_ != FsmState::STATE_HANDOVER)
    beginHandover(rc_invalid_seen_ ? "Malformed RC frame" : "CH5 released or RC event queue overflow");
  //! Update robot_state, fc_state and rc_input
  if (!updateRobotState()) {
    if (fsm_state_ == FsmState::STATE_MANUAL) {
      // An edge while inputs are unhealthy must not become a deferred takeoff.
      rc_input_.resetGuidedModeTrigger();
      rc_input_.resetTrajModeTrigger();
    } else {
      if (fsm_state_ != FsmState::STATE_HANDOVER)
        beginHandover(fault_reason_.empty() ? "Required input invalid" : fault_reason_);
      updateHandover();
      if (fsm_state_ == FsmState::STATE_HANDOVER)
        publishPerchingEmergencyRecovery();
    }
    return;
  }
  if (fsm_state_ == FsmState::STATE_HANDOVER) {
    updateHandover();
    // Retain a bounded hover command while waiting for FCU takeover, only
    // while all controller inputs are still valid. Never send zero thrust.
    if (fsm_state_ != FsmState::STATE_HANDOVER) return;
  }
  tic_toc_.toc("ros_update");

  consumePerchingCommand();
  // The flight loop is the only owner of motor output, including deliberate
  // zero-thrust output. Input health and pilot/FCU takeover checks run first.
  if (updatePerching()) return;

  //! Update FSM state and reference
  if (!perchingActive()) updateFsmStateAndReference();
  tic_toc_.toc("reference_update");

  robot_command_.stamp = now_;
  //! Controller preparation
  static_cast<GeometricController*>(controller_.get())->setContactHold(
      perching_status_.phase == flygripper_msgs::PerchingStatus::OPENING);
  controller_->prepare(robot_state_);
  //! Rotor control process for guided and traj mode
  if ((fsm_state_ == FsmState::STATE_GUIDED ||
      fsm_state_ == FsmState::STATE_TRAJ ||
      fsm_state_ == FsmState::STATE_HANDOVER) &&
      guided_entry_verified_ && fc_state_.armed && fc_state_.mode == "GUIDED_NOGPS") {
    //! Controller execution
    auto* geometric = static_cast<GeometricController*>(controller_.get());
    static flygripper_common::RuntimeProfiler control_profiler(
        "trajectory_tracking_control", 1000);
    {
      flygripper_common::ScopedRuntimeSample profile_sample(control_profiler);
      const double started = steadySeconds();
      controller_->getCommand(robot_state_, references_, robot_command_, now_);
      compute_duration_ = steadySeconds() - started;
    }
    if (geometric->thrustSafetyViolated()) {
      attitude_safety_handover_ = true;
      beginHandover("Unsafe thrust direction or non-finite acceleration");
    }
    tic_toc_.toc("control");

    //! External Force Estimation
    if (fsm_param_->admittance_mode == AdmittanceMode::DISABLE) {
      ext_wrench_est_->reset();
    } else {
      // get external force from L1 estimator and torque from indi
      Vector<3> external_force =
          dynamic_cast<GeometricController *>(controller_.get())
              ->getExternalWrench("l1")
              .head(3);
      double external_yaw_torque =
          dynamic_cast<GeometricController *>(controller_.get())
              ->getExternalWrench("indi")(5);
      // assign external force xyz and torque yaw
      external_wrench_.data << external_force, external_yaw_torque;
      // update to estimator real interactive wrench
      // if external wrench is smaller than threshold, set interactive wrench to
      // ZERO else remove the internal wrench from external wrench to get real
      // interactive wrench
      if (!ext_wrench_est_->update(
              external_wrench_.data, external_wrench_.data,
              fsm_param_->external_force_threshold,
              fsm_param_->external_torque_threshold,
              fsm_param_->external_wrench_trigger_duration)) {
        external_wrench_.data.setZero();
      }
      // Publish interactive wrench
      toRosMsg(external_wrench_, interactive_wrench_msg_);
      interactive_wrench_pub_.publish(interactive_wrench_msg_);
    }

    //! [ROS] Publish controller staff
    if (!robot_command_.valid()) {
      ROS_ERROR_STREAM_THROTTLE(1.0, "Rotor command is not valid! \n" << robot_command_);
    } else {
      // [ROS] First publish rotor commands
      if (robot_command_.rotor_mode == RobotCommandMode::MODE_SINGLE_ROTOR) {
        for (int i = 0; i < rpm_maps_.size(); ++i) {
          robot_command_.rotor_velocities_command(i) = rpm_maps_[i].map(
              robot_command_.rotor_velocities(i), battery_.voltage);
        }

        toRosMsg(robot_command_, rotor_command_msg_);
        rotor_command_pub_.publish(rotor_command_msg_);
        trajectory_control_published_this_cycle_ = true;
      }
      //      else if (robot_command_.rotor_mode ==
      //      RobotCommandMode::MODE_BODY_RATE_CUSTOMIZED){
      //        if (actuator_state_.isReceived(now_,
      //        fsm_param_->actuator_state_timeout) && actuator_state_.valid()){
      //          toRosMsg(robot_command_,
      //                   -actuator_state_.position*0.005, //TODO: mapping from
      //                   servo rotate rad to morphing delta length
      //                   rotor_command_msg_);
      //        } else {
      //          toRosMsg(robot_command_,
      //                   0.0,
      //                   rotor_command_msg_);
      //        }
      //        rotor_command_pub_.publish(rotor_command_msg_);
      //      }
      else if (robot_command_.rotor_mode == RobotCommandMode::MODE_ATTITUDE) {
        robot_command_.attitude_command =
            imu_.att * odom_.att.inverse() *
            robot_command_.attitude; // Align with FCU Frame
        // Frame alignment must preserve the configured maximum tilt as well
        // as an upright, finite target.
        const auto& q = robot_command_.attitude_command;
        if (!q.coeffs().allFinite() || !std::isfinite(q.norm()) || q.norm() < 1e-6 ||
            !flygripper_controller::uprightTiltWithinLimit(
                (q.normalized() * Vector<3>::UnitZ()).z(), robot_param_->tilt_angle_max)) {
          attitude_safety_handover_ = true;
          beginHandover("Unsafe FCU attitude after frame alignment (invalid or tilt limit exceeded)");
        }
        if (attitude_safety_handover_) {
          // Latch this fallback until mode feedback confirms FCU takeover.
          // Use FCU yaw directly; do not reapply the suspect frame alignment.
          const Quaternion current = imu_.att.normalized();
          const double yaw = std::atan2(
              2.0 * (current.w() * current.z() + current.x() * current.y()),
              1.0 - 2.0 * (current.y() * current.y() + current.z() * current.z()));
          robot_command_.attitude_command = Quaternion(
              Eigen::AngleAxis<Scalar>(yaw, Vector<3>::UnitZ()));
          robot_command_.collective_thrust = robot_param_->mass * kGravity;
        }
        robot_command_.attitude_command.normalize();
        robot_command_.collective_thrust_command = thrust_acc_map_.map(
            robot_command_.collective_thrust / robot_param_->mass);
        if (!attitude_safety_handover_)
          robot_command_.collective_thrust_command =
              perchingRecoveryThrottle(robot_command_.collective_thrust_command);
        if (!std::isfinite(robot_command_.collective_thrust_command) ||
            !robot_command_.attitude_command.coeffs().allFinite()) {
          beginHandover("Non-finite attitude/thrust command");
          return;
        }
        robot_command_.collective_thrust_command =
            std::clamp(robot_command_.collective_thrust_command, 0.0, 1.0);
        toRosMsg(robot_command_, thrust_att_command_msg_);
        thrust_att_command_pub_.publish(thrust_att_command_msg_);
        if (!perchingActive() && fsm_state_ == FsmState::STATE_TRAJ) {
          last_powered_throttle_ = robot_command_.collective_thrust_command;
          last_powered_throttle_at_ = steadySeconds();
        }
        trajectory_control_published_this_cycle_ = true;
      } else {
        ROS_WARN_THROTTLE(1.0, "other mode is not implemented.");
      }

      // [ROS] Publish robot_state
      toRosMsg(robot_state_, robot_state_msg_);
      robot_state_pub_.publish(robot_state_msg_);

      // [ROS] Publish references
      toRosMsg(references_, reference_msg_);
      reference_pub_.publish(reference_msg_);

      // [ROS] Publish controller state and relevant data from controller
      ctrl_state_pub_.publish(
          dynamic_cast<GeometricController *>(controller_.get())
              ->getControllerMsg());
      ext_indi_pub_.publish(
          dynamic_cast<GeometricController *>(controller_.get())
              ->getExternalWrenchMsg("indi"));
      ext_l1_pub_.publish(dynamic_cast<GeometricController *>(controller_.get())
                              ->getExternalWrenchMsg("l1"));
      ext_tact_pub_.publish(
          dynamic_cast<GeometricController *>(controller_.get())
              ->getExternalWrenchMsg("tact"));
      tic_toc_.toc("control_publish");
    }
  }

  tic_toc_.toc("morph_publish");
  ROS_INFO_THROTTLE(10.0, "\n------------- Timing ------------- \n%s",
                    tic_toc_
                        .analyze({"ros_update", "reference_update", "control",
                                  "control_publish", "morph_publish"})
                        .c_str());
}

void Fsm::updateParameter() {
  // [ROS] Update dynamic parameters
  fsm_param_config_.getParams(*fsm_param_);
  robot_param_config_.getParams(*robot_param_);
}

bool Fsm::updateRobotState() {
  static double last_imu_stamp = -1.0;
  bool ret = !rc_snapshot_overflow_ && !rc_invalid_seen_;
  fault_reason_ = rc_snapshot_overflow_ ? "RC event queue overflow; " : "";
  if (rc_invalid_seen_) fault_reason_ += "Malformed RC frame; ";
  const auto freshness_reason = [](double stamp, double received, double gap, double timeout) {
    if (received < 0.0) return "NO MESSAGE PROCESSED YET";
    if (!std::isfinite(stamp) || stamp <= 0.0) return "INVALID HEADER TIMESTAMP";
    if (gap > timeout) return "NO RECENT CALLBACK (stream/queue/scheduling)";
    return "OLD HEADER TIMESTAMP (check callback_gap for input stall versus delayed samples)";
  };
  // Preserve measurement-age gating; callback time is diagnostic only.
  if (!std::isfinite(imu_.stamp) || now_ - imu_.stamp > fsm_param_->imu_timeout ||
      imu_callback_gap_ < 0.0 || imu_callback_gap_ > fsm_param_->imu_timeout) {
    fault_reason_ += "IMU stale; ";
    ret = false;
    ROS_ERROR_THROTTLE(1.0,
        "\033[1;31m[IMU freshness] %s; header_age=%.1f ms, callback_gap=%.1f ms, limit=%.1f ms. Companion control unavailable.\033[0m",
        freshness_reason(imu_.stamp, imu_receive_stamp_, imu_callback_gap_, fsm_param_->imu_timeout),
        (now_ - imu_.stamp) * 1000.0,
        imu_callback_gap_ * 1000.0,
        fsm_param_->imu_timeout * 1000.0);
  } else if (!imu_.valid()) {
    ret = false;
    fault_reason_ += "IMU invalid; ";
    ROS_WARN_THROTTLE(1.0, "IMU data invalid!");
  }

  if (!std::isfinite(odom_.stamp) || now_ - odom_.stamp > fsm_param_->odom_timeout ||
      odom_callback_gap_ < 0.0 || odom_callback_gap_ > fsm_param_->odom_timeout) {
    fault_reason_ += "EKF odometry stale; ";
    ret = false;
    ROS_ERROR_THROTTLE(1.0,
        "\033[1;31m[EKF odometry freshness] %s; header_age=%.1f ms, callback_gap=%.1f ms, limit=%.1f ms. This does not prove raw mocap loss.\033[0m",
        freshness_reason(odom_.stamp, odom_receive_stamp_, odom_callback_gap_, fsm_param_->odom_timeout),
        (now_ - odom_.stamp) * 1000.0,
        odom_callback_gap_ * 1000.0,
        fsm_param_->odom_timeout * 1000.0);
  } else if (!odom_.valid()) {
    ret = false;
    fault_reason_ += "EKF odometry invalid; ";
    ROS_WARN_THROTTLE(1.0, "Odometry data invalid!");
  }
  const bool quaternion_valid = std::abs(imu_.att.norm() - 1.0) < 0.05 &&
                                std::abs(odom_.att.norm() - 1.0) < 0.05;
  if (!quaternion_valid) fault_reason_ += "quaternion norm invalid; ";
  ret &= quaternion_valid;

  // ESC freshness is required below only for single-rotor output. Attitude
  // output intentionally works without ESC telemetry; do not log false alarms.

  if (ret) { // update robot_state from imu, odom, rotor_speed and
             // actuator_state(x)
    robot_state_.stamp = now_;
    // from odom
    robot_state_.pos = odom_.pos;
    robot_state_.setQ(odom_.att);
    robot_state_.vel = odom_.vel;
    // from imu
    if (last_imu_stamp < 0 || imu_.stamp < last_imu_stamp) {
      robot_state_.alpha = Vector<3>::Zero();
    } else if (imu_.stamp - last_imu_stamp > 1e-6) {
      robot_state_.alpha = 1 / (imu_.stamp - last_imu_stamp) *
                           (imu_.omega - robot_state_.omega); // body frame
    }
    last_imu_stamp = imu_.stamp;
    robot_state_.omega = imu_.omega;                          // body frame
    robot_state_.acc = odom_.att * imu_.acc + kGravityVector; // world frame
    // from rotor speed
    // Attitude output does not require ESC feedback. Do not seed the
    // controller's diagnostic filters with infinities when ESC is absent.
    robot_state_.rot_speed = rotor_speed_.valid()
        ? rotor_speed_.rot_speed : Vector<4>::Zero();

    if (actuator_state_.isReceived(now_, fsm_param_->actuator_state_timeout) &&
        actuator_state_.valid()) {
      // TODO： given by actuator state real time
      Scalar l = 0.21 / std::sqrt(2.0) / 2.0;
      robot_state_.rot1_pos << l, -l;
      robot_state_.rot2_pos << -l, l;
      robot_state_.rot3_pos << l, l;
      robot_state_.rot4_pos << -l, -l;
    } else {
      // given by initial value from parameters
      robot_state_.rot1_pos << robot_param_->rotor_position.col(0);
      robot_state_.rot2_pos << robot_param_->rotor_position.col(1);
      robot_state_.rot3_pos << robot_param_->rotor_position.col(2);
      robot_state_.rot4_pos << robot_param_->rotor_position.col(3);
    }
  }

  if (!rc_input_.isReceived(now_, fsm_param_->rc_input_timeout)) {
    fault_reason_ += "RC stale; ";
    ret = false;
  } else if (!rc_input_.valid()) {
    ret = false;
    fault_reason_ += "RC invalid; ";
    ROS_WARN_THROTTLE(1.0, "RC In data invalid!");
  } else {
    velocity_control_.des_vel.x() =
        rc_input_.channels[AXIS_PITCH] * fsm_param_->velocity_scale_x;
    velocity_control_.des_vel.y() =
        rc_input_.channels[AXIS_ROLL] * fsm_param_->velocity_scale_y;
    velocity_control_.des_vel.z() =
        rc_input_.channels[AXIS_THROTTLE] * fsm_param_->velocity_scale_z;
    velocity_control_.des_yaw_rate =
        rc_input_.channels[AXIS_YAW] * fsm_param_->velocity_scale_yaw;
  }

  if (!battery_.isReceived(now_, fsm_param_->battery_timeout)) {
    //ret = false;
  } else if (!battery_.valid()) {
    //ret = false;
    ROS_WARN_THROTTLE(1.0, "Battery data invalid!");
  }
  if (!fcStateFresh()) fault_reason_ += "FCU state stale/disconnected; ";
  ret &= fcStateFresh();
  // MAVROS maps FCU boot time to ROS time using a filtered timesync offset.
  // Small sub-frame future timestamps are normal; large jumps still fail.
  const bool sensor_clock_valid = imu_.stamp > 0.0 && odom_.stamp > 0.0 &&
      imu_.stamp <= now_ + fsm_param_->sensor_future_tolerance &&
      odom_.stamp <= now_ + fsm_param_->sensor_future_tolerance;
  if (!sensor_clock_valid)
    ROS_WARN_THROTTLE(1.0, "\033[1;31mSensor clock invalid: IMU age %.6f s, odometry age %.6f s\033[0m",
                      now_ - imu_.stamp, now_ - odom_.stamp);
  if (!sensor_clock_valid) fault_reason_ += "sensor clock invalid; ";
  ret &= sensor_clock_valid;
  if (robot_command_.rotor_mode == RobotCommandMode::MODE_SINGLE_ROTOR)
    ret &= rotor_speed_.isReceived(now_, fsm_param_->rotor_speed_timeout) &&
           rotor_speed_.valid();
  return ret;
}

void Fsm::updateFsmStateAndReference() {
  switch (fsm_state_) {
  case FsmState::STATE_MANUAL: {
    //! [STATE] MANUAL => ...
    if (rc_input_.isGuidedModeTriggered()) {
      rc_input_.resetGuidedModeTrigger();
      if (!prepareGuidedEntry()) break;
      velocity_teleop_ready_ = false;
      session_command_mode_ = robot_command_.rotor_mode;
      mode_request_start_ = now_;
      last_mode_request_ = -INF;
      setFsmState(FsmState::STATE_WAIT_GUIDED);
    }
    break;
  }
  case FsmState::STATE_WAIT_GUIDED: {
    if (!rc_input_.isGuidedMode() || !fc_state_.armed) {
      beginHandover("Guided entry cancelled or FCU disarmed");
      break;
    }
    if (now_ < mode_request_start_ ||
        now_ - mode_request_start_ > fsm_param_->mode_switch_timeout) {
      // A late GUIDED confirmation must also be handed back to the FCU.
      beginHandover("GUIDED_NOGPS entry verification/confirmation timed out");
      break;
    }
    const auto worker = mode_worker_->status();
    // Mode feedback alone cannot verify thrust semantics. Require a successful
    // request from this session, including when the FCU was already in GUIDED.
    if (worker.completed > guided_entry_completed_baseline_ &&
        worker.result_mode == "GUIDED_NOGPS" && worker.success)
      guided_entry_verified_ = true;
    if (guided_entry_verified_ && fc_state_.mode == "GUIDED_NOGPS") {
      if (takeoff_requested_) {
        if (!std::isfinite(extended_state_stamp_) ||
            now_ - extended_state_stamp_ > fsm_param_->extended_state_timeout ||
            extended_state_.landed_state != mavros_msgs::ExtendedState::LANDED_STATE_ON_GROUND ||
            robot_state_.vel.norm() > fsm_param_->takeoff_start_speed_max) {
          beginHandover("Ground state changed while waiting for GUIDED_NOGPS");
          break;
        }
        auto settings = fsm_param_->takeoff;
        settings.height = fsm_param_->takeoff_height;
        if (!takeoff_plan_.start({robot_state_.pos.x(), robot_state_.pos.y(),
                                 robot_state_.pos.z()}, robot_state_.getYaw(), now_, settings)) {
          beginHandover("Invalid takeoff limits or timeout too short for ascent");
          break;
        }
        ROS_INFO("Takeoff: relative height %.2f m, profile %.2f s",
                 settings.height, takeoff_plan_.duration());
        setReferenceType(ReferenceType::TYPE_TAKEOFF);
        sampler_->getTakeoffReference(takeoff_plan_, references_, now_,
                                      robot_command_.rotor_mode);
      } else {
        setReferenceType(ReferenceType::TYPE_HOVER);
        sampler_->getHoverReference(robot_state_, references_, now_,
                                    robot_command_.rotor_mode, true);
      }
      setFsmState(FsmState::STATE_GUIDED);
      break;
    }
    if (now_ - last_mode_request_ >= 1.0) {
      last_mode_request_ = now_;
      requestFcMode("GUIDED_NOGPS");
    }
    break;
  }
  case FsmState::STATE_HANDOVER:
    sampler_->getHoverReference(robot_state_, references_, now_,
                                robot_command_.rotor_mode);
    break;
  case FsmState::STATE_GUIDED: {
    //! [STATE] GUIDED => ...
    if (!rc_input_.isGuidedMode()) {
      beginHandover("CH5 released");
      break;
    } else if (ref_type_ != ReferenceType::TYPE_TAKEOFF &&
               rc_input_.isTrajModeTriggered()) {
      rc_input_.resetTrajModeTrigger();
      // Double check if fc state is set to guided
      if (!fc_state_.guided) {
        ROS_ERROR("Failure in GUIDED => TRAJ: FC state is not guided!");
        break;
      }
      //! [STATE] GUIDED => TRAJ by rc
      // Do not reuse a trajectory received in a previous trajectory session.
      // consumeInputs() has already recorded its sequence in applied_.
      trajectory_ = Trajectory();
      pending_trajectory_message_.reset();
      active_trajectory_message_.reset();
      trajectory_message_decoded_ = false;
      awaiting_first_trajectory_ = true;
      setFsmState(FsmState::STATE_TRAJ);
      if (fsm_param_->traj_trigger) {
        publishTrajTrigger();
      }
      //! [TYPE] ANY => HOVER while switching from STATE GUIDED to TRAJ
      setReferenceType(ReferenceType::TYPE_HOVER);
      //! NOTE: use the previous reference state instead of robot_state
      //! to avoid reference jump from STATE GUIDED to TRAJ cause of constant
      //! error of quadrotor.
      sampler_->getHoverReference(references_.front().state, references_, now_,
                                  robot_command_.rotor_mode, true);
      break;
    }

    // Ground arming usually leaves the throttle stick low. Require centering
    // after ascent before joystick velocity control can command a descent.
    if (ref_type_ != ReferenceType::TYPE_TAKEOFF &&
        velocity_control_.des_vel.norm() <= kVelocityDeadZone &&
        std::abs(velocity_control_.des_yaw_rate) <= kVelocityDeadZone)
      velocity_teleop_ready_ = true;

    //! [STATE] GUIDED
    //-------------------------------------- ReferenceType::TYPE_TAKEOFF
    //----------------------------------------
    if (ref_type_ == ReferenceType::TYPE_TAKEOFF) {
      // A CH6 edge during ascent is consumed, so it cannot trigger a delayed
      // trajectory transition immediately after takeoff.
      rc_input_.resetTrajModeTrigger();
      const auto status = takeoff_plan_.update(
          now_, {robot_state_.pos.x(), robot_state_.pos.y(), robot_state_.pos.z()},
          {robot_state_.vel.x(), robot_state_.vel.y(), robot_state_.vel.z()});
      if (status == TakeoffPlan::Status::COMPLETE) {
        sampler_->getTakeoffReference(takeoff_plan_, references_, now_,
                                      robot_command_.rotor_mode);
        setReferenceType(ReferenceType::TYPE_HOVER);
        sampler_->getHoverReference(references_.front().state, references_,
                                    now_, robot_command_.rotor_mode, true);
        takeoff_plan_.reset();
        ROS_INFO("Takeoff complete; holding the takeoff target.");
      } else if (status == TakeoffPlan::Status::RUNNING) {
        sampler_->getTakeoffReference(takeoff_plan_, references_, now_,
                                      robot_command_.rotor_mode);
      } else {
        beginHandover(std::string("Takeoff failed: ") + TakeoffPlan::statusName(status));
      }
    }
    //-------------------------------------- ReferenceType::TYPE_VEL
    //----------------------------------------
    else if (fsm_param_->velocity_teleop && velocity_teleop_ready_ &&
             (velocity_control_.des_vel.norm() > kVelocityDeadZone ||
              std::abs(velocity_control_.des_yaw_rate) > kVelocityDeadZone)) {
      //! [TYPE] ANY => VEL
      if (ref_type_ != ReferenceType::TYPE_VEL) {
        setReferenceType(ReferenceType::TYPE_VEL);
        //! NOTE: use the previous reference state instead of robot_state
        //! to avoid reference jump from TYPE HOVER to VEL cause of static
        //! error of quadrotor.
        sampler_->getVelocityReference(
            references_.front().state, velocity_control_, references_, now_,
            robot_command_.rotor_mode, true, fsm_param_->world_frame_velocity);
      } else {
        //! [TYPE] VEL
        sampler_->getVelocityReference(
            robot_state_, velocity_control_, references_, now_,
            robot_command_.rotor_mode, false, fsm_param_->world_frame_velocity);
      }
    } else if (ref_type_ == ReferenceType::TYPE_VEL) {
      //! [TYPE] VEL => HOVER
      setReferenceType(ReferenceType::TYPE_HOVER);
      //! NOTE: use the previous reference state instead of robot_state
      //! to avoid reference jump from TYPE VEL to HOVER cause of static error
      //! of quadrotor.
      sampler_->getHoverReference(references_.front().state, references_, now_,
                                  robot_command_.rotor_mode, true);
    }
    //-------------------------------------- ReferenceType::TYPE_HOVER
    //----------------------------------------

    else if (ref_type_ != ReferenceType::TYPE_HOVER) {
      ROS_WARN(
          "ATTENTION: SHOULD NOT CALL THIS BRANCH IF YOUR FSM IS WELL BUILT, "
          "FORCE IT TO BE TYPE HOVER");
      setReferenceType(ReferenceType::TYPE_HOVER);
      //! NOTE: use the previous reference state instead of robot_state
      //! to avoid reference jump from TYPE HOVER to ADMITTANCE cause of
      //! constant error of quadrotor.
      sampler_->getHoverReference(references_.front().state, references_, now_,
                                  robot_command_.rotor_mode, true);
    } else {
      //! [TYPE] HOVER;
      sampler_->getHoverReference(robot_state_, references_, now_,
                                  robot_command_.rotor_mode);
    }

    break;
  }
  case FsmState::STATE_TRAJ: {
    //! [STATE] TRAJ => ...
    if (!rc_input_.isGuidedMode()) {
      beginHandover("CH5 released during trajectory control");
      break;
    } else if (rc_traj_release_seen_ || !rc_input_.isTrajMode()) {
      //! [STATE] TRAJ => GUIDED by rc
      setFsmState(FsmState::STATE_GUIDED);
      rc_input_.resetTrajModeTrigger();  // A queued low/high pair cannot re-enter next cycle.
      //! [TYPE] ANY => HOVER while switching from STATE TRAJ to GUIDED
      setReferenceType(ReferenceType::TYPE_HOVER);
      //! NOTE: use the previous reference state instead of robot_state
      //! to avoid reference jump from STATE TRAJ to GUIDED cause of constant
      //! error of quadrotor.
      sampler_->getHoverReference(references_.front().state, references_, now_,
                                  robot_command_.rotor_mode, true);
      break;
    }

    //! [STATE] TRAJ
    //-------------------------------------- ReferenceType::TYPE_TRAJ
    //----------------------------------------
    if (trajectory_.isReceived(now_) && trajectory_.valid() &&
        trajectory_.available(now_)) {
      if (awaiting_first_trajectory_) {
        const double t = now_ - trajectory_.getStartTime();
        const Vector<3> p = trajectory_.xyz_mode == Trajectory::XYZMode::XYZ_MIN_JERK
            ? trajectory_.xyz_min_jerk_poly.getPos(t)
            : trajectory_.xyz_min_snap_poly.getPos(t);
        const Vector<3> v = trajectory_.xyz_mode == Trajectory::XYZMode::XYZ_MIN_JERK
            ? trajectory_.xyz_min_jerk_poly.getVel(t)
            : trajectory_.xyz_min_snap_poly.getVel(t);
        bool heading_discontinuous = false;
        if (trajectory_.heading_mode == Trajectory::HeadingMode::HEADING_POLY) {
          const auto yawOf = [](const Vector<4>& a) {
            const Quaternion q(a[3], a[0], a[1], a[2]);
            const Vector<3> forward = q.normalized() * Vector<3>::UnitX();
            return std::atan2(forward.y(), forward.x());
          };
          const double yaw = trajectory_.heading_poly.getPos(t)[0];
          heading_discontinuous =
              std::abs(std::remainder(yaw - yawOf(robot_state_.att), 2.0 * M_PI)) > M_PI / 6.0 ||
              std::abs(std::remainder(yaw - yawOf(references_.front().state.att), 2.0 * M_PI)) > M_PI / 6.0;
        }
        // Old missions may still publish freshly stamped hold trajectories
        // before their trigger callback runs. Reject those as well as cached
        // messages; do not jump away from either the vehicle or its reference.
        if ((p - robot_state_.pos).norm() > 0.15 ||
            (p - references_.front().state.pos).norm() > 0.15 ||
            (v - robot_state_.vel).norm() > 0.30 || heading_discontinuous) {
          ROS_WARN_THROTTLE(1.0, "Rejecting first trajectory: position, velocity or heading differs from the current vehicle/reference");
          trajectory_ = Trajectory();
          active_trajectory_message_.reset();
          trajectory_message_decoded_ = false;
          break;
        }
      }
      //! [TYPE] ANY => TRAJ
      if (ref_type_ != ReferenceType::TYPE_TRAJ) {
        setReferenceType(ReferenceType::TYPE_TRAJ);
      }
      //! [TYPE] TRAJ
      if (!sampler_->getTrajectoryReference(robot_state_, trajectory_,
                                            references_, now_,
                                            robot_command_.rotor_mode)) {
        //! [TYPE] TRAJ => HOVER
        setReferenceType(ReferenceType::TYPE_HOVER);
        //! NOTE: Force switching to hovering mode with current robot_state
        //! because of trajectory problem
        //!       i.e. tracking error is too large //TODO: better to have
        sampler_->getHoverReference(robot_state_, references_, now_,
                                    robot_command_.rotor_mode, true);
      } else {
        trajectory_sampled_this_cycle_ = true;
        awaiting_first_trajectory_ = false;
      }
    } else if (ref_type_ == ReferenceType::TYPE_TRAJ) {
      //! [TYPE] TRAJ => HOVER
      setReferenceType(ReferenceType::TYPE_HOVER);
      //! NOTE: use the previous reference state instead of robot_state
      //! to avoid reference jump from TYPE TRAJ to HOVER cause of constant
      //! error of quadrotor.
      sampler_->getHoverReference(references_.front().state, references_, now_,
                                  robot_command_.rotor_mode, true);
    }

    //-------------------------------------- ReferenceType::TYPE_VEL
    //----------------------------------------
    //      else if (fsm_param_->velocity_teleop &&
    //               (velocity_control_.des_vel.norm() > kVelocityDeadZone ||
    //                std::abs(velocity_control_.des_yaw_rate) >
    //                kVelocityDeadZone)){
    //        //! [TYPE] ANY => VEL
    //        if (ref_type_ != ReferenceType::TYPE_VEL){
    //          setReferenceType(ReferenceType::TYPE_VEL);
    //          //! NOTE: use the previous reference state instead of
    //          robot_state
    //          //! to avoid reference jump from TYPE HOVER to VEL cause of
    //          static error of quadrotor.
    //          sampler_->getVelocityReference(references_.front().state,
    //                                         velocity_control_,
    //                                         references_, now_,
    //                                         robot_command_.rotor_mode,
    //                                         true,
    //                                         fsm_param_->world_frame_velocity);
    //        } else{
    //          //! [TYPE] VEL
    //          sampler_->getVelocityReference(robot_state_,
    //                                         velocity_control_,
    //                                         references_, now_,
    //                                         robot_command_.rotor_mode,
    //                                         false,
    //                                         fsm_param_->world_frame_velocity);
    //        }
    //      }
    //      else if(ref_type_ == ReferenceType::TYPE_VEL) {
    //        //! [TYPE] VEL => HOVER
    //        setReferenceType(ReferenceType::TYPE_HOVER);
    //        //! NOTE: use the previous reference state instead of
    //        robot_state
    //        //! to avoid reference jump from TYPE VEL to HOVER cause of
    //        static error of quadrotor.
    //        sampler_->getHoverReference(references_.front().state,
    //        references_, now_, robot_command_.rotor_mode, true);
    //      }

    //-------------------------------------- TYPE_ADMITTANCE
    //----------------------------------------
    else if (fsm_param_->admittance_mode != AdmittanceMode::DISABLE &&
             !external_wrench_.data.isApproxToConstant(0.0)) {
      //! [TYPE] ANY => ADMITTANCE
      // rebuild the external wrench for admittance system
      external_wrench_.ext_force.x() = fsm_param_->enable_admittance_x
                                           ? external_wrench_.ext_force.x()
                                           : 0.0;
      external_wrench_.ext_force.y() = fsm_param_->enable_admittance_y
                                           ? external_wrench_.ext_force.y()
                                           : 0.0;
      external_wrench_.ext_force.z() = fsm_param_->enable_admittance_z
                                           ? external_wrench_.ext_force.z()
                                           : 0.0;
      external_wrench_.ext_torque_yaw = fsm_param_->enable_admittance_yaw
                                            ? external_wrench_.ext_torque_yaw
                                            : 0.0;

      if (ref_type_ != ReferenceType::TYPE_ADMITTANCE) {
        setReferenceType(ReferenceType::TYPE_ADMITTANCE);
        //! NOTE: use the previous reference state instead of robot_state
        //! to avoid reference jump from TYPE HOVER to ADMITTANCE cause of
        //! constant error of quadrotor.
        sampler_->getAdmittanceReference(references_.front().state,
                                         external_wrench_, references_, now_,
                                         robot_command_.rotor_mode, true);
      } else {
        //! [TYPE] ADMITTANCE
        sampler_->getAdmittanceReference(robot_state_, external_wrench_,
                                         references_, now_,
                                         robot_command_.rotor_mode, false);
      }
    } else if (ref_type_ == ReferenceType::TYPE_ADMITTANCE) {
      //! [TYPE] ADMITTANCE => HOVER
      setReferenceType(ReferenceType::TYPE_HOVER);
      // NOTE: use last reference instead of current robot state as hover
      // reference to avoid control instability
      sampler_->getHoverReference(references_.front().state, references_, now_,
                                  robot_command_.rotor_mode, true);
    }
    //-------------------------------------- ReferenceType::TYPE_HOVER
    //----------------------------------------
    else if (ref_type_ != ReferenceType::TYPE_HOVER) {
      ROS_WARN(
          "ATTENTION: SHOULD NOT CALL THIS BRANCH IF YOUR FSM IS WELL BUILT, "
          "FORCE IT TO BE TYPE HOVER");
      setReferenceType(ReferenceType::TYPE_HOVER);
      //! NOTE: use the previous reference state instead of robot_state
      //! to avoid reference jump from TYPE HOVER to ADMITTANCE cause of
      //! constant error of quadrotor.
      sampler_->getHoverReference(references_.front().state, references_, now_,
                                  robot_command_.rotor_mode, true);
    } else {
      //! [TYPE] HOVER
      sampler_->getHoverReference(robot_state_, references_, now_,
                                  robot_command_.rotor_mode);
    }
    break;
  }
  default:
    break;
  }
  return;
}

void Fsm::setFsmState(const FsmState &fsm_state) {
  if (fsm_state_ != fsm_state) {
    ROS_INFO("\033[32mState: %s => %s\033[32m",
             FsmStateToString(fsm_state_).c_str(),
             FsmStateToString(fsm_state).c_str());
    fsm_state_ = fsm_state;
  }
}

void Fsm::setReferenceType(const ReferenceType &ref_type) {
  if (ref_type_ != ref_type) {
    ROS_INFO("\033[34m Ref: %s => %s\033[34m",
             ReferenceTypeToString(ref_type_).c_str(),
             ReferenceTypeToString(ref_type).c_str());
    ref_type_ = ref_type;
  }
}

void Fsm::publishTrajTrigger() {
  geometry_msgs::PoseStamped trigger;
  trigger.header.stamp = ros::Time::now();
  trigger.pose.orientation.w = 1.0;
  traj_trigger_pub_.publish(trigger);
}

bool Fsm::fcStateFresh() const {
  return std::isfinite(fc_state_.stamp) && fc_state_.connected &&
         now_ >= fc_state_.stamp &&
         now_ - fc_state_.stamp <= fsm_param_->fc_state_timeout;
}

void Fsm::publishFlightTrajectoryStatus() {
  flygripper_msgs::FlightTrajectoryStatus status;
  status.header.stamp = ros::Time::now();
  const double publish_time = status.header.stamp.toSec();
  status.fcu_state_fresh = std::isfinite(fc_state_.stamp) && fc_state_.connected &&
      publish_time >= fc_state_.stamp &&
      publish_time - fc_state_.stamp <= fsm_param_->fc_state_timeout;
  status.fcu_armed = fc_state_.armed;
  status.fcu_guided_nogps = fc_state_.mode == "GUIDED_NOGPS";
  status.fsm_state = static_cast<uint8_t>(fsm_state_);
  status.reference_type = static_cast<uint8_t>(ref_type_);
  const auto& message = active_trajectory_message_;
  if (message) {
    status.trajectory_id = message->id;
    status.trajectory_start = message->header.stamp;
  }
  status.trajectory_active = message && trajectory_message_decoded_ &&
      trajectory_.id >= 0 && static_cast<uint32_t>(trajectory_.id) == message->id &&
      trajectory_.stamp == message->header.stamp.toSec() &&
      publish_time >= trajectory_.stamp &&
      publish_time <= trajectory_.stamp + trajectory_.duration &&
      status.fcu_state_fresh && status.fcu_armed && status.fcu_guided_nogps &&
      guided_entry_verified_ && !attitude_safety_handover_ &&
      fsm_state_ == FsmState::STATE_TRAJ && ref_type_ == ReferenceType::TYPE_TRAJ &&
      trajectory_sampled_this_cycle_ && trajectory_control_published_this_cycle_;

  const double steady_now = steadySeconds();
  const bool changed = !flight_status_initialized_ ||
      status.trajectory_active != last_flight_status_active_ ||
      (status.trajectory_active &&
       (status.trajectory_id != last_flight_status_id_ ||
        status.trajectory_start != last_flight_status_start_));
  if (!changed && steady_now - last_flight_status_publish_ < 0.05) return;
  flight_trajectory_status_pub_.publish(status);
  flight_status_initialized_ = true;
  last_flight_status_active_ = status.trajectory_active;
  last_flight_status_id_ = status.trajectory_id;
  last_flight_status_start_ = status.trajectory_start;
  last_flight_status_publish_ = steady_now;
}

bool Fsm::requestFcMode(const std::string& mode) {
  return mode_worker_->request(mode, mode == "GUIDED_NOGPS" &&
      robot_command_.rotor_mode == RobotCommandMode::MODE_ATTITUDE);
}

bool Fsm::prepareGuidedEntry() {
  guided_entry_verified_ = false;
  guided_entry_mode_ = fc_state_.mode;
  takeoff_requested_ = false;
  velocity_teleop_ready_ = false;
  if (!fcStateFresh() || !fc_state_.armed) {
    ROS_ERROR("Guided entry rejected: connect and arm the FCU with the transmitter first.");
    return false;
  }
  if (robot_state_.vel.norm() > kMaxVelocityToHoverMode) {
    ROS_ERROR("Guided entry rejected: vehicle speed is too high.");
    return false;
  }
  if (fc_state_.mode != "STABILIZE" && fc_state_.mode != "ALT_HOLD" &&
      fc_state_.mode != "LOITER" && fc_state_.mode != "POSHOLD" &&
      fc_state_.mode != "GUIDED_NOGPS") {
    ROS_ERROR_STREAM("Guided entry rejected from FCU mode " << fc_state_.mode);
    return false;
  }
  if (robot_command_.rotor_mode != RobotCommandMode::MODE_ATTITUDE &&
      robot_command_.rotor_mode != RobotCommandMode::MODE_SINGLE_ROTOR) {
    ROS_ERROR("Guided entry rejected: unsupported command_mode.");
    return false;
  }
  if (fsm_param_->auto_takeoff) {
    if (robot_command_.rotor_mode != RobotCommandMode::MODE_ATTITUDE ||
        !std::isfinite(extended_state_stamp_) || now_ < extended_state_stamp_ ||
        now_ - extended_state_stamp_ > fsm_param_->extended_state_timeout ||
        (extended_state_.landed_state != mavros_msgs::ExtendedState::LANDED_STATE_ON_GROUND &&
         extended_state_.landed_state != mavros_msgs::ExtendedState::LANDED_STATE_IN_AIR)) {
      ROS_ERROR("Auto takeoff requires attitude output and fresh, known /mavros/extended_state.");
      return false;
    }
    takeoff_requested_ = extended_state_.landed_state ==
        mavros_msgs::ExtendedState::LANDED_STATE_ON_GROUND;
    if (takeoff_requested_) {
      auto settings = fsm_param_->takeoff;
      settings.height = fsm_param_->takeoff_height;
      TakeoffPlan validation;
      if (robot_state_.vel.norm() > fsm_param_->takeoff_start_speed_max ||
          !validation.start({robot_state_.pos.x(), robot_state_.pos.y(), robot_state_.pos.z()},
                            robot_state_.getYaw(), now_, settings)) {
        ROS_ERROR("Takeoff rejected: excessive start speed, invalid profile limits or timeout.");
        return false;
      }
    }
  }
  // The worker verifies GUID_OPTIONS before sending GUIDED_NOGPS. Only a new
  // result may authorize this session; an earlier success must not be reused.
  const auto worker = mode_worker_->status();
  if (worker.busy || worker.pending) return false;
  attitude_safety_handover_ = false;
  guided_entry_completed_baseline_ = worker.completed;
  return true;
}

void Fsm::stopAutonomy() {
  abortPerching("FCU/pilot ended autonomous control");
  perching_recovery_at_ = -1.0;
  last_powered_throttle_ = NAN;
  last_powered_throttle_at_ = -1.0;
  perching_rescue_until_ = -1.0;
  perching_emergency_until_ = -1.0;
  mode_worker_->cancel();
  guided_entry_verified_ = false;
  takeoff_plan_.reset();
  takeoff_requested_ = false;
  rc_input_.resetGuidedModeTrigger();
  rc_input_.resetTrajModeTrigger();
  setReferenceType(ReferenceType::TYPE_HOVER);
  setFsmState(FsmState::STATE_MANUAL);
}

void Fsm::beginHandover(const std::string& reason) {
  // During deliberate unloading, waiting for a mode RPC/Guided timeout with
  // the old low/zero thrust is unsafe. Restore the last verified-flight output
  // immediately, even when a sensor just expired; pilot/FCU mode changes above
  // this path still win and never cause automatic re-arming.
  if (perchingActive() && perching_status_.phase != flygripper_msgs::PerchingStatus::RETREAT &&
      fc_state_.armed && fc_state_.mode == "GUIDED_NOGPS" &&
      guided_entry_verified_ && robot_command_.rotor_mode == RobotCommandMode::MODE_ATTITUDE) {
    // A stale /mavros/state must not leave our own zero-thrust command in force.
    // Use only the already verified session and finite cached flight output;
    // newly observed mode changes/disarming still take precedence above.
    perching_emergency_until_ = steadySeconds() +
        std::clamp(perching_config_.recovery_duration, 0.5, 2.0);
    publishPerchingThrust(perching_status_.saved_throttle);
    perching_rescue_until_ = steadySeconds() + perching_config_.recovery_duration;
  }
  abortPerching(reason);
  perching_recovery_at_ = -1.0;
  last_handover_reason_ = reason;
  if (fsm_state_ == FsmState::STATE_HANDOVER) return;
  const bool was_waiting = fsm_state_ == FsmState::STATE_WAIT_GUIDED;
  handover_from_wait_ = was_waiting;
  mode_worker_->cancel();
  ROS_ERROR_STREAM("Ending companion control: " << reason
                   << ". Requesting " << fsm_param_->handover_mode);
  takeoff_plan_.reset();
  takeoff_requested_ = false;
  rc_input_.resetGuidedModeTrigger();
  rc_input_.resetTrajModeTrigger();
  setReferenceType(ReferenceType::TYPE_HOVER);
  if (robot_state_.pos.allFinite() && robot_state_.att.allFinite())
    sampler_->getHoverReference(robot_state_, references_, now_,
                                robot_command_.rotor_mode, true);
  setFsmState(FsmState::STATE_HANDOVER);
  mode_request_start_ = now_;
  last_mode_request_ = now_;
  if ((fcStateFresh() || perching_emergency_until_ > steadySeconds()) && fc_state_.armed &&
      (fc_state_.mode == "GUIDED_NOGPS" || was_waiting)) {
    requestFcMode(fsm_param_->handover_mode);
  } else if (fcStateFresh()) {
    stopAutonomy();
  }
}

void Fsm::updateHandover() {
  if (!fcStateFresh()) {
    // A verified perching session may request takeover while heartbeat feedback
    // is missing. Bound retries to the same recovery window; never request GUIDED.
    if (perching_emergency_until_ > steadySeconds() && fc_state_.armed &&
        fc_state_.mode == "GUIDED_NOGPS" && now_ - last_mode_request_ >= 1.0) {
      last_mode_request_ = now_;
      requestFcMode(fsm_param_->handover_mode);
    }
    ROS_ERROR_THROTTLE(1.0, "FCU state unavailable during takeover; waiting for FCU failsafe/pilot.");
    return;
  }
  // An in-flight GUIDED RPC cannot be recalled. Keep cancellation handover
  // alive briefly so a late GUIDED confirmation is handed back to the FCU.
  if (fc_state_.armed && fc_state_.mode == guided_entry_mode_ && handover_from_wait_ &&
      (mode_worker_->status().busy || now_ - mode_request_start_ < fsm_param_->mode_switch_timeout)) return;
  if (!fc_state_.armed || fc_state_.mode != "GUIDED_NOGPS") {
    ROS_WARN_STREAM("Companion control stopped; FCU mode is " << fc_state_.mode);
    stopAutonomy();
    return;
  }
  if (now_ - last_mode_request_ >= 1.0 || now_ < last_mode_request_) {
    last_mode_request_ = now_;
    requestFcMode(fsm_param_->handover_mode);
  }
  if (now_ - mode_request_start_ > fsm_param_->mode_switch_timeout)
    ROS_ERROR_THROTTLE(1.0, "FCU takeover not confirmed. Pilot intervention required.");
}

void Fsm::publishRuntimeDiagnostics() {
  const double now = steadySeconds();
  if (now - last_diagnostic_ < 0.1) return;
  last_diagnostic_ = now;
  diagnostic_msgs::DiagnosticArray array;
  array.header.stamp = ros::Time::now();
  diagnostic_msgs::DiagnosticStatus status;
  status.name = ros::this_node::getName() + "/control_runtime";
  status.hardware_id = "companion";
  status.level = fault_reason_.empty() ? diagnostic_msgs::DiagnosticStatus::OK
                                      : diagnostic_msgs::DiagnosticStatus::ERROR;
  status.message = fault_reason_.empty() ? "Inputs valid" : fault_reason_;
  const auto add = [&status](const std::string& key, double value) {
    diagnostic_msgs::KeyValue pair;
    pair.key = key;
    pair.value = std::to_string(value);
    status.values.push_back(pair);
  };
  add("imu_header_age_ms", (now_ - imu_.stamp) * 1000.0);
  add("odom_header_age_ms", (now_ - odom_.stamp) * 1000.0);
  // Preserve existing keys, now with callback entry measured before any lock.
  add("imu_age_at_callback_ms", applied_.imu.message
      ? (applied_.imu.timing.callback_ros - applied_.imu.message->header.stamp.toSec()) * 1000.0 : -1.0);
  add("odom_age_at_callback_ms", applied_.odom.message
      ? (applied_.odom.timing.callback_ros - applied_.odom.message->header.stamp.toSec()) * 1000.0 : -1.0);
  add("imu_callback_gap_ms", imu_callback_gap_ * 1000.0);
  add("odom_callback_gap_ms", odom_callback_gap_ * 1000.0);
  // Sample the live buffers too: an old control snapshot must not be mistaken
  // for a stopped input worker. All maxima below are since node startup.
  const auto timing = [&add](const std::string& name, const auto& input) {
    const auto& t = input.timing;
    add(name + "_callbacks", input.callbacks);
    add(name + "_accepted_updates", input.count);
    add(name + "_rejected_updates", input.rejected_updates);
    add(name + "_sequence_gaps", input.sequence_gaps);
    add(name + "_live_callback_gap_ms", input.count
        ? (steadySeconds() - input.received_steady) * 1000.0 : -1.0);
    add(name + "_callback_max_gap_ms", input.max_gap * 1000.0);
    add(name + "_receipt_ros_s", t.receipt_ros);
    add(name + "_callback_ros_s", t.callback_ros);
    add(name + "_callback_steady_s", t.callback_steady);
    add(name + "_lock_acquired_steady_s", input.lock_acquired_steady);
    add(name + "_saved_steady_s", input.saved_steady);
    add(name + "_age_at_receipt_ms", input.message
        ? (t.receipt_ros - input.message->header.stamp.toSec()) * 1000.0 : -1.0);
    add(name + "_queue_delay_ms", input.queue_delay * 1000.0);
    add(name + "_queue_delay_max_ms", input.max_queue_delay * 1000.0);
    add(name + "_lock_wait_ms", input.lock_wait * 1000.0);
    add(name + "_lock_wait_max_ms", input.max_lock_wait * 1000.0);
    add(name + "_store_ms", input.store_duration * 1000.0);
    add(name + "_store_max_ms", input.max_store_duration * 1000.0);
  };
  const auto live = [&add, &timing](const std::string& name, const auto& buffer, const auto& applied) {
    const auto snapshot = buffer.trySnapshot();
    add(name + "_live_snapshot_busy", !snapshot);
    timing(name, snapshot ? *snapshot : applied);
  };
  live("imu", imu_input_, applied_.imu);
  live("odom", odom_input_, applied_.odom);
  live("rc", rc_input_buffer_, applied_.rc);
  live("fc", fc_input_, applied_.fc);
  live("trajectory", trajectory_input_, applied_.trajectory);
  add("imu_timeout_ms", fsm_param_->imu_timeout * 1000.0);
  add("odom_timeout_ms", fsm_param_->odom_timeout * 1000.0);
  add("control_period_ms", loop_period_ * 1000.0);
  add("control_period_max_ms", max_loop_period_ * 1000.0);
  add("input_snapshot_ms", input_copy_duration_ * 1000.0);
  add("input_lock_copy_ms", input_lock_copy_duration_ * 1000.0);
  add("input_decode_ms", input_decode_duration_ * 1000.0);
  add("input_snapshot_misses", input_snapshot_misses_);
  add("parameter_refresh_ms", parameter_duration_ * 1000.0);
  add("control_compute_ms", compute_duration_ * 1000.0);
  add("control_work_ms", control_duration_ * 1000.0);
  add("control_work_max_ms", max_control_duration_ * 1000.0);
  add("l1_enabled", static_cast<GeometricController*>(controller_.get())->l1Enabled());
  add("l1_initialization_count",
      static_cast<GeometricController*>(controller_.get())->l1InitializationCount());
  add("attitude_safety_handover", attitude_safety_handover_);
  const auto worker = mode_worker_->status();
  add("mode_request_busy", worker.busy);
  add("mode_request_pending", worker.pending);
  add("mode_request_elapsed_ms", worker.elapsed * 1000.0);
  add("mode_request_last_duration_ms", worker.last_duration * 1000.0);
  add("mode_request_completed", worker.completed);
  add("mode_request_ignored", worker.ignored);
  add("mode_request_success", worker.success);
  diagnostic_msgs::KeyValue mode;
  mode.key = "mode_request_active"; mode.value = worker.active_mode;
  status.values.push_back(mode);
  mode.key = "last_handover_reason"; mode.value = last_handover_reason_;
  status.values.push_back(mode);
  mode.key = "fsm_state"; mode.value = FsmStateToString(fsm_state_);
  status.values.push_back(mode);
  mode.key = "fc_mode"; mode.value = fc_state_.mode;
  status.values.push_back(mode);
  array.status.push_back(status);
  runtime_diagnostic_pub_.publish(array);
  max_loop_period_ = max_control_duration_ = 0.0;
}

}; // namespace flygripper_flight
