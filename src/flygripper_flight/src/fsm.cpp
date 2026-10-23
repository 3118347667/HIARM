#include "flygripper_flight/fsm.hpp"

#include "flygripper_common/type/const.hpp"
#include "flygripper_common/utils/realtime.hpp"
#include "flygripper_common/utils/runtime_profiler.hpp"
#include "flygripper_msgs/GraspState.h"
#include "mavros_msgs/SetMode.h"
#include "mavros_msgs/ParamGet.h"

namespace flygripper_flight {
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
  input_spinner_ = std::make_unique<ros::AsyncSpinner>(1, &update_queue_);
  control_spinner_ = std::make_unique<ros::AsyncSpinner>(1, &control_queue_);
  input_spinner_->start();
  ros::NodeHandle control_nh(nh);
  control_nh.setCallbackQueue(&control_queue_);
  loop_timer_ = control_nh.createTimer(ros::Duration(1.0 / fsm_param_->loop_frequency),
                               &Fsm::controlLoop, this);
  control_spinner_->start();
}

Fsm::~Fsm() {
  loop_timer_.stop();
  if (control_spinner_) control_spinner_->stop();
  if (input_spinner_) input_spinner_->stop();
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
  //! rc from mavros
  rc_options_ = ros::SubscribeOptions::create<mavros_msgs::RCIn>(
      "/mavros/rc/in", 10, boost::bind(&Fsm::rosRcCallback, this, _1),
      ros::VoidConstPtr(), &update_queue_);
  rc_options_.transport_hints = ros::TransportHints().tcpNoDelay();
  rc_sub_ = nh.subscribe(rc_options_);
  //! fc_state from mavros
  fc_state_options_ = ros::SubscribeOptions::create<mavros_msgs::State>(
      "/mavros/state", 10, boost::bind(&Fsm::rosFcStateCallback, this, _1),
      ros::VoidConstPtr(), &update_queue_);
  fc_state_options_.transport_hints = ros::TransportHints().tcpNoDelay();
  fc_state_sub_ = nh.subscribe(fc_state_options_);
  extended_state_options_ = ros::SubscribeOptions::create<mavros_msgs::ExtendedState>(
      "/mavros/extended_state", 10,
      boost::bind(&Fsm::rosExtendedStateCallback, this, _1),
      ros::VoidConstPtr(), &update_queue_);
  extended_state_sub_ = nh.subscribe(extended_state_options_);
  //! odom
  odom_options_ = ros::SubscribeOptions::create<nav_msgs::Odometry>(
      "odom", 2, boost::bind(&Fsm::rosOdomCallback, this, _1),
      ros::VoidConstPtr(), &update_queue_);
  odom_options_.transport_hints = ros::TransportHints().tcpNoDelay();
  odom_sub_ = nh.subscribe(odom_options_);
  //! imu
  imu_options_ = ros::SubscribeOptions::create<sensor_msgs::Imu>(
      "imu", 2, boost::bind(&Fsm::rosImuCallback, this, _1),
      ros::VoidConstPtr(), &update_queue_);
  imu_options_.transport_hints = ros::TransportHints().tcpNoDelay();
  imu_sub_ = nh.subscribe(imu_options_);
  //! actuator_state
  actuator_state_options_ =
      ros::SubscribeOptions::create<flygripper_msgs::ActuatorState>(
          "actuator_state", 10,
          boost::bind(&Fsm::rosActuatorStateCallback, this, _1),
          ros::VoidConstPtr(), &update_queue_);
  actuator_state_options_.transport_hints = ros::TransportHints().tcpNoDelay();
  actuator_state_sub_ = nh.subscribe(actuator_state_options_);
  //! battery from mavros
  battery_options_ = ros::SubscribeOptions::create<sensor_msgs::BatteryState>(
      "/mavros/battery", 10, boost::bind(&Fsm::rosBatteryCallback, this, _1),
      ros::VoidConstPtr(), &update_queue_);
  battery_options_.transport_hints = ros::TransportHints().tcpNoDelay();
  battery_sub_ = nh.subscribe(battery_options_);
  //! rotor_speed from mavros
  rotor_speed_options_ =
      ros::SubscribeOptions::create<mavros_msgs::ESCTelemetry>(
          "rotor_speed", 10, boost::bind(&Fsm::rosRotorSpeedCallback, this, _1),
          ros::VoidConstPtr(), &update_queue_);
  rotor_speed_options_.transport_hints = ros::TransportHints().tcpNoDelay();
  rotor_speed_sub_ = nh.subscribe(rotor_speed_options_);
  //! trajectory
  traj_options_ =
      ros::SubscribeOptions::create<flygripper_msgs::PolynomialTrajectory>(
          "trajectory", 10, boost::bind(&Fsm::rosTrajCallback, this, _1),
          ros::VoidConstPtr(), &update_queue_);
  traj_options_.transport_hints = ros::TransportHints().tcpNoDelay();
  traj_sub_ = nh.subscribe(traj_options_);

  runtime_diagnostic_pub_ = nh.advertise<diagnostic_msgs::DiagnosticArray>("runtime_diagnostics", 10);

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

void Fsm::rosOdomCallback(nav_msgs::OdometryConstPtr msg) {
  std::lock_guard<std::mutex> lock(input_mutex_);
  received_.odom.store(msg);
}

void Fsm::rosImuCallback(sensor_msgs::ImuConstPtr msg) {
  std::lock_guard<std::mutex> lock(input_mutex_);
  received_.imu.store(msg);
}

void Fsm::rosActuatorStateCallback(flygripper_msgs::ActuatorStateConstPtr msg) {
  std::lock_guard<std::mutex> lock(input_mutex_);
  received_.actuator.store(msg);
}

void Fsm::rosBatteryCallback(sensor_msgs::BatteryStateConstPtr msg) {
  std::lock_guard<std::mutex> lock(input_mutex_);
  received_.battery.store(msg);
}

void Fsm::rosRotorSpeedCallback(mavros_msgs::ESCTelemetryConstPtr msg) {
  std::lock_guard<std::mutex> lock(input_mutex_);
  received_.rotor.store(msg);
}

void Fsm::rosRcCallback(mavros_msgs::RCInConstPtr msg) {
  std::lock_guard<std::mutex> lock(input_mutex_);
  received_.rc.store(msg);
  if (rc_events_.size() >= 64) { rc_events_.pop_front(); rc_overflow_ = true; }
  rc_events_.push_back(received_.rc);
}

void Fsm::rosFcStateCallback(mavros_msgs::StateConstPtr msg) {
  std::lock_guard<std::mutex> lock(input_mutex_);
  received_.fc.store(msg);
}

void Fsm::rosExtendedStateCallback(mavros_msgs::ExtendedStateConstPtr msg) {
  std::lock_guard<std::mutex> lock(input_mutex_);
  received_.extended.store(msg);
}

void Fsm::rosTrajCallback(flygripper_msgs::PolynomialTrajectoryConstPtr msg) {
  std::lock_guard<std::mutex> lock(input_mutex_);
  received_.trajectory.store(msg);
}

void Fsm::consumeInputs() {
  const double started = steadySeconds();
  Inputs input;
  std::deque<InputSlot<mavros_msgs::RCIn>> rc_events;
  bool overflow;
  {
    std::lock_guard<std::mutex> lock(input_mutex_);
    input = received_;
    rc_events.swap(rc_events_);
    overflow = rc_overflow_;
    rc_overflow_ = false;
  }
  rc_snapshot_overflow_ = overflow;
  rc_release_seen_ = overflow;
  for (const auto& event : rc_events) {
    fromRosMsg(event.message, rc_input_, true);
    rc_input_.stamp = event.received_ros;
    rc_input_.update();
    if (!rc_input_.isGuidedMode()) rc_release_seen_ = true;
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
    fromRosMsg(input.trajectory.message, trajectory_);
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
}

void Fsm::controlLoop(const ros::TimerEvent &event) {
  compute_duration_ = 0.0;
  loop_started_ = steadySeconds();
  loop_period_ = last_loop_started_ > 0.0 ? loop_started_ - last_loop_started_ : 0.0;
  last_loop_started_ = loop_started_;
  max_loop_period_ = std::max(max_loop_period_, loop_period_);
  ScopeExit finish([this] {
    control_duration_ = steadySeconds() - loop_started_;
    max_control_duration_ = std::max(max_control_duration_, control_duration_);
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
    beginHandover("CH5 released or RC event queue overflow");
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

  //! Update FSM state and reference
  updateFsmStateAndReference();
  tic_toc_.toc("reference_update");

  robot_command_.stamp = now_;
  //! Controller preparation
  controller_->prepare(robot_state_);
  //! Rotor control process for guided and traj mode
  if ((fsm_state_ == FsmState::STATE_GUIDED ||
      fsm_state_ == FsmState::STATE_TRAJ ||
      fsm_state_ == FsmState::STATE_HANDOVER) &&
      guided_entry_verified_ && fc_state_.armed && fc_state_.mode == "GUIDED_NOGPS") {
    //! Controller execution
    static flygripper_common::RuntimeProfiler control_profiler(
        "trajectory_tracking_control", 1000);
    {
      flygripper_common::ScopedRuntimeSample profile_sample(control_profiler);
      const double started = steadySeconds();
      controller_->getCommand(robot_state_, references_, robot_command_, now_);
      compute_duration_ = steadySeconds() - started;
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
      ROS_ERROR_STREAM("Rotor command is not valid! \n" << robot_command_);
    } else {
      // [ROS] First publish rotor commands
      if (robot_command_.rotor_mode == RobotCommandMode::MODE_SINGLE_ROTOR) {
        for (int i = 0; i < rpm_maps_.size(); ++i) {
          robot_command_.rotor_velocities_command(i) = rpm_maps_[i].map(
              robot_command_.rotor_velocities(i), battery_.voltage);
        }

        toRosMsg(robot_command_, rotor_command_msg_);
        rotor_command_pub_.publish(rotor_command_msg_);
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
        robot_command_.collective_thrust_command = thrust_acc_map_.map(
            robot_command_.collective_thrust / robot_param_->mass);
        if (!std::isfinite(robot_command_.collective_thrust_command) ||
            !robot_command_.attitude_command.coeffs().allFinite()) {
          beginHandover("Non-finite attitude/thrust command");
          return;
        }
        robot_command_.collective_thrust_command =
            std::clamp(robot_command_.collective_thrust_command, 0.0, 1.0);
        toRosMsg(robot_command_, thrust_att_command_msg_);
        thrust_att_command_pub_.publish(thrust_att_command_msg_);
      } else {
        ROS_WARN("other mode is not implemented.");
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
  bool ret = !rc_snapshot_overflow_;
  fault_reason_ = rc_snapshot_overflow_ ? "RC event queue overflow; " : "";
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
    ROS_WARN("IMU data invalid!");
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
    ROS_WARN("Odometry data invalid!");
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
    ROS_WARN("RC In data invalid!");
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
    ROS_WARN("Battery data invalid!");
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
    } else if (!rc_input_.isTrajMode()) {
      //! [STATE] TRAJ => GUIDED by rc
      setFsmState(FsmState::STATE_GUIDED);
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
      };
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
  guided_entry_completed_baseline_ = worker.completed;
  return true;
}

void Fsm::stopAutonomy() {
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
  if (fcStateFresh() && fc_state_.armed &&
      (fc_state_.mode == "GUIDED_NOGPS" || was_waiting)) {
    requestFcMode(fsm_param_->handover_mode);
  } else if (fcStateFresh()) {
    stopAutonomy();
  }
}

void Fsm::updateHandover() {
  if (!fcStateFresh()) {
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
  // Age already present at callback entry, including timestamp mapping errors.
  add("imu_age_at_callback_ms", applied_.imu.message
      ? (applied_.imu.received_ros - applied_.imu.message->header.stamp.toSec()) * 1000.0 : -1.0);
  add("odom_age_at_callback_ms", applied_.odom.message
      ? (applied_.odom.received_ros - applied_.odom.message->header.stamp.toSec()) * 1000.0 : -1.0);
  add("imu_callback_gap_ms", imu_callback_gap_ * 1000.0);
  add("odom_callback_gap_ms", odom_callback_gap_ * 1000.0);
  add("imu_callback_max_gap_ms", applied_.imu.max_gap * 1000.0);
  add("odom_callback_max_gap_ms", applied_.odom.max_gap * 1000.0);
  // Also expose the live receive thread, independently of the control snapshot.
  double live_imu, live_odom;
  {
    std::lock_guard<std::mutex> lock(input_mutex_);
    const double sampled = steadySeconds();
    live_imu = received_.imu.count ? (sampled - received_.imu.received_steady) * 1000.0 : -1.0;
    live_odom = received_.odom.count ? (sampled - received_.odom.received_steady) * 1000.0 : -1.0;
  }
  add("imu_live_callback_gap_ms", live_imu);
  add("odom_live_callback_gap_ms", live_odom);
  add("imu_callbacks", applied_.imu.count);
  add("odom_callbacks", applied_.odom.count);
  add("imu_sequence_gaps", applied_.imu.sequence_gaps);
  add("odom_sequence_gaps", applied_.odom.sequence_gaps);
  add("imu_timeout_ms", fsm_param_->imu_timeout * 1000.0);
  add("odom_timeout_ms", fsm_param_->odom_timeout * 1000.0);
  add("control_period_ms", loop_period_ * 1000.0);
  add("control_period_max_ms", max_loop_period_ * 1000.0);
  add("input_snapshot_ms", input_copy_duration_ * 1000.0);
  add("parameter_refresh_ms", parameter_duration_ * 1000.0);
  add("control_compute_ms", compute_duration_ * 1000.0);
  add("control_work_ms", control_duration_ * 1000.0);
  add("control_work_max_ms", max_control_duration_ * 1000.0);
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
