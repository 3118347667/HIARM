#include "flygripper_flight/fsm.hpp"

#include "flygripper_common/type/const.hpp"
#include "flygripper_common/utils/realtime.hpp"
#include "mavros_msgs/SetMode.h"

namespace flygripper_flight{
Fsm::Fsm(ros::NodeHandle &nh, ros::NodeHandle& pnh){
  //! fsm parameter
  if (!fsm_param_config_.init(pnh)){
    throw ros::Exception("Failed to initialize parameter config");
  }
  fsm_param_ = std::make_shared<FsmParameter>();
  fsm_param_config_.getParams(*fsm_param_);

  //! robot parameter
  ros::NodeHandle robot_param_nh(nh, "robot_parameter");
  if (!robot_param_config_.init(robot_param_nh)){
    throw ros::Exception("Failed to initialize robot parameter config");
  }
  robot_param_ = std::make_shared<RobotParameter>();
  robot_param_config_.getParams(*robot_param_);

  //! rpm mapping
  rpm_maps_.resize(4);
  for (int i = 0; i < rpm_maps_.size(); ++i) {
    if (robot_param_->dshot_rpm_map_method == 0){
      if(!rpm_maps_[i].init_simple_model(robot_param_->dshot_rpm_map_coeff)){
        throw ros::Exception("Failed to initialize dshot_rpm_map_coeff config");
      }
    }
    else if (robot_param_->dshot_rpm_map_method == 1){
      if(!rpm_maps_[i].init_precise_model(robot_param_->dshot_rpm_volt_map_coeff)){
        throw ros::Exception("Failed to initialize dshot_rpm_volt_map_coeff config");
      }
    }
    else if (robot_param_->dshot_rpm_map_method == 2){
      std::filesystem::path rpm_map_file =
          std::filesystem::path(robot_param_->dshot_rpm_volt_map_file)/
              std::filesystem::path("rpm"+std::to_string(i)+"_map.csv");
      if (!rpm_maps_[i].init_precise_model(rpm_map_file)){
        throw ros::Exception("Failed to initialize dshot_rpm_volt_map_file config");
      }
    }
  }

  //! thrust acc mapping
  if (!thrust_acc_map_.init_simple_model(robot_param_->hover_percentage)){
    throw ros::Exception("Failed to initialize thrust_acc_map_coeff config");
  }

  //! external wrench estimator
  ext_wrench_est_ = std::make_shared<ExternalWrenchEstimator<4>>(fsm_param_->external_wrench_buffer_size,
                                                                 fsm_param_->external_wrench_cutoff_frequency,
                                                                 fsm_param_->loop_frequency);
  external_wrench_.data = Eigen::Vector4d::Zero();

  //! default state and reference type
  fsm_state_ = FsmState::STATE_MANUAL;
  ref_type_ = fsm_param_->auto_takeoff ? ReferenceType::TYPE_TAKEOFF : ReferenceType::TYPE_HOVER;
  ROS_INFO("\033[32mInitial State: %s, Type: %s.\033[32m",
           FsmStateToString(fsm_state_).c_str(),
           ReferenceTypeToString(ref_type_).c_str());

  //! ros setup of subscriber, publisher, etc
  rosSetup(nh);

  //! initialize sampler and controller
  sampler_ = std::make_shared<Sampler>(robot_param_, fsm_param_);
  controller_ = std::make_shared<GeometricController>(robot_param_,
                                                      fsm_param_->loop_frequency);
}

Fsm::~Fsm(){
  publishZeroCommand();
};

void Fsm::publishZeroCommand(){
  ROS_WARN("Start Shutting down");
  robot_command_.setZero();
  if (robot_command_.rotor_mode == RobotCommandMode::MODE_SINGLE_ROTOR){
    for (int i = 0; i < rpm_maps_.size(); ++i) {
      robot_command_.rotor_velocities_command(i) = rpm_maps_[i].map(robot_command_.rotor_velocities(i),
                                                                    battery_.voltage);
    }
    toRosMsg(robot_command_, rotor_command_msg_);
    rotor_command_pub_.publish(rotor_command_msg_);
  }
//  else if (robot_command_.rotor_mode == RobotCommandMode::MODE_BODY_RATE_CUSTOMIZED){
//    if (actuator_state_.isReceived(now_, fsm_param_->actuator_state_timeout) && actuator_state_.valid()){
//      toRosMsg(robot_command_,
//               -actuator_state_.position*0.005, //TODO: mapping from servo rotate rad to morphing delta length
//               rotor_command_msg_);
//    } else {
//      toRosMsg(robot_command_,
//               0.0,
//               rotor_command_msg_);
//    }
//    rotor_command_pub_.publish(rotor_command_msg_);
//  }
  else if (robot_command_.rotor_mode == RobotCommandMode::MODE_ATTITUDE){
    robot_command_.collective_thrust_command = thrust_acc_map_.map(robot_command_.collective_thrust/robot_param_->mass);
    toRosMsg(robot_command_, thrust_att_command_msg_);
    thrust_att_command_pub_.publish(thrust_att_command_msg_);
  }else {
    ROS_WARN("other mode is not implemented.");
  }
  ROS_WARN("Shutting down completed");
}

void Fsm::rosSetup(ros::NodeHandle &nh){
  //! rc from mavros
  rc_options_ =
      ros::SubscribeOptions::create<mavros_msgs::RCIn>(
          "/mavros/rc/in",
          10,
          boost::bind(&Fsm::rosRcCallback, this, _1),
          ros::VoidConstPtr(),
          &update_queue_);
  rc_options_.transport_hints = ros::TransportHints().tcpNoDelay();
  rc_sub_ = nh.subscribe(rc_options_);
  //! fc_state from mavros
  fc_state_options_ =
      ros::SubscribeOptions::create<mavros_msgs::State>(
          "/mavros/state",
          10,
          boost::bind(&Fsm::rosFcStateCallback, this, _1),
          ros::VoidConstPtr(),
          &update_queue_);
  fc_state_options_.transport_hints = ros::TransportHints().tcpNoDelay();
  fc_state_sub_ = nh.subscribe(fc_state_options_);
  //! odom
  odom_options_ =
      ros::SubscribeOptions::create<nav_msgs::Odometry>(
          "odom",
          10,
          boost::bind(&Fsm::rosOdomCallback, this, _1),
          ros::VoidConstPtr(),
          &update_queue_);
  odom_options_.transport_hints = ros::TransportHints().tcpNoDelay();
  odom_sub_ = nh.subscribe(odom_options_);
  //! imu
  imu_options_ =
      ros::SubscribeOptions::create<sensor_msgs::Imu>(
          "imu",
          10,
          boost::bind(&Fsm::rosImuCallback, this, _1),
          ros::VoidConstPtr(),
          &update_queue_);
  imu_options_.transport_hints = ros::TransportHints().tcpNoDelay();
  imu_sub_ = nh.subscribe(imu_options_);
  //! actuator_state
  actuator_state_options_ =
      ros::SubscribeOptions::create<flygripper_msgs::ActuatorState>(
          "actuator_state",
          10,
          boost::bind(&Fsm::rosActuatorStateCallback, this, _1),
          ros::VoidConstPtr(),
          &update_queue_);
  actuator_state_options_.transport_hints = ros::TransportHints().tcpNoDelay();
  actuator_state_sub_ = nh.subscribe(actuator_state_options_);
  //! battery from mavros
  battery_options_ =
      ros::SubscribeOptions::create<sensor_msgs::BatteryState>(
          "/mavros/battery",
          10,
          boost::bind(&Fsm::rosBatteryCallback, this, _1),
          ros::VoidConstPtr(),
          &update_queue_);
  battery_options_.transport_hints = ros::TransportHints().tcpNoDelay();
  battery_sub_ = nh.subscribe(battery_options_);
  //! rotor_speed from mavros
  rotor_speed_options_ =
      ros::SubscribeOptions::create<mavros_msgs::ESCTelemetry>(
          "rotor_speed",
          10,
          boost::bind(&Fsm::rosRotorSpeedCallback, this, _1),
          ros::VoidConstPtr(),
          &update_queue_);
  rotor_speed_options_.transport_hints = ros::TransportHints().tcpNoDelay();
  rotor_speed_sub_ = nh.subscribe(rotor_speed_options_);
  //! trajectory
  traj_options_ =
      ros::SubscribeOptions::create<flygripper_msgs::PolynomialTrajectory>(
          "trajectory",
          10,
          boost::bind(&Fsm::rosTrajCallback, this, _1),
          ros::VoidConstPtr(),
          &update_queue_);
  traj_options_.transport_hints = ros::TransportHints().tcpNoDelay();
  traj_sub_ = nh.subscribe(traj_options_);

  // robot command
  rotor_command_pub_ = nh.advertise<mavros_msgs::ActuatorControl>("rotor_command", 10);
  actuator_command_pub_ = nh.advertise<flygripper_msgs::ActuatorCommand>("actuator_command", 10);
  thrust_att_command_pub_ = nh.advertise<mavros_msgs::AttitudeTarget>("/mavros/setpoint_raw/attitude", 10);
  traj_trigger_pub_ = nh.advertise<geometry_msgs::PoseStamped>("trigger",10);
  reference_pub_ = nh.advertise<nav_msgs::Path>("reference", 10);
  ext_indi_pub_ = nh.advertise<geometry_msgs::WrenchStamped>("external_wrench_indi", 10);
  ext_l1_pub_ = nh.advertise<geometry_msgs::WrenchStamped>("external_wrench_l1", 10);
  interactive_wrench_pub_ = nh.advertise<geometry_msgs::WrenchStamped>("interactive_wrench", 10);
  robot_state_pub_ = nh.advertise<flygripper_msgs::RobotState>("robot_state", 10);
  ctrl_state_pub_ = nh.advertise<flygripper_msgs::GeometricControllerData>("geometric_controller/data", 10);
  mavros_setmode_srv_ = nh.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");

  loop_timer_ = nh.createTimer(ros::Duration(1.0/fsm_param_->loop_frequency),
                               &Fsm::controlLoop, this);
}

void Fsm::rosOdomCallback(nav_msgs::OdometryConstPtr msg){
  fromRosMsg(msg, odom_ );
}

void Fsm::rosImuCallback(sensor_msgs::ImuConstPtr msg){
  fromRosMsg(msg, imu_ );
}

void Fsm::rosActuatorStateCallback(flygripper_msgs::ActuatorStateConstPtr msg){
  fromRosMsg(msg, actuator_state_);
}

void Fsm::rosBatteryCallback(sensor_msgs::BatteryStateConstPtr msg){
  fromRosMsg(msg, battery_);
}

void Fsm::rosRotorSpeedCallback(mavros_msgs::ESCTelemetryConstPtr msg){
  fromRosMsg(msg, rotor_speed_);
}

void Fsm::rosRcCallback(mavros_msgs::RCInConstPtr msg){
  fromRosMsg(msg, rc_input_, true);
  rc_input_.update(); // update mode trigger
}

void Fsm::rosFcStateCallback(mavros_msgs::StateConstPtr msg){
  fromRosMsg(msg, fc_state_);
}

void Fsm::rosTrajCallback(flygripper_msgs::PolynomialTrajectoryConstPtr msg){
  fromRosMsg(msg, trajectory_);
}

void Fsm::controlLoop(const ros::TimerEvent& event){
  checkCycleTime(1.0 / fsm_param_->loop_frequency,
                 fsm_param_->loop_time_error_threshold,
                 dt_);
  tic_toc_.tic();
  now_ = event.current_real.toSec();
  //! [ROS] Update all ros callback functions to update data and parameters.
  update_queue_.callAvailable();
  updateParameter();
  //    ROS_INFO_STREAM("Param: "<< *robot_param_);
  robot_command_.rotor_mode = fsm_param_->command_mode;
  //! Update robot_state, fc_state and rc_input
  if (!updateRobotState()) {
    return;
  }
  tic_toc_.toc("ros_update");

  //! Update FSM state and reference
  updateFsmStateAndReference();
  tic_toc_.toc("reference_update");

  robot_command_.stamp = now_;
  //! Controller preparation
  controller_->prepare(robot_state_);
  //! Rotor control process for guided and traj mode
  if (fsm_state_ == FsmState::STATE_GUIDED || fsm_state_ == FsmState::STATE_TRAJ) {
    //! Controller execution
    controller_->getCommand(robot_state_, references_, robot_command_, now_);
    tic_toc_.toc("control");

    //! External Force Estimation
    if(fsm_param_->admittance_mode == AdmittanceMode::DISABLE){
      ext_wrench_est_->reset();
    } else{
      // get external force from L1 estimator and torque from indi
      Vector<3> external_force = dynamic_cast<GeometricController*>(controller_.get())
          ->getExternalWrench("l1").head(3);
      double external_yaw_torque = dynamic_cast<GeometricController*>(controller_.get())
                                                ->getExternalWrench("indi")(5);
      // assign external force xyz and torque yaw
      external_wrench_.data << external_force, external_yaw_torque;
      // update to estimator real interactive wrench
      // if external wrench is smaller than threshold, set interactive wrench to ZERO
      // else remove the internal wrench from external wrench to get real interactive wrench
      if (!ext_wrench_est_->update(external_wrench_.data,
                                   external_wrench_.data,
                                   fsm_param_->external_force_threshold,
                                   fsm_param_->external_torque_threshold,
                                   fsm_param_->external_wrench_trigger_duration)){
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
      if (robot_command_.rotor_mode == RobotCommandMode::MODE_SINGLE_ROTOR){
        for (int i = 0; i < rpm_maps_.size(); ++i) {
          robot_command_.rotor_velocities_command(i) = rpm_maps_[i].map(robot_command_.rotor_velocities(i),
                                                                        battery_.voltage);
        }

        toRosMsg(robot_command_, rotor_command_msg_);
        rotor_command_pub_.publish(rotor_command_msg_);
      }
//      else if (robot_command_.rotor_mode == RobotCommandMode::MODE_BODY_RATE_CUSTOMIZED){
//        if (actuator_state_.isReceived(now_, fsm_param_->actuator_state_timeout) && actuator_state_.valid()){
//          toRosMsg(robot_command_,
//                   -actuator_state_.position*0.005, //TODO: mapping from servo rotate rad to morphing delta length
//                   rotor_command_msg_);
//        } else {
//          toRosMsg(robot_command_,
//                   0.0,
//                   rotor_command_msg_);
//        }
//        rotor_command_pub_.publish(rotor_command_msg_);
//      }
      else if (robot_command_.rotor_mode == RobotCommandMode::MODE_ATTITUDE){
        robot_command_.attitude_command = imu_.att * odom_.att.inverse() * robot_command_.attitude;  // Align with FCU Frame
        robot_command_.collective_thrust_command = thrust_acc_map_.map(robot_command_.collective_thrust/robot_param_->mass);
        toRosMsg(robot_command_, thrust_att_command_msg_);
        thrust_att_command_pub_.publish(thrust_att_command_msg_);
      }else {
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
          dynamic_cast<GeometricController*>(controller_.get())
              ->getControllerMsg());
      ext_indi_pub_.publish(
          dynamic_cast<GeometricController*>(controller_.get())
              ->getExternalWrenchMsg("indi"));
      ext_l1_pub_.publish(
          dynamic_cast<GeometricController*>(controller_.get())
              ->getExternalWrenchMsg("l1"));
      tic_toc_.toc("control_publish");
    }
  }

  tic_toc_.toc("morph_publish");
  ROS_INFO_THROTTLE(10.0,
                    "\n------------- Timing ------------- \n%s",
                    tic_toc_.analyze({"ros_update",
                                      "reference_update",
                                      "control",
                                      "control_publish",
                                      "morph_publish"}).c_str());
}

void Fsm::updateParameter(){
  // [ROS] Update dynamic parameters
  fsm_param_config_.getParams(*fsm_param_);
  robot_param_config_.getParams(*robot_param_);
}

bool Fsm::updateRobotState(){
  static double last_imu_stamp = -1.0;
  bool ret = true;
  if(!imu_.isReceived(now_, fsm_param_->imu_timeout)){
    ret = false;
  } else if(!imu_.valid()){
    ret = false;
    ROS_WARN("IMU data invalid!");
  }

  if(!odom_.isReceived(now_, fsm_param_->odom_timeout)){
    ret = false;
  }
  else if(!odom_.valid()){
    ret = false;
    ROS_WARN("Odometry data invalid!");
  }

  if (!rotor_speed_.isReceived(now_, fsm_param_->rotor_speed_timeout)){
    ret = false;
  }
  else if(!rotor_speed_.valid()){
    ret = false;
    ROS_WARN("ESC telemetry data invalid!");
  }

  if (ret){ // update robot_state from imu, odom, rotor_speed and actuator_state(x)
    robot_state_.stamp = now_;
    // from odom
    robot_state_.pos = odom_.pos;
    robot_state_.setQ(odom_.att);
    robot_state_.vel = odom_.vel;
    // from imu
    if (last_imu_stamp < 0){
      robot_state_.alpha = Vector<3>::Zero();
    } else{
      robot_state_.alpha = 1/(imu_.stamp-last_imu_stamp) * (imu_.omega - robot_state_.omega);  // body frame
    }
    last_imu_stamp = imu_.stamp;
    robot_state_.omega = imu_.omega;  // body frame
    robot_state_.acc = odom_.att * imu_.acc+ kGravityVector; // world frame
    // from rotor speed
    robot_state_.rot_speed = rotor_speed_.rot_speed;

    if(actuator_state_.isReceived(now_, fsm_param_->actuator_state_timeout) && actuator_state_.valid()){
      // TODO： given by actuator state real time
      Scalar l = 0.21/ std::sqrt(2.0)/2.0;
      robot_state_.rot1_pos << l, -l;
      robot_state_.rot2_pos << -l, l;
      robot_state_.rot3_pos << l, l;
      robot_state_.rot4_pos << -l, -l;
    }
    else {
      // given by initial value from parameters
      robot_state_.rot1_pos << robot_param_->rotor_position.col(0);
      robot_state_.rot2_pos << robot_param_->rotor_position.col(1);
      robot_state_.rot3_pos << robot_param_->rotor_position.col(2);
      robot_state_.rot4_pos << robot_param_->rotor_position.col(3);
    }
  }

  if(!rc_input_.isReceived(now_, fsm_param_->rc_input_timeout)){
    ret = false;
  }
  else if(!rc_input_.valid()){
    ret = false;
    ROS_WARN("RC In data invalid!");
  } else {
    velocity_control_.des_vel.x() = rc_input_.channels[AXIS_PITCH]    * fsm_param_->velocity_scale_x;
    velocity_control_.des_vel.y() = rc_input_.channels[AXIS_ROLL]     * fsm_param_->velocity_scale_y;
    velocity_control_.des_vel.z() = rc_input_.channels[AXIS_THROTTLE] * fsm_param_->velocity_scale_z;
    velocity_control_.des_yaw_rate = rc_input_.channels[AXIS_YAW]     * fsm_param_->velocity_scale_yaw;
  }

  if (!battery_.isReceived(now_, fsm_param_->battery_timeout)){
    ret = false;
  }
  else if(!battery_.valid()){
    ret = false;
    ROS_WARN("Battery data invalid!");
  }
  ret &= fc_state_.isReceived(now_, fsm_param_->fc_state_timeout);
  return ret;
}

void Fsm::updateFsmStateAndReference(){
  switch (fsm_state_) {
    case FsmState::STATE_MANUAL:
    {
      //! [STATE] MANUAL => ...
      if (rc_input_.isGuidedModeTriggered()){
        rc_input_.resetGuidedModeTrigger();
        // Check if robot velocity is too large
        if (robot_state_.vel.norm() > kMaxVelocityToHoverMode){
          ROS_ERROR("REJECT MANUAL => GUIDED: Robot velocity %f is too large!", robot_state_.vel.norm());
          break;
        }
        // Set fc guided mode via mavros
        if(!setFcGuidedMode(true)){
          break;
        }
        //! [STATE] MANUAL -> GUIDED triggered by rc
        setFsmState(FsmState::STATE_GUIDED);
        //! [TYPE] ANY -> TAKEOFF/HOVER
        setReferenceType(fsm_param_->auto_takeoff ? ReferenceType::TYPE_TAKEOFF : ReferenceType::TYPE_HOVER);
        //! NOTE: assign the FIRST reference state from robot_state, DO NOT CHANGE
        sampler_->getHoverReference(robot_state_, references_, now_, robot_command_.rotor_mode, true);
      }
      break;
    }
    case FsmState::STATE_GUIDED:
    {
      //! [STATE] GUIDED => ...
      if (!rc_input_.isGuidedMode()){
        // Set fc manual mode via mavros
        if(!setFcGuidedMode(false)){
          break;
        }
        //! [STATE] GUIDED => MANUAL by rc
        setFsmState(FsmState::STATE_MANUAL);
        break;
      }
      else if (rc_input_.isTrajModeTriggered()){
        rc_input_.resetTrajModeTrigger();
        // Double check if fc state is set to guided
        if (!fc_state_.guided){
          ROS_ERROR("Failure in GUIDED => TRAJ: FC state is not guided!");
          break;
        }
        //! [STATE] GUIDED => TRAJ by rc
        setFsmState(FsmState::STATE_TRAJ);
        if (fsm_param_->traj_trigger){
          publishTrajTrigger();
        }
        //! [TYPE] ANY => HOVER while switching from STATE GUIDED to TRAJ
        setReferenceType(ReferenceType::TYPE_HOVER);
        //! NOTE: use the previous reference state instead of robot_state 
        //! to avoid reference jump from STATE GUIDED to TRAJ cause of constant error of quadrotor.
        sampler_->getHoverReference(references_.front().state, references_, now_, robot_command_.rotor_mode, true);
        break;
      }

      //! [STATE] GUIDED
      //-------------------------------------- ReferenceType::TYPE_TAKEOFF ----------------------------------------
      if (ref_type_ == ReferenceType::TYPE_TAKEOFF){
        if (robot_state_.pos.z() > fsm_param_->takeoff_height - 0.02){
          //! [TYPE] TAKEOFF => HOVER
          setReferenceType(ReferenceType::TYPE_HOVER);
          //! NOTE: use the previous reference state instead of robot_state 
          //! to avoid reference jump from TYPE TAKEOFF to HOVER cause of constant error of quadrotor.
          sampler_->getHoverReference(references_.front().state, references_, now_, robot_command_.rotor_mode, true);
        } else{
          //! [TYPE] TAKEOFF;
          sampler_->getTakeoffReference(robot_state_, references_,
                                        now_, robot_command_.rotor_mode,
                                        fsm_param_->takeoff_height);
        }
      }
      //-------------------------------------- ReferenceType::TYPE_VEL ----------------------------------------
      else if (fsm_param_->velocity_teleop &&
          (velocity_control_.des_vel.norm() > kVelocityDeadZone ||
              std::abs(velocity_control_.des_yaw_rate) > kVelocityDeadZone)){
        //! [TYPE] ANY => VEL
        if (ref_type_ != ReferenceType::TYPE_VEL){
          setReferenceType(ReferenceType::TYPE_VEL);
          //! NOTE: use the previous reference state instead of robot_state 
          //! to avoid reference jump from TYPE HOVER to VEL cause of static error of quadrotor.
          sampler_->getVelocityReference(references_.front().state,
                                         velocity_control_,
                                         references_, now_, robot_command_.rotor_mode,
                                         true, fsm_param_->world_frame_velocity);
        } else{
          //! [TYPE] VEL
          sampler_->getVelocityReference(robot_state_,
                                         velocity_control_,
                                         references_, now_, robot_command_.rotor_mode,
                                         false, fsm_param_->world_frame_velocity);
        }
      }
      else if(ref_type_ == ReferenceType::TYPE_VEL) {
        //! [TYPE] VEL => HOVER
        setReferenceType(ReferenceType::TYPE_HOVER);
        //! NOTE: use the previous reference state instead of robot_state 
        //! to avoid reference jump from TYPE VEL to HOVER cause of static error of quadrotor.
        sampler_->getHoverReference(references_.front().state, references_, now_, robot_command_.rotor_mode, true);
      }
      //-------------------------------------- ReferenceType::TYPE_HOVER ----------------------------------------

      else if(ref_type_ != ReferenceType::TYPE_HOVER) {
        ROS_WARN("ATTENTION: SHOULD NOT CALL THIS BRANCH IF YOUR FSM IS WELL BUILT, FORCE IT TO BE TYPE HOVER");
        setReferenceType(ReferenceType::TYPE_HOVER);
        //! NOTE: use the previous reference state instead of robot_state
        //! to avoid reference jump from TYPE HOVER to ADMITTANCE cause of constant error of quadrotor.
        sampler_->getHoverReference(references_.front().state, references_, now_, robot_command_.rotor_mode, true);
      }
      else{
        //! [TYPE] HOVER;
        sampler_->getHoverReference(robot_state_, references_, now_, robot_command_.rotor_mode);
      }

      break;
    }
    case FsmState::STATE_TRAJ:
    {
      //! [STATE] TRAJ => ...
      if (!rc_input_.isGuidedMode()) {
        // Set fc manual mode via mavros
        if(!setFcGuidedMode(false)){
          break;
        }
        //! [STATE] TRAJ => MANUAL by rc
        setFsmState(FsmState::STATE_MANUAL);
        break;
      }
      else if (!rc_input_.isTrajMode()){
        //! [STATE] TRAJ => GUIDED by rc
        setFsmState(FsmState::STATE_GUIDED);
        //! [TYPE] ANY => HOVER while switching from STATE TRAJ to GUIDED
        setReferenceType(ReferenceType::TYPE_HOVER);
        //! NOTE: use the previous reference state instead of robot_state 
        //! to avoid reference jump from STATE TRAJ to GUIDED cause of constant error of quadrotor.
        sampler_->getHoverReference(references_.front().state, references_, now_, robot_command_.rotor_mode, true);
        break;
      }

      //! [STATE] TRAJ
      //-------------------------------------- ReferenceType::TYPE_TRAJ ----------------------------------------
      if (trajectory_.isReceived(now_)
          && trajectory_.valid()
          && trajectory_.available(now_)){
        //! [TYPE] ANY => TRAJ
        if (ref_type_ != ReferenceType::TYPE_TRAJ){
          setReferenceType(ReferenceType::TYPE_TRAJ);
        }
        //! [TYPE] TRAJ
        if (!sampler_->getTrajectoryReference(robot_state_, trajectory_, references_, now_,
                                              robot_command_.rotor_mode)){
          //! [TYPE] TRAJ => HOVER
          setReferenceType(ReferenceType::TYPE_HOVER);
          //! NOTE: Force switching to hovering mode with current robot_state
          //! because of trajectory problem
          //!       i.e. tracking error is too large //TODO: better to have
          sampler_->getHoverReference(robot_state_, references_, now_, robot_command_.rotor_mode, true);
        };
      }
      else if (ref_type_ == ReferenceType::TYPE_TRAJ){
        //! [TYPE] TRAJ => HOVER
        setReferenceType(ReferenceType::TYPE_HOVER);
        //! NOTE: use the previous reference state instead of robot_state
        //! to avoid reference jump from TYPE TRAJ to HOVER cause of constant error of quadrotor.
        sampler_->getHoverReference(references_.front().state, references_, now_, robot_command_.rotor_mode, true);
      }

      //-------------------------------------- ReferenceType::TYPE_VEL ----------------------------------------
//      else if (fsm_param_->velocity_teleop &&
//               (velocity_control_.des_vel.norm() > kVelocityDeadZone ||
//                std::abs(velocity_control_.des_yaw_rate) > kVelocityDeadZone)){
//        //! [TYPE] ANY => VEL
//        if (ref_type_ != ReferenceType::TYPE_VEL){
//          setReferenceType(ReferenceType::TYPE_VEL);
//          //! NOTE: use the previous reference state instead of robot_state
//          //! to avoid reference jump from TYPE HOVER to VEL cause of static error of quadrotor.
//          sampler_->getVelocityReference(references_.front().state,
//                                         velocity_control_,
//                                         references_, now_, robot_command_.rotor_mode,
//                                         true, fsm_param_->world_frame_velocity);
//        } else{
//          //! [TYPE] VEL
//          sampler_->getVelocityReference(robot_state_,
//                                         velocity_control_,
//                                         references_, now_, robot_command_.rotor_mode,
//                                         false, fsm_param_->world_frame_velocity);
//        }
//      }
//      else if(ref_type_ == ReferenceType::TYPE_VEL) {
//        //! [TYPE] VEL => HOVER
//        setReferenceType(ReferenceType::TYPE_HOVER);
//        //! NOTE: use the previous reference state instead of robot_state
//        //! to avoid reference jump from TYPE VEL to HOVER cause of static error of quadrotor.
//        sampler_->getHoverReference(references_.front().state, references_, now_, robot_command_.rotor_mode, true);
//      }

      //-------------------------------------- TYPE_ADMITTANCE ----------------------------------------
      else if (fsm_param_->admittance_mode != AdmittanceMode::DISABLE && !external_wrench_.data.isApproxToConstant(0.0)){
        //! [TYPE] ANY => ADMITTANCE
        // rebuild the external wrench for admittance system
        external_wrench_.ext_force.x() = fsm_param_->enable_admittance_x ? external_wrench_.ext_force.x() : 0.0;
        external_wrench_.ext_force.y() = fsm_param_->enable_admittance_y ? external_wrench_.ext_force.y() : 0.0;
        external_wrench_.ext_force.z() = fsm_param_->enable_admittance_z ? external_wrench_.ext_force.z() : 0.0;
        external_wrench_.ext_torque_yaw = fsm_param_->enable_admittance_yaw ? external_wrench_.ext_torque_yaw : 0.0;

        if (ref_type_ != ReferenceType::TYPE_ADMITTANCE){
          setReferenceType(ReferenceType::TYPE_ADMITTANCE);
          //! NOTE: use the previous reference state instead of robot_state
          //! to avoid reference jump from TYPE HOVER to ADMITTANCE cause of constant error of quadrotor.
          sampler_->getAdmittanceReference(references_.front().state,
                                           external_wrench_,
                                           references_, now_, robot_command_.rotor_mode,
                                           true);
        } else{
          //! [TYPE] ADMITTANCE
          sampler_->getAdmittanceReference(robot_state_,
                                           external_wrench_,
                                           references_, now_, robot_command_.rotor_mode,
                                           false);
        }
      }
      else if(ref_type_ == ReferenceType::TYPE_ADMITTANCE) {
        //! [TYPE] ADMITTANCE => HOVER
        setReferenceType(ReferenceType::TYPE_HOVER);
        // NOTE: use last reference instead of current robot state as hover reference to avoid control instability
        sampler_->getHoverReference(references_.front().state, references_, now_, robot_command_.rotor_mode, true);
      }
      //-------------------------------------- ReferenceType::TYPE_HOVER ----------------------------------------
      else if(ref_type_ != ReferenceType::TYPE_HOVER) {
        ROS_WARN("ATTENTION: SHOULD NOT CALL THIS BRANCH IF YOUR FSM IS WELL BUILT, FORCE IT TO BE TYPE HOVER");
        setReferenceType(ReferenceType::TYPE_HOVER);
        //! NOTE: use the previous reference state instead of robot_state
        //! to avoid reference jump from TYPE HOVER to ADMITTANCE cause of constant error of quadrotor.
        sampler_->getHoverReference(references_.front().state, references_, now_, robot_command_.rotor_mode, true);
      }
      else {
        //! [TYPE] HOVER
        sampler_->getHoverReference(robot_state_, references_, now_, robot_command_.rotor_mode);
      }
      break;
    }
    default:
      break;
  }
  return;
}

void Fsm::setFsmState(const FsmState& fsm_state){
  if(fsm_state_ != fsm_state){
    ROS_INFO("\033[32mState: %s => %s\033[32m",
             FsmStateToString(fsm_state_).c_str(),
             FsmStateToString(fsm_state).c_str());
    fsm_state_ = fsm_state;
  }
}

void Fsm::setReferenceType(const ReferenceType& ref_type){
  if(ref_type_ != ref_type){
    ROS_INFO("\033[34m Ref: %s => %s\033[34m",
             ReferenceTypeToString(ref_type_).c_str(),
             ReferenceTypeToString(ref_type).c_str());
    ref_type_ = ref_type;
  }
}

void Fsm::publishTrajTrigger(){
  geometry_msgs::PoseStamped trigger;
  trigger.header.stamp = ros::Time::now();
  trigger.pose.orientation.w = 1.0;
  traj_trigger_pub_.publish(trigger);
}

bool Fsm::setFcGuidedMode(bool on)
{
  mavros_msgs::SetMode setmode_srv;
  if (on)
  {
    if (fc_state_.mode == "GUIDED_NOGPS"){
      ROS_WARN("Already in GUIDED mode!");
      return true;
    }
    setmode_srv.request.custom_mode = "GUIDED_NOGPS";
    if (!(mavros_setmode_srv_.call(setmode_srv) && setmode_srv.response.mode_sent))
    {
      ROS_ERROR("Failure in MANUAL => GUIDED: Setting GUIDED_NOGPS mode rejected by FCU!");
      return false;
    } else {
      ROS_INFO("Setting GUIDED_NOGPS mode by FCU.");
    }
  }
  else
  {
    if (fc_state_.mode == "STABILIZE"){
      ROS_WARN("Already in STABILIZE mode!");
      return true;
    }
    setmode_srv.request.custom_mode = "STABILIZE";
    if (!(mavros_setmode_srv_.call(setmode_srv) && setmode_srv.response.mode_sent))
    {
      ROS_ERROR("Failure in GUIDED => MANUAL: Setting STABILIZE mode rejected by FCU!");
      return false;
    } else{
      ROS_INFO("Setting STABILIZE mode by FCU.");
    }
  }
  return true;
}

};
