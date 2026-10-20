#pragma once

#include <iostream>

// ROS message
#include "ros/ros.h"

// Realtime buffer
#include <realtime_tools/realtime_buffer.h>
#include <realtime_tools/realtime_publisher.h>

// ROS message
#include "std_msgs/Float64.h"
#include "flygripper_msgs/ActuatorState.h"
#include "flygripper_msgs/ActuatorCommand.h"

// flygripper common
#include "flygripper_common/ros/joint_state_publisher.hpp"
#include "flygripper_common/utils/realtime.hpp"
#include "flygripper_common/type/rc.hpp"
#include "flygripper_common/type/actuator_command.hpp"

// flygripper actuator
#include "flygripper_actuator/servo/servo_control_param.hpp"
#include "flygripper_actuator/servo/feetech_servo.hpp" // TODO: bug occurs if include this before ros.h

using flygripper_common::JointStatePublisher;
using flygripper_common::JointState;
using flygripper_common::Rc;

namespace flygripper_actuator{
namespace servo {
class ActuatorControl {
 public:
  ActuatorControl(ros::NodeHandle &nh, ros::NodeHandle &pnh):
      enable_control_(false),
      teleop_mode_(TeleopMode::MODE_DISABLE)
  {
    if (!param_config_.init(pnh)) {
      throw ros::Exception("Failed to initialize parameter config");
    }
    param_config_.getParams(params_);

    actuator_ = std::make_shared<FeetechServo>(params_.actuator_interface,
                                               params_.baud_rate, 95);
    actuator_->config(params_.actuator_id, params_.actuator_product, params_.actuator_param);
    actuator_state_ = actuator_->getState(static_cast<uint32_t>(params_.actuator_id));
    actuator_command_ = actuator_->getCommand(static_cast<uint32_t>(params_.actuator_id));

    command_sub_ = nh.subscribe("actuator_command", 1, &ActuatorControl::commandCallback, this);
    rc_in_sub_ = nh.subscribe("/mavros/rc/in", 1, &ActuatorControl::RcInCallback, this);
    state_pub_ =
        std::make_shared<realtime_tools::RealtimePublisher<flygripper_msgs::ActuatorState>>(nh, "actuator_state", 1);

    if (params_.enable_joint_state_publisher) {
      joint_state_publisher_.reset(new JointStatePublisher(nh, pnh));
      joint_states_.push_back(JointState{.name=params_.actuator_joint_name, .position=0.0, .velocity=0.0, .effort=0.0});
    }

    desired_update_time_ = 1.0 / params_.loop_frequency;
    control_timer_ = nh.createTimer(ros::Duration(desired_update_time_), &ActuatorControl::update, this);

  }

  void shutdown(){
    control_timer_.stop();
    actuator_->terminate();
  }

 private:

  void update(const ros::TimerEvent &event) {

    checkCycleTime(desired_update_time_,
                   params_.loop_time_error_threshold,
                   dt_);
    ros::Time now = event.current_real;

    // Update parameter and reference command process
    auto command = command_buffer_.readFromRT();
    auto rc = rc_buffer_.readFromRT();

    param_config_.getParams(params_);

    // Actuator read current state process
    actuator_->read();
    setTeleopMode(params_.teleop_mode);
    //! Finite State Machine
    // Priority: MODE_PARAM > MODE_CMD > MODE_RC
    // MODE_PARAM
    if (teleop_mode_ == TeleopMode::MODE_PARAM){
      command->mode = flygripper_common::ActuatorCommandMode::MODE_POSITION;
      command->position = params_.reference_position;
    }
    // MODE_RC
    else if(teleop_mode_ == TeleopMode::MODE_RC &&
             rc->isReceived(now.toSec(), 1.0) && rc->valid()){
      command->mode = flygripper_common::ActuatorCommandMode::MODE_POSITION;
      //! rad: [-12.5, 0] => deg [-716.56, 0]
      command->position = -(1 - rc->channels[8]) / 2.0 * 12.5;
      if( rc->channels[9] > 0 ) {
        command->position = params_.one_morphing_angle;
      }
    }
    // MODE_CMD
    else if(teleop_mode_ == TeleopMode::MODE_CMD &&
//              command->isReceived(now.toSec(), 1.0) && //NOTED: timeout is allowed for discontinuous control
             command->valid()){
    }
    // MODE_CMD invalid, MODE_RC timeout/invalid or MODE_DISABLE
    else {
      command->mode = flygripper_common::ActuatorCommandMode::MODE_RELEASE;
    }

    // Enable control to enable actuator with torque mode
    if (command->mode != flygripper_common::ActuatorCommandMode::MODE_RELEASE) {
      if (!enable_control_) {
        enable_control_ = true;
        actuator_->enableTorque(true);
      }
    }
    else if (enable_control_) {
      enable_control_ = false;
      actuator_->enableTorque(false);
    }

    if (enable_control_){
//      ROS_INFO_THROTTLE(0.5, "enable_control:%f",command->reference_position);
      // Control process
      double error, error_dot = 0.0;
      actuator_command_->stamp = now;
      switch (command->mode) {
      case flygripper_common::ActuatorCommandMode::MODE_POSITION:
      {
        actuator_command_->position = command->position;
        actuator_command_->velocity = 0;
        actuator_command_->acceleration = 0;
        break;
      }
      case flygripper_common::ActuatorCommandMode::MODE_POSITION_VELOCITY:
      {
        actuator_command_->position = command->position;
        actuator_command_->velocity = command->velocity;
        actuator_command_->acceleration = 0;
        break;
      }
      case flygripper_common::ActuatorCommandMode::MODE_POSITION_VELOCITY_EFFORT:
      {
        actuator_command_->position = command->position;
        actuator_command_->velocity = command->velocity;
        actuator_command_->acceleration = command->effort;  // TODO: effort to acc
        break;
      }
      default:
      {
        ROS_ERROR_STREAM("Unknown command mode: "
                         << flygripper_common::ActuatorCommandModeToString(command->mode).c_str());
      }
      }
      // Actuator write command process
      actuator_->write();
    }

    // Print actuator state with realtime tools
    if (state_pub_->trylock()) {
      state_pub_->msg_.header.stamp = now;
      state_pub_->msg_.position = actuator_state_->position;
      state_pub_->msg_.velocity = actuator_state_->velocity;
      state_pub_->msg_.effort = actuator_state_->effort;

      state_pub_->msg_.voltage = actuator_state_->voltage;
      state_pub_->msg_.current = actuator_state_->current;
      state_pub_->msg_.temperature = actuator_state_->temperature;
      state_pub_->msg_.overload = actuator_state_->overload;
      state_pub_->unlockAndPublish();
    }

    if (params_.enable_joint_state_publisher) {
      joint_states_[0].position = actuator_state_->position;
      joint_states_[0].velocity = actuator_state_->velocity;
      joint_states_[0].effort = actuator_state_->effort;
      joint_state_publisher_->publish(now, joint_states_);
    }
  }

  void RcInCallback(const mavros_msgs::RCInConstPtr &msg) {
    Rc<10> rc;
    fromRosMsg(msg, rc, false);
    rc_buffer_.writeFromNonRT(rc);
  }

  void commandCallback(flygripper_msgs::ActuatorCommandConstPtr msg) {
    flygripper_common::ActuatorCommand teleop_command;
    fromRosMsg(msg,teleop_command);
    command_buffer_.writeFromNonRT(teleop_command);
  }

  void setTeleopMode(const TeleopMode& mode){
    if (teleop_mode_ != mode) {
      ROS_INFO("Actuator Teleop Mode: %s => %s.",
               TeleopModeToString(teleop_mode_).c_str(),
               TeleopModeToString(mode).c_str() );
      teleop_mode_ = mode;
    }
  }

  TeleopMode teleop_mode_;
  ros::Subscriber command_sub_, rc_in_sub_;
  std::shared_ptr<realtime_tools::RealtimePublisher<flygripper_msgs::ActuatorState>> state_pub_;
  bool enable_control_;
  ros::Timer control_timer_;
  realtime_tools::RealtimeBuffer<flygripper_common::ActuatorCommand> command_buffer_;
  realtime_tools::RealtimeBuffer<Rc<10>> rc_buffer_;

  JointStatePublisher::Ptr joint_state_publisher_;
  std::vector<JointState> joint_states_;

  double desired_update_time_;
  ros::Duration dt_;

  ParameterConfig param_config_;
  Parameters params_;

  FeetechServo::Ptr actuator_;
  // Borrowed from actuator_'s maps; only FeetechServo owns these objects.
  ActuatorState* actuator_state_ = nullptr;
  ActuatorCommand* actuator_command_ = nullptr;

};
}
}
