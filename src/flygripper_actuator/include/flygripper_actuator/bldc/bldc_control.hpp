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
#include "flygripper_msgs/PidControllerState.h"

// flygripper common
#include "flygripper_common/controller/pid.hpp"
#include "flygripper_common/ros/joint_state_publisher.hpp"

// flygripper actuator
#include "flygripper_actuator/bldc/bldc_control_param.hpp"
#include "flygripper_actuator/bldc/can_bldc.hpp" // TODO: bug occurs if include this before ros.h
#include "flygripper_common/utils/realtime.hpp"

using flygripper_common::Pid;
using flygripper_common::JointStatePublisher;
using flygripper_common::JointState;

namespace flygripper_actuator{
namespace bldc {
class ActuatorControl {
 public:
  ActuatorControl(ros::NodeHandle &nh, ros::NodeHandle &pnh) {
    if (!param_config_.init(pnh)) {
      throw ros::Exception("Failed to initialize parameter config");
    }
    param_config_.getParams(params_);

    actuator_ = std::make_shared<CanActuator>(params_.actuator_bus, params_.actuator_interface, 95);
    actuator_->config(params_.actuator_id, params_.actuator_product, params_.actuator_param);
    actuator_state_.reset(actuator_->getState(static_cast<uint32_t>(params_.actuator_id)));
    actuator_command_.reset(actuator_->getCommand(static_cast<uint32_t>(params_.actuator_id)));

    pid_controller_ = std::make_shared<Pid>(params_.pid);

    command_sub_ = nh.subscribe("actuator_command", 1, &ActuatorControl::commandCallback, this);
    state_pub_ =
        std::make_shared<realtime_tools::RealtimePublisher<flygripper_msgs::ActuatorState>>(nh, "actuator_state", 1);
    ctrl_state_pub_ =
        std::make_shared<realtime_tools::RealtimePublisher<flygripper_msgs::PidControllerState>>(nh, "pid_state", 1);

    if (params_.enable_joint_state_publisher) {
      joint_state_publisher_.reset(new JointStatePublisher(nh, pnh));
      joint_states_.push_back(JointState{.name=params_.actuator_joint_name, .position=0.0, .velocity=0.0, .effort=0.0});
    }

    desired_update_time_ = 1.0 / params_.loop_frequency;
    control_timer_ = nh.createTimer(ros::Duration(desired_update_time_), &ActuatorControl::update, this);

  }

 private:

  void update(const ros::TimerEvent &event) {

    checkCycleTime(desired_update_time_,
                   params_.loop_time_error_threshold,
                   dt_);
    ros::Time now = event.current_real;

    // Update parameter and reference command process
    auto command = command_buffer_.readFromRT();
    param_config_.getParams(params_);
    pid_controller_->setParams(params_.pid);

    // Actuator read current state process
    actuator_->read();

    // Control process
    double error, error_dot = 0.0;
    actuator_command_->stamp = now;
    if (command->mode == flygripper_msgs::ActuatorCommand::MODE_POSITION) {
      error = command->reference_position - actuator_state_->position;
      actuator_command_->effort = pid_controller_->update(error, dt_.toNSec());
    } else if (command->mode == flygripper_msgs::ActuatorCommand::MODE_POSITION_VELOCITY) {
      error = command->reference_position - actuator_state_->position;
      error_dot = command->reference_velocity - actuator_state_->velocity;
      actuator_command_->effort = pid_controller_->update(error, error_dot, dt_.toNSec());
    } else if (command->mode == flygripper_msgs::ActuatorCommand::MODE_POSITION_VELOCITY_EFFORT) {
      error = command->reference_position - actuator_state_->position;
      error_dot = command->reference_velocity - actuator_state_->velocity;
      actuator_command_->effort = pid_controller_->update(error, error_dot, command->reference_effort, dt_.toNSec());}
    else {
      ROS_ERROR_STREAM("Unknown command mode: " << command->mode);
    }
    // Actuator write command process
    actuator_->write();

    // Print actuator state and controller state process
    if (state_pub_->trylock()) {
      state_pub_->msg_.header.stamp = now;
      state_pub_->msg_.position = actuator_state_->position;
      state_pub_->msg_.velocity = actuator_state_->velocity;
      state_pub_->msg_.effort = actuator_state_->effort;
      state_pub_->unlockAndPublish();
    }

    if (ctrl_state_pub_->trylock()) {
      ctrl_state_pub_->msg_.header.stamp = now;
      ctrl_state_pub_->msg_.time_step = dt_.toSec();
      ctrl_state_pub_->msg_.error = error;
      ctrl_state_pub_->msg_.error_dot = error_dot;
      ctrl_state_pub_->msg_.reference_value = command->reference_position;
      ctrl_state_pub_->msg_.reference_value_dot = command->reference_velocity;
      ctrl_state_pub_->msg_.current_value = actuator_state_->position;
      ctrl_state_pub_->msg_.current_value_dot = actuator_state_->velocity;
      ctrl_state_pub_->msg_.command = actuator_command_->effort;
      ctrl_state_pub_->unlockAndPublish();
    }

    if (params_.enable_joint_state_publisher) {
      joint_states_[0].position = actuator_state_->position;
      joint_states_[0].velocity = actuator_state_->velocity;
      joint_states_[0].effort = actuator_state_->effort;
      joint_state_publisher_->publish(now, joint_states_);
    }

  }

  void commandCallback(const flygripper_msgs::ActuatorCommand &msg) {
    command_buffer_.writeFromNonRT(msg);
  }

  ros::Subscriber command_sub_;
  std::shared_ptr<realtime_tools::RealtimePublisher<flygripper_msgs::ActuatorState>> state_pub_;
  std::shared_ptr<realtime_tools::RealtimePublisher<flygripper_msgs::PidControllerState>> ctrl_state_pub_;
  ros::Timer control_timer_;
  realtime_tools::RealtimeBuffer<flygripper_msgs::ActuatorCommand> command_buffer_;

  JointStatePublisher::Ptr joint_state_publisher_;
  std::vector<JointState> joint_states_;

  double desired_update_time_;
  ros::Duration dt_;

  ParameterConfig param_config_;
  Parameters params_;

  CanActuator::Ptr actuator_;
  ActuatorState::Ptr actuator_state_;
  ActuatorCommand::Ptr actuator_command_;

  Pid::Ptr pid_controller_;

};
}
}