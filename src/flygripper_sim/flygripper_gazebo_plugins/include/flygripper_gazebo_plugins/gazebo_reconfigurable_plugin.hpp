#pragma once
#pragma once

#include <deque>
#include <Eigen/Eigen>

// ROS
#include <ros/ros.h>
#include <pluginlib/class_loader.h>

#include "flygripper_msgs/ActuatorCommand.h"
#include "flygripper_msgs/ActuatorState.h"

// Gazebo
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <gazebo/common/Plugin.hh>

#include "flygripper_gazebo_plugins/gazebo_utils.hpp"

namespace gazebo {


class GazeboReconfigurablePlugin : public ModelPlugin {

 public:
  GazeboReconfigurablePlugin(){ }

  virtual ~GazeboReconfigurablePlugin() = default;

  virtual void Load(physics::ModelPtr model,
                    sdf::ElementPtr sdf){
    ROS_INFO("Loading %s plugin", kPluginName.c_str());

    // Save pointers to the model
    model_ = model;
    sdf_ = sdf;

    // Error message if the model couldn't be found
    if (!model_)
    {
      ROS_ERROR("%s plugin (ns = %s) couldn't get model from pointer",
                kPluginName.c_str(), robot_namespace_.c_str());
      return;
    }

    world_ = model_->GetWorld();
    getSdfParam<double>(kPluginName, sdf_, "commandRateLimit", command_rate_limit_, 0.0);
    last_update_time_ = world_->SimTime().Double();
    // Check that ROS has been initialized
    if(!ros::isInitialized())
    {
      ROS_FATAL("A ROS node for Gazebo has not been initialized, unable to load plugin %s. "
                "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)",
                kPluginName.c_str());
      return;
    }

    // Get robot namespace
    getSdfParam<std::string>(kPluginName,
                             sdf_, "robotNamespace", robot_namespace_, model_->GetName());

    // Get the rotor number
    std::string group_name = "reconfigurableJoint";
    int reconfigurable_joint_num = getSdfElementCount(kPluginName, sdf_,
                                                      group_name);
    if (reconfigurable_joint_num == 0){
      ROS_ERROR("%s plugin error: No reconfigurable joint found", kPluginName.c_str());
      return;
    }

    reconfigurable_joint_params_.resize(reconfigurable_joint_num);

    for (int i = 0; i < reconfigurable_joint_num; ++i) {

      auto grouped_sdf = getSdfElementByAttribute(kPluginName, sdf_,
                                                  group_name, "id", i);
      if (!grouped_sdf){
        return;
      }
//      else{
//        ROS_INFO("Find reconfigurable joint with attribute %s: %d", "id", i);
//      }
      auto & param = reconfigurable_joint_params_[i];
      if (!getSdfParam<std::string>(kPluginName,
                                    grouped_sdf, "jointName",
                                    param.joint_name)){
        return;
      }
      param.joint = model_->GetJoint(param.joint_name);
      if (!param.joint){
        ROS_FATAL("%s plugin error: joint named: %s does not exist\n",
                  kPluginName.c_str(), param.joint_name.c_str());
        return;
      }

      // Get offset, multiplier and sensitiveness
      getSdfParam<double>(kPluginName, grouped_sdf,
                          "multiplier", param.multiplier, 1.0);
      getSdfParam<double>(kPluginName,grouped_sdf,
                          "offset", param.offset, 0.0);
      getSdfParam<double>(kPluginName,grouped_sdf,
                          "sensitiveness", param.sensitiveness, 0.0);
      ROS_WARN("%s plugin debug: sensitiveness is %f\n",
               kPluginName.c_str(), param.sensitiveness);
      if (param.sensitiveness < 0.0){
        param.sensitiveness = -param.sensitiveness;
        ROS_WARN("%s plugin error: sensitiveness must be positive, set to %f\n",
                 kPluginName.c_str(), param.sensitiveness);
      }

      getSdfParam<double>(kPluginName, grouped_sdf,
                          "kp", param.kp, 1);
      getSdfParam<double>(kPluginName, grouped_sdf,
                          "kd", param.kd, 0.0);
      param.max_effort = param.joint->GetEffortLimit(0);
      param.max_velocity = param.joint->GetVelocityLimit(0);
    }

    // ROS
    // Get parameters/settings for namespace from ROS param server
    model_nh_ = ros::NodeHandle(robot_namespace_);
    if (!getSdfParam<std::string>(kPluginName, sdf_,
                                  "rosCommandTopicName", ros_command_topic_name_)){
      return;
    }
    if (!getSdfParam<std::string>(kPluginName, sdf_,
                                  "rosFeedbackTopicName", ros_feedback_topic_name_)){
      return;
    }

    ros_setup_ = false;

    actuator_command_.header.stamp = ros::Time::now();
    actuator_command_.reference_position = 0;
    actuator_command_.reference_velocity = 0;
    actuator_command_.reference_effort = 0;

    // Update connection
    update_connection_ =
        gazebo::event::Events::ConnectWorldUpdateBegin
            (boost::bind(&GazeboReconfigurablePlugin::Update, this, _1));

    ROS_INFO("%s plugin loaded in namespace: %s",
             kPluginName.c_str(), robot_namespace_.c_str());
  }

  void SetupROS(){
    // TODO: subscribe in a seperate queue and callback in update
    ros_command_sub_ = model_nh_.subscribe<flygripper_msgs::ActuatorCommand>(
        ros_command_topic_name_, 1,
        &GazeboReconfigurablePlugin::RosCommandCallback, this);
    ros_feedback_pub_ = model_nh_.advertise<flygripper_msgs::ActuatorState>(
        ros_feedback_topic_name_, 1);
  }

  void RosCommandCallback(const flygripper_msgs::ActuatorCommand::ConstPtr& msg){
    actuator_command_ = *msg;
  }

  void Update(const common::UpdateInfo& update_info){
    if (!ros_setup_){
      SetupROS();
      ros_setup_ = true;
      ROS_INFO("%s plugin: Setup ROS settings.", kPluginName.c_str());
    }
    auto current_time = update_info.simTime;
    double current_time_sec = current_time.Double();
    double dt = current_time_sec - last_update_time_;
    last_update_time_ = current_time_sec;

    PassiveJointControl(dt);
    PublishActuatorState();
  }

  void Reset(){
    if (ros_setup_) {
      ros_command_sub_.shutdown();
    }
    ros_setup_ = false;
  }

 private:

  void PassiveJointControl(double dt){
    // Smooth the physical actuator target instead of applying an instantaneous 0 -> 5 step.
    double target = actuator_command_.reference_position;
    if (command_rate_limit_ > 0.0) {
      double step = command_rate_limit_ * std::max(0.0, dt);
      filtered_reference_position_ += ignition::math::clamp(target-filtered_reference_position_, -step, step);
    } else {
      filtered_reference_position_ = target;
    }
    // TODO: Debug for oscillation of tendon driven as first one that should be published
    double largest_tracking_error = -1.0;

    for (auto & param : reconfigurable_joint_params_){
      auto & joint = param.joint;
      double cur_pos = joint->Position(0);
      double cur_vel = joint->GetVelocity(0);

      double reference_position = 0.0;
      double reference_velocity = 0.0;
      // if out of sensitiveness
      if (std::abs(filtered_reference_position_) >= param.sensitiveness){
        reference_position = filtered_reference_position_ > 0 ?
                             filtered_reference_position_ - param.sensitiveness :
                             filtered_reference_position_ + param.sensitiveness;
        reference_velocity = actuator_command_.reference_velocity;
      }
      // if need recover to zero, current position is out of sensitiveness
      else if (cur_pos/param.multiplier >= param.sensitiveness){
        reference_velocity = actuator_command_.reference_velocity;
      }

      double des_pos = param.multiplier * reference_position + param.offset;
      double des_vel = param.multiplier * reference_velocity;
      // As joint dynamics can not be ideally modeled, so feedforward effor term IS NOT USED
      //      double des_effort = actuator_command_.reference_effort / multiplier_;


      double cmd_velocity = param.kp * (des_pos - cur_pos) + des_vel;
      cmd_velocity = ignition::math::clamp(cmd_velocity,
                                           -param.max_velocity,
                                           param.max_velocity);
      double cmd_effort = param.kd * (cmd_velocity - cur_vel); // + des_effort;
      cmd_effort = ignition::math::clamp(cmd_effort,
                                         -param.max_effort,
                                         param.max_effort);

      // Report the least settled joint, so a finished slider cannot hide a moving hinge.
      double effective_position = (cur_pos - param.offset) / param.multiplier;
      if (param.sensitiveness > 0.0 && std::abs(effective_position) > 0.01) {
        effective_position += effective_position > 0.0 ? param.sensitiveness : -param.sensitiveness;
      }
      double error = std::abs(effective_position - actuator_command_.reference_position);
      if (error > largest_tracking_error) {
        largest_tracking_error = error;
        actuator_state_.position = effective_position;
        actuator_state_.velocity = cur_vel / param.multiplier;
        actuator_state_.effort = cmd_effort * param.multiplier;
      }
      // TODO: Control enable or not, try to simulate bouncing and tendon property
      bool control_enabled = true;

      // Debug
//       ROS_INFO("name: %s, des_pos: %f, cur_pos: %f, des_vel: %f, cur_vel: %f, cmd_effort: %f, control: %d",
//       param.joint_name.c_str(), des_pos, cur_pos, des_vel, cur_vel, cmd_effort, int(control_enabled));

      if (control_enabled){
        joint->SetForce(0, cmd_effort);
      } else{
        joint->SetForce(0, 0);
      }

    }

  }

  void PublishActuatorState(){
    actuator_state_.header.stamp = ros::Time::now();
    ros_feedback_pub_.publish(actuator_state_);
  }
  double command_rate_limit_ = 0.0;
  double filtered_reference_position_ = 0.0;
  const std::string kPluginName = "GazeboReconfigurablePlugin";

  // Pointer to the gazebo objects
  physics::ModelPtr model_;
  physics::WorldPtr world_;
  sdf::ElementPtr sdf_;

  // Parameters
  std::string robot_namespace_;
  std::string actuator_joint_name_;
  physics::JointPtr actuator_joint_;

  // Reconfigurable joints params
  struct ReconfigurableJointParams{
    std::string  joint_name;
    physics::JointPtr joint;
    std::string link_name;
    physics::LinkPtr link;
    double offset;
    double multiplier;
    double sensitiveness;
    double max_effort;
    double max_velocity;
    double kp;
    double kd;
  };
  std::vector<ReconfigurableJointParams> reconfigurable_joint_params_;

  double last_update_time_;

  // Pointer to the update event connection.
  event::ConnectionPtr update_connection_;

  // ros
  ros::NodeHandle model_nh_;
  ros::Subscriber ros_command_sub_;
  flygripper_msgs::ActuatorCommand actuator_command_;
  std::string ros_command_topic_name_;

  ros::Publisher ros_feedback_pub_;
  flygripper_msgs::ActuatorState actuator_state_;
  std::string ros_feedback_topic_name_;
  bool ros_setup_;
};

} // namespace gazebo {

