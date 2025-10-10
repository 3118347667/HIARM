#pragma once

// ROS
#include <ros/ros.h>
#include <pluginlib/class_loader.h>
#include <geometry_msgs/WrenchStamped.h>

// Gazebo
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <gazebo/common/Plugin.hh>

#include "flygripper_gazebo_plugins/gazebo_utils.hpp"


namespace gazebo {

class GazeboInteractivePlugin : public ModelPlugin {

 public:
  GazeboInteractivePlugin(){ }

  virtual ~GazeboInteractivePlugin() = default;

  virtual void Load(physics::ModelPtr model, sdf::ElementPtr sdf){
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

    // Read parameters
    std::string link_name;
    if( !getSdfParam<std::string>(kPluginName,sdf_,
                                  "linkName", link_name) ||
        !getSdfParam<double>(kPluginName,sdf_,
                             "commandTimeout", command_timeout_) ||
        !getSdfParam<bool>(kPluginName,sdf_,
                           "relativeForce", relative_force_) ||
        !getSdfParam<bool>(kPluginName,sdf_,
                           "relativeTorque", relative_torque_)){
      return;
    }

    apply_link_ = model_->GetLink(link_name);
    if (!apply_link_){
      ROS_FATAL("%s plugin error: link named: %s does not exist\n",
                kPluginName.c_str(), link_name.c_str());
      return;
    }

    // ROS
    // Get parameters/settings for namespace from ROS param server
    model_nh_ = ros::NodeHandle(robot_namespace_);
    if (!getSdfParam<std::string>(kPluginName, sdf_,
                                  "rosCommandTopicName", ros_command_topic_name_)){
      return;
    }
    ros_setup_ = false;

    last_update_time_ = world_->SimTime().Double();
    last_recv_command_time_ = world_->SimTime().Double();

    // Update connection
    update_connection_ =
        gazebo::event::Events::ConnectWorldUpdateBegin
            (boost::bind(&GazeboInteractivePlugin::Update, this, _1));

    ROS_INFO("%s plugin loaded in namespace: %s",
             kPluginName.c_str(), robot_namespace_.c_str());
  }

  void SetupROS(){
    ros_command_sub_ = model_nh_.subscribe<geometry_msgs::WrenchStamped>(
        ros_command_topic_name_, 1,
        &GazeboInteractivePlugin::RosCommandCallback, this);
  }

  void Update(const common::UpdateInfo& update_info){
    if (!ros_setup_){
      SetupROS();
      ros_setup_ = true;
      ROS_INFO("%s plugin: Setup ROS settings.", kPluginName.c_str());
    }

    double dt = update_info.simTime.Double() - last_update_time_;
    last_update_time_ = update_info.simTime.Double();

    // update interactive force and torque
    if (applied_){
      if (last_update_time_ - last_recv_command_time_ < command_timeout_){
        if (relative_force_){
          apply_link_->AddRelativeForce(interactive_force_);
        }
        else{
          apply_link_->AddForce(interactive_force_);
        }
        if (relative_torque_){
          apply_link_->AddRelativeTorque(interactive_torque_);
        }
        else{
          apply_link_->AddTorque(interactive_torque_);
        }
      }else{
        ROS_INFO("%s plugin: interactive force and torque stop applying due to command timeout %f.",
                 kPluginName.c_str(), command_timeout_);
        applied_ = false;
      }
    }
  }

  void Reset(){
    if (ros_setup_) {
      ros_command_sub_.shutdown();
    }
    ros_setup_ = false;
    interactive_force_ = GzVector3d::Zero;
    interactive_torque_ = GzVector3d::Zero;
    applied_ = false;
  }

  void RosCommandCallback(const geometry_msgs::WrenchStamped::ConstPtr& msg){
    applied_ = true;
    last_recv_command_time_ = world_->SimTime().Double();
    interactive_force_.X() = msg->wrench.force.x;
    interactive_force_.Y() = msg->wrench.force.y;
    interactive_force_.Z() = msg->wrench.force.z;
    interactive_torque_.X() = msg->wrench.torque.x;
    interactive_torque_.Y() = msg->wrench.torque.y;
    interactive_torque_.Z() = msg->wrench.torque.z;
  }

 private:
  const std::string kPluginName = "GazeboInteractivePlugin";

  // Pointer to the gazebo objects
  physics::ModelPtr model_;
  sdf::ElementPtr sdf_;
  physics::WorldPtr world_;

  physics::LinkPtr apply_link_;
  GzVector3d interactive_force_;
  GzVector3d interactive_torque_;
  bool relative_force_;
  bool relative_torque_;
  bool applied_;

  double last_update_time_;
  double last_recv_command_time_;
  double command_timeout_;

  // Pointer to the update event connection.
  event::ConnectionPtr update_connection_;

  // Parameters
  std::string robot_namespace_;

  // ros
  ros::NodeHandle model_nh_;
  bool ros_setup_;


  // ros subscribe command and apply to rotor joint
  std::string ros_command_topic_name_;
  ros::Subscriber ros_command_sub_;
};

} // namespace gazebo {

