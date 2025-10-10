#pragma once

// ROS
#include <ros/ros.h>
#include <pluginlib/class_loader.h>
#include "std_msgs/Float64.h"
#include "std_msgs/String.h"

// Gazebo
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <gazebo/common/Plugin.hh>

#include "flygripper_gazebo_plugins/gazebo_utils.hpp"


namespace gazebo {


class GazeboTemplatePlugin : public ModelPlugin {

 public:

  GazeboTemplatePlugin()
  { }

  virtual ~GazeboTemplatePlugin() = default;

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
                             sdf_, "robotNamespace",
                             robot_namespace_,
                             model_->GetName());

    // Get publish frequency
    double publish_frequency;
    getSdfParam<double>(kPluginName, sdf_,
                        "publishFrequency", publish_frequency, 100);
    publish_duration_ = 1.0 / publish_frequency;
    double max_step_size = world_->Physics()->GetMaxStepSize();
    if (publish_duration_ < max_step_size) {
      ROS_WARN("%s plugin: publishFrequency < physics engine's "
               "max step size, you may not be publishing at the desired rate.",
               kPluginName.c_str());
      publish_duration_ = max_step_size;
    }

    // ROS
    // Get parameters/settings for namespace from ROS param server
    model_nh_ = ros::NodeHandle(robot_namespace_);

    // Update connection
    update_connection_ =
        gazebo::event::Events::ConnectWorldUpdateBegin
            (boost::bind(&GazeboTemplatePlugin::Update, this, _1));

    ROS_INFO("%s plugin loaded in namespace: %s",
             kPluginName.c_str(), robot_namespace_.c_str());
  }

  void SetupROS(){
    ros_command_sub_ = model_nh_.subscribe<std_msgs::Float64>("gazebo_subscribe", 1,
                                                              &GazeboTemplatePlugin::rosCommandCallback, this);
    ros_feedback_pub_ = model_nh_.advertise<std_msgs::String>("gazebo_publish", 1);
  }

  void Update(const common::UpdateInfo& update_info){
    if (!ros_setup_){
      SetupROS();
      ros_setup_ = true;
      ROS_INFO("%s plugin: Setup ROS settings.", kPluginName.c_str());
    }

    auto current_time = update_info.simTime;
    double current_time_sec = current_time.Double();

    if (current_time - last_update_time_ >= publish_duration_){
      // Time issue
      last_update_time_ = current_time_sec;

      std_msgs::String msg;
      msg.data = std::to_string(float_data_);
      if (ros_feedback_pub_.getNumSubscribers() > 0){
        ros_feedback_pub_.publish(msg);
      }
    }

//    ROS_INFO_THROTTLE(1, "%s plugin: Updating", kPluginName.c_str());
  }


  void Reset(){
    if (ros_setup_) {
      ros_command_sub_.shutdown();
    }
    ros_setup_ = false;
  }

  void rosCommandCallback(const std_msgs::Float64::ConstPtr& msg){
    float_data_ = msg->data;
  }

 private:
  const std::string kPluginName = "GazeboTemplatePlugin";
  // Pointer to the gazebo objects
  physics::ModelPtr model_;
  sdf::ElementPtr sdf_;
  physics::WorldPtr world_;

  double publish_duration_;
  double last_update_time_;
  double float_data_;

  // Pointer to the update event connection.
  event::ConnectionPtr update_connection_;

  // Parameters
  std::string robot_namespace_;

  // ros
  ros::NodeHandle model_nh_;
  bool ros_setup_;
  std::string ros_command_topic_name_;
  std::string ros_feedback_topic_name_;
  ros::Subscriber ros_command_sub_;
  ros::Publisher ros_feedback_pub_;


};

} // namespace gazebo {

