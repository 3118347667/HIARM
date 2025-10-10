#pragma once

#include <deque>
#include <Eigen/Eigen>

// ROS
#include <ros/ros.h>
#include <pluginlib/class_loader.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>

// Gazebo
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <gazebo/common/Plugin.hh>

#include "flygripper_gazebo_plugins/gazebo_utils.hpp"


namespace gazebo {

class GazeboOdometryPlugin : public ModelPlugin {

 public:
  using NormalDistribution = std::normal_distribution<double>;
  using OdometryQueue = std::deque<std::pair<double, nav_msgs::Odometry>>;

  GazeboOdometryPlugin():
      random_generator_(random_device_())
  { }

  virtual ~GazeboOdometryPlugin() = default;

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

    // Get link name and link
    if (!getSdfParam<std::string>(kPluginName,
                                  sdf_, "linkName", link_name_)){
      return;
    }
    link_ = model_->GetLink(link_name_);
    if (!link_){
      ROS_FATAL("%s plugin error: link named: %s does not exist\n",
                kPluginName.c_str(), link_name_.c_str());
      return;
    }

    // TODO: robot base_link odom should be calculated by algorithms but not sim
//    parent_link_ = world_->EntityByName(kParentFrameId);
//
//    if (parent_link_ == NULL) {
//      ROS_FATAL("plugin error: Couldn't find specified parent link: %s.",
//                      kParentFrameId.c_str());
//      return;
//    }

    // Get noise parameter
    unsigned int random_engine_seed;
    getSdfParam<unsigned int>(kPluginName,
                              sdf_,
                              "randomEngineSeed",
                              random_engine_seed,
                              std::chrono::system_clock::now().time_since_epoch().count());
    random_generator_.seed(random_engine_seed);

    GzVector3d noise_normal_position, noise_normal_quaternion,
        noise_normal_linear_velocity, noise_normal_angular_velocity;

    getSdfParam<GzVector3d>(kPluginName, sdf_,
                          "noiseNormalPosition", noise_normal_position, GzVector3d::Zero);
    getSdfParam<GzVector3d>(kPluginName, sdf_,
                          "noiseNormalQuaternion", noise_normal_quaternion, GzVector3d::Zero);
    getSdfParam<GzVector3d>(kPluginName, sdf_,
                          "noiseNormalLinearVelocity", noise_normal_linear_velocity, GzVector3d::Zero);
    getSdfParam<GzVector3d>(kPluginName, sdf_,
                          "noiseNormalAngularVelocity", noise_normal_angular_velocity, GzVector3d::Zero);

    position_n_[0] = NormalDistribution(0, noise_normal_position.X());
    position_n_[1] = NormalDistribution(0, noise_normal_position.Y());
    position_n_[2] = NormalDistribution(0, noise_normal_position.Z());

    attitude_n_[0] = NormalDistribution(0, noise_normal_quaternion.X());
    attitude_n_[1] = NormalDistribution(0, noise_normal_quaternion.Y());
    attitude_n_[2] = NormalDistribution(0, noise_normal_quaternion.Z());

    linear_velocity_n_[0] =
        NormalDistribution(0, noise_normal_linear_velocity.X());
    linear_velocity_n_[1] =
        NormalDistribution(0, noise_normal_linear_velocity.Y());
    linear_velocity_n_[2] =
        NormalDistribution(0, noise_normal_linear_velocity.Z());

    angular_velocity_n_[0] =
        NormalDistribution(0, noise_normal_angular_velocity.X());
    angular_velocity_n_[1] =
        NormalDistribution(0, noise_normal_angular_velocity.Y());
    angular_velocity_n_[2] =
        NormalDistribution(0, noise_normal_angular_velocity.Z());

    // Get Noise covariance matrix
    // Fill in covariance of pose
    Eigen::Map<Eigen::Matrix<double, 6, 6> > pose_covariance(
        pose_covariance_matrix_.data());
    Eigen::Matrix<double, 6, 1> pose_covd;

    pose_covd << noise_normal_position.X() * noise_normal_position.X(),
        noise_normal_position.Y() * noise_normal_position.Y(),
        noise_normal_position.Z() * noise_normal_position.Z(),
        noise_normal_quaternion.X() * noise_normal_quaternion.X(),
        noise_normal_quaternion.Y() * noise_normal_quaternion.Y(),
        noise_normal_quaternion.Z() * noise_normal_quaternion.Z();
    pose_covariance = pose_covd.asDiagonal();

    // Fill in covariance of twist
    Eigen::Map<Eigen::Matrix<double, 6, 6> > twist_covariance(
        twist_covariance_matrix_.data());
    Eigen::Matrix<double, 6, 1> twist_covd;

    twist_covd << noise_normal_linear_velocity.X() *
        noise_normal_linear_velocity.X(),
        noise_normal_linear_velocity.Y() * noise_normal_linear_velocity.Y(),
        noise_normal_linear_velocity.Z() * noise_normal_linear_velocity.Z(),
        noise_normal_angular_velocity.X() * noise_normal_angular_velocity.X(),
        noise_normal_angular_velocity.Y() * noise_normal_angular_velocity.Y(),
        noise_normal_angular_velocity.Z() * noise_normal_angular_velocity.Z();
    twist_covariance = twist_covd.asDiagonal();

    // ROS
    // Get parameters/settings for namespace from ROS param server
    model_nh_ = ros::NodeHandle(robot_namespace_);
    if (!getSdfParam<std::string>(kPluginName, sdf_,
                                  "rosOdomTopicName", ros_odom_topic_name_)){
      return;
    }
    ros_setup_ = false;
    getSdfParam<std::string>(kPluginName, sdf_,
                             "frameId", frame_id_, "world");

    getSdfParam<std::string>(kPluginName, sdf_,
                             "childFrameId", child_frame_id_, robot_namespace_);

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
    last_update_time_ = world_->SimTime().Double();

    getSdfParam<double>(kPluginName, sdf_,
                        "measurementDelay", measurement_delay_, 0.0);

    // Init ros odom msg
    ros_odom_msg_.header.frame_id = frame_id_;
    ros_odom_msg_.child_frame_id = child_frame_id_;
    for (int i = 0; i < pose_covariance_matrix_.size(); ++i) {
      ros_odom_msg_.pose.covariance[i] = pose_covariance_matrix_[i];
      ros_odom_msg_.twist.covariance[i] = twist_covariance_matrix_[i];
    }

    // Update connection
    update_connection_ =
        gazebo::event::Events::ConnectWorldUpdateBegin
            (boost::bind(&GazeboOdometryPlugin::Update, this, _1));

    ROS_INFO("%s plugin loaded in namespace: %s",
             kPluginName.c_str(), robot_namespace_.c_str());

  }

  void SetupROS(){
    ros_odom_pub_ = model_nh_.advertise<nav_msgs::Odometry>(
        ros_odom_topic_name_, 1);
  }

  void Update(const common::UpdateInfo& update_info){
    if (!ros_setup_){
      SetupROS();
      ros_setup_ = true;
      ROS_INFO("%s plugin: Setup ROS settings.", kPluginName.c_str());
    }

    auto current_time = update_info.simTime;
    double current_time_sec = current_time.Double();

    // Update queue
    // trick with current_time with type of common::Time operation to avoid double equal issue
    if (current_time - last_update_time_ >= publish_duration_){
      // Time issue
      last_update_time_ = current_time_sec;

      // Get pose, linear and angular velocity from Gazebo

      GzPose3d T_W_B = link_->WorldCoGPose();
      // linear velocity is defined in world frame
      GzVector3d linear_velocity_B = link_->RelativeLinearVel();
      GzVector3d linear_velocity_W = link_->WorldLinearVel();
      // angular velocity is defined in body frame
      GzVector3d angular_velocity_B = link_->RelativeAngularVel();

      // Trans to Eigen and calculate attitude distortions
      Eigen::Vector3d theta;
      theta << attitude_n_[0](random_generator_),
          attitude_n_[1](random_generator_),
          attitude_n_[2](random_generator_);
      Eigen::Quaterniond q_n = quaternionFromSmallAngle(theta);
      q_n.normalize();
      Eigen::Quaterniond q = convertToEigen(T_W_B.Rot()) * q_n;
      // Push ros odom msg to queue
      ros_odom_msg_.header.stamp.sec = current_time.sec; // real time
      ros_odom_msg_.header.stamp.nsec = current_time.nsec;

      ros_odom_msg_.pose.pose.position.x = T_W_B.Pos().X() + position_n_[0](random_generator_);
      ros_odom_msg_.pose.pose.position.y = T_W_B.Pos().Y() + position_n_[1](random_generator_);
      ros_odom_msg_.pose.pose.position.z = T_W_B.Pos().Z() + position_n_[2](random_generator_);
      ros_odom_msg_.pose.pose.orientation.x = q.x();
      ros_odom_msg_.pose.pose.orientation.y = q.y();
      ros_odom_msg_.pose.pose.orientation.z = q.z();
      ros_odom_msg_.pose.pose.orientation.w = q.w();
      // linear velocity is defined in world frame
      ros_odom_msg_.twist.twist.linear.x = linear_velocity_W.X() + linear_velocity_n_[0](random_generator_);
      ros_odom_msg_.twist.twist.linear.y = linear_velocity_W.Y() + linear_velocity_n_[1](random_generator_);
      ros_odom_msg_.twist.twist.linear.z = linear_velocity_W.Z() + linear_velocity_n_[2](random_generator_);

      ros_odom_msg_.twist.twist.angular.x = angular_velocity_B.X() + angular_velocity_n_[0](random_generator_);
      ros_odom_msg_.twist.twist.angular.y = angular_velocity_B.Y() + angular_velocity_n_[1](random_generator_);
      ros_odom_msg_.twist.twist.angular.z = angular_velocity_B.Z() + angular_velocity_n_[2](random_generator_);
      ros_odom_msg_queue_.push_back(std::make_pair(current_time_sec + measurement_delay_, ros_odom_msg_));

      // publish tf without delay
      tf::Transform W_T_B;
      W_T_B.setOrigin( tf::Vector3(T_W_B.Pos().X(), T_W_B.Pos().Y(), T_W_B.Pos().Z()));
      W_T_B.setRotation(tf::Quaternion(q.x(), q.y(), q.z(), q.w()));
      // NOTED: TF_REPEATED_DATA ignoring data with redundant timestamp for frame Problem
      // because tf publish in the same timestamp
      ros::Time tf_time = ros::Time::now().fromSec(current_time_sec); //ros::Time::now();
      tf_broadcaster_.sendTransform(tf::StampedTransform(W_T_B, tf_time,
                                                         frame_id_, child_frame_id_));
    }

    // Publish ros odom msg from queue
    // trick with current_time with type of common::Time operation to avoid double equal issue
    while (ros_odom_msg_queue_.size() > 0 && current_time - ros_odom_msg_queue_.front().first >= 0 ){
      if (ros_odom_pub_.getNumSubscribers() > 0){
        ros_odom_pub_.publish(ros_odom_msg_queue_.front().second);
      }
      ros_odom_msg_queue_.pop_front();
    }
  }

  void Reset(){
    if (ros_setup_) {
      ros_odom_pub_.shutdown();
    }
    ros_setup_ = false;
  }

 private:
  const std::string kPluginName = "GazeboOdometryPlugin";
//  const std::string kParentFrameId = "base_link";
  // Pointer to the gazebo objects
  physics::ModelPtr model_;
  sdf::ElementPtr sdf_;
  physics::WorldPtr world_;
  physics::LinkPtr link_;
//  physics::EntityPtr parent_link_;

  double last_update_time_;
  std::random_device random_device_;
  std::mt19937 random_generator_;

  NormalDistribution position_n_[3];
  NormalDistribution attitude_n_[3];
  NormalDistribution linear_velocity_n_[3];
  NormalDistribution angular_velocity_n_[3];
  std::array<double, 6*6> pose_covariance_matrix_;
  std::array<double, 6*6> twist_covariance_matrix_;

  // Pointer to the update event connection.
  event::ConnectionPtr update_connection_;

  // Parameters
  std::string robot_namespace_;
  std::string link_name_;
  std::string frame_id_;
  std::string child_frame_id_;
  double publish_duration_;
  double measurement_delay_;
  // Tf
  tf::TransformBroadcaster tf_broadcaster_;

  // ros
  ros::NodeHandle model_nh_;
  bool ros_setup_;
  ros::Publisher ros_odom_pub_;
  std::string ros_odom_topic_name_;
  OdometryQueue ros_odom_msg_queue_;
  nav_msgs::Odometry ros_odom_msg_;
};

} // namespace gazebo {

