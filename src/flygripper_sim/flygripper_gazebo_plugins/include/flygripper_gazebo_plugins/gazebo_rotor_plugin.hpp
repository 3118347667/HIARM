#pragma once

// ROS
#include <ros/ros.h>
#include <pluginlib/class_loader.h>
#include <mavros_msgs/ESCTelemetry.h>
#include <mavros_msgs/ActuatorControl.h>

// Gazebo
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <gazebo/common/Plugin.hh>

#include "flygripper_gazebo_plugins/gazebo_utils.hpp"

// TODO： add the cmd buffer to simulate the delay of the communication
namespace gazebo {

namespace turning_direction {
const static int CCW = 1;
const static int CW = -1;
} // namespace turning_direction
static constexpr double kRadPerSecToRpm = 30.0  / M_PI;

class GazeboRotorPlugin : public ModelPlugin {

 public:
  GazeboRotorPlugin(){ }

  virtual ~GazeboRotorPlugin() = default;

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


    // Get the rotor number
    std::string group_name = "rotor";
    rotor_num_ = getSdfElementCount(kPluginName, sdf_,
                                    group_name);
    if (rotor_num_ == 0){
      ROS_ERROR("%s plugin error: No rotor found", kPluginName.c_str());
      return;
    }

    // Init rotor variables with size of rotor_num_
    rotor_links_name.resize(rotor_num_);
    rotor_joints_name.resize(rotor_num_);
    rotor_turning_directions_.resize(rotor_num_);

    rotor_links_.resize(rotor_num_);
    rotor_joints_.resize(rotor_num_);

    rotor_command_velocities_.resize(rotor_num_, 0);
    rotor_feedback_velocities_.resize(rotor_num_, 0);
    ros_feedback_msg_.esc_telemetry.resize(rotor_num_);

    // Get the rotor joints, links, turning_directions and init filters
    std::string rotor_turning_direction;
    for (int i = 0; i < rotor_num_; ++i) {
      auto grouped_sdf = getSdfElementByAttribute(kPluginName, sdf_,
                                                  group_name, "id", i);
      if (!grouped_sdf){
        return;
      }
//      else{
//        ROS_INFO("Find rotor with attribute %s: %d", "id", i);
//      }
      if (!getSdfParam(kPluginName, grouped_sdf, "jointName", rotor_joints_name[i]) ||
          !getSdfParam(kPluginName, grouped_sdf, "linkName", rotor_links_name[i]) ||
          !getSdfParam(kPluginName, grouped_sdf, "turningDirection", rotor_turning_direction)){
        return;
      }
      if (rotor_turning_direction == "cw"){
        rotor_turning_directions_[i] = turning_direction::CW;
      }
      else if (rotor_turning_direction == "ccw"){
        rotor_turning_directions_[i] = turning_direction::CCW;
      }
      else{
        ROS_ERROR("%s plugin error: Invalid turning direction %s",
                  kPluginName.c_str(),
                  rotor_turning_direction.c_str());
      }

      rotor_joints_[i] = model_->GetJoint(rotor_joints_name[i]);
      if (rotor_joints_[i] == NULL){
        ROS_ERROR("%s plugin error: Couldn't find specified joint \"%s\".",
                  kPluginName.c_str(),
                  rotor_joints_name[i].c_str());
        return;
      }

      rotor_links_[i]= model_->GetLink(rotor_links_name[i]);
      if (rotor_links_[i] == NULL){
        ROS_ERROR("%s plugin error: Couldn't find specified link \"%s\".",
                  kPluginName.c_str(),
                  rotor_links_name[i].c_str());
        return;
      }

      rotor_velocity_filters_.push_back(std::make_unique<FirstOrderFilter<double>>(0.0,
                                                                                   time_constant_up_,
                                                                                   time_constant_down_));
      ROS_INFO("%s plugin: rotor %d jointName: %s, linkName: %s, turningDirections: %d",
               kPluginName.c_str(),
               i,
               rotor_joints_name[i].c_str(),
               rotor_links_name[i].c_str(),
               rotor_turning_directions_[i]);
    }


    double moment_coefficient;
    // Read other rotor parameters
    if( !getSdfParam<double>(kPluginName,sdf_,
                             "motorConstant", motor_coefficient_) ||
        !getSdfParam<double>(kPluginName,sdf_,
                             "momentConstant", moment_coefficient) ||
        !getSdfParam<double>(kPluginName,sdf_,
                             "rotorDragCoefficient", rotor_drag_coefficient_) ||
        !getSdfParam<double>(kPluginName,sdf_,
                             "rollingMomentCoefficient", rolling_moment_coefficient_) ||
        !getSdfParam<double>(kPluginName,sdf_,
                             "maxRotVelocity", max_rot_velocity_) ||
        !getSdfParam<double>(kPluginName,sdf_,
                             "timeConstantUp", time_constant_up_) ||
        !getSdfParam<double>(kPluginName,sdf_,
                             "timeConstantDown", time_constant_down_) ||
        !getSdfParam<double>(kPluginName, sdf_,
                             "rotorVelocitySlowdownSim", rotor_velocity_slowdown_sim_) ||
        !getSdfParam<GzVector3d>(kPluginName, sdf_,
                                 "dshotRpmMappingCoefficient", dshot_rpm_coefficient_)){
      return;
    }

    // Here all rotor speed and its relevant coefficients
    // are measured in unit of round per minute

    // Transform from round/m to rad/s
    max_rot_velocity_ /= kRadPerSecToRpm;
    // Calculate ratio of moment_coefficient / motor_coefficient
    moment_motor_ratio_ = moment_coefficient / motor_coefficient_;
    // Transform coefficient from N/(round/m)^2 to N/(rad/s)^2
    motor_coefficient_ *= kRadPerSecToRpm * kRadPerSecToRpm;

    // Transform coefficient from N/(round/m)/(m/s) to N/(rad/s)/(m/s)
    rotor_drag_coefficient_ *= kRadPerSecToRpm;
    // Transform coefficient from N*m/(round/m)/(m/s) to N*m/(rad/s)/(m/s)
    rolling_moment_coefficient_ *= kRadPerSecToRpm;

    // ROS
    // Get parameters/settings for namespace from ROS param server
    model_nh_ = ros::NodeHandle(robot_namespace_);
    if (!getSdfParam<std::string>(kPluginName, sdf_,
                                  "rosCommandTopicName", ros_command_topic_name_) ||
        !getSdfParam<std::string>(kPluginName, sdf_,
                                  "rosFeedbackTopicName", ros_feedback_topic_name_)){
      return;
    }
    ros_setup_ = false;

    last_update_time_ = world_->SimTime().Double();
    last_recv_command_time_ = world_->SimTime().Double();

    // Update connection
    update_connection_ =
        gazebo::event::Events::ConnectWorldUpdateBegin
            (boost::bind(&GazeboRotorPlugin::Update, this, _1));


    ROS_INFO("%s plugin loaded in namespace: %s",
             kPluginName.c_str(), robot_namespace_.c_str());
  }

  void SetupROS(){
    // TODO: subscribe in a seperate queue and callback in update
    ros_command_sub_ = model_nh_.subscribe<mavros_msgs::ActuatorControl>(
        ros_command_topic_name_, 1,
        &GazeboRotorPlugin::RosCommandCallback, this);
    ros_feedback_pub_ = model_nh_.advertise<mavros_msgs::ESCTelemetry>(
        ros_feedback_topic_name_, 1);
  }

  void Update(const common::UpdateInfo& update_info){
    if (!ros_setup_){
      SetupROS();
      ros_setup_ = true;
      ROS_INFO("%s plugin: Setup ROS settings.", kPluginName.c_str());
    }

    double dt = update_info.simTime.Double() - last_update_time_;
    last_update_time_ = update_info.simTime.Double();

    UpdateAerodynamics(dt);
    RosPublish();
  }

  // TODO: wind speed[x]
  void UpdateAerodynamics(double dt){
    for (int i=0; i < rotor_num_; i++){
      double rotor_feedback_velocity_sim = rotor_joints_[i]->GetVelocity(0);
      if (std::abs(rotor_feedback_velocity_sim) / (2 * M_PI) > 1 / (2 * dt)) {
        ROS_WARN("%s plugin warning: Aliasing on rotor might occur. "
                 "Consider making smaller simulation time steps "
                 "or raising the rotorVelocitySlowdownSim param.",
                 kPluginName.c_str());
      }
      // Get current rotor_feedback_velocity as [rotor_feedback_velocity_sim * rotor_velocity_slowdown_sim_]
      double rotor_feedback_velocity = rotor_feedback_velocity_sim * rotor_velocity_slowdown_sim_;

      // Get the direction of the rotor rotation.
      int rotor_velocity_sign =
          (rotor_feedback_velocity  > 0) - (rotor_feedback_velocity < 0);

      // Assuming symmetric propellers (or rotors) for the thrust calculation.
      double thrust = rotor_turning_directions_[i] * rotor_velocity_sign *
          rotor_feedback_velocity * rotor_feedback_velocity *
          motor_coefficient_;
//      ROS_INFO_THROTTLE(0.2, "id: %d, rotor_feedback_velocity: %f, thrust: %f", i,
//                        rotor_feedback_velocity, thrust);

      // Apply a force to the link.
      rotor_links_[i]->AddRelativeForce(GzVector3d (0, 0, thrust));
//      ROS_INFO("id: %d, thrust: %f", i, thrust);

      // Forces from Philppe Martin's and Erwan Salaün's
      // 2010 IEEE Conference on Robotics and Automation paper
      // The True Role of Accelerometer Feedback in Quadrotor Control
      // - \omega * \lambda_1 * V_A^{\perp}
      GzVector3d joint_axis = rotor_joints_[i]->GlobalAxis(0);
      GzVector3d body_velocity_W = rotor_links_[i]->WorldLinearVel();
      GzVector3d relative_wind_velocity_W = body_velocity_W ;// - wind_speed_W_;
      GzVector3d body_velocity_perpendicular =
          relative_wind_velocity_W -
              (relative_wind_velocity_W.Dot(joint_axis) * joint_axis);
      GzVector3d air_drag = -std::abs(rotor_feedback_velocity) *
          rotor_drag_coefficient_ *
          body_velocity_perpendicular;
      // Apply air_drag to link.
      rotor_links_[i]->AddForce(air_drag);
      // Moments get the parent link, such that the resulting torques can be
      // applied.
      physics::Link_V parent_links = rotor_links_[i]->GetParentJointsLinks();
      // The tansformation from the parent_link to the link_.
      GzPose3d pose_difference =
          rotor_links_[i]->WorldCoGPose() - parent_links.at(0)->WorldCoGPose();
      GzVector3d drag_torque(
          0, 0, -rotor_turning_directions_[i] * thrust * moment_motor_ratio_);
      // Transforming the drag torque into the parent frame to handle
      // arbitrary rotor orientations.
      GzVector3d drag_torque_parent_frame =
          pose_difference.Rot().RotateVector(drag_torque);
      parent_links.at(0)->AddRelativeTorque(drag_torque_parent_frame);

      GzVector3d rolling_moment;
      // - \omega * \mu_1 * V_A^{\perp}
      rolling_moment = -std::abs(rotor_feedback_velocity) *
          rolling_moment_coefficient_ *
          body_velocity_perpendicular;
      parent_links.at(0)->AddTorque(rolling_moment);
      rotor_feedback_velocities_[i] = rotor_velocity_sign * rotor_feedback_velocity;

      // TODO： time delay
      // Apply the filter on the rotor's velocity.
      if (last_update_time_ - last_recv_command_time_ < kCommandTimeout){

        double rotor_velocity = std::min(max_rot_velocity_,
                                         rotor_command_velocities_[i]);
        rotor_velocity = rotor_velocity_filters_[i]->update(rotor_velocity, dt);

        // Make sure max force is set, as it may be reset to 0 by a world reset any
        // time. (This cannot be done during Reset() because the change will be undone
        // by the Joint's reset function afterwards.)
        rotor_joints_[i]->SetVelocity(
            0, rotor_turning_directions_[i] * rotor_velocity /
                rotor_velocity_slowdown_sim_);
//        ROS_INFO_THROTTLE(0.2, "id: %d, rotor command velocity: %f", i, rotor_velocity);

      } else {
        ROS_WARN_COND(i==0 && rotor_command_velocities_[i] > 1e-5,
                      "%s plugin: Did not receive motor commands for %f "
                      "seconds, stopping all rotors.",
                      kPluginName.c_str(),
                      last_update_time_ - last_recv_command_time_);
        rotor_command_velocities_[i] = 0.0;
        rotor_joints_[i]->SetVelocity( 0, 0);
      }
    }
  }

  void Reset(){
    if (ros_setup_) {
      ros_command_sub_.shutdown();
    }
    ros_setup_ = false;
    for (int i=0; i < rotor_num_; i++){
      rotor_command_velocities_[i] = 0;
      rotor_velocity_filters_[i] = std::make_unique<FirstOrderFilter<double>>(0.0,
                                                                              time_constant_up_,
                                                                              time_constant_down_);
    }

  }

  void RosCommandCallback(const mavros_msgs::ActuatorControl::ConstPtr& msg){
    if (msg->group_mix != mavros_msgs::ActuatorControl::PX4_MIX_FLIGHT_CONTROL){
      ROS_ERROR_THROTTLE(3.0, "%s plugin error: The group_mix of mavros_msgs::ActuatorControl message is not PX4_MIX_FLIGHT_CONTROL mode.",
                kPluginName.c_str());
      return;
    }
    if (msg->controls.size() < rotor_num_){
      ROS_ERROR_THROTTLE(3.0, "%s plugin error: The rotor number (%ld) in message is smaller than the rotor number(%d) in gazebo model.",
                kPluginName.c_str(),
                msg->controls.size(), rotor_num_);
      return;
    }
    last_recv_command_time_ = world_->SimTime().Double();
    for (int i = 0; i < rotor_num_; i++){
      // Mapping from dshot to rpm, then convert to rad/s
      rotor_command_velocities_[i] = DshotToRpm(msg->controls[i]) / kRadPerSecToRpm;
    }
  }

  void RosPublish(){
    if (ros_feedback_pub_.getNumSubscribers() > 0) {
      ros_feedback_msg_.header.stamp = ros::Time::now();
      for (int i = 0; i < rotor_num_; i++) {
        // To round/m
        ros_feedback_msg_.esc_telemetry[i].rpm = rotor_feedback_velocities_[i] * kRadPerSecToRpm;
      }
      ros_feedback_pub_.publish(ros_feedback_msg_);
    }
  }

 private:

  double DshotToRpm(const double& dshot){

    //! Model 1: [k0, k1, k2], y = k0 + k1 * x + k2 * x * x
    //! where x is rpm, y is dshot command
    const double a = dshot_rpm_coefficient_[2];
    const double b = dshot_rpm_coefficient_[1];
    const double c = dshot_rpm_coefficient_[0]-dshot;
    double discriminant = b * b - 4 * a * c;
    if (discriminant < 0){
      ROS_WARN("%s plugin warning: Negative discriminant in DshotToRpm", kPluginName.c_str());
      discriminant = 0;
    }
    return (-b + std::sqrt(discriminant)) / (2 * a);

  }


  const std::string kPluginName = "GazeboRotorPlugin";
  const double kCommandTimeout = 0.1;
  // Pointer to the gazebo objects
  physics::ModelPtr model_;
  sdf::ElementPtr sdf_;
  physics::WorldPtr world_;

  std::vector<physics::JointPtr> rotor_joints_;
  std::vector<physics::LinkPtr> rotor_links_;
  std::vector<std::string> rotor_joints_name;
  std::vector<std::string> rotor_links_name;
  std::vector<int> rotor_turning_directions_;

  double last_update_time_;
  double last_recv_command_time_;

  // Pointer to the update event connection.
  event::ConnectionPtr update_connection_;

  // Parameters
  std::string robot_namespace_;

  int rotor_num_;

  // Parameters for rotor model
  double motor_coefficient_;
  double moment_motor_ratio_;
  // Parameters for dshot rpm mapping model
  GzVector3d dshot_rpm_coefficient_;

  // Parameters for rotor drag model
  double rotor_drag_coefficient_;
  double rolling_moment_coefficient_;

  // Parameters for rotor velocity saturation
  double max_rot_velocity_;

  // Parameters for rotor velocity filter
  double time_constant_up_;
  double time_constant_down_;

  // Parameters for slow down rotor joint velocities in sim
  double rotor_velocity_slowdown_sim_;


  // ros
  ros::NodeHandle model_nh_;
  bool ros_setup_;

  // ros subscribe command and apply to rotor joint
  std::string ros_command_topic_name_;
  ros::Subscriber ros_command_sub_;

  std::vector<double> rotor_command_velocities_;
  std::vector<FirstOrderFilter<double>::Ptr> rotor_velocity_filters_;

  // ros publish feedback
  std::string ros_feedback_topic_name_;
  ros::Publisher ros_feedback_pub_;
  std::vector<double> rotor_feedback_velocities_;
  mavros_msgs::ESCTelemetry ros_feedback_msg_;
};

} // namespace gazebo {

