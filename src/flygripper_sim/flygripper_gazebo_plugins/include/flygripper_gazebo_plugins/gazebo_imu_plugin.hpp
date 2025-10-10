#pragma once
#pragma once

#include <deque>
#include <Eigen/Eigen>

// ROS
#include <ros/ros.h>
#include <pluginlib/class_loader.h>
#include <sensor_msgs/Imu.h>

// Gazebo
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <gazebo/common/Plugin.hh>

#include "flygripper_gazebo_plugins/gazebo_utils.hpp"



namespace gazebo {

// Default values for use with ADIS16448 IMU
static constexpr double kDefaultAdisGyroscopeNoiseDensity =
    2.0 * 35.0 / 3600.0 / 180.0 * M_PI;
static constexpr double kDefaultAdisGyroscopeRandomWalk =
    2.0 * 4.0 / 3600.0 / 180.0 * M_PI;
static constexpr double kDefaultAdisGyroscopeBiasCorrelationTime =
    1.0e+3;
static constexpr double kDefaultAdisGyroscopeTurnOnBiasSigma =
    0.5 / 180.0 * M_PI;
static constexpr double kDefaultAdisAccelerometerNoiseDensity =
    2.0 * 2.0e-3;
static constexpr double kDefaultAdisAccelerometerRandomWalk =
    2.0 * 3.0e-3;
static constexpr double kDefaultAdisAccelerometerBiasCorrelationTime =
    300.0;
static constexpr double kDefaultAdisAccelerometerTurnOnBiasSigma =
    20.0e-3 * 9.8;

// Earth's gravity in Huzhou (lat=+30.861degN, lon=+120.058degE, h=+1587m, WGS84)
static constexpr double kDefaultGravityMagnitude = 9.78903;

struct ImuParameters {
  /// Gyroscope noise density (two-sided spectrum) [rad/s/sqrt(Hz)]
  double gyroscope_noise_density;
  /// Gyroscope bias random walk [rad/s/s/sqrt(Hz)]
  double gyroscope_random_walk;
  /// Gyroscope bias correlation time constant [s]
  double gyroscope_bias_correlation_time;
  /// Gyroscope turn on bias standard deviation [rad/s]
  double gyroscope_turn_on_bias_sigma;

  /// Accelerometer noise density (two-sided spectrum) [m/s^2/sqrt(Hz)]
  double accelerometer_noise_density;
  /// Accelerometer bias random walk. [m/s^2/s/sqrt(Hz)]
  double accelerometer_random_walk;
  /// Accelerometer bias correlation time constant [s]
  double accelerometer_bias_correlation_time;
  /// Accelerometer turn on bias standard deviation [m/s^2]
  double accelerometer_turn_on_bias_sigma;

  /// Norm of the gravitational acceleration [m/s^2]
  double gravity_magnitude;
};

class GazeboImuPlugin : public ModelPlugin {

 public:
  using NormalDistribution = std::normal_distribution<double>;

  GazeboImuPlugin():
      random_generator_(random_device_())
  { }

  virtual ~GazeboImuPlugin() = default;

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

    // Get noise parameter
    getSdfParam<double>(kPluginName, sdf_,
                        "gyroscopeNoiseDensity",
                        imu_parameters_.gyroscope_noise_density,
                        kDefaultAdisGyroscopeNoiseDensity);
    getSdfParam<double>(kPluginName, sdf_,
                        "gyroscopeBiasRandomWalk",
                        imu_parameters_.gyroscope_random_walk,
                        kDefaultAdisGyroscopeRandomWalk);
    getSdfParam<double>(kPluginName, sdf_,
                        "gyroscopeBiasCorrelationTime",
                        imu_parameters_.gyroscope_bias_correlation_time,
                        kDefaultAdisGyroscopeBiasCorrelationTime);
    assert(imu_parameters_.gyroscope_bias_correlation_time > 0.0);
    getSdfParam<double>(kPluginName, sdf_,
                        "gyroscopeTurnOnBiasSigma",
                        imu_parameters_.gyroscope_turn_on_bias_sigma,
                        kDefaultAdisGyroscopeTurnOnBiasSigma);
    getSdfParam<double>(kPluginName, sdf_,
                        "accelerometerNoiseDensity",
                        imu_parameters_.accelerometer_noise_density,
                        kDefaultAdisAccelerometerNoiseDensity);
    getSdfParam<double>(kPluginName, sdf_,
                        "accelerometerRandomWalk",
                        imu_parameters_.accelerometer_random_walk,
                        kDefaultAdisAccelerometerRandomWalk);
    getSdfParam<double>(kPluginName, sdf_,
                        "accelerometerBiasCorrelationTime",
                        imu_parameters_.accelerometer_bias_correlation_time,
                        kDefaultAdisAccelerometerBiasCorrelationTime);
    assert(imu_parameters_.accelerometer_bias_correlation_time > 0.0);
    getSdfParam<double>(kPluginName, sdf_,
                        "accelerometerTurnOnBiasSigma",
                        imu_parameters_.accelerometer_turn_on_bias_sigma,
                        kDefaultAdisAccelerometerTurnOnBiasSigma);

    unsigned int random_engine_seed;
    getSdfParam<unsigned int>(kPluginName,
                              sdf_,
                              "randomEngineSeed",
                              random_engine_seed,
                              std::chrono::system_clock::now().time_since_epoch().count());
    random_generator_.seed(random_engine_seed);
    standard_normal_distribution_ = std::normal_distribution<double>(0.0, 1.0);

    gravity_W_ = world_->Gravity();
    imu_parameters_.gravity_magnitude = gravity_W_.Length();

    for (int i = 0; i < 3; ++i) {
      gyroscope_turn_on_bias_[i] =
          imu_parameters_.gyroscope_turn_on_bias_sigma *
              standard_normal_distribution_(random_generator_);
      accelerometer_turn_on_bias_[i] =
          imu_parameters_.accelerometer_turn_on_bias_sigma *
              standard_normal_distribution_(random_generator_);
    }

    gyroscope_bias_.setZero();
    accelerometer_bias_.setZero();

    // ROS
    // Get parameters/settings for namespace from ROS param server
    model_nh_ = ros::NodeHandle(robot_namespace_);
    if (!getSdfParam<std::string>(kPluginName, sdf_,
                                  "rosImuTopicName", ros_imu_topic_name_)){
      return;
    }
    ros_setup_ = false;

    // Get frame id
    // TODO: namespace/link_name_ or custome?
    getSdfParam<std::string>(kPluginName, sdf_,
                             "frameId", frame_id_, link_name_);


    // Get topic publish frequency
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
    last_publish_time_ = last_update_time_ = world_->SimTime().Double();

    // Init ros imu message
    ros_imu_msg_.header.frame_id = frame_id_;
    // We assume uncorrelated noise on the 3 channels -> only set diagonal
    // elements. Only the broadband noise component is considered, specified as a
    // continuous-time density (two-sided spectrum); not the true covariance of
    // the measurements.
    for (int i = 0; i < 9; i++) {
      if (i == 0 || i == 4 || i == 8) {
        ros_imu_msg_.angular_velocity_covariance[i] =
            imu_parameters_.gyroscope_noise_density *
                imu_parameters_.gyroscope_noise_density;

        ros_imu_msg_.linear_acceleration_covariance[i] =
            imu_parameters_.accelerometer_noise_density *
                imu_parameters_.accelerometer_noise_density;
      } else {
        ros_imu_msg_.angular_velocity_covariance[i] = 0.0;

        ros_imu_msg_.linear_acceleration_covariance[i] = 0.0;

      }

      if (i == 0 || i==3 || i==4 || i==7 || i==8) {
        ros_imu_msg_.orientation_covariance[i] = -1.0;
      } else {
        ros_imu_msg_.orientation_covariance[i] = 0.0;
      }
    }

    // Update connection
    update_connection_ =
        gazebo::event::Events::ConnectWorldUpdateBegin
            (boost::bind(&GazeboImuPlugin::Update, this, _1));

    ROS_INFO("%s plugin loaded in namespace: %s",
             kPluginName.c_str(), robot_namespace_.c_str());
  }

  void SetupROS(){
    ros_imu_pub_ = model_nh_.advertise<sensor_msgs::Imu>(
        ros_imu_topic_name_, 1);
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
    // Get acc and gyro from Gazebo
    GzPose3d T_W_I = link_->WorldPose();
    GzQuaterniond q_W_I = T_W_I.Rot();
    GzVector3d acceleration_I =
        link_->RelativeLinearAccel() - q_W_I.RotateVectorReverse(gravity_W_);
    GzVector3d angular_vel_I = link_->RelativeAngularVel();
    // Trans to Eigen
    Eigen::Vector3d linear_acceleration_I(acceleration_I.X(), acceleration_I.Y(),
                                          acceleration_I.Z());
    Eigen::Vector3d angular_velocity_I(angular_vel_I.X(), angular_vel_I.Y(),
                                       angular_vel_I.Z());
    // Add noise with update rate
    AddNoise(linear_acceleration_I, angular_velocity_I, dt);

    // Publish ros imu msg
    // Trick with current_time with type of common::Time operation to avoid double equal issue
    if (current_time - last_publish_time_ >= publish_duration_){
      last_publish_time_ = current_time_sec;

      ros_imu_msg_.header.stamp.sec = current_time.sec;
      ros_imu_msg_.header.stamp.nsec = current_time.nsec;
      ros_imu_msg_.angular_velocity.x = angular_velocity_I.x();
      ros_imu_msg_.angular_velocity.y = angular_velocity_I.y();
      ros_imu_msg_.angular_velocity.z = angular_velocity_I.z();

      ros_imu_msg_.linear_acceleration.x = linear_acceleration_I.x();
      ros_imu_msg_.linear_acceleration.y = linear_acceleration_I.y();
      ros_imu_msg_.linear_acceleration.z = linear_acceleration_I.z();

      ros_imu_msg_.orientation.x = q_W_I.X();
      ros_imu_msg_.orientation.y = q_W_I.Y();
      ros_imu_msg_.orientation.z = q_W_I.Z();
      ros_imu_msg_.orientation.w = q_W_I.W();

      if (ros_imu_pub_.getNumSubscribers() > 0){
        ros_imu_pub_.publish(ros_imu_msg_);
      }
    }

  }

  void Reset(){
    if (ros_setup_) {
      ros_imu_pub_.shutdown();
    }
    ros_setup_ = false;
  }

  void AddNoise(
      Eigen::Ref<Eigen::Vector3d> linear_acceleration,
      Eigen::Ref<Eigen::Vector3d> angular_velocity,
      const double dt){
    // TODO numerical validation of linear acceleration and angular velocity
//    GZ_ASSERT(linear_acceleration != nullptr, "Linear acceleration was null.");
//    GZ_ASSERT(angular_velocity != nullptr, "Angular velocity was null.");
    GZ_ASSERT(dt > 0.0, "Change in time must be greater than 0.");

    // Gyrosocpe
    double tau_g = imu_parameters_.gyroscope_bias_correlation_time;
    // Discrete-time standard deviation equivalent to an "integrating" sampler
    // with integration time dt.
    double sigma_g_d = 1 / sqrt(dt) * imu_parameters_.gyroscope_noise_density;
    double sigma_b_g = imu_parameters_.gyroscope_random_walk;
    // Compute exact covariance of the process after dt [Maybeck 4-114].
    double sigma_b_g_d = sqrt(-sigma_b_g * sigma_b_g * tau_g / 2.0 *
        (exp(-2.0 * dt / tau_g) - 1.0));
    // Compute state-transition.
    double phi_g_d = exp(-1.0 / tau_g * dt);
    // Simulate gyroscope noise processes and add them to the true angular rate.
    for (int i = 0; i < 3; ++i) {
      gyroscope_bias_[i] =
          phi_g_d * gyroscope_bias_[i] +
              sigma_b_g_d * standard_normal_distribution_(random_generator_);
      angular_velocity[i] =
          angular_velocity[i] + gyroscope_bias_[i] +
              sigma_g_d * standard_normal_distribution_(random_generator_) +
              gyroscope_turn_on_bias_[i];
    }

    // Accelerometer
    double tau_a = imu_parameters_.accelerometer_bias_correlation_time;
    // Discrete-time standard deviation equivalent to an "integrating" sampler
    // with integration time dt.
    double sigma_a_d = 1 / sqrt(dt) * imu_parameters_.accelerometer_noise_density;
    double sigma_b_a = imu_parameters_.accelerometer_random_walk;
    // Compute exact covariance of the process after dt [Maybeck 4-114].
    double sigma_b_a_d = sqrt(-sigma_b_a * sigma_b_a * tau_a / 2.0 *
        (exp(-2.0 * dt / tau_a) - 1.0));
    // Compute state-transition.
    double phi_a_d = exp(-1.0 / tau_a * dt);
    // Simulate accelerometer noise processes and add them to the true linear
    // acceleration.
    for (int i = 0; i < 3; ++i) {
      accelerometer_bias_[i] =
          phi_a_d * accelerometer_bias_[i] +
              sigma_b_a_d * standard_normal_distribution_(random_generator_);
      linear_acceleration[i] =
          linear_acceleration[i] + accelerometer_bias_[i] +
              sigma_a_d * standard_normal_distribution_(random_generator_) +
              accelerometer_turn_on_bias_[i];
    }
  }

 private:
  const std::string kPluginName = "GazeboImuPlugin";

  // Pointer to the gazebo objects
  physics::ModelPtr model_;
  physics::WorldPtr world_;
  sdf::ElementPtr sdf_;
  physics::LinkPtr link_;

  double last_update_time_;
  double last_publish_time_;

  // Pointer to the update event connection.
  event::ConnectionPtr update_connection_;

  // Parameters
  std::string robot_namespace_;
  std::string link_name_;
  std::string frame_id_;
  double publish_duration_;

  ImuParameters imu_parameters_;
  std::random_device random_device_;
  std::mt19937 random_generator_;
  NormalDistribution standard_normal_distribution_;

  GzVector3d gravity_W_;
  Eigen::Vector3d gyroscope_turn_on_bias_;
  Eigen::Vector3d accelerometer_turn_on_bias_;
  Eigen::Vector3d gyroscope_bias_;
  Eigen::Vector3d accelerometer_bias_;

  // ros
  ros::NodeHandle model_nh_;
  bool ros_setup_;
  ros::Publisher ros_imu_pub_;
  std::string ros_imu_topic_name_;
  sensor_msgs::Imu ros_imu_msg_;
};

} // namespace gazebo {

