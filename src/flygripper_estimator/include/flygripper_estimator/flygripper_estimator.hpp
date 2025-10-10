#pragma once
#include <ros/ros.h>

#include <algorithm>

#include "flygripper_common/filter/butterworth.hpp"
#include "flygripper_common/ros/parameters.hpp"
#include "flygripper_common/type/actuator_state.hpp"
#include "flygripper_common/type/base.hpp"
#include "flygripper_common/type/const.hpp"
#include "flygripper_common/type/imu.hpp"
#include "flygripper_common/type/odometry.hpp"
#include "flygripper_common/type/robot_state.hpp"
#include "flygripper_common/type/rotor_speed.hpp"

namespace flygripper_estimator{

// using namespace flygripper_common;
using flygripper_common::Scalar;
using flygripper_common::Matrix;
using flygripper_common::Vector;

using flygripper_common::getParam;
using flygripper_common::ButterworthLPF;
using flygripper_common::kGravityVector;
using flygripper_common::RobotState;
using flygripper_common::Imu;
using flygripper_common::Odometry;
using flygripper_common::RotorSpeed;
using flygripper_common::ActuatorState;


struct Parameter{
  Scalar lpf_cutoff_frequency;
  Scalar lpf_sampling_frequency;
  Scalar update_frequency;
  Scalar timeout;
  Parameter(const ros::NodeHandle& pnh){
    if (!getParam("lpf_cutoff_frequency", lpf_cutoff_frequency, pnh) ||
        !getParam("lpf_sampling_frequency", lpf_sampling_frequency, pnh) ||
        !getParam("update_frequency", update_frequency, pnh) ||
        !getParam("timeout", timeout, pnh)){
      throw ros::Exception("Failed to load parameters");
    }
  }

};

class Estimator{
 public:
  Estimator(ros::NodeHandle& nh, ros::NodeHandle& pnh):param_(pnh)
  {
    imu_sub_ = nh.subscribe<sensor_msgs::Imu>("imu",
                                              100,
                                              boost::bind(&Estimator::rosCallback<sensor_msgs::Imu, Imu>, this, _1, std::ref(imu_data_)),
                                              ros::VoidConstPtr(),
                                              ros::TransportHints().tcpNoDelay());
    odom_sub_ = nh.subscribe<nav_msgs::Odometry>("odom",
                                                 100,
                                                 boost::bind(&Estimator::rosCallback<nav_msgs::Odometry, Odometry>, this, _1, std::ref(odom_data_)),
                                                 ros::VoidConstPtr(),
                                                 ros::TransportHints().tcpNoDelay());
    actuator_state_sub_ = nh.subscribe<flygripper_msgs::ActuatorState>("actuator_state",
                                                                       100,
                                                                       boost::bind(&Estimator::rosCallback<flygripper_msgs::ActuatorState, ActuatorState>, this, _1, std::ref(actuator_state_data_)),
                                                                       ros::VoidConstPtr(),
                                                                       ros::TransportHints().tcpNoDelay());
    rotor_speed_sub_ = nh.subscribe<mavros_msgs::ESCTelemetry>("rotor_speed",
                                                               100,
                                                               boost::bind(&Estimator::rosCallback<mavros_msgs::ESCTelemetry, RotorSpeed>, this, _1, std::ref(rotor_speed_data_)),
                                                               ros::VoidConstPtr(),
                                                               ros::TransportHints().tcpNoDelay());

    odom_pub_ = nh.advertise<nav_msgs::Odometry>("center_odom", 100);
    robot_state_pub_ = nh.advertise<flygripper_msgs::RobotState>("robot_state", 100);

    acc_lpf_ = std::make_shared<ButterworthLPF<3>>(param_.lpf_cutoff_frequency, param_.lpf_sampling_frequency);
    omega_lpf_ = std::make_shared<ButterworthLPF<3>>(param_.lpf_cutoff_frequency, param_.lpf_sampling_frequency);
    rotor_speed_lpf_ = std::make_shared<ButterworthLPF<4>>(param_.lpf_cutoff_frequency, param_.lpf_sampling_frequency);

    update_timer_ = nh.createTimer(ros::Duration(1.0/param_.update_frequency),
                                   &Estimator::update,
                                   this);
  }

  ~Estimator() = default;

  //! General callback
  template< typename Msg, typename Data>
  void rosCallback(boost::shared_ptr<Msg const> msg, Data& data){
    fromRosMsg(msg, data);
  }

  void update(ros::TimerEvent const& event){
    // TODO: implement EKF
    double now = event.current_real.toSec();
    if(imu_data_.isReceived(now, param_.timeout) && imu_data_.valid() &&
        odom_data_.isReceived(now, param_.timeout) && odom_data_.valid() &&
        rotor_speed_data_.isReceived(now, param_.timeout)&& rotor_speed_data_.valid() &&
        actuator_state_data_.isReceived(now, param_.timeout)&& actuator_state_data_.valid())
    {
      omega_lpf_->apply(imu_data_.omega);
      acc_lpf_->apply(imu_data_.acc);
      rotor_speed_lpf_->apply(rotor_speed_data_.rot_speed);

      robot_state_.stamp = ros::Time::now().toSec();
      robot_state_.pos = odom_data_.pos;
      robot_state_.setQ(odom_data_.att);
      robot_state_.vel = odom_data_.vel;
      robot_state_.omega = imu_data_.omega; // lpf : omega_lpf_->get();
      robot_state_.acc = odom_data_.att * imu_data_.acc+ kGravityVector; //lpf :  odom_data_.att * acc_lpf_->get() + kGravityVector; // acc in world frame
      robot_state_.alpha = omega_lpf_->getDerivative();

      robot_state_.rot_speed = rotor_speed_lpf_->get();
      robot_state_.rot1_pos = actuator_state_data_.rot1_pos;
      robot_state_.rot2_pos = actuator_state_data_.rot2_pos;
      robot_state_.rot3_pos = actuator_state_data_.rot3_pos;
      robot_state_.rot4_pos = actuator_state_data_.rot4_pos;

      // as imu is installed in the left bottom (rotor 2) of the body,
      // we need to rotate the position, velocity and acceleration data to the center of the body
      // TODO: modify afterwards
//      const Vector<3> body_pos_in_imu = Vector<3>(-robot_state_.rot2_pos.x(),
//                                                  -robot_state_.rot2_pos.y(),
//                                                  0.0);
//      robot_state_.pos += robot_state_.getR() * body_pos_in_imu;
//      robot_state_.vel += robot_state_.getR() * (robot_state_.omega.cross(body_pos_in_imu));
//      robot_state_.acc += robot_state_.getR() *
//          (robot_state_.omega.cross(robot_state_.omega.cross(body_pos_in_imu)) +
//              robot_state_.alpha.cross(body_pos_in_imu));

      toRosMsg(robot_state_, robot_state_msg_);
      robot_state_pub_.publish(robot_state_msg_);
      toRosMsg(robot_state_, odom_msg_);
      odom_pub_.publish(odom_msg_);
    }

  }

 private:
  Parameter param_;
  ros::Timer update_timer_;

  ros::Subscriber imu_sub_;
  Imu imu_data_;

  ros::Subscriber odom_sub_;
  Odometry odom_data_;

  ros::Subscriber rotor_speed_sub_;
  RotorSpeed rotor_speed_data_;

  ros::Subscriber actuator_state_sub_;
  ActuatorState actuator_state_data_;

  RobotState robot_state_;
  flygripper_msgs::RobotState robot_state_msg_;
  ros::Publisher robot_state_pub_;

  nav_msgs::Odometry odom_msg_;
  ros::Publisher odom_pub_;

  ButterworthLPF<3>::Ptr acc_lpf_, omega_lpf_;
  ButterworthLPF<4>::Ptr rotor_speed_lpf_;
};
}
