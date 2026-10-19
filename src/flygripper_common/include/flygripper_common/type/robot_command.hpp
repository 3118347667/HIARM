#pragma once
#include <memory>
#include <Eigen/Dense>
#include <iostream>
#include "mavros_msgs/AttitudeTarget.h"
#include "mavros_msgs/ActuatorControl.h"

#include "flygripper_msgs/RotorCommand.h"
#include "flygripper_msgs/ActuatorCommand.h"

#include "flygripper_common/ros/conversions.hpp"
#include "flygripper_common/type/base.hpp"

namespace flygripper_common{
enum class  RobotCommandMode: int{
  MODE_ATTITUDE = 0,
  MODE_BODY_RATE = 1,
  MODE_SINGLE_ROTOR = 2,
  MODE_BODY_RATE_CUSTOMIZED = 3,
  MODE_SIZE = 4
};

inline std::string CommandModeToString(RobotCommandMode command_mode){
  switch(command_mode){
  case RobotCommandMode::MODE_SINGLE_ROTOR:
    return "SINGLE_ROTOR";
  case RobotCommandMode::MODE_BODY_RATE:
    return "BODY_RATE";
  case RobotCommandMode::MODE_ATTITUDE:
    return "ATTITUDE";
  default:
    return "Unknown";
  }
}

class RobotCommand
{
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  using Ptr = std::shared_ptr<RobotCommand>;
  Scalar stamp{INF};
  /* ------------------ Rotor Command ---------------------- */
  RobotCommandMode rotor_mode{RobotCommandMode::MODE_SIZE};
  //! for attitude mode and body rate mode as well as body rate customized mode
  Scalar collective_thrust{INF};
  Scalar collective_thrust_command{INF};
  //! for attitude mode
  Quaternion attitude{Vector<4>::Constant(INF)};
  Quaternion attitude_command{Vector<4>::Constant(INF)};
  //! for body rate mode as well as body rate customized mode
  Vector<3> bodyrates{Vector<3>::Constant(INF)};
  //! for single rotor mode
  Vector<3> torque{Vector<3>::Constant(INF)}; // intermediate variables
  Vector<4> rotor_thrusts{Vector<4>::Constant(INF)}; // intermediate variables
  Vector<4> rotor_velocities{Vector<4>::Constant(INF)};
  Vector<4> rotor_velocities_command{Vector<4>::Constant(INF)};
  //! for body rate customized mode
  // TODO: mess
  int compensation_method{-1};
  Scalar delta_mass{INF};

 public:
  RobotCommand(){}
  RobotCommand(const Scalar& time,
               const RobotCommandMode& mode){
    stamp = time;
    rotor_mode = mode;
    setZero();
  }
  ~RobotCommand() = default;

  void setAttitude(const Scalar angle,
                   const Vector<3>& axis = Vector<3>::UnitZ()) {
    attitude = Quaternion(Eigen::AngleAxis<Scalar>(angle, axis));
  }

  void setZero(){
    switch (rotor_mode) {
    case RobotCommandMode::MODE_ATTITUDE:
      attitude.setIdentity();
      collective_thrust = 0;
      collective_thrust_command = 0;
      break;
    case RobotCommandMode::MODE_BODY_RATE:
      bodyrates.setZero();
      collective_thrust = 0;
      collective_thrust_command = 0;
      break;
    case RobotCommandMode::MODE_SINGLE_ROTOR:
      collective_thrust = 0;
      torque.setZero();
      rotor_thrusts.setZero();
      rotor_velocities.setZero();
      rotor_velocities_command.setZero();
      break;
    case RobotCommandMode::MODE_BODY_RATE_CUSTOMIZED:
      bodyrates.setZero();
      collective_thrust = 0;
      break;
    default:
      break;
    }
  }

  bool operator==(const RobotCommand& rhs) const {
    if (stamp != rhs.stamp) {
      return false;
    }
    if (rotor_mode != rhs.rotor_mode) {
      return false;
    }
    switch (rotor_mode) {
    case RobotCommandMode::MODE_ATTITUDE:
      return (collective_thrust == rhs.collective_thrust &&
          attitude.coeffs().isApprox(rhs.attitude.coeffs(), 1e-3));
    case RobotCommandMode::MODE_BODY_RATE:
      return (collective_thrust == rhs.collective_thrust &&
          bodyrates.isApprox(rhs.bodyrates, 1e-3));
    case RobotCommandMode::MODE_SINGLE_ROTOR:
      return ( rotor_velocities.isApprox(rhs.rotor_velocities, 1e-3));
    case RobotCommandMode::MODE_BODY_RATE_CUSTOMIZED:
      return (collective_thrust == rhs.collective_thrust &&
          bodyrates.isApprox(rhs.bodyrates, 1e-3));
    default:
      return false;
    }
  }

  inline bool valid() const {
    return std::isfinite(stamp) &&
           ((rotor_mode == RobotCommandMode::MODE_ATTITUDE && attitude.coeffs().allFinite() && std::isfinite(collective_thrust)) ||
            (rotor_mode == RobotCommandMode::MODE_BODY_RATE && bodyrates.allFinite() && std::isfinite(collective_thrust)) ||
            (rotor_mode == RobotCommandMode::MODE_SINGLE_ROTOR && rotor_velocities.allFinite()) ||
            (rotor_mode == RobotCommandMode::MODE_BODY_RATE_CUSTOMIZED && bodyrates.allFinite() && std::isfinite(collective_thrust)));
  }

  inline bool isReceived(const Scalar &now, Scalar timeout=0.1) const{
    if (!std::isfinite(stamp)){
      ROS_ERROR_ONCE("RobotCommand has never been received!");
      return false;
    } else if (now - stamp > timeout){
      ROS_ERROR("RobotCommand is not received for %f seconds!", now - stamp);
      return false;
    }
    return true;
  }

  friend std::ostream& operator<<(std::ostream& os, const RobotCommand& p) {
    os.precision(3);
    os << std::scientific;
    os << "rotor_mode: " << CommandModeToString(p.rotor_mode) << std::endl;
    os << "stamp: " << p.stamp << std::endl;
//    switch (p.rotor_mode) {
//        case RobotCommandMode::MODE_SINGLE_ROTOR:
    os << "collective_thrust: " << p.collective_thrust << std::endl;
    os << "torque: " << p.torque.transpose() << std::endl;
    os << "rotor_thrusts: " << p.rotor_thrusts.transpose() << std::endl;
    os << "rotor_velocities: " << p.rotor_velocities.transpose() << std::endl;
//          break;
//        case RobotCommandMode::MODE_BODY_RATE:
//          os << "collective_thrust: " << p.collective_thrust << std::endl;
    os << "bodyrates: " << p.bodyrates.transpose() << std::endl;
//          break;
//        case RobotCommandMode::MODE_ATTITUDE:
//          os << "collective_thrust: " << p.collective_thrust << std::endl;
    os << "attitude: " << p.attitude.coeffs().transpose() << std::endl;
//          break;
//        default:
//          break;
//    }
    os.precision();
    os.unsetf(std::ios::scientific);
    return os;
  }

};

//! Robot Command <-> Rotor Command Msg TODO:DEPRECATED for rotor command message
inline void fromRosMsg(flygripper_msgs::RotorCommandConstPtr msg, RobotCommand& data){
  data.stamp = msg->header.stamp.toSec();
  data.rotor_mode = static_cast<RobotCommandMode>(msg->mode);

  switch (data.rotor_mode) {
  case RobotCommandMode::MODE_SINGLE_ROTOR:
    if(msg->rotor_velocities.size() != 4){
      ROS_ERROR("RotorCommand message rotor velocities size mismatch, "
                "expected 4, got %ld",
                msg->rotor_velocities.size());
      return;
    }
    for (int i = 0; i < 4; i++){
      data.rotor_velocities(i) = msg->rotor_velocities[i];
    }
    break;
  case RobotCommandMode::MODE_BODY_RATE:
    data.collective_thrust = msg->collective_thrust;
    fromRosVec3(msg->bodyrates, data.bodyrates);
    break;
  case RobotCommandMode::MODE_ATTITUDE:
    data.collective_thrust = msg->collective_thrust;
    fromRosQuat(msg->attitude, data.attitude);
    break;
  default:
    ROS_ERROR("Unknown RobotCommandMode");
    break;
  }
}
//TODO:DEPRECATED for rotor command message
inline void toRosMsg(const RobotCommand& data, flygripper_msgs::RotorCommand& msg){
  msg.header.stamp = ros::Time(data.stamp);
  msg.mode = static_cast<int>(data.rotor_mode);

  switch (data.rotor_mode) {
  case RobotCommandMode::MODE_SINGLE_ROTOR:
    msg.rotor_velocities.resize(4);
    for (int i = 0; i < 4; i++){
      msg.rotor_velocities[i] = data.rotor_velocities(i);
    }
    break;
  case RobotCommandMode::MODE_BODY_RATE:
    msg.collective_thrust = data.collective_thrust;
    toRosVec3(data.bodyrates, msg.bodyrates);
    break;
  case RobotCommandMode::MODE_ATTITUDE:
    msg.collective_thrust = data.collective_thrust;
    toRosQuat(data.attitude, msg.attitude);
    break;
  default:
    ROS_ERROR("Unknown RobotCommandMode");
    break;
  }
}

//! Robot Command -> Mavros ActuatorControl Msg in MODE_SINGLE_ROTOR
inline void toRosMsg(const RobotCommand& data, mavros_msgs::ActuatorControl& msg)
{
  if (data.rotor_mode == RobotCommandMode::MODE_SINGLE_ROTOR) {
    if (!data.rotor_velocities_command.allFinite()) {
      ROS_WARN("Invalid data for mavros_msgs::ActuatorControl message in MODE_SINGLE_ROTOR.");
      return;
    }

    msg.header.stamp = ros::Time(data.stamp);
    msg.header.frame_id = std::string("FCU");
    msg.group_mix = mavros_msgs::ActuatorControl::PX4_MIX_FLIGHT_CONTROL; // =0

    for (int i = 0; i < 4; i++) {
      msg.controls[i] = data.rotor_velocities_command[i];
    }
    for (int i = 4; i < 8; i++) {
      msg.controls[i] = 0.0;
    }
  }
  else {
    ROS_WARN("Only MODE_SINGLE_ROTOR is supported for mavros_msgs::ActuatorControl message.");
    return;
  }
}

//! Robot Command -> Mavros ActuatorControl Msg in MODE_BODY_RATE_CUSTOMIZED
inline void toRosMsg(const RobotCommand& data,
                     const Scalar actuator_position,
                     mavros_msgs::ActuatorControl& msg)
{
  if (data.rotor_mode == RobotCommandMode::MODE_BODY_RATE_CUSTOMIZED) {
    if (!data.bodyrates.allFinite() ||
        !std::isfinite(data.collective_thrust) ||
        !std::isfinite(actuator_position) ||
        !std::isfinite(data.delta_mass) ||
        data.compensation_method < 0 || data.compensation_method > 2) {
      ROS_WARN("Invalid data for mavros_msgs::ActuatorControl message in MODE_BODY_RATE_CUSTOMIZED.");
      return;
    }

    msg.header.stamp = ros::Time(data.stamp);
    msg.header.frame_id = std::string("FCU");
    msg.group_mix = mavros_msgs::ActuatorControl::PX4_MIX_PAYLOAD; // =2
    msg.controls[0] = data.collective_thrust;

    // transform from ros baselink frame(front, left, up) to fc frame(front, right, down)
    // which refer to implementation in mavros setpoint raw plugin in /mavros/src/plugins/setpoint_raw.cp
    Eigen::Affine3d baselink_to_fc(Quaternion(
        Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitZ()) *
        Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitY()) *
        Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX())));
    Vector<3> bodyrates_fc = baselink_to_fc * data.bodyrates;
    msg.controls[1] = bodyrates_fc[0];
    msg.controls[2] = bodyrates_fc[1];
    msg.controls[3] = bodyrates_fc[2];

    msg.controls[4] = static_cast<Scalar>(data.compensation_method);
    msg.controls[5] = actuator_position;
    msg.controls[6] = data.delta_mass;

    msg.controls[7] = 0.0;

  }
  else {
    ROS_WARN("Only MODE_BODY_RATE_CUSTOMIZED is supported for mavros_msgs::ActuatorControl message.");
    return;
  }
}

//! Robot Command -> Mavros AttitudeTarget Msg
inline void toRosMsg(const RobotCommand& data, mavros_msgs::AttitudeTarget& msg)
{

  if (data.rotor_mode == RobotCommandMode::MODE_ATTITUDE){
    if( data.attitude_command.coeffs().allFinite() &&
        std::isfinite(data.collective_thrust_command)){
      msg.header.stamp = ros::Time(data.stamp);
      msg.header.frame_id = std::string("FCU");
      msg.type_mask = mavros_msgs::AttitudeTarget::IGNORE_ROLL_RATE |
          mavros_msgs::AttitudeTarget::IGNORE_PITCH_RATE |
          mavros_msgs::AttitudeTarget::IGNORE_YAW_RATE;
      toRosQuat(data.attitude_command, msg.orientation);
      msg.thrust = data.collective_thrust_command;
    } else{
      ROS_WARN("Invalid data for attitude mode mavros_msgs::AttitudeTarget message.");
      return;
    }

  } else if (data.rotor_mode == RobotCommandMode::MODE_BODY_RATE){

    if(data.bodyrates.allFinite() &&
        std::isfinite(data.collective_thrust_command)){
      msg.header.stamp = ros::Time(data.stamp);
      msg.header.frame_id = std::string("FCU");
      msg.type_mask = mavros_msgs::AttitudeTarget::IGNORE_ATTITUDE;
      toRosVec3(data.bodyrates, msg.body_rate);
      msg.thrust = data.collective_thrust_command;
    } else{
      ROS_WARN("Invalid data for bodyrate mode mavros_msgs::AttitudeTarget message.");
      return;
    }

  } else {
    ROS_WARN("Only MODE_ATTITUDE and MODE_BODY_RATE "
             "are supported for mavros_msgs::AttitudeTarget message.");
    return;
  }
}

} //namespace flygripper_common