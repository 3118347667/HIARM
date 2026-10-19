#pragma once
#include <memory>
#include <Eigen/Dense>
#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "flygripper_msgs/RobotState.h"

#include "flygripper_common/ros/conversions.hpp"
#include "flygripper_common/type/const.hpp"
#include "flygripper_common/type/base.hpp"

namespace flygripper_common{

class RobotState{
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  using Ptr = std::shared_ptr<RobotState>;
  enum StateIndex: unsigned int{
    POS = 0,  // pos: position
    POS_X = 0,
    POS_Y = 1,
    POS_Z = 2,
    N_POS = 3,

    ATT = 3,  // att: attitude
    ATT_X = 3,
    ATT_Y = 4,
    ATT_Z = 5,
    ATT_W = 6,
    N_ATT = 4,

    VEL = 7, // vel: linear velocity
    VEL_X = 7,
    VEL_Y = 8,
    VEL_Z = 9,
    N_VEL = 3,

    OME = 10, // omega: angular velocity
    OME_X = 10,
    OME_Y = 11,
    OME_Z = 12,
    N_OME = 3,

    ACC = 13, // acc: linear acceleration
    ACC_X = 13,
    ACC_Y = 14,
    ACC_Z = 15,
    N_ACC = 3,

    ALPHA = 16, // alpha: angular acceleration
    ALPHA_X = 16,
    ALPHA_Y = 17,
    ALPHA_Z = 18,
    N_ALPHA = 3,

    ROT_SPD = 19, // rotor Omega
    ROT1_SPD = 19,
    ROT2_SPD = 20,
    ROT3_SPD = 21,
    ROT4_SPD = 22,
    N_ROT = 4,

    ROT1_POS = 23,  // rotor1 position: x, y
    ROT1_POS_X = 23,
    ROT1_POS_Y = 24,
    N_ROT_POS = 2,

    ROT2_POS = 25,  // rotor2 position: x, y
    ROT2_POS_X = 25,
    ROT2_POS_Y = 26,

    ROT3_POS = 27,  // rotor3 position: x, y
    ROT3_POS_X = 27,
    ROT3_POS_Y = 28,

    ROT4_POS = 29,  // rotor4 position: x, y
    ROT4_POS_X = 29,
    ROT4_POS_Y = 30,

    JERK = 31, // jerk: linear jerk
    JERK_X = 31,
    JERK_Y = 32,
    JERK_Z = 33,
    N_JERK = 3,

    SNAP = 34, // snap: linear snap
    SNAP_X = 34,
    SNAP_Y = 35,
    SNAP_Z = 36,
    N_SNAP = 3,

    N_STATE = 37
  };
  Scalar stamp{INF};
  Vector<StateIndex::N_STATE> x{Vector<StateIndex::N_STATE>::Constant(INF)};

  // Rotor State
  Ref<Vector<3>> pos{x.segment<StateIndex::N_POS>(StateIndex::POS)};
  Ref<Vector<4>> att{x.segment<StateIndex::N_ATT>(StateIndex::ATT)};
  Ref<Vector<3>> vel{x.segment<StateIndex::N_VEL>(StateIndex::VEL)};
  Ref<Vector<3>> omega{x.segment<StateIndex::N_OME>(StateIndex::OME)};
  Ref<Vector<3>> acc{x.segment<StateIndex::N_ACC>(StateIndex::ACC)};
  Ref<Vector<3>> alpha{x.segment<StateIndex::N_ALPHA>(StateIndex::ALPHA)};

  Ref<Vector<3>> jerk{x.segment<StateIndex::N_JERK>(StateIndex::JERK)};
  Ref<Vector<3>> snap{x.segment<StateIndex::N_SNAP>(StateIndex::SNAP)};

  Ref<Vector<4>> rot_speed{x.segment<StateIndex::N_ROT>(StateIndex::ROT_SPD)};

  // Morphing State
  Eigen::Map<Eigen::Matrix<double, N_ROT_POS, N_ROT>> rot_pos{x.segment<N_ROT_POS*N_ROT>(ROT1_POS).data()};
  Ref<Vector<2>> rot1_pos{x.segment<StateIndex::N_ROT_POS>(StateIndex::ROT1_POS)};
  Ref<Vector<2>> rot2_pos{x.segment<StateIndex::N_ROT_POS>(StateIndex::ROT2_POS)};
  Ref<Vector<2>> rot3_pos{x.segment<StateIndex::N_ROT_POS>(StateIndex::ROT3_POS)};
  Ref<Vector<2>> rot4_pos{x.segment<StateIndex::N_ROT_POS>(StateIndex::ROT4_POS)};


 public:
  RobotState() {
    // NOTE：Invalid with default constructor
  }

  RobotState(const Scalar& t){
    stamp = t;
    setZero();
  }

  RobotState(const Scalar& t,
             Vector<3>& position,
             const Scalar yaw):
      RobotState(t)
  {
    pos = position;
    setQ(yaw);
  }

  void setZero(){
    x.setZero();
    x(ATT_W) = 1.0;
    //TODO: set init actuator state
  }

  ~RobotState() = default;

  void setQ(const Quaternion quaternion) {
    x(StateIndex::ATT_W) = quaternion.w();
    x(StateIndex::ATT_X) = quaternion.x();
    x(StateIndex::ATT_Y) = quaternion.y();
    x(StateIndex::ATT_Z) = quaternion.z();
  }

  void setQ(const Scalar angle, const Vector<3>& axis = Vector<3>::UnitZ()) {
    const Quaternion q_aa(Eigen::AngleAxis<Scalar>(angle, axis));
    x(StateIndex::ATT_W) = q_aa.w();
    x(StateIndex::ATT_X) = q_aa.x();
    x(StateIndex::ATT_Y) = q_aa.y();
    x(StateIndex::ATT_Z) = q_aa.z();
  }

  Quaternion getQ() const {
    return Quaternion(x(ATT_W), x(ATT_X), x(ATT_Y), x(ATT_Z));
  }

  Matrix<3, 3> getR() const {
    return getQ().toRotationMatrix();
  }

  Scalar getYaw(const Scalar yaw = INF) const {
    const Vector<3> B_x = getQ() * Vector<3>::UnitX();
    const Vector<3> B_x_proj = Vector<3>(B_x.x(), B_x.y(), 0);
    if (B_x_proj.norm() < 1e-3) return yaw;
    const Vector<3> B_x_proj_norm = B_x_proj.normalized();
    const Vector<3> cross = Vector<3>::UnitX().cross(B_x_proj_norm);
    const Scalar angle = asin(cross.z());
    if (B_x_proj_norm.x() >= 0.0) return angle;
    if (B_x_proj_norm.y() >= 0.0) return M_PI - angle;
    return -M_PI - angle;
  }

  RobotState getHoverState() const {
    RobotState hover_state;
    hover_state.setZero();
    hover_state.stamp = stamp;
    hover_state.pos = pos;
    hover_state.setQ(getYaw(0.0));
    return hover_state;
  }

  bool operator==(const RobotState& rhs) const {
    return stamp == rhs.stamp && x.isApprox(rhs.x);
  }

  bool isApprox(const RobotState& rhs, const Scalar tol) const {
    return std::abs(stamp - rhs.stamp) < tol && x.isApprox(rhs.x, tol);
  }

  friend std::ostream& operator<<(std::ostream& os, const RobotState& state){
    os.precision(6);
    os << std::scientific;
    os << "Robot State at " << state.stamp << "s:\n";
    os.precision(3);
    os << "pos=    [" << state.pos.transpose()   << "]" << std::endl;
    os << "att=    [" << state.att.transpose()   << "]" << std::endl;
    os << "vel=    [" << state.vel.transpose()   << "]" << std::endl;
    os << "omega=  [" << state.omega.transpose() << "]" << std::endl;
    os << "alpha=  [" << state.alpha.transpose() << "]" << std::endl;
    os << "jerk=   [" << state.jerk.transpose()  << "]" << std::endl;
    os << "snap=   [" << state.snap.transpose()  << "]" << std::endl;
    os << "rot_speed= [" << state.rot_speed.transpose() << "]" << std::endl;
    os << "rot1_pos=  [" << state.rot1_pos.transpose()  << "]" << std::endl;
    os << "rot2_pos=  [" << state.rot2_pos.transpose()  << "]" << std::endl;
    os << "rot3_pos=  [" << state.rot3_pos.transpose()  << "]" << std::endl;
    os << "rot4_pos=  [" << state.rot4_pos.transpose()  << "]" << std::endl;
    os.precision();
    os.unsetf(std::ios::scientific);
    return os;
  }

  inline bool valid() const{
    return x.allFinite() && std::isfinite(stamp);
  }

  inline bool isReceived(const Scalar &now, Scalar timeout=0.1) const{
    if (!std::isfinite(stamp)){
      ROS_ERROR_ONCE("RobotState is not received!");
      return false;
    } else if (now - stamp > timeout){
      ROS_ERROR_THROTTLE(0.5, "RobotState is not received for %f seconds!", now-stamp);
      return false;
    }
    return true;
  }

};

//! RobotState <-> flygripper_msgs::RobotState
inline void fromRosMsg(flygripper_msgs::RobotStateConstPtr msg, RobotState& data){
  data = RobotState(msg->header.stamp.toSec()); // TODO: ros::Time::now();

  fromRosVec3(msg->position, data.pos);
  fromRosQuat(msg->attitude, data.att);
  fromRosVec3(msg->linear_velocity, data.vel);
  fromRosVec3(msg->angular_velocity, data.omega);
  fromRosVec3(msg->linear_acceleration, data.acc);
  fromRosVec3(msg->angular_acceleration, data.alpha);

  if (msg->rotor_speed.size() != 4 || msg->rotor_position.size() != 4){
    ROS_ERROR("Rotor speed or position message size mismatch, "
        "expected 4, got rotor speed size %ld, rotor position size %ld",
        msg->rotor_speed.size(), msg->rotor_position.size());
    return;
  }

  for (int i = 0; i < 4; i++){
    data.rot_speed(i) = msg->rotor_speed[i];
  }

  data.rot1_pos.x() = msg->rotor_position[0].x;
  data.rot1_pos.y() = msg->rotor_position[0].y;

  data.rot2_pos.x() = msg->rotor_position[1].x;
  data.rot2_pos.y() = msg->rotor_position[1].y;

  data.rot3_pos.x() = msg->rotor_position[2].x;
  data.rot3_pos.y() = msg->rotor_position[2].y;

  data.rot4_pos.x() = msg->rotor_position[3].x;
  data.rot4_pos.y() = msg->rotor_position[3].y;
}

inline void toRosMsg(const RobotState& data, flygripper_msgs::RobotState& msg){

  msg.header.stamp = ros::Time(data.stamp);
  msg.header.frame_id = kFixedFrame;
  toRosVec3(data.pos, msg.position);
  toRosQuat(data.att, msg.attitude);
  toRosVec3(data.vel, msg.linear_velocity);
  toRosVec3(data.omega, msg.angular_velocity);
  toRosVec3(data.acc, msg.linear_acceleration);
  toRosVec3(data.alpha, msg.angular_acceleration);

  msg.rotor_speed.resize(4);
  for (int i = 0; i < 4; i++){
    msg.rotor_speed[i] = data.rot_speed(i);
  }
  msg.rotor_position.resize(4);
  msg.rotor_position[0].x = data.rot1_pos.x();
  msg.rotor_position[0].y = data.rot1_pos.y();
  msg.rotor_position[0].z = 0;
  msg.rotor_position[1].x = data.rot2_pos.x();
  msg.rotor_position[1].y = data.rot2_pos.y();
  msg.rotor_position[1].z = 0;
  msg.rotor_position[2].x = data.rot3_pos.x();
  msg.rotor_position[2].y = data.rot3_pos.y();
  msg.rotor_position[2].z = 0;
  msg.rotor_position[3].x = data.rot4_pos.x();
  msg.rotor_position[3].y = data.rot4_pos.y();
  msg.rotor_position[3].z = 0;
}

//! RobotState <-> nav_msgs::Odometry
inline void toRosMsg(const RobotState& data, nav_msgs::Odometry& msg){
  msg.header.stamp = ros::Time(data.stamp);
  msg.header.frame_id = kFixedFrame;
  toRosVec3(data.pos, msg.pose.pose.position);
  toRosQuat(data.att, msg.pose.pose.orientation);
  toRosVec3(data.vel, msg.twist.twist.linear);
  toRosVec3(data.omega, msg.twist.twist.angular);
}

} //namespace flygripper_common