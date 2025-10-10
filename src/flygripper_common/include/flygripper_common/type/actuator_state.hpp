#pragma once
#include <Eigen/Dense>
#include "flygripper_msgs/ActuatorState.h"

#include "flygripper_common/type/base.hpp"
namespace flygripper_common{
class ActuatorState{
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  using Ptr = std::shared_ptr<ActuatorState>;
  Scalar stamp{INF};
  Scalar position{INF};
  Scalar velocity{INF};
  Scalar effort{INF};
  Scalar current{INF};
  bool overload{false};

//  Vector<2> rot1_pos{Vector<2>::Constant(INF)};
//  Vector<2> rot2_pos{Vector<2>::Constant(INF)};
//  Vector<2> rot3_pos{Vector<2>::Constant(INF)};
//  Vector<2> rot4_pos{Vector<2>::Constant(INF)};

 public:
  ActuatorState(){}
  ~ActuatorState() = default;

  inline bool valid() const {
    return
//        rot1_pos.allFinite() &&
//        rot2_pos.allFinite() &&
//        rot3_pos.allFinite() &&
//        rot4_pos.allFinite() &&
        std::isfinite(position) &&
        std::isfinite(velocity) &&
        std::isfinite(effort) &&
        std::isfinite(current) &&
        std::isfinite(stamp);
  }

  inline bool isReceived(const Scalar &now, Scalar timeout=0.1) const {
    if (!std::isfinite(stamp)){
//      ROS_ERROR_THROTTLE(2.0, "ActuatorState has never been received!");
      return false;
    } else if (now - stamp > timeout){
      ROS_ERROR_THROTTLE(1.0, "ActuatorState is not received for %f seconds!", now - stamp);
      return false;
    }
    return true;
  }

//  inline void kinematics(const Scalar& joint_position) {
//    //TODO: hard code for forward kinematics !!!!
//    // l = (part_width + part_distance - joint_position * ratio)/2
//
//    // flygripper simple
////    Scalar l = (0.1 + 0.15 - joint_position * 0.02)/2;
//
//    // flygripper
////    Scalar l = (0.17- joint_position * 0.01)/2;
//    //
//    //
//    // 3-----17.5-----1
//    // |
//    // 14.5
//    // |
//    // 2----16--------4
//
//    rot1_pos << 0.0725, -0.0875;
//    rot2_pos << -0.0725, 0.08;
//    rot3_pos << 0.0725, 0.0875;
//    rot4_pos << -0.0725, -0.08;
//  }
};

//! ActuatorState <-> flygripper_msgs::ActuatorState
inline void fromRosMsg(flygripper_msgs::ActuatorStateConstPtr msg, ActuatorState& data){
  data.stamp = msg->header.stamp.toSec(); // TODO: ros::Time::now();
  data.position = msg->position;
  data.velocity = msg->velocity;
  data.effort = msg->effort;
  data.current = msg->current;
  data.overload = msg->overload;
//  data.kinematics(data.position);
}

} //namespace flygripper_common