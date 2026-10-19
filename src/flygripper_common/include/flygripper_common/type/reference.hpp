#pragma once
#include <memory>
#include <Eigen/Dense>
#include "ros/ros.h"
#include "nav_msgs/Path.h"

#include "flygripper_common/ros/conversions.hpp"
#include "flygripper_common/type/robot_command.hpp"
#include "flygripper_common/type/robot_state.hpp"
#include "flygripper_common/type/const.hpp"

namespace flygripper_common{

struct Reference {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  using Ptr = std::shared_ptr<Reference>;
  RobotState state;
  RobotCommand input;

  Reference() = default;
  Reference(const RobotState& rstate, const RobotCommand& rinput):
      state(rstate), input(rinput) {}
  ~Reference() = default;

};
using ReferenceVector = std::vector<Reference>;

inline void toRosMsg(const ReferenceVector& data, nav_msgs::Path& msg){
  msg.header.stamp = ros::Time::now();
  msg.header.frame_id = kFixedFrame;
  msg.poses.resize(data.size());
  for (size_t i = 0; i < data.size(); ++i){
    msg.poses[i].header.stamp = ros::Time(data[i].state.stamp);
    msg.poses[i].header.frame_id = kFixedFrame;
    toRosVec3(data[i].state.pos, msg.poses[i].pose.position);
    toRosQuat(data[i].state.att, msg.poses[i].pose.orientation);
  }
}


}
