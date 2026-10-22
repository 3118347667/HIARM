#pragma once
#include <Eigen/Dense>
#include "nav_msgs/Odometry.h"

#include "flygripper_common/ros/conversions.hpp"
#include "flygripper_common/type/base.hpp"

namespace flygripper_common{
class Odometry
{
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  using Ptr =  std::shared_ptr<Odometry>;
  Scalar stamp{INF};
  Vector<3> pos{Vector<3>::Constant(INF)};
  Vector<3> vel{Vector<3>::Constant(INF)};
  Quaternion att{Vector<4>::Constant(INF)};
  Vector<3> omega{Vector<3>::Constant(INF)};
  bool velocity_in_body{false};
 public:
  Odometry(){}
  explicit Odometry(bool velocity_in_body):
      velocity_in_body(velocity_in_body){
  }
  ~Odometry() = default;

  inline bool valid() const{
    return pos.allFinite() && att.coeffs().allFinite() &&
        vel.allFinite() && omega.allFinite() &&
        std::isfinite(stamp);
  }

  inline bool isReceived(const Scalar &now, Scalar timeout=0.1) const{
    if (!std::isfinite(stamp)){
      ROS_ERROR_THROTTLE(2.0, "Odometry has never been received!");
      return false;
    } else if (now - stamp > timeout){
      ROS_ERROR_THROTTLE(1.0,
          "\033[1;31mOdometry timestamp is stale: age=%.6f s, timeout=%.6f s. "
          "This does not prove raw mocap loss.\033[0m", now - stamp, timeout);
      return false;
    }
    return true;
  }

  Matrix<3, 3> getR() const {
    return att.toRotationMatrix();
  }

  Scalar getYaw(const Scalar yaw = INF) const {
    const Vector<3> B_x = att * Vector<3>::UnitX();
    const Vector<3> B_x_proj = Vector<3>(B_x.x(), B_x.y(), 0);
    if (B_x_proj.norm() < 1e-3) return yaw;
    const Vector<3> B_x_proj_norm = B_x_proj.normalized();
    const Vector<3> cross = Vector<3>::UnitX().cross(B_x_proj_norm);
    const Scalar angle = asin(cross.z());
    if (B_x_proj_norm.x() >= 0.0) return angle;
    if (B_x_proj_norm.y() >= 0.0) return M_PI - angle;
    return -M_PI - angle;
  }
};

//! Odometry <-> nav_msgs::Odometry
inline void fromRosMsg(nav_msgs::OdometryConstPtr msg, Odometry& data){
  data.stamp = ros::Time::now().toSec(); // TODO: ros::Time::now() vs msg->header.stamp
  fromRosVec3(msg->pose.pose.position, data.pos);
  fromRosQuat(msg->pose.pose.orientation, data.att);
  fromRosVec3(msg->twist.twist.linear, data.vel);
  if (data.velocity_in_body){
    data.vel = data.att * data.vel;
  }
  fromRosVec3(msg->twist.twist.angular, data.omega);
}

} //namespace flygripper_common
