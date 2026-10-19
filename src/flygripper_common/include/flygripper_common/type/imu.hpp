#pragma once
#include <Eigen/Dense>
#include "sensor_msgs/Imu.h"

#include "flygripper_common/ros/conversions.hpp"
#include "flygripper_common/type/base.hpp"
namespace flygripper_common{
class Imu
{
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  using Ptr = std::shared_ptr<Imu>;

  Scalar stamp{INF};
  Quaternion att{Vector<4>::Constant(INF)};
  Vector<3> omega{Vector<3>::Constant(INF)};
  Vector<3> acc{Vector<3>::Constant(INF)};

 public:
  Imu(){}
  ~Imu() = default;

  inline bool valid() const {
    return att.coeffs().allFinite() && omega.allFinite() &&
        acc.allFinite() && std::isfinite(stamp);
  }

  inline bool isReceived(const Scalar &now, Scalar timeout=0.1) const{
    if (!std::isfinite(stamp)){
      ROS_ERROR_THROTTLE(2.0, "Imu has never been received!");
      return false;
    } else if (now - stamp > timeout){
      ROS_ERROR_THROTTLE(1.0, "Imu is not received for %f seconds!", now - stamp);
      return false;
    }
    return true;
  }

};

//! Imu <-> sensor_msgs::Imu
inline void fromRosMsg(sensor_msgs::ImuConstPtr msg, Imu& data){
  data.stamp = ros::Time::now().toSec(); // TODO: ros::Time::now() vs msg->header.stamp
  fromRosQuat(msg->orientation, data.att);
  fromRosVec3(msg->angular_velocity, data.omega);
  fromRosVec3(msg->linear_acceleration, data.acc);
}

} //namespace flygripper_common