#pragma once
#include <Eigen/Dense>
#include "sensor_msgs/BatteryState.h"

#include "flygripper_common/ros/conversions.hpp"
#include "flygripper_common/type/base.hpp"

namespace flygripper_common{
class Battery
{
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  using Ptr =  std::shared_ptr<Battery>;
  Scalar stamp{INF};
  Scalar voltage{INF};
  Scalar percentage{INF};
 public:
  Battery() = default;
  ~Battery() = default;

  inline bool valid() const{
    return std::isfinite(stamp) && std::isfinite(voltage) &&
           std::isfinite(percentage);
  }

  inline bool isReceived(const Scalar &now, Scalar timeout=1.0) const{
    if (!std::isfinite(stamp)){
      ROS_ERROR_THROTTLE(2.0, "Battery has never been received!");
      return false;
    } else if (now - stamp > timeout){
      ROS_ERROR_THROTTLE(1.0, "Battery is not received for %f seconds!", timeout);
      return false;
    }
    return true;
  }
};

//! Odometry <-> nav_msgs::Odometry
inline void fromRosMsg(sensor_msgs::BatteryStateConstPtr msg, Battery& data){
  data.stamp = ros::Time::now().toSec(); // TODO: ros::Time::now() vs msg->header.stamp
  data.voltage = msg->voltage;
  data.percentage = msg->percentage;
}

} //namespace flygripper_common
