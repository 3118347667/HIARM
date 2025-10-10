#pragma once
#include <Eigen/Dense>
#include "mavros_msgs/ESCTelemetry.h"

#include "flygripper_common/ros/conversions.hpp"
#include "flygripper_common/type/base.hpp"

namespace flygripper_common{
class RotorSpeed
{
 public:
  using Ptr = std::shared_ptr<RotorSpeed>;
  Scalar stamp{INF};
  Vector<4> rot_speed{Vector<4>::Constant(INF)};

 public:
  RotorSpeed(){}
  ~RotorSpeed() = default;

  inline bool valid() const {
    return rot_speed.allFinite() &&
        std::isfinite(stamp);
  }

  inline bool isReceived(const Scalar &now, Scalar timeout=0.1) const{
    if (!std::isfinite(stamp)){
      ROS_ERROR_THROTTLE(2.0, "EscTelemetry has never been received!");
      return false;
    } else if (now - stamp > timeout){
      ROS_ERROR_THROTTLE(1.0, "EscTelemetry is not received for %f seconds!", now - stamp);
      return false;
    }
    return true;
  }
};

//! RotorSpeed <-> mavros_msgs::ESCTelemetry
inline void fromRosMsg(mavros_msgs::ESCTelemetryConstPtr msg, RotorSpeed& data){
  data.stamp = ros::Time::now().toSec(); // TODO: ros::Time::now() vs msg->header.stamp
  if (msg->esc_telemetry.size() >= data.rot_speed.size()) {
    for (int i=0; i<msg->esc_telemetry.size(); i++){
      data.rot_speed(i) = msg->esc_telemetry[i].rpm;
    }
  } else{
    ROS_ERROR("Rotor speed message size mismatch, expected %ld, got %ld",
              data.rot_speed.size(), msg->esc_telemetry.size());
  }

}

} //namespace flygripper_common