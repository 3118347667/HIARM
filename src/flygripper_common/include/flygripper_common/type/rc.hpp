#pragma once
#include <memory>
#include "mavros_msgs/RCIn.h"

#include "flygripper_common/type/base.hpp"

namespace flygripper_common{
template<int ChannelSize=4>
class Rc{
 public:
  using Ptr = std::shared_ptr<Rc>;

  static constexpr Scalar kDeadZone = 0.25;
  static constexpr Scalar kEpslon = 1e-5;
  static constexpr Scalar kScaledMax = 1.0;
  static constexpr Scalar kScaledMid = 0.0;
  static constexpr Scalar kScaledMin = -1.0;

  //static constexpr Scalar kRawMax = 1940.0; //1933
  //static constexpr Scalar kRawMin= 1060.0; //1066
  static constexpr Scalar kRawMax = 2000.0; //1933
  static constexpr Scalar kRawMin= 1000.0; //1066
  static constexpr Scalar kRawMid = (kRawMax + kRawMin)/2;
  static constexpr Scalar kRawScale = (kRawMax - kRawMin)/2;

  Scalar stamp{INF};
  Vector<ChannelSize> channels{Vector<ChannelSize>::Constant(INF)};
  Rc() {}
  ~Rc() = default;

  bool isAllChannelCentered(){
    bool centered = true;
    for (int i = 0; i < ChannelSize; i++)
    {
      centered &= abs(channels[i]) < kEpslon;
    }
    return centered;
  }

  inline bool valid() const{
    // Check if stamp valid
    if (!std::isfinite(stamp)){
      return false;
    }
    // Check if axis valid
    for (int i = 0; i < ChannelSize; i++)
    {
      if (channels[i] > kScaledMax + EPSILON || channels[i] < kScaledMin - EPSILON){
        ROS_ERROR_THROTTLE(0.5, "RC input axis from channel %d is not valid!", i);
        return false;
      }
    }
    return true;
  }

  inline bool isReceived(const Scalar &now, Scalar timeout=0.1) const{
    if (!std::isfinite(stamp)){
      ROS_ERROR_THROTTLE(2.0, "RC input has never been received!");
      return false;
    } else if (now - stamp > timeout){
      ROS_ERROR_THROTTLE(1.0, "RC input is not received for %f seconds.", now - stamp);
      return false;
    }
    return true;
  }
};

//! Rc <-> mavros_msgs::RCIn
template <int ChannelSize>
inline void fromRosMsg(mavros_msgs::RCInConstPtr msg, Rc<ChannelSize>& rc,
                       bool enable_dead_zone = true){
  // Update time stamp
  rc.stamp = ros::Time::now().toSec();

  Scalar dead_zone;
  if (enable_dead_zone){
    dead_zone = rc.kDeadZone;
  } else{
    dead_zone = rc.kEpslon;
  }
  if(msg->channels.size() >= ChannelSize){
    // Channel 0-9: roll, pitch, throttle, yaw, guided, plan, arm, kill-switch, morph remap from -1.0 to 1.0
    for (int i = 0; i < ChannelSize; i++)
    {
      rc.channels[i] = (static_cast<Scalar>(msg->channels[i]) - rc.kRawMid) / rc.kRawScale;
      if (rc.channels[i] > dead_zone){
        rc.channels[i] = (rc.channels[i] - dead_zone) / (rc.kScaledMax - dead_zone);
      }
      else if (rc.channels[i] < -dead_zone){
        rc.channels[i] = (rc.channels[i] + dead_zone) / (rc.kScaledMax - dead_zone);
      }
      else{
        rc.channels[i] = rc.kScaledMid;
      }
    }
  } else{
    ROS_ERROR_THROTTLE(3.0, " RCIn message channel size mismatch, "
        "expected %d, got %ld", ChannelSize, msg->channels.size());
  }
}

} //namespace flygripper_flight
