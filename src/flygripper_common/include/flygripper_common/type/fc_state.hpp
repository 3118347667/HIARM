#pragma once
#include <memory>
#include "mavros_msgs/State.h"
#include "mavros_msgs/ExtendedState.h"

#include "flygripper_common/type/base.hpp"

namespace flygripper_common{

class FcState{
 public:
  using Ptr = std::shared_ptr<FcState>;

  Scalar stamp{INF};
  FcState() {}
  ~FcState() = default;

  bool connected{false};
  bool armed{false};
  bool ground{false};
  bool guided{false};
  bool manual{false};
  std::string mode;

  inline bool isReceived(const Scalar &now, Scalar timeout=5.0) const{
    if (!std::isfinite(stamp)){
      ROS_ERROR_THROTTLE(2.0, "FC state has never been received!");
      return false;
    } else if (now - stamp > timeout){
      ROS_ERROR_THROTTLE(1.0, "FC state is not received for %f seconds.", now - stamp);
      return false;
    }
    return true;
  }
};

//! FcState <- mavros_msgs::State
inline void fromRosMsg(mavros_msgs::StateConstPtr msg, FcState& fc_state){
  fc_state.stamp = ros::Time::now().toSec();//msg->header.stamp.toSec();
  fc_state.connected = msg->connected;
  fc_state.armed = msg->armed;
  fc_state.ground = msg->system_status == 3;
  fc_state.guided = msg->mode == "GUIDED_NOGPS";
  fc_state.manual = msg->manual_input;
  fc_state.mode =  msg->mode;
}


} //namespace flygripper_flight
