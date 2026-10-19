#pragma once
#include "flygripper_common/type/base.hpp"
#include "flygripper_msgs/ActuatorCommand.h"

namespace flygripper_common{
enum class ActuatorCommandMode: int{
  MODE_POSITION = 0,
  MODE_POSITION_VELOCITY = 1,
  MODE_POSITION_VELOCITY_EFFORT = 2,
  MODE_RELEASE = 3
};

inline std::string ActuatorCommandModeToString(ActuatorCommandMode mode){
  switch(mode){
  case ActuatorCommandMode::MODE_POSITION:
    return "POSITION";
  case ActuatorCommandMode::MODE_POSITION_VELOCITY:
    return "POSITION_VELOCITY";
  case ActuatorCommandMode::MODE_POSITION_VELOCITY_EFFORT:
    return "POSITION_VELOCITY_EFFORT";
  case ActuatorCommandMode::MODE_RELEASE:
    return "RELEASE";
  default:
    return "Unknown";
  }
}

class ActuatorCommand
{
public:
 using Ptr = std::shared_ptr<ActuatorCommand>;

 Scalar stamp{INF};
 Scalar position{INF};
 Scalar velocity{INF};
 Scalar effort{INF};
 ActuatorCommandMode mode{ActuatorCommandMode::MODE_RELEASE};

  ActuatorCommand() = default;
  ~ActuatorCommand() = default;

  bool valid() const {
    return std::isfinite(stamp) &&
           (mode == ActuatorCommandMode::MODE_RELEASE ||
            (mode == ActuatorCommandMode::MODE_POSITION && std::isfinite(position)) ||
            (mode == ActuatorCommandMode::MODE_POSITION && std::isfinite(position) && std::isfinite(velocity)) ||
            (mode == ActuatorCommandMode::MODE_POSITION && std::isfinite(position) && std::isfinite(velocity) && std::isfinite(effort)));
  }

  bool isReceived(const Scalar &now, Scalar timeout=0.1) const{
    if (!std::isfinite(stamp)){
      ROS_ERROR_ONCE("Actuator Command has never been received!");
      return false;
    } else if (now - stamp > timeout){
      ROS_ERROR("Actuator Command is not received for %f seconds!", now - stamp);
      return false;
    }
    return true;
  }

  friend std::ostream& operator<<(std::ostream& os, const ActuatorCommand& p) {
    os.precision(3);
    os << std::scientific;
    os << "mode: " << ActuatorCommandModeToString(p.mode) << std::endl;
    os << "stamp: " << p.stamp << std::endl;
    os << "position: " << p.position << std::endl;
    os << "velocity: " << p.velocity << std::endl;
    os << "effort: " << p.effort << std::endl;
    os.precision();
    os.unsetf(std::ios::scientific);
    return os;
  }

};

//! RobotCommand <-> flygripper_msgs::ActuatorCommand
inline void fromRosMsg(flygripper_msgs::ActuatorCommandConstPtr msg, ActuatorCommand& data){
  data.stamp = msg->header.stamp.toSec();
  data.mode = static_cast<ActuatorCommandMode>(msg->mode);
  data.position = msg->reference_position;
  data.velocity = msg->reference_velocity;
  data.effort = msg->reference_effort;
}

inline void toRosMsg(const ActuatorCommand& data, flygripper_msgs::ActuatorCommand& msg){
  msg.header.stamp = ros::Time(data.stamp);
  msg.mode = static_cast<int>(data.mode);
  msg.reference_position = data.position;
  msg.reference_velocity = data.velocity;
  msg.reference_effort = data.effort;
}

}