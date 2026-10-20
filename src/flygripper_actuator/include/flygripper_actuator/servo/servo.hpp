#pragma once
#include <ros/ros.h>

#include <chrono>
#include <memory>

#include "flygripper_common/filter/butterworth.hpp"

namespace flygripper_actuator {
namespace servo {
//using TimeStamp = std::chrono::steady_clock::time_point;
using TimeStamp = ros::Time;

struct ActuatorState {
  using Ptr = std::shared_ptr<ActuatorState>;
  int32_t ecd;
  int32_t last_ecd;
  int16_t ecd_rpm;

  double current;
  double voltage;

  uint32_t seq = 0; // start from 0
  TimeStamp stamp;
  int round = 0; // start from 0
  double frequency;
  double position;
  double velocity;
  double effort;
  double temperature;
  bool overload;
  friend std::ostream &operator<<(std::ostream &os, const ActuatorState &data) {
    os.precision(3);
    os << std::scientific;
    os << "seq: " << data.seq << std::endl
       << "stamp: " << std::to_string(data.stamp.sec) + " sec " + std::to_string(data.stamp.nsec) + " nsec" << std::endl
       << "round: " << data.round << std::endl
       << "frequency: " << data.frequency << std::endl
       << "position: " << data.position << std::endl
       << "velocity: " << data.velocity << std::endl
       << "effort: " << data.effort << std::endl
       << "current: " << data.current << std::endl
        << "voltage: " << data.voltage << std::endl
       << "temperature: " << data.temperature << std::endl
       << "overload: " <<  ( data.overload ?  "TRUE" : "FALSE" ) << std::endl;
    os.precision();
    os.unsetf(std::ios::scientific);
    return os;
  }
};

struct ActuatorCommand {
  using Ptr = std::shared_ptr<ActuatorCommand>;
  TimeStamp stamp;
  double position = 0.0;
  double velocity = 0.0;
  double acceleration = 0.0;
  friend std::ostream &operator<<(std::ostream &os, const ActuatorCommand &data) {
    os.precision(3);
    os << std::scientific;
    os << "stamp: " << std::to_string(data.stamp.sec) + " sec "
        + std::to_string(data.stamp.nsec) + " nsec"
       << std::endl
       << "position: " << data.position << std::endl
       << "velocity: " << data.velocity << std::endl
       << "acceleration: " << data.acceleration << std::endl;

    os.precision();
    os.unsetf(std::ios::scientific);
    return os;
  }
};

struct ActuatorParam {
  using Ptr = std::shared_ptr<ActuatorParam>;
  double act2pos;
  double act2vel;
  double act2effort;
  double effort2act;
  double max_out;
  double offset_angle;

  friend std::ostream &operator<<(std::ostream &os, const ActuatorParam &data) {
    os.precision(3);
    os << std::scientific;
    os << "act2pos: " << data.act2pos << std::endl
       << "act2vel: " << data.act2vel << std::endl
       << "act2effort: " << data.act2effort << std::endl
       << "effort2act: " << data.effort2act << std::endl
       << "max_out: " << data.max_out << std::endl
       << "offset_angle: " << data.offset_angle << std::endl;
    os.precision();
    os.unsetf(std::ios::scientific);
    return os;
  }
};
}
}
