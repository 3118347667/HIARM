#pragma once
#include <ros/ros.h>
#include <queue>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <queue>

#include "flygripper_common/type/base.hpp"
#include "flygripper_common/type/const.hpp"

namespace flygripper_flight{
using namespace flygripper_common;
class ThrustAccMap {
 public:
  ThrustAccMap():
      map_type_(MapType::NONE), P_(1e6)
//                   compensation_scale_(1.0)
  {};
  ~ThrustAccMap() = default;

  bool valid() const{
    return (map_type_ == MapType::WITHOUT_VOLT_LINEAR_FIT && std::isfinite(command2acc_coeff_));
  }

  bool init_simple_model(Scalar hover_percentage){
    if (hover_percentage < 0.1 ||
        hover_percentage > 0.8){
      ROS_ERROR("Error in thrust acc map coefficients: hover_percentage should be between 0.1 and 0.8.");
      return false;
    }
    command2acc_coeff_ = kGravity / hover_percentage;
    map_type_ = MapType::WITHOUT_VOLT_LINEAR_FIT;
    return true;
  }

  void resetMap(Scalar hover_percentage){
    P_ = 1e6;
    command2acc_coeff_ = kGravity / hover_percentage;

    //    compensation_scale_ = 1.0;
  }

  Scalar map(const Scalar& thrust_acc, const Scalar voltage = 14.0) const
  {
    if (map_type_ == MapType::WITHOUT_VOLT_LINEAR_FIT){
      return thrust_acc / command2acc_coeff_;
    }
//    else if (map_type_ == MapType::WITH_VOLT_CURVE_FIT){
//      return 0.0
//    }
    else {
      ROS_ERROR("It is invalid to call map function before initialize it, "
                "please call init function to init it first");
      return 0.0;
    }
  };

  bool updateMap(Scalar cur_command,
                 Scalar cur_thrust_acc, // body frame
                 Scalar cur_voltage = 14.0){
    // 1.Push into queue, should update the map right after mapping to get the latest command
    ros::Time now = ros::Time::now();
    command_queue_.push(std::pair<ros::Time, Scalar>(now, cur_command));
    while (command_queue_.size() > 100){ // keep the queue small size
      command_queue_.pop();
    }

    // 2. Pop the prev_command
    //    from time range (generally 35-45 ms), based on system response time
    while (command_queue_.size() >= 1) {
      std::pair<ros::Time, Scalar> prev_stamped_command = command_queue_.front();
      double time_passed = (now - prev_stamped_command.first).toSec();
      if (time_passed > response_time_upper_bound_)
      {
        command_queue_.pop();
        continue;
      }
      if (time_passed < response_time_lower_bound_)
      {
        return false;
      }

      /**************************************************************************/
      /* Update the Model with Recursive Least Squares algorithm with vanishing memory
       * command = compensation_scale_ * map(thrust_acc, voltage) , dynamic update compensation_scale_
       * thrust_acc = command2acc_coeff_ * command,  dynamic update command2acc_coeff_   */
      /**************************************************************************/
      Scalar command;
      Scalar prev_command = prev_stamped_command.second;
      if (map_type_ == MapType::WITHOUT_VOLT_LINEAR_FIT) {
        Scalar gamma = 1 / (rho2_ + prev_command * P_ * prev_command);
        Scalar K = gamma * P_ * prev_command;
        command2acc_coeff_ = command2acc_coeff_ +
                             K * (cur_thrust_acc - prev_command * command2acc_coeff_);
        P_ = (1 - K * prev_command) * P_ / rho2_;
      } else {
        ROS_ERROR(
            "Thrust mapping need to init the model before update the map.");
        return false;
      }

      // evaluation
      const Scalar hover_percentage = kGravity / command2acc_coeff_;
      if (hover_percentage < 0.1 ||
          hover_percentage > 0.8){
        ROS_ERROR("Estimated hover_percentage >0.8 or <0.1! Perhaps the accel vibration is too high!");
        command2acc_coeff_ = hover_percentage > 0.8 ? kGravity / 0.8 : command2acc_coeff_;
        command2acc_coeff_ = hover_percentage < 0.1 ? kGravity / 0.1 : command2acc_coeff_;
      }
      ROS_INFO_THROTTLE(5.0, "Estimated hover_percentage: %f", hover_percentage);
    }
    return true;
  }

 private:



  //! Map type
  enum class MapType{
    NONE = -1,
    WITHOUT_VOLT_LINEAR_FIT = 0,
  };
  MapType map_type_;


  //! without voltage mapping: curve fit approach
  Scalar command2acc_coeff_{NAN};

  //! Update map using recursive least squares algorithm with vanishing memory
//  Scalar compensation_scale_{1.0};
  Scalar P_{1e6};
  const double rho2_{0.998};
  const Scalar response_time_lower_bound_{0.035};
  const Scalar response_time_upper_bound_{0.045};
  std::queue<std::pair<ros::Time, Scalar>> command_queue_;

};

}