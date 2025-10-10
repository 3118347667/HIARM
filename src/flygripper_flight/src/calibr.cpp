#include <ros/ros.h>
#include <cmath>
#include "flygripper_flight/rpm_map.hpp"
#include "flygripper_flight/rc_utils.hpp"
#include "flygripper_common/type/robot_parameter.hpp"
#include "flygripper_common/type/robot_command.hpp"
#include "flygripper_common/type/battery.hpp"

#include "mavros_msgs/ActuatorControl.h"
using namespace flygripper_flight;
using namespace flygripper_common;
RcInput rc_input;
Battery battery;
void batteryCallback(sensor_msgs::BatteryStateConstPtr msg){
  fromRosMsg(msg, battery);
}

void rcCallback(const mavros_msgs::RCIn::ConstPtr& msg){
  fromRosMsg(msg, rc_input, false);
  rc_input.update(); // update mode trigger
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "flygripper_flight_calibr");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  ros::NodeHandle robot_param_nh(nh, "robot_parameter");
  RobotParameterConfig robot_param_config;
  if (!robot_param_config.init(robot_param_nh)){
    ROS_ERROR("Failed to initialize robot parameter config");
    return 1;
  }
  // robot parameter
  RobotParameter::Ptr robot_param = std::make_shared<RobotParameter>();
  robot_param_config.getParams(*robot_param);

  // init rpm_map
  std::vector<RpmMap> rpm_maps;
  rpm_maps.resize(4);
  for (int i = 0; i < rpm_maps.size(); ++i) {
    if (robot_param->dshot_rpm_map_method == 0){
      if(!rpm_maps[i].init_simple_model(robot_param->dshot_rpm_map_coeff)){
        return false;
      }
    }
    else if (robot_param->dshot_rpm_map_method == 1){
      if(!rpm_maps[i].init_precise_model(robot_param->dshot_rpm_volt_map_coeff)){
        return false;
      }
    }
    else if (robot_param->dshot_rpm_map_method == 2){
      std::filesystem::path rpm_map_file =
          std::filesystem::path(robot_param->dshot_rpm_volt_map_file)/
              std::filesystem::path("rpm"+std::to_string(i)+"_map.csv");
      if (!rpm_maps[i].init_precise_model(rpm_map_file)){
        return false;
      }
    }
  }

  // init min max speed
  Scalar rotor_speed_min, rotor_speed_max;
  rotor_speed_min = std::sqrt(robot_param->rotor_thrusts_min[0] / robot_param->thrust_coeff);
  rotor_speed_max = std::sqrt(robot_param->rotor_thrusts_max[0] / robot_param->thrust_coeff);
  ROS_INFO("rotor_speed_min: %f, rotor_speed_max: %f", rotor_speed_min, rotor_speed_max);
  // rc sub
  ros::Subscriber rc_sub = nh.subscribe<mavros_msgs::RCIn>("/mavros/rc/in",
                                                           100,
                                                           &rcCallback,
                                                           ros::VoidConstPtr(),
                                                           ros::TransportHints().tcpNoDelay());
  //battery sub
  ros::Subscriber batt_sub = nh.subscribe<sensor_msgs::BatteryState>("/mavros/battery",
                                                                     100,
                                                                     &batteryCallback,
                                                                     ros::VoidConstPtr(),
                                                                     ros::TransportHints().tcpNoDelay());
  // cmd pub
  ros::Publisher rotor_mavros_command_pub = nh.advertise<mavros_msgs::ActuatorControl>("/mavros/actuator_control", 1);
  ros::Publisher rotor_command_pub = nh.advertise<flygripper_msgs::RotorCommand>("rotor_command", 1);
  mavros_msgs::ActuatorControl control_msg;
  flygripper_msgs::RotorCommand rotor_command_msg;
  Scalar control_frequency;
  bool calibr_mode;
  pnh.param("control_frequency", control_frequency, 333.3);
  pnh.param("calibration_mode", calibr_mode, true);
  ROS_INFO("control_frequency: %f, mode: %s", control_frequency, calibr_mode ? "calibration" : "evaluation");
  ros::Rate rate(control_frequency);
  RobotCommand robot_command(ros::Time::now().toSec(), RobotCommandMode::MODE_SINGLE_ROTOR);
  while (ros::ok()){
    Scalar now = ros::Time::now().toSec();
    ros::spinOnce();
    robot_param_config.getParams(*robot_param);
    if(!robot_param->valid()) {
      ROS_ERROR_THROTTLE(1.0, "Wait for valid robot parameter...");
    }
    else if(!rc_input.isReceived(now) || !rc_input.valid()) {
      ROS_ERROR_THROTTLE(1.0, "Wait for valid rc message...");
    }
    else if(!battery.isReceived(now) || !battery.valid()) {
      ROS_ERROR_THROTTLE(1.0, "Wait for valid battery message...");
    }
    else{
      double rc_val = (rc_input.channels[AXIS_THROTTLE] + 1.0) / 2; // 0 to 1

      if(calibr_mode) { // calibration mode w/o voltage
        control_msg.header.stamp = ros::Time::now();
        control_msg.header.frame_id = std::string("FCU");
        control_msg.group_mix =
            mavros_msgs::ActuatorControl::PX4_MIX_FLIGHT_CONTROL;
        for (int i = 0; i < 4; i++) {
          control_msg.controls[i] = rc_val;
        }
        for (int i = 4; i < 8; i++) {
          control_msg.controls[i] = 0.0;
        }
      }
      else{ // evaluation mode w or w/o voltage
        robot_command.stamp = now;
        rotor_speed_min = std::sqrt(robot_param->rotor_thrusts_min[0] / robot_param->thrust_coeff);
        rotor_speed_max = std::sqrt(robot_param->rotor_thrusts_max[0] / robot_param->thrust_coeff);

        robot_command.rotor_velocities = Vector<4>::Constant(
            rotor_speed_min + rc_val * (rotor_speed_max - rotor_speed_min)
        );

        for (int i = 0; i < rpm_maps.size(); ++i) {
          robot_command.rotor_velocities_command(i) =
              rpm_maps[i].map(robot_command.rotor_velocities(i), battery.voltage);
        }

        // mavros message
        toRosMsg(robot_command, control_msg);
        // debug message with rotor_command
        toRosMsg(robot_command, rotor_command_msg);
        ROS_INFO_THROTTLE(0.5, "command rotor speed: %f in the range of [%f, %f]",
                          robot_command.rotor_velocities[0], rotor_speed_min, rotor_speed_max);
        rotor_command_pub.publish(rotor_command_msg);
      }
      rotor_mavros_command_pub.publish(control_msg);
    }
    rate.sleep();
  }
  return 0;
}


