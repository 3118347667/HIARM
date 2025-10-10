#include <ros/ros.h>
#include <cmath>
#include "flygripper_msgs/GeometricControllerData.h"
#include "sensor_msgs/BatteryState.h"

#include "flygripper_common/type/robot_parameter.hpp"
#include "flygripper_common/type/robot_command.hpp"
#include "flygripper_common/type/battery.hpp"
#include "flygripper_flight/rpm_map.hpp"

#include "mavros_msgs/ActuatorControl.h"

using namespace flygripper_common;
using namespace flygripper_flight;

RobotCommand::Ptr robot_command_ptr;
Battery battery;
void ctrlCallback(const flygripper_msgs::GeometricControllerData::ConstPtr& msg){
  for (int i = 0; i < 4; ++i) {
    robot_command_ptr->rotor_velocities(i) = msg->rotor_velocities_desired[i];
  }
}
void battCallback(const sensor_msgs::BatteryState::ConstPtr& msg){
  fromRosMsg(msg, battery);
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

  // ctrl sub
  ros::Subscriber ctrl_sub = nh.subscribe<flygripper_msgs::GeometricControllerData>("/flygripper/geometric_controller/data",
                                                                                    100,
                                                                                    &ctrlCallback,
                                                                                    ros::VoidConstPtr(),
                                                                                    ros::TransportHints().tcpNoDelay());

  ros::Subscriber batt_sub = nh.subscribe<sensor_msgs::BatteryState>("/mavros/battery",
                                                                     100,
                                                                     &battCallback,
                                                                     ros::VoidConstPtr(),
                                                                     ros::TransportHints().tcpNoDelay());
  // cmd pub
  ros::Publisher command_pub = nh.advertise<mavros_msgs::ActuatorControl>("/mavros/actuator_control_0", 1);
  ros::Publisher command1_pub = nh.advertise<mavros_msgs::ActuatorControl>("/mavros/actuator_control_1", 1);

  robot_command_ptr = std::make_shared<RobotCommand>(ros::Time::now().toSec(),
                                                     RobotCommandMode::MODE_SINGLE_ROTOR);
  mavros_msgs::ActuatorControl control_msg0;
  mavros_msgs::ActuatorControl control_msg1;
  // robot parameter
  RobotParameter::Ptr robot_param = std::make_shared<RobotParameter>();
  robot_param_config.getParams(*robot_param);
  // init rpm_map
  std::vector<RpmMap> rpm_maps;
  std::vector<RpmMap> rpm_maps2;
  rpm_maps.resize(4);
  rpm_maps2.resize(4);
  for (int i = 0; i < rpm_maps.size(); ++i) {
    if(!rpm_maps[i].init_simple_model(robot_param->dshot_rpm_map_coeff)){
      return false;
    }
    std::filesystem::path rpm_map_file =
        std::filesystem::path(robot_param->dshot_rpm_volt_map_file)/
            std::filesystem::path("rpm"+std::to_string(i)+"_map.csv");
    if (!rpm_maps2[i].init_precise_model(rpm_map_file)){
      return false;
    }
  }


  ros::Rate rate(333);
  while (ros::ok()){
    Scalar now = ros::Time::now().toSec();
    robot_command_ptr->stamp = now;
    ros::spinOnce();
    robot_param_config.getParams(*robot_param);
    if(!robot_param->valid()) {
      ROS_ERROR_THROTTLE(1.0, "Wait for valid robot parameter...");
    }
    else if(robot_command_ptr->rotor_velocities(0)<1.0){
      ROS_WARN_THROTTLE(1.0, "Wait for rotor vel...");
    }
    else if(!battery.isReceived(now) || !battery.valid()) {
      ROS_WARN_THROTTLE(1.0, "Wait for valid battery message...");
    }
    else{
      for (int i = 0; i < 4; ++i) {
        robot_command_ptr->rotor_velocities_command(i) = rpm_maps[i].map(
            robot_command_ptr->rotor_velocities(i));
      }
      toRosMsg(*robot_command_ptr, control_msg0);

      for (int i = 0; i < 4; ++i) {
        robot_command_ptr->rotor_velocities_command(i) =
            rpm_maps2[i].map(robot_command_ptr->rotor_velocities(i),
                            battery.voltage);
      }
      toRosMsg(*robot_command_ptr, control_msg1);

      command_pub.publish(control_msg0);
      command1_pub.publish(control_msg1);

      ROS_INFO_THROTTLE(0.5, "command rotor speed: %f, %f, %f, %f \n "
                             "actuator_0: %f, %f, %f, %f \n"
                             "actuator_1: %f, %f, %f, %f",
                        robot_command_ptr->rotor_velocities[0],
                        robot_command_ptr->rotor_velocities[1],
                        robot_command_ptr->rotor_velocities[2],
                        robot_command_ptr->rotor_velocities[3],
                        control_msg0.controls[0],
                        control_msg0.controls[1],
                        control_msg0.controls[2],
                        control_msg0.controls[3],
                        control_msg1.controls[0],
                        control_msg1.controls[1],
                        control_msg1.controls[2],
                        control_msg1.controls[3]);


    }
    rate.sleep();
  }
  return 0;
}


