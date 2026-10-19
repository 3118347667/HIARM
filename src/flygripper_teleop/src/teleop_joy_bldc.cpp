#include "ros/ros.h"
#include "sensor_msgs/Joy.h"

#include "flygripper_msgs/RotorCommand.h"
#include "flygripper_msgs/ActuatorCommand.h"


std::vector<float> joy_axis;

void joyCallback(const sensor_msgs::Joy::ConstPtr& msg){
  joy_axis = msg->axes;
}

int main(int argc, char **argv) {

  ros::init(argc, argv, "teleop_joy_node");
  ros::NodeHandle nh, pnh("~");
  std::string actuator_mode;
  pnh.param<std::string>("actuator_mode", actuator_mode, "position");
  ROS_INFO("actuator_mode: %s", actuator_mode.c_str());
  double loop_frequency;
  pnh.param<double>("loop_frequency", loop_frequency, 500.0);
  ROS_INFO("loop_frequency: %f", loop_frequency);
  std::string actuator_topic, rotor_topic;
  pnh.param<std::string>("actuator_topic", actuator_topic, "/flygripper/actuator_command");
  ROS_INFO("actuator_topic: %s", actuator_topic.c_str());
  pnh.param<std::string>("rotor_topic", rotor_topic, "/flygripper/rotor_command");
  ROS_INFO("rotor_topic: %s", rotor_topic.c_str());
  double actuator_scale, rotor_scale;
  pnh.param<double>("actuator_scale", actuator_scale, 10.0);
  pnh.param<double>("rotor_scale", rotor_scale, 10.0);
  ROS_INFO("actuator_scale: %f", actuator_scale);
  ROS_INFO("rotor_scale: %f", rotor_scale);


  auto joy_sub = nh.subscribe("joy", 1, joyCallback);
  auto actuator_pub = nh.advertise<flygripper_msgs::ActuatorCommand>(actuator_topic, 1);
  auto rotor_pub = nh.advertise<flygripper_msgs::RotorCommand>(rotor_topic, 1);

  flygripper_msgs::ActuatorCommand actuator_msg;
  actuator_msg.mode = flygripper_msgs::ActuatorCommand::MODE_POSITION;
  actuator_msg.reference_position = 0.0;
  actuator_msg.reference_velocity = 0.0;

  flygripper_msgs::RotorCommand rotor_msg;
  rotor_msg.mode = flygripper_msgs::RotorCommand::MODE_SINGLE_ROTOR;
  rotor_msg.rotor_velocities.resize(4, 0.0);

  joy_axis.resize(6, 0.0);
  ros::Rate rate(loop_frequency);
  while (ros::ok()){
    ros::spinOnce();

    actuator_msg.header.stamp = ros::Time::now();
    if(actuator_mode=="position"){
      // joy_axis[1]  vs -joy_axis[2]
      actuator_msg.reference_position = -joy_axis[2] * actuator_scale;
      ROS_INFO_THROTTLE(0.5, "actuator_msg.reference_position: %f", actuator_msg.reference_position);
    }
    else if(actuator_mode=="velocity"){
      // joy_axis[1]  vs -joy_axis[2]
      actuator_msg.reference_position += -joy_axis[2] * actuator_scale /loop_frequency;
      ROS_INFO_THROTTLE(0.5, "actuator_msg.reference_position: %f", actuator_msg.reference_position);
    } else{
      ROS_ERROR_ONCE("Invalid actuator mode: %s", actuator_mode.c_str());
    }

    // rotor
    rotor_msg.header.stamp = ros::Time::now();
    for(int i=0; i<4; i++){
      double coeff = (i==0)? 0.4/0.3: 1.0;
      // joy_axis[3] vs -joy_axis[1]
      rotor_msg.rotor_velocities[i] = -joy_axis[1] * rotor_scale * coeff;
    }
    ROS_INFO_THROTTLE(0.5, "rotor_msg.rotor_velocities: [%f, %f, %f, %f]",
      rotor_msg.rotor_velocities[0], rotor_msg.rotor_velocities[1],
      rotor_msg.rotor_velocities[2], rotor_msg.rotor_velocities[3]);
    rotor_pub.publish(rotor_msg);

    actuator_pub.publish(actuator_msg);
    rate.sleep();
  }
}