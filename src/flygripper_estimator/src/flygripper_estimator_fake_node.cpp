#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

#include "flygripper_common/ros/parameters.hpp"
#include "flygripper_common/type/robot_state.hpp"

int main(int argc, char** argv)
{
  using namespace flygripper_common;
  ros::init(argc, argv, "flygripper_estimator_fake_node");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  ros::Publisher robot_state_pub = nh.advertise<flygripper_msgs::RobotState>("robot_state", 100);
  ros::Publisher odom_pub = nh.advertise<nav_msgs::Odometry>("odom", 100);
  tf::TransformBroadcaster tf_broadcaster;

  double update_frequency;
  if (!getParam("update_frequency", update_frequency, pnh)){
    throw ros::Exception("Failed to load parameters");
  }
  ros::Rate rate(update_frequency);
  RobotState robot_state;
  flygripper_msgs::RobotState robot_state_msg;
  nav_msgs::Odometry odom_msg;
  while(ros::ok()){
    ros::spinOnce();
    ros::Time now = ros::Time::now();
    robot_state.stamp = now.toSec();
    robot_state.pos = Vector<3>::Zero();
    robot_state.setQ(Quaternion::Identity());
    robot_state.vel = Vector<3>::Zero();
    robot_state.omega = Vector<3>::Zero();
    robot_state.acc = Vector<3>::Zero(); // acc in world frame
    robot_state.alpha = Vector<3>::Zero();
    robot_state.rot_speed = Vector<4>::Zero();
    robot_state.rot1_pos  << 0.0725, -0.0875;
    robot_state.rot2_pos << -0.0725, 0.08;
    robot_state.rot3_pos << 0.0725, 0.0875;
    robot_state.rot4_pos << -0.0725, -0.08;

    // publish tf without delay
    tf::Transform W_T_B;
    W_T_B.setOrigin( tf::Vector3(robot_state.pos.x(),
                                robot_state.pos.y(),
                                robot_state.pos.z()) );
    W_T_B.setRotation(tf::Quaternion(robot_state.getQ().x(), robot_state.getQ().y(),
                                     robot_state.getQ().z(), robot_state.getQ().w()));
    ros::Time tf_time = now; //ros::Time::now();
    tf_broadcaster.sendTransform(tf::StampedTransform(W_T_B, tf_time,
                                                      "world", "flygripper"));

    toRosMsg(robot_state, robot_state_msg);
    toRosMsg(robot_state, odom_msg);
    robot_state_pub.publish(robot_state_msg);
    odom_pub.publish(odom_msg);


    rate.sleep();
  }
  return 0;
}