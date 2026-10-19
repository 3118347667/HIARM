#ifndef POINT_H
#define POINT_H

#include "fsto/config.h"
#include "gcopter/gcopter.hpp"
#include <flygripper_msgs/PolynomialTrajectory.h>

#include <chrono>
#include <cmath>
#include <iostream>
#include <memory>

#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <ros/console.h>
#include <ros/ros.h>

#include "flygripper_common/trajectory/polynomial_trajectory.hpp"
using flygripper_common::PolynomialTrajectory;

class MavGlobalPlanner {
public:
  MavGlobalPlanner(Config &conf, ros::NodeHandle &nh_);

  Config config;
  ros::NodeHandle nh;

  ros::Subscriber goalSub;
  void goalCallBack(const geometry_msgs::PoseStamped::ConstPtr &msg);

  ros::Subscriber odomSub;
  Eigen::Vector3d curOdom;
  ros::Time odomStamp;
  bool odomInitialized;
  void odomCallBack(const nav_msgs::Odometry::ConstPtr &msg);

  ros::Publisher trajPub;

  static void
  polynomialTrajConverter(const PolynomialTrajectory<3, 7> &traj,
                          flygripper_msgs::PolynomialTrajectory &msg,
                          ros::Time &iniStamp, const int &headingMode = 0);
};

#endif
