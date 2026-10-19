#pragma once

#include <iostream>
#include <memory>
#include <unordered_map>
#include <chrono>
#include <cmath>

#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include "flygripper_common/type/trajectory.hpp"
#include "flygripper_common/type/const.hpp"
enum class Color{
  RED,
  BLUE,
  GREEN,
  WHITE,
  YELLOW,
  GREEN_BLUE
};
class Visualizer
{
 public:
  using Ptr = std::shared_ptr<Visualizer>;
  Visualizer() = default;
  Visualizer(ros::NodeHandle &nh)
      : nh_(nh)
  { }
  
  void setMarkerColor(visualization_msgs::Marker& marker,
                      Color color = Color::RED,
                      double alpha = 1) {
    marker.color.a = alpha;
    switch (color) {
    case Color::WHITE:
      marker.color.r = 1;
      marker.color.g = 1;
      marker.color.b = 1;
      break;
    case Color::RED:
      marker.color.r = 1;
      marker.color.g = 0;
      marker.color.b = 0;
      break;
    case Color::GREEN:
      marker.color.r = 0;
      marker.color.g = 1;
      marker.color.b = 0;
      break;
    case Color::BLUE:
      marker.color.r = 0;
      marker.color.g = 0;
      marker.color.b = 1;
      break;
    case Color::YELLOW:
      marker.color.r = 1;
      marker.color.g = 1;
      marker.color.b = 0;
      break;
    case Color::GREEN_BLUE:
      marker.color.r = 0;
      marker.color.g = 1;
      marker.color.b = 1;
      break;
    }
  }
  
  void visualizeWaypoints(const Eigen::Matrix3Xd &route,
                                 std::string topic_name="/visualizer/waypoints",
                                 Color color = Color::RED,
                                 const ros::Time& stamp=ros::Time::now())
  {
    checkPublisher<visualization_msgs::Marker>(topic_name);

    visualization_msgs::Marker waypoints_marker;
    waypoints_marker.id = 0;
    waypoints_marker.type = visualization_msgs::Marker::SPHERE_LIST;
    waypoints_marker.header.stamp = stamp;
    waypoints_marker.header.frame_id = flygripper_common::kFixedFrame;
    waypoints_marker.pose.orientation.w = 1.00;
    waypoints_marker.action = visualization_msgs::Marker::ADD;
    waypoints_marker.ns = topic_name;
    setMarkerColor(waypoints_marker, color, 1.0);
    waypoints_marker.scale.x = 0.25;
    waypoints_marker.scale.y = 0.25;
    waypoints_marker.scale.z = 0.25;

    for (int i = 0; i < route.cols(); ++i)
    {
      geometry_msgs::Point point;
      point.x = route.col(i)(0);
      point.y = route.col(i)(1);
      point.z = route.col(i)(2);
      waypoints_marker.points.push_back(point);
    }
    publisher_map_[topic_name].publish(waypoints_marker);
  }

  void visualizeWaypoints(const std::vector<Eigen::Vector3d> &route,
                                 std::string topic_name="/visualizer/waypoints",
                                 Color color = Color::RED,
                                 const ros::Time& stamp=ros::Time::now())
  {
    checkPublisher<visualization_msgs::Marker>(topic_name);

    visualization_msgs::Marker waypoints_marker;
    waypoints_marker.id = 0;
    waypoints_marker.type = visualization_msgs::Marker::SPHERE_LIST;
    waypoints_marker.header.stamp = stamp;
    waypoints_marker.header.frame_id = flygripper_common::kFixedFrame;
    waypoints_marker.pose.orientation.w = 1.00;
    waypoints_marker.action = visualization_msgs::Marker::ADD;
    waypoints_marker.ns = topic_name;
    setMarkerColor(waypoints_marker, color, 1.0);
    waypoints_marker.scale.x = 0.25;
    waypoints_marker.scale.y = 0.25;
    waypoints_marker.scale.z = 0.25;

    for (size_t i = 0; i < route.size(); ++i)
    {
      geometry_msgs::Point point;
      point.x = route[i].x();
      point.y = route[i].y();
      point.z = route[i].z();
      waypoints_marker.points.push_back(point);
    }
    publisher_map_[topic_name].publish(waypoints_marker);
  }

  template<int XYZ_Order, int Yaw_Order>
  void visualizeTrajectory(const flygripper_common::PolynomialTrajectory<3, XYZ_Order> &xyz_poly,
                                  const flygripper_common::PolynomialTrajectory<1, Yaw_Order> &yaw_poly,
                                  std::string topic_name="/visualizer/trajectory",
                                  const double dT=0.01,
                                  const ros::Time& stamp=ros::Time::now())
  {
    checkPublisher<nav_msgs::Path>(topic_name);

    nav_msgs::Path path;
    path.header.stamp = stamp;
    path.header.frame_id = flygripper_common::kFixedFrame;
    path.poses.clear();
    if (xyz_poly.getPieceNum() > 0)
    {
      const double T = dT ;
//      std::min(dT, xyz_poly.getTotalDuration() * dT);
      for (double t = 0; t < xyz_poly.getTotalDuration(); t += T)
      {
        Eigen::Vector3d p = xyz_poly.getPos(t);
        double yaw = yaw_poly.getPos(t)(0);

        geometry_msgs::PoseStamped pose;
        pose.header.stamp = stamp+ros::Duration(t);
        pose.header.frame_id = flygripper_common::kFixedFrame;
        pose.pose.position.x = p(0);
        pose.pose.position.y = p(1);
        pose.pose.position.z = p(2);
        Eigen::Quaterniond q(Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()));
        pose.pose.orientation.x = q.x();
        pose.pose.orientation.y = q.y();
        pose.pose.orientation.z = q.z();
        pose.pose.orientation.w = q.w();
        path.poses.push_back(pose);
      }
      publisher_map_[topic_name].publish(path);
    }
  }

  template<int XYZ_Order>
  void visualizeTrajectory(const flygripper_common::PolynomialTrajectory<3, XYZ_Order> &xyz_poly,
                                  std::string topic_name="/visualizer/trajectory",
                                  const double dT=0.01,
                                  const ros::Time& stamp=ros::Time::now())
  {
    checkPublisher<nav_msgs::Path>(topic_name);

    nav_msgs::Path path;
    path.header.stamp = stamp;
    path.header.frame_id = flygripper_common::kFixedFrame;
    path.poses.clear();
    if (xyz_poly.getPieceNum() > 0)
    {
      const double T = dT ;
      //      std::min(dT, xyz_poly.getTotalDuration() * dT);
      for (double t = 0; t < xyz_poly.getTotalDuration(); t += T)
      {
        Eigen::Vector3d p = xyz_poly.getPos(t);

        geometry_msgs::PoseStamped pose;
        pose.header.stamp = stamp+ros::Duration(t);
        pose.header.frame_id = flygripper_common::kFixedFrame;
        pose.pose.position.x = p(0);
        pose.pose.position.y = p(1);
        pose.pose.position.z = p(2);
        path.poses.push_back(pose);
      }
      publisher_map_[topic_name].publish(path);
    }
  }
  template<class MsgType>
  void checkPublisher(std::string topic_name) {
    auto got = publisher_map_.find(topic_name);
    if (got == publisher_map_.end()) {
      ros::Publisher pub =
          nh_.advertise<MsgType>(topic_name, 10);
      publisher_map_[topic_name] = pub;
      ros::Duration(0.1).sleep();
    }
  }
 private:
  ros::NodeHandle nh_;
  using PublisherMap = std::unordered_map<std::string, ros::Publisher>;
  PublisherMap publisher_map_;
};

