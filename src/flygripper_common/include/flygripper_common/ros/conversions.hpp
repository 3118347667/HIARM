#pragma once
#include "ros/ros.h"
#include "geometry_msgs/Quaternion.h"
#include "geometry_msgs/Vector3.h"

#include "flygripper_common/type/base.hpp"
namespace flygripper_common{

inline void fromRosMsg(const boost::shared_ptr<geometry_msgs::Quaternion const>& msg, Quaternion& quat) {
    quat.w() = msg->w;
    quat.x() = msg->x;
    quat.y() = msg->y;
    quat.z() = msg->z;
}

inline void fromRosMsg(const boost::shared_ptr<geometry_msgs::Quaternion const>& msg, Ref<Vector<4>> vec) {
    vec(0) = msg->x;
    vec(1) = msg->y;
    vec(2) = msg->z;
    vec(3) = msg->w;
}

// Vector3 conversions for geometry_msgs::Vector3
inline void fromRosMsg(const boost::shared_ptr<geometry_msgs::Vector3 const>& msg, Ref<Vector<3>> vec) {
    vec.x() = msg->x;
    vec.y() = msg->y;
    vec.z() = msg->z;
}

/** Basic conversions between ROS and Eigen types
 */
inline void fromRosQuat(
    const geometry_msgs::Quaternion msg, Quaternion& quat)
{
  quat.w() = msg.w;
  quat.x() = msg.x;
  quat.y() = msg.y;
  quat.z() = msg.z;

}

inline void fromRosQuat(
    const geometry_msgs::Quaternion msg, Ref<Vector<4>> vec)
{
  vec.w() = msg.w;
  vec.x() = msg.x;
  vec.y() = msg.y;
  vec.z() = msg.z;
}

inline void toRosQuat(
    const Quaternion quat, geometry_msgs::Quaternion& msg )
{
  msg.w = quat.w();
  msg.x = quat.x();
  msg.y = quat.y();
  msg.z = quat.z();
}

inline void toRosQuat(
    const Vector<4> vec, geometry_msgs::Quaternion& msg )
{
  msg.w = vec.w();
  msg.x = vec.x();
  msg.y = vec.y();
  msg.z = vec.z();
}

template<typename T>
inline void fromRosVec3(const T& msg, Ref<Vector<3>> vec)
{
  vec.x() = msg.x;
  vec.y() = msg.y;
  vec.z() = msg.z;
}

template<typename T>
inline void toRosVec3(ConstRef<Vector<3>> vec, T& msg)
{
  msg.x = vec.x();
  msg.y = vec.y();
  msg.z = vec.z();
}

}
