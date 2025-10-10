#pragma once
#include <ros/ros.h>
#include <XmlRpcException.h>

#include "flygripper_common/type/base.hpp"
namespace flygripper_common{

inline double xmlRpcGetDouble(const XmlRpc::XmlRpcValue& value)
{
  if (value.getType() == XmlRpc::XmlRpcValue::TypeInt)
  {
    const int tmp = value;
    return (double)tmp;
  }
  else
    return value;
}

inline bool xmlRpcGetDouble(const XmlRpc::XmlRpcValue& xml_value,
                            const int& id, double& target_value)
{
  if (xml_value.size() > id)
  {
    ROS_ASSERT((xml_value[id].getType() == XmlRpc::XmlRpcValue::TypeDouble) ||
        (xml_value[id].getType() == XmlRpc::XmlRpcValue::TypeInt));
    target_value =  xmlRpcGetDouble(xml_value[id]);
    return true;
  }
  else
    return false;
}

inline bool xmlRpcGetDouble(XmlRpc::XmlRpcValue& xml_value,
                            const std::string& name, double& target_value)
{
  if (xml_value.hasMember(name))
  {
    ROS_ASSERT((xml_value[name].getType() == XmlRpc::XmlRpcValue::TypeDouble) ||
        (xml_value[name].getType() == XmlRpc::XmlRpcValue::TypeInt));
    target_value = xmlRpcGetDouble(xml_value[name]);
    return true;
  }
  else
    return false;
}

/**
 * @brief Get parameter from parameter server
 * @tparam T: type of parameter
 * @param name: name of parameter
 * @param parameter: parameter
 * @param pnh: node handle
 * @return true if parameter is loaded successfully
 */
template<typename T>
inline bool getParam(const std::string& name, T& parameter,
              const ros::NodeHandle& pnh = ros::NodeHandle("~"))
{
  if (pnh.getParam(name, parameter))
  {
    ROS_INFO_STREAM(
        "[" << pnh.getNamespace() << "] " << name << " = " << parameter);
    return true;
  }
  ROS_ERROR_STREAM(
      "[" << pnh.getNamespace() << "] Could not load parameter "
          << pnh.getNamespace() << "/"<< name);
  return false;
}

/**
 * @brief Get parameter from parameter server, if parameter is not found, set to default value
 * @tparam T: type of parameter
 * @param name: name of parameter
 * @param parameter: parameter
 * @param default_value: default value
 * @param pnh: node handle
 * @return true if parameter is loaded successfully
 */
template<typename T>
inline bool getParamDefault(const std::string& name, T& parameter, const T& default_value,
                     const ros::NodeHandle& pnh = ros::NodeHandle("~"))
{
  if (pnh.getParam(name, parameter))
    ROS_INFO_STREAM(
        "[" << pnh.getNamespace() << "] " << name << " = " << parameter);
  else
  {
    parameter = default_value;
    ROS_WARN_STREAM(
        "[" << pnh.getNamespace() << "] " << name << " = " << parameter
            << " set to default value");
  }
  return true;
}

inline bool getParamDefault(const std::string& name,
                            Ref<Vector<3>> vector,
                            const Ref<const Vector<3>>& default_vector,
                            const ros::NodeHandle& pnh = ros::NodeHandle("~"))
{
  if (pnh.getParam(name + "/x", vector.x())
      && pnh.getParam(name + "/y", vector.y())
      && pnh.getParam(name + "/z", vector.z())){
    ROS_INFO_STREAM(
        "[" << pnh.getNamespace() << "] " << name
            << " x: " << vector.x()
            << ", y: " << vector.y()
            << ", z: " << vector.z());
  }
  else
  {
    vector = default_vector;
    ROS_WARN_STREAM(
        "[" << pnh.getNamespace() << "] " << name <<" set to default value "
            << " x: " << vector.x()
                   << ", y: " << vector.y()
                   << ", z: " << vector.z());
  }
  return true;
}

/**
 * @brief Get parameter (std::vector) from parameter server
 * @tparam T: type of parameter
 * @param name: name of parameter
 * @param parameter: parameter(std::vector)
 * @param pnh: node handle
 * @return true if parameter is loaded successfully
 */
template<typename T>
inline bool getParam(const std::string& name, std::vector<T>& parameter,
              const ros::NodeHandle& pnh = ros::NodeHandle("~"))
{
  if (pnh.getParam(name, parameter))
  {
    std::stringstream parameter_str;
    for (auto i: parameter){ parameter_str<<i<<" "; }
    ROS_INFO_STREAM(
        "[" << pnh.getNamespace() << "] " << name << " = [ " << parameter_str.str() <<" ]");
    return true;
  }
  ROS_ERROR_STREAM(
      "[" << pnh.getNamespace() << "] Could not load parameter "
          << pnh.getNamespace() << "/"<< name);
  return false;
}

/**
 * @brief Get parameter(Eigen::Vector2d) from parameter server
 * @param name: name of parameter
 * @param vector: parameter(Eigen::Vector2d)
 * @param pnh: node handle
 * @return true if parameter is loaded successfully
 */
template<>
inline bool getParam(const std::string& name, Vector<2>& vector,
                     const ros::NodeHandle& pnh)
{
  if (pnh.getParam(name + "/x", vector.x())
      && pnh.getParam(name + "/y", vector.y())){
    ROS_INFO_STREAM(
        "[" << pnh.getNamespace() << "] " << name
            << " x: " << vector.x()
            << ", y: " << vector.y());
    return true;
  }
  else{
    ROS_ERROR_STREAM(
        "[" << pnh.getNamespace() << "] Could not load parameter "
            << pnh.getNamespace() << "/"<< name);
    return false;
  }
}

/**
 * @brief Get parameter(Eigen::Vector3d) from parameter server
 * @param name: name of parameter
 * @param vector: parameter(Eigen::Vector3d)
 * @param pnh: node handle
 * @return true if parameter is loaded successfully
 */
template<>
inline bool getParam(const std::string& name, Vector<3>& vector,
              const ros::NodeHandle& pnh)
{
  if (pnh.getParam(name + "/x", vector.x())
      && pnh.getParam(name + "/y", vector.y())
      && pnh.getParam(name + "/z", vector.z())){
    ROS_INFO_STREAM(
        "[" << pnh.getNamespace() << "] " << name
            << " x: " << vector.x()
            << ", y: " << vector.y()
            << ", z: " << vector.z());
    return true;
  }
  else{
    ROS_ERROR_STREAM(
        "[" << pnh.getNamespace() << "] Could not load parameter "
            << pnh.getNamespace() << "/"<< name);
    return false;
  }
}

/**
 * @brief Get parameter(Eigen::Quaterniond) from parameter server
 * @param name: name of parameter
 * @param q: parameter(Eigen::Quaterniond)
 * @param pnh: node handle
 * @return true if parameter is loaded successfully
 */
template<>
inline bool getParam(const std::string& name, Quaternion& q,
              const ros::NodeHandle& pnh)
{
  if (pnh.getParam(name + "/x", q.x()) && pnh.getParam(name + "/y", q.y())
      && pnh.getParam(name + "/z", q.z()) && pnh.getParam(name + "/w", q.w())){
    ROS_INFO_STREAM(
        "[" << pnh.getNamespace() << "] " << name
            << " w: " << q.w()
            << ", x: " << q.x()
            << ", y: " << q.y()
            << ", z: " << q.z());
    return true;
  }
  else{
    ROS_ERROR_STREAM(
        "[" << pnh.getNamespace() << "] Could not load parameter "
            << pnh.getNamespace() << "/"<< name);
    return false;
  }
}

/**
 * @brief Get parameter(std::vector<Eigen::Vector3d>) from parameter server
 * @param name: name of parameter
 * @param waypoints: parameter(std::vector<Eigen::Vector3d>)
 * @param pnh: node handle
 * @return true if parameter is loaded successfully
 */
template<>
inline bool getParam(const std::string& name, std::vector<Vector<3>> & waypoints,
              const ros::NodeHandle& pnh)
{
  unsigned int index = 0;
  std::string name_prefix;
  Vector<3> tmp_waypoint;
  while(true) {
    /** Parameter Naming:
     *          name = "waypoints"
     *          waypoints/p0/x = 1.0,
     *          waypoints/p0/y = 2.0,
     *          waypoints/p0/z = 1.0,
     *          ...
     *
     *  YAML File:
     *  waypoints:
     *      p0: {x:1.0, y:2.0, z:1.0}
     *
     **/
    name_prefix = name + "/p" + std::to_string(index);

    if (!pnh.getParam(name_prefix + "/x", tmp_waypoint.x())){
      ROS_INFO_STREAM(
          "[" << pnh.getNamespace() << "] " << "Load "<< index << "waypoints");
      return true;
    }

    if (!pnh.getParam(name_prefix + "/y", tmp_waypoint.y())){
      ROS_ERROR_STREAM(
          "[" << pnh.getNamespace() << "] " << "Miss "
              << pnh.getNamespace() << "/"<< name_prefix +"/y");
      return false;
    }

    if (!pnh.getParam(name_prefix + "/z", tmp_waypoint.z())){
      ROS_ERROR_STREAM(
          "[" << pnh.getNamespace() << "] " << "Miss "
              << pnh.getNamespace() << "/"<< name_prefix +"/z");
      return false;
    }
    waypoints.push_back(tmp_waypoint);
    index++;
  }
}



} // namespace common