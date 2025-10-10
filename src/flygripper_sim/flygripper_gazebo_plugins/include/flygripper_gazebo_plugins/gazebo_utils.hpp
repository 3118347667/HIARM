#pragma once
#include <ros/ros.h>
#include <gazebo/common/common.hh>
#include <Eigen/Eigen>

namespace gazebo
{
// Gazebo types
using GzVector3d = ignition::math::Vector3d;
using GzPose3d = ignition::math::Pose3d;
using GzQuaterniond = ignition::math::Quaterniond;
using GzMatrix4d = ignition::math::Matrix4d;
using GzMatrix3d = ignition::math::Matrix3d;
using GzBox = ignition::math::AxisAlignedBox;

/// --------------------------------- Param ------------------------------------
// Read parameters with default value
template <typename T>
inline bool getSdfParam(const std::string& plugin_name,
                        const sdf::ElementPtr& sdf,
                        const std::string& param_name, T& param,
                        const T& default_value)
{
  if (sdf->HasElement(param_name)) {
    param = sdf->GetElement(param_name)->Get<T>();
    return true;
  } else {
    param = default_value;
    ROS_WARN_STREAM(plugin_name << " plugin warning: "
                                << "Please specify " << param_name.c_str()
                                << ", set to default value: " << param);
    return false;
  }
}

// Read parameters without default value
template <typename T>
inline bool getSdfParam(const std::string& plugin_name,
                        const sdf::ElementPtr& sdf,
                        const std::string& param_name, T& param)
{
  if (sdf->HasElement(param_name)) {
    param = sdf->GetElement(param_name)->Get<T>();
    return true;
  } else {
    ROS_ERROR("%s plugin error: "
              "Please specify %s.",
              plugin_name.c_str(),
              param_name.c_str());
    return false;
  }
}

// Read multiple parameters without default value
template <typename T>
inline bool getSdfParams(const std::string& plugin_name,
                        const sdf::ElementPtr& sdf,
                        const std::string& param_name, std::vector<T>& param)
{
  if (sdf->HasElement(param_name)) {
    sdf::ElementPtr param_elem = sdf->GetElement(param_name);
    for (; param_elem != NULL;
           param_elem = param_elem->GetNextElement(param_name))
    {
      param.push_back(param_elem->Get<T>());
    }
    return true;
  } else {
    ROS_ERROR("%s plugin error: "
              "Please specify %s.",
              plugin_name.c_str(),
              param_name.c_str());
    return false;
  }
}

// Certain format nested parameters utils
template <typename T>
inline bool getSdfNestedParam(const std::string& plugin_name,
                              const sdf::ElementPtr& sdf,
                              const std::string& param_name, T& param)
{
  std::string delimiter = "/";
  size_t pos = 0;
  std::string splitted_param_name = param_name;
  sdf::ElementPtr splitted_sdf = sdf;
  std::string subgroup_name;
  // iteration subgroup
  while ((pos = splitted_param_name.find(delimiter)) != std::string::npos) {
    subgroup_name = splitted_param_name.substr(0, pos);
    if (splitted_sdf->HasElement(subgroup_name)) {
      splitted_sdf = splitted_sdf->GetElement(subgroup_name);
    } else {
      ROS_ERROR("%s plugin error: "
                "The subgroup %s in parameter %s does not exist.",
                plugin_name.c_str(),
                subgroup_name.c_str(),
                param_name.c_str());
      return false;
    }
    splitted_param_name.erase(0, pos + delimiter.length());
  }

  if (splitted_sdf->HasElement(splitted_param_name)) {
    param = splitted_sdf->GetElement(splitted_param_name)->Get<T>();
    return true;
  } else {
    ROS_ERROR("%s plugin error: "
              "Please specify %s in parameter %s.",
              plugin_name.c_str(),
              splitted_param_name.c_str(),
              param_name.c_str());
    return false;
  }
}

template <typename T>
inline bool getSdfNestedParam(const std::string& plugin_name,
                              const sdf::ElementPtr& sdf,
                              const std::string& param_name, T& param,
                              const T& default_value)
{
  std::string delimiter = "/";
  size_t pos = 0;
  std::string splitted_param_name = param_name;
  sdf::ElementPtr splitted_sdf = sdf;
  std::string subgroup_name;
  // iteration subgroup
  while ((pos = splitted_param_name.find(delimiter)) != std::string::npos) {
    subgroup_name = splitted_param_name.substr(0, pos);
    if (splitted_sdf->HasElement(subgroup_name)) {
      splitted_sdf = splitted_sdf->GetElement(subgroup_name);
    } else {
      ROS_ERROR("%s plugin error: "
                "The subgroup %s in parameter %s does not exist.",
                plugin_name.c_str(),
                subgroup_name.c_str(),
                param_name.c_str());
      return false;
    }
    splitted_param_name.erase(0, pos + delimiter.length());
  }
  // get parameter
  if (splitted_sdf->HasElement(splitted_param_name)) {
    param = splitted_sdf->GetElement(splitted_param_name)->Get<T>();
    return true;
  } else {
    param = default_value;
    ROS_WARN_STREAM(plugin_name << " plugin warning: "
                                << "Please specify " << splitted_param_name.c_str()
                                << " in parameter " << param_name.c_str()
                                << ", set to default value: " << param);
    return false;
  }
}

template <typename A>
inline sdf::ElementPtr getSdfElementByAttribute(const std::string& plugin_name,
                                                const sdf::ElementPtr& sdf,
                                                const std::string& group_name,
                                                const std::string& attribute_name,
                                                const A& attribute_value)
{
  std::string delimiter = "/";
  size_t pos = 0;
  std::string splitted_name = group_name;
  sdf::ElementPtr splitted_sdf = sdf;
  std::string subgroup_name;
  // subgroup iteration
  while ((pos = splitted_name.find(delimiter)) != std::string::npos) {
    subgroup_name = splitted_name.substr(0, pos);
    if (splitted_sdf->HasElement(subgroup_name)) {
      splitted_sdf = splitted_sdf->GetElement(subgroup_name);
    } else {
      ROS_ERROR("%s plugin error: "
                "The subgroup %s in parameter group %s does not exist.",
                plugin_name.c_str(),
                subgroup_name.c_str(),
                group_name.c_str());
      return nullptr;
    }
    splitted_name.erase(0, pos + delimiter.length());
  }
  // match the last subgroup
  if (splitted_sdf->HasElement(splitted_name)) {
    splitted_sdf = splitted_sdf->GetElement(splitted_name);
  } else {
    ROS_ERROR("%s plugin error: "
              "The %s in parameter group %s does not exist.",
              plugin_name.c_str(),
              splitted_name.c_str(),
              group_name.c_str());
    return nullptr;
  }

  // last subgroup has attribute name
  if (!splitted_sdf->HasAttribute(attribute_name)) {
    ROS_ERROR("%s plugin error: "
              "The attribute %s in parameter group %s does not exist.",
              plugin_name.c_str(),
              attribute_name.c_str(),
              group_name.c_str());
    return nullptr;
  }

  A read_attribute_value;
  // subgroup matches attribute value
  while ((read_attribute_value =
              splitted_sdf->Get<A>(attribute_name) != attribute_value)) {
    splitted_sdf = splitted_sdf->GetNextElement(subgroup_name);
    if (!splitted_sdf) {
      ROS_ERROR_STREAM(plugin_name << " plugin error: "
                                   << "The attribute "
                                   << attribute_name.c_str() << " in parameter group "
                                   << group_name.c_str() << " does not match the value "
                                   << attribute_value);
      return nullptr;
    }
  }

  return splitted_sdf;
}

inline int getSdfElementCount(const std::string& plugin_name,
                              const sdf::ElementPtr& sdf,
                              const std::string& group_name)
{
  std::string delimiter = "/";
  size_t pos = 0;
  std::string splitted_name = group_name;
  sdf::ElementPtr splitted_sdf = sdf;
  std::string subgroup_name;
  // subgroup iteration
  while ((pos = splitted_name.find(delimiter)) != std::string::npos) {
    subgroup_name = splitted_name.substr(0, pos);
    if (splitted_sdf->HasElement(subgroup_name)) {
      splitted_sdf = splitted_sdf->GetElement(subgroup_name);
    } else {
      ROS_ERROR("%s plugin error: "
                "The subgroup %s in parameter group %s does not exist.",
                plugin_name.c_str(),
                subgroup_name.c_str(),
                group_name.c_str());
      return 0;
    }
    splitted_name.erase(0, pos + delimiter.length());
  }
  // match the last subgroup
  if (splitted_sdf->HasElement(splitted_name)) {
    splitted_sdf = splitted_sdf->GetElement(splitted_name);
  } else {
    ROS_ERROR("%s plugin error: "
              "The %s in parameter group %s does not exist.",
              plugin_name.c_str(),
              splitted_name.c_str(),
              group_name.c_str());
    return 0;
  }

  int element_count = 0;
  while (splitted_sdf) {
    element_count++;
    splitted_sdf = splitted_sdf->GetNextElement(splitted_name);
  }
  return element_count;
}

/// ---------------------------- FirstOrderFilter ------------------------------
// First Order Filter
template <typename T>
class FirstOrderFilter
{
  ///           Short reveiw of discrete time implementation of first order system: Laplace:
  ///             X(s)/U(s) = 1/(tau*s + 1)
  ///           continous time system:
  ///             dx(t) = (-1/tau)*x(t) + (1/tau)*u(t)
  ///           discretized system (ZoH):
  ///             x(k+1) = exp(samplingTime*(-1/tau))*x(k) + (1 - exp(samplingTime*(-1/tau))) * u(k)
 public:
  using Ptr = std::unique_ptr<FirstOrderFilter<T>>;
  FirstOrderFilter(T initial_state, double time_constant_up,
                   double time_constant_down)
      : last_state_(initial_state),
        time_constant_up_(time_constant_up),
        time_constant_down_(time_constant_down)
  {
  }

  FirstOrderFilter(T initial_state, double time_constant)
      : last_state_(initial_state),
        time_constant_up_(time_constant),
        time_constant_down_(time_constant)
  {
  }

  ~FirstOrderFilter() {}

  T update(T input, double dt)
  {
    T state;
    if (input > last_state_) {
      // Calcuate the outputState if accelerating.
      double alpha_up = exp(-dt / time_constant_up_);
      // x(k+1) = Ad*x(k) + Bd*u(k)
      state = alpha_up * last_state_ + (1 - alpha_up) * input;

    } else {
      // Calculate the outputState if decelerating.
      double alpha_down = exp(-dt / time_constant_down_);
      state = alpha_down * last_state_ + (1 - alpha_down) * input;
    }
    last_state_ = state;
    return state;
  }

 protected:
  double time_constant_up_;
  double time_constant_down_;
  T last_state_;
};


/// ---------------------------- Eigen Conversion ------------------------------
template <class Derived>
inline Eigen::Quaternion<typename Derived::Scalar> quaternionFromSmallAngle(
    const Eigen::MatrixBase<Derived>& theta)
{
  using Scalar = typename Derived::Scalar;
  EIGEN_STATIC_ASSERT_FIXED_SIZE(Derived);
  EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Derived, 3);
  const Scalar q_squared = theta.squaredNorm() / 4.0;

  if (q_squared < 1) {
    return Eigen::Quaternion<Scalar>(sqrt(1 - q_squared), theta[0] * 0.5,
                                     theta[1] * 0.5, theta[2] * 0.5);
  } else {
    const Scalar w = 1.0 / sqrt(1 + q_squared);
    const Scalar f = w * 0.5;
    return Eigen::Quaternion<Scalar>(w, theta[0] * f, theta[1] * f,
                                     theta[2] * f);
  }
}

template <typename Scalar>
inline Eigen::Quaternion<Scalar> convertToEigen(
    const ignition::math::Quaternion<Scalar> quaternion)
{
  return Eigen::Quaternion<Scalar>(quaternion.W(), quaternion.X(),
                                   quaternion.Y(), quaternion.Z());
}
}