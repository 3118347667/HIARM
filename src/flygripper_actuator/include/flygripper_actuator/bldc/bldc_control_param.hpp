#pragma once

#include "flygripper_common/controller/pid.hpp"
#include "flygripper_common/ros/dynamic_parameter_config.hpp"
#include "flygripper_common/ros/parameters.hpp"

#include "flygripper_actuator/BLDCConfig.h"
#include "flygripper_actuator/bldc/bldc.hpp"


using flygripper_common::PidParams;
using flygripper_common::xmlRpcGetDouble;
using flygripper_common::DynamicParameterConfig;

namespace flygripper_actuator{
namespace bldc{

struct Parameters{
  // dynamic parameters
  PidParams pid;

  // static parameters
  double loop_frequency;
  double loop_time_error_threshold;

  std::string actuator_bus;
  std::string actuator_interface;
  int actuator_id;
  std::string actuator_product;
  ActuatorParam actuator_param;
  std::string actuator_joint_name;

  bool enable_joint_state_publisher;

  friend std::ostream& operator<<(std::ostream& os, const Parameters& p) {
    os.precision(3);
    os << std::scientific;
    os << "pid: " << std::endl << p.pid;
    os << "loop_frequency: " << p.loop_frequency << std::endl
       << "loop_time_error_threshold: " << p.loop_time_error_threshold << std::endl

       << "actuator_bus: " << p.actuator_bus << std::endl
       << "actuator_interface: " << p.actuator_interface << std::endl
       << "actuator_id: " << p.actuator_id << std::endl
       << "actuator_product: " << p.actuator_product << std::endl
       << "actuator_param: " << std::endl << p.actuator_param
       << "actuator_joint_name: " << p.actuator_joint_name << std::endl

       << "enable_joint_state_publisher: " << p.enable_joint_state_publisher << std::endl;
    os.precision();
    os.unsetf(std::ios::scientific);
    return os;
  }
};

class ParameterConfig:
    public DynamicParameterConfig<Parameters, BLDCConfig> {
 public:
  ParameterConfig() = default;
  virtual ~ParameterConfig() = default;

  virtual bool initParams(ros::NodeHandle& pnh, Parameters& params) override{
    // config from ROS static parameters
    pnh.param("loop_frequency", params.loop_frequency, 10.0);
    pnh.param("loop_time_error_threshold", params.loop_time_error_threshold, 1.0/params.loop_frequency);

    pnh.param("actuator_bus", params.actuator_bus, std::string("socket_can"));
    pnh.param("actuator_interface", params.actuator_interface, std::string("can0"));
    pnh.param("actuator_id", params.actuator_id, 0x201);
    pnh.param("actuator_product", params.actuator_product, std::string("rm2006"));
    pnh.param("actuator_joint_name", params.actuator_joint_name, std::string("actuator_joint"));

    pnh.param("enable_joint_state_publisher", params.enable_joint_state_publisher, false);

    bool find_actuator = false;
    if (pnh.hasParam("actuators")) {
      XmlRpc::XmlRpcValue actuator_params;
      pnh.getParam("actuators", actuator_params);
      if (actuator_params.getType() == XmlRpc::XmlRpcValue::TypeStruct) {
        for (auto it = actuator_params.begin(); it != actuator_params.end(); ++it) {
          std::string name = it->first;
          XmlRpc::XmlRpcValue param = it->second;
          if (params.actuator_product == name && param.getType() == XmlRpc::XmlRpcValue::TypeStruct){
            find_actuator = true;
            ROS_INFO("Loading actuator parameters for %s", name.c_str());
            if (!xmlRpcGetDouble(param, "act2pos", params.actuator_param.act2pos)){
              ROS_ERROR("Failed to load actuator %s parameter act2pos", name.c_str());
              return false;
            }
            if (!xmlRpcGetDouble(param, "act2vel", params.actuator_param.act2vel)){
              ROS_ERROR("Failed to load actuator %s parameter act2vel", name.c_str());
              return false;
            }
            if (!xmlRpcGetDouble(param, "act2effort", params.actuator_param.act2effort)){
              ROS_ERROR("Failed to load actuator %s parameter act2effort", name.c_str());
              return false;
            }
            if (!xmlRpcGetDouble(param, "effort2act", params.actuator_param.effort2act)){
              ROS_ERROR("Failed to load actuator %s parameter effort2act", name.c_str());
              return false;
            }
            if (!xmlRpcGetDouble(param, "max_out", params.actuator_param.max_out)){
              ROS_ERROR("Failed to load actuator %s parameter max_out", name.c_str());
              return false;
            }
            if (!xmlRpcGetDouble(param, "offset_angle", params.actuator_param.offset_angle)){
              ROS_ERROR("Failed to load actuator %s parameter offset_angle", name.c_str());
              return false;
            }
            if (!xmlRpcGetDouble(param, "lpf_cutoff_freq", params.actuator_param.lpf_cutoff_freq)){
              ROS_ERROR("Failed to load actuator %s parameter lpf_cutoff_freq", name.c_str());
              return false;
            }
            if (!xmlRpcGetDouble(param, "lpf_sampling_freq", params.actuator_param.lpf_sampling_freq)){
              ROS_ERROR("Failed to load actuator %s parameter lpf_sampling_freq", name.c_str());
              return false;
            }
          }
        }
        if (!find_actuator){
          ROS_ERROR("Failed to find actuator %s, "
              "please check if it is a valid actuator product name",
              params.actuator_product.c_str());
          return false;
        }
      }
    } else {
      ROS_ERROR("Failed to load actuator parameters");
      return false;
    }

    // config from ROS dynamic parameters
    pnh.param("k_p", params.pid.k_p, 0.0);
    pnh.param("k_i", params.pid.k_i, 0.0);
    pnh.param("k_d", params.pid.k_d, 0.0);
    pnh.param("integral_zone", params.pid.integral_zone, 0.0);
    pnh.param("integral_min", params.pid.integral_min, 0.0);
    pnh.param("integral_max", params.pid.integral_max, 0.0);
    pnh.param("dead_zone", params.pid.dead_zone, 0.0);
    pnh.param("command_min", params.pid.command_min, 0.0);
    pnh.param("command_max", params.pid.command_max, 0.0);

    return true;
  }

  void dynamicParams2Config(const Parameters& params, BLDCConfig& config) override{
    // convert params to dynamic reconfig
    config.k_p = params.pid.k_p;
    config.k_i = params.pid.k_i;
    config.k_d = params.pid.k_d;
    config.integral_zone = params.pid.integral_zone;
    config.integral_min = params.pid.integral_min;
    config.integral_max = params.pid.integral_max;
    config.dead_zone = params.pid.dead_zone;
    config.command_min = params.pid.command_min;
    config.command_max = params.pid.command_max;
  }

  void config2DynamicParams(const BLDCConfig& config, Parameters& params) override{
    // convert dynamic reconfig to params
    params.pid.k_p = config.k_p;
    params.pid.k_i = config.k_i;
    params.pid.k_d = config.k_d;
    params.pid.integral_zone = config.integral_zone;
    params.pid.integral_min = config.integral_min;
    params.pid.integral_max = config.integral_max;
    params.pid.dead_zone = config.dead_zone;
    params.pid.command_min = config.command_min;
    params.pid.command_max = config.command_max;
  }

};
}
}