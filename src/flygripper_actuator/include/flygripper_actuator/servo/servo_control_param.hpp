#pragma once

#include "flygripper_common/controller/pid.hpp"
#include "flygripper_common/ros/dynamic_parameter_config.hpp"
#include "flygripper_common/ros/parameters.hpp"

#include "flygripper_actuator/ServoConfig.h"
#include "flygripper_actuator/servo/servo.hpp"



using flygripper_common::xmlRpcGetDouble;
using flygripper_common::DynamicParameterConfig;

namespace flygripper_actuator{
namespace servo{

enum class TeleopMode: int {
  MODE_DISABLE = 0,
  MODE_PARAM = 1,
  MODE_CMD = 2,
  MODE_RC = 3
};

inline std::string TeleopModeToString(const TeleopMode& mode){
  switch(mode){
  case TeleopMode::MODE_DISABLE:
    return "DISABLE";
  case TeleopMode::MODE_PARAM:
    return "PARAM";
  case TeleopMode::MODE_CMD:
    return "CMD";
  case TeleopMode::MODE_RC:
    return "RC";
  default:
    return "Unknown";
  }
}

struct Parameters{
  // static parameters
  double loop_frequency;
  double loop_time_error_threshold;

  std::string actuator_bus;
  std::string actuator_interface;
  int actuator_id;
  int baud_rate;
  std::string actuator_product;
  ActuatorParam actuator_param;
  std::string actuator_joint_name;

  bool enable_joint_state_publisher;

  // dynamic parameters
  TeleopMode teleop_mode;
  double reference_position;
  double one_morphing_angle;
  friend std::ostream& operator<<(std::ostream& os, const Parameters& p) {
    os.precision(3);
    os << std::scientific;
    os << "loop_frequency: " << p.loop_frequency << std::endl
       << "loop_time_error_threshold: " << p.loop_time_error_threshold << std::endl

       << "actuator_bus: " << p.actuator_bus << std::endl
       << "actuator_interface: " << p.actuator_interface << std::endl
       << "baud_rate: " << p.baud_rate << std::endl

       << "actuator_id: " << p.actuator_id << std::endl
       << "actuator_product: " << p.actuator_product << std::endl
       << "actuator_param: " << std::endl << p.actuator_param
       << "actuator_joint_name: " << p.actuator_joint_name << std::endl

       << "enable_joint_state_publisher: " << p.enable_joint_state_publisher << std::endl;
    os << "teleop_mode: " << TeleopModeToString(p.teleop_mode) << std::endl;
    os << "reference_position: " << p.reference_position << std::endl;
    os << "one_morphing_angle: "<< p.one_morphing_angle << std::endl;
    os.precision();
    os.unsetf(std::ios::scientific);
    return os;
  }
};



class ParameterConfig:
    public DynamicParameterConfig<Parameters, ServoConfig> {
 public:
  ParameterConfig() = default;
  virtual ~ParameterConfig() = default;

  virtual bool initParams(ros::NodeHandle& pnh, Parameters& params) override{
    // config from ROS static parameters
    pnh.param("loop_frequency", params.loop_frequency, 10.0);
    pnh.param("loop_time_error_threshold", params.loop_time_error_threshold, 1.0/params.loop_frequency);

    pnh.param("actuator_bus", params.actuator_bus, std::string("uart"));
    pnh.param("baud_rate", params.baud_rate, 1000000);
    pnh.param("actuator_interface", params.actuator_interface, std::string("/dev/FEETECH_SERVO"));

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
    pnh.param("reference_position", params.reference_position, 0.0);
    int teleop_mode_int;
    pnh.param("teleop_mode", teleop_mode_int, 0);
    params.teleop_mode = static_cast<TeleopMode>(teleop_mode_int);
    pnh.param("one_morphing_angle", params.one_morphing_angle, 0.0);
    return true;
  }

  void dynamicParams2Config(const Parameters& params, ServoConfig& config) override{
    // convert params to dynamic reconfig
    config.reference_position = static_cast<int>(params.reference_position/M_PI*180);
    config.teleop_mode = static_cast<int>(params.teleop_mode);
    config.one_morphing_angle = static_cast<double>(params.one_morphing_angle);
  }

  void config2DynamicParams(const ServoConfig& config, Parameters& params) override{
    // convert dynamic reconfig to params
    params.reference_position = static_cast<double>(config.reference_position/180.0*M_PI);
    params.teleop_mode = static_cast<TeleopMode>(config.teleop_mode);
    params.one_morphing_angle = static_cast<double>(config.one_morphing_angle);
  }
};
}
}