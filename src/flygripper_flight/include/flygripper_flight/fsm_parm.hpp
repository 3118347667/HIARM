#pragma once

#include "flygripper_common/ros/dynamic_parameter_config.hpp"
#include "flygripper_common/ros/parameters.hpp"
#include "flygripper_common/type/robot_command.hpp"

#include "flygripper_flight/RosFsmParameterConfig.h"

using namespace flygripper_common;

namespace flygripper_flight{
enum class ControllerType: int{
  GEOMETRIC_CONTROLLER=0,
  NMPC_CONTROLLER=1
};

inline std::string ControllerTypeToString(ControllerType controller_type){
  switch(controller_type){
  case ControllerType::GEOMETRIC_CONTROLLER:
    return "Geometric Controller";
  case ControllerType::NMPC_CONTROLLER:
    return "NMPC Controller";
  default:
    return "Unknown";
  }
}

enum class AdmittanceMode: int{
  DISABLE=0,
  ADMITTANCE=1,
  ZERO_FORCE=2,
};

inline  std::string AdmittanceModeToString(AdmittanceMode admittance_mode){
  switch(admittance_mode){
  case AdmittanceMode::DISABLE:
    return "DISABLE";
  case AdmittanceMode::ADMITTANCE:
    return "ADMITTANCE";
  case AdmittanceMode::ZERO_FORCE:
    return "ZERO_FORCE";
  default:
    return "Unknown";
  }
}



struct FsmParameter{
  using Ptr=std::shared_ptr<FsmParameter>;
  // static parameters
  Scalar loop_frequency;
  Scalar loop_time_error_threshold;
  Scalar imu_timeout;
  Scalar odom_timeout;
  Scalar rotor_speed_timeout;
  Scalar actuator_state_timeout;
  Scalar fc_state_timeout;
  Scalar rc_input_timeout;
  Scalar battery_timeout;

  // dynamic parameters
  ControllerType controller;
  RobotCommandMode command_mode;
  bool auto_takeoff;
  Scalar takeoff_height;
  bool traj_trigger;

  bool velocity_teleop;
  Scalar velocity_scale_x;
  Scalar velocity_scale_y;
  Scalar velocity_scale_z;
  Scalar velocity_scale_yaw;
  bool world_frame_velocity;

  AdmittanceMode admittance_mode;
  bool enable_admittance_x, enable_admittance_y, enable_admittance_z, enable_admittance_yaw;
  Vector<4> coeff_M, coeff_D, coeff_K;
  Vector<4> admittance_max_vel, admittance_max_acc;

  Scalar external_force_threshold, external_torque_threshold;
  Scalar external_wrench_trigger_duration;
  int external_wrench_buffer_size;
  Scalar external_wrench_cutoff_frequency;

  friend std::ostream& operator<<(std::ostream& os, const FsmParameter& p) {
    os.precision(3);
    os << std::scientific;
    os << "loop_frequency: " << p.loop_frequency << std::endl;
    os << "loop_time_error_threshold: " << p.loop_time_error_threshold << std::endl;
    os << "imu_timeout: " << p.imu_timeout << std::endl;
    os << "odom_timeout: " << p.odom_timeout << std::endl;
    os << "rotor_speed_timeout: " << p.rotor_speed_timeout << std::endl;
    os << "actuator_state_timeout: " << p.actuator_state_timeout << std::endl;
    os << "fc_state_timeout: " << p.fc_state_timeout << std::endl;
    os << "rc_input_timeout: " << p.rc_input_timeout << std::endl;
    os << "battery_timeout: " << p.battery_timeout << std::endl;

    os << "controller: " << ControllerTypeToString(p.controller) << std::endl;
    os << "controller_mode: " << CommandModeToString(p.command_mode) << std::endl;

    os << "auto_takeoff: " << (p.auto_takeoff ? "True" : "False") << std::endl;
    os << "takeoff_height: " << p.takeoff_height << std::endl;

    os << "traj_trigger: " << (p.traj_trigger ? "True" : "False") << std::endl;

    os << "velocity_teleop: " << (p.velocity_teleop ? "True" : "False") << std::endl;
    os << "velocity_scale_x: " << p.velocity_scale_x << std::endl;
    os << "velocity_scale_y: " << p.velocity_scale_y << std::endl;
    os << "velocity_scale_z: " << p.velocity_scale_z << std::endl;
    os << "world_frame_velocity: " << (p.world_frame_velocity ? "True" : "False") << std::endl;

    os << "admittance_mode: " << AdmittanceModeToString(p.admittance_mode) << std::endl;
    os << "coeff_M:  [" << p.coeff_M.transpose() << "]" << std::endl;
    os << "coeff_D:  [" << p.coeff_D.transpose() << "]" << std::endl;
    os << "coeff_K:  [" << p.coeff_K.transpose() << "]" << std::endl;

    os.precision();
    os.unsetf(std::ios::scientific);
    return os;
  }
};

class FsmParameterConfig:
    public DynamicParameterConfig<FsmParameter, RosFsmParameterConfig> {
 public:
  FsmParameterConfig() = default;
  virtual ~FsmParameterConfig() = default;

  virtual bool initParams(ros::NodeHandle& pnh, FsmParameter& params) override{
    // config from ROS parameters [static is marked]
    getParamDefault("loop_frequency", params.loop_frequency, 333.0, pnh); //static
    getParamDefault("loop_time_error_threshold", params.loop_time_error_threshold, 1.0/params.loop_frequency, pnh); //static
    getParamDefault("imu_timeout", params.imu_timeout, 0.1, pnh); //static
    getParamDefault("odom_timeout", params.odom_timeout, 0.1, pnh); //static
    getParamDefault("rotor_speed_timeout", params.rotor_speed_timeout, 0.1, pnh); //static
    getParamDefault("actuator_state_timeout", params.actuator_state_timeout, 0.1, pnh); //static
    getParamDefault("fc_state_timeout", params.fc_state_timeout, 0.1, pnh); //static
    getParamDefault("rc_input_timeout", params.rc_input_timeout, 5.0, pnh); //static
    getParamDefault("battery_timeout", params.battery_timeout, 5.0, pnh); //static

    int controller, command_mode;
    if (!getParam("controller", controller, pnh)) return false; //static
    params.controller = static_cast<ControllerType>(controller);
    if (!getParam("command_mode", command_mode, pnh)) return false; //static
    params.command_mode = static_cast<RobotCommandMode>(command_mode);

    getParamDefault("auto_takeoff", params.auto_takeoff, false, pnh);
    getParamDefault("takeoff_height", params.takeoff_height, 1.0, pnh);

    getParamDefault("traj_trigger", params.traj_trigger, false, pnh);

    getParamDefault("velocity_teleop", params.velocity_teleop, false, pnh);
    getParamDefault("velocity_scale_x", params.velocity_scale_x, 0.3, pnh); //static
    getParamDefault("velocity_scale_y", params.velocity_scale_y, 0.3, pnh); //static
    getParamDefault("velocity_scale_z", params.velocity_scale_z, 0.1, pnh); //static
    getParamDefault("velocity_scale_yaw", params.velocity_scale_yaw, 0.0, pnh); //static
    getParamDefault("world_frame_velocity", params.world_frame_velocity, false, pnh);

    int admittance_mode;
    getParamDefault("admittance_mode", admittance_mode, 0, pnh);
    params.admittance_mode =  static_cast<AdmittanceMode>(admittance_mode);
    getParamDefault("enable_admittance_x", params.enable_admittance_x, true, pnh);
    getParamDefault("enable_admittance_y", params.enable_admittance_y, true, pnh);
    getParamDefault("enable_admittance_z", params.enable_admittance_z, true, pnh);
    getParamDefault("enable_admittance_yaw", params.enable_admittance_yaw, true, pnh);

    getParamDefault("coeff_M_x",   params.coeff_M(0), 0.5, pnh);
    getParamDefault("coeff_M_y",   params.coeff_M(1), 0.5, pnh);
    getParamDefault("coeff_M_z",   params.coeff_M(2), 0.5, pnh);
    getParamDefault("coeff_M_yaw", params.coeff_M(3), 0.5, pnh);
    getParamDefault("coeff_D_x",   params.coeff_D(0), 0.5, pnh);
    getParamDefault("coeff_D_y",   params.coeff_D(1), 0.5, pnh);
    getParamDefault("coeff_D_z",   params.coeff_D(2), 0.5, pnh);
    getParamDefault("coeff_D_yaw", params.coeff_D(3), 0.5, pnh);
    getParamDefault("coeff_K_x",   params.coeff_K(0), 0.5, pnh);
    getParamDefault("coeff_K_y",   params.coeff_K(1), 0.5, pnh);
    getParamDefault("coeff_K_z",   params.coeff_K(2), 0.5, pnh);
    getParamDefault("coeff_K_yaw", params.coeff_K(3), 0.5, pnh);

    getParamDefault("admittance_max_vel_x", params.admittance_max_vel(0), 0.5, pnh);
    getParamDefault("admittance_max_vel_y", params.admittance_max_vel(1), 0.5, pnh);
    getParamDefault("admittance_max_vel_z", params.admittance_max_vel(2), 0.5, pnh);
    getParamDefault("admittance_max_vel_yaw", params.admittance_max_vel(3), 0.5, pnh);
    getParamDefault("admittance_max_acc_x", params.admittance_max_acc(0), 0.5, pnh);
    getParamDefault("admittance_max_acc_y", params.admittance_max_acc(1), 0.5, pnh);
    getParamDefault("admittance_max_acc_z", params.admittance_max_acc(2), 0.5, pnh);
    getParamDefault("admittance_max_acc_yaw", params.admittance_max_acc(3), 0.5, pnh);


    if (!getParam("external_wrench_buffer_size", params.external_wrench_buffer_size, pnh)) return false;  //static
    if (!getParam("external_force_threshold", params.external_force_threshold, pnh)) return false;
    if (!getParam("external_torque_threshold", params.external_torque_threshold, pnh)) return false;

    if (!getParam("external_wrench_trigger_duration", params.external_wrench_trigger_duration, pnh)) return false;
    if (!getParam("external_wrench_cutoff_frequency", params.external_wrench_cutoff_frequency, pnh)) return false;

    return true;
  }

  void dynamicParams2Config(const FsmParameter& params, RosFsmParameterConfig& config) override{
    // convert params to dynamic reconfig
    config.controller = static_cast<int>(params.controller);
    config.command_mode = static_cast<int>(params.command_mode);
    config.auto_takeoff = params.auto_takeoff;
    config.takeoff_height = params.takeoff_height;
    config.traj_trigger = params.traj_trigger;
    config.velocity_teleop = params.velocity_teleop;
    config.world_frame_velocity = params.world_frame_velocity;

    config.admittance_mode = static_cast<int>(params.admittance_mode);
    config.enable_admittance_x = params.enable_admittance_x;
    config.enable_admittance_y = params.enable_admittance_y;
    config.enable_admittance_z = params.enable_admittance_z;
    config.enable_admittance_yaw = params.enable_admittance_yaw;

    config.coeff_M_x = params.coeff_M(0);
    config.coeff_M_y = params.coeff_M(1);
    config.coeff_M_z = params.coeff_M(2);
    config.coeff_M_yaw = params.coeff_M(3);
    config.coeff_D_x = params.coeff_D(0);
    config.coeff_D_y = params.coeff_D(1);
    config.coeff_D_z = params.coeff_D(2);
    config.coeff_D_yaw = params.coeff_D(3);
    config.coeff_K_x = params.coeff_K(0);
    config.coeff_K_y = params.coeff_K(1);
    config.coeff_K_z = params.coeff_K(2);
    config.coeff_K_yaw = params.coeff_K(3);
    config.admittance_max_vel_x = params.admittance_max_vel(0);
    config.admittance_max_vel_y = params.admittance_max_vel(1);
    config.admittance_max_vel_z = params.admittance_max_vel(2);
    config.admittance_max_vel_yaw = params.admittance_max_vel(3);
    config.admittance_max_acc_x = params.admittance_max_acc(0);
    config.admittance_max_acc_y = params.admittance_max_acc(1);
    config.admittance_max_acc_z = params.admittance_max_acc(2);
    config.admittance_max_acc_yaw = params.admittance_max_acc(3);

    config.external_force_threshold = params.external_force_threshold;
    config.external_torque_threshold = params.external_torque_threshold;
    config.external_wrench_trigger_duration = params.external_wrench_trigger_duration;
    config.external_wrench_cutoff_frequency = params.external_wrench_cutoff_frequency;
  }

  void config2DynamicParams(const RosFsmParameterConfig& config, FsmParameter& params) override{
    // convert dynamic reconfig to params
    params.controller = static_cast<ControllerType>(config.controller);
    params.command_mode = static_cast<RobotCommandMode>(config.command_mode);
    params.auto_takeoff = config.auto_takeoff;
    params.takeoff_height = config.takeoff_height;
    params.traj_trigger = config.traj_trigger;
    params.velocity_teleop = config.velocity_teleop;
    params.world_frame_velocity = config.world_frame_velocity;

    params.admittance_mode = static_cast<AdmittanceMode>(config.admittance_mode);
    params.enable_admittance_x = config.enable_admittance_x;
    params.enable_admittance_y = config.enable_admittance_y;
    params.enable_admittance_z = config.enable_admittance_z;
    params.enable_admittance_yaw = config.enable_admittance_yaw;

    params.coeff_M << config.coeff_M_x, config.coeff_M_y, config.coeff_M_z, config.coeff_M_yaw;
    params.coeff_D << config.coeff_D_x, config.coeff_D_y, config.coeff_D_z, config.coeff_D_yaw;
    params.coeff_K << config.coeff_K_x, config.coeff_K_y, config.coeff_K_z, config.coeff_K_yaw;
    params.admittance_max_vel << config.admittance_max_vel_x, config.admittance_max_vel_y,
        config.admittance_max_vel_z, config.admittance_max_vel_yaw;
    params.admittance_max_acc << config.admittance_max_acc_x, config.admittance_max_acc_y,
        config.admittance_max_acc_z, config.admittance_max_acc_yaw;

    params.external_force_threshold = config.external_force_threshold;
    params.external_torque_threshold = config.external_torque_threshold;
    params.external_wrench_trigger_duration = config.external_wrench_trigger_duration;
    params.external_wrench_cutoff_frequency = config.external_wrench_cutoff_frequency;
  }
};
}

