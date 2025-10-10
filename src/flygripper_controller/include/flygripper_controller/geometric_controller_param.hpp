#pragma once
#include <memory>
#include "flygripper_common/type/base.hpp"
#include "flygripper_common/ros/parameters.hpp"
#include "flygripper_common/ros/dynamic_parameter_config.hpp"

#include "flygripper_controller/RosGeometricControllerParameterConfig.h"

using namespace flygripper_common;
namespace flygripper_controller
{
//! Disturbance Compensation
enum class CompensationMethod: int{
  NONE=0,
  INDI=1,
  L1=2
};

inline std::string CompensationMethodToString(CompensationMethod compensation_method){
  switch(compensation_method){
    case CompensationMethod::NONE:
      return "None";
    case CompensationMethod::INDI:
      return "INDI Controller";
    case CompensationMethod::L1:
      return "L1 Adaptive Controller";
    default:
      return "Unknown";
  }
}

class GeometricControllerParameter
{
 public:
  using Ptr = std::shared_ptr<GeometricControllerParameter>;
  //! low pass filter parameters
  Scalar acc_cutoff_freq;
  Scalar omega_cutoff_freq;
  Scalar rotor_cutoff_freq;
  Scalar ctrl_omega_xy_cutoff_freq;
  Scalar ctrl_omega_z_cutoff_freq;

  Scalar indi_acc_cutoff_freq;
  Scalar indi_alpha_cutoff_freq;

  //! geometric controller parameters
  Vector<3> k_pos;
  Vector<3> k_vel;
  Vector<3> k_att;
  Vector<3> k_omega;
  Vector<3> kd_omega;
  Vector<3> ki_omega;
  Vector<3> ki_omega_max;
  Vector<3> ff_omega;

  Vector<3> pos_err_bound;
  Vector<3> vel_err_bound;

  Vector<3> L1_force_bound;
  Vector<3> INDI_force_bound;
  Vector<3> L1_torque_bound;
  Vector<3> INDI_torque_bound;

  //! disturbance compensation
  CompensationMethod acc_compensation;
  CompensationMethod alpha_compensation;
  bool feedback_linearization;
  bool thrust_projection;
  bool tilt_prioritization;

  //! L1 adaptive controller parameters
  Scalar As_v;
  Scalar As_omega;
  Scalar thrust_flt1_cutoff_freq;
  Scalar torque_flt1_cutoff_freq, torque_flt2_cutoff_freq;

  // TODO: debug
  int magic_number;
  friend std::ostream& operator<<(std::ostream& os,
                                  const GeometricControllerParameter& p) {
    os.precision(3);
    os << std::scientific;
    os << "acc_cutoff_freq: " << p.acc_cutoff_freq << std::endl;
    os << "omega_cutoff_freq: " << p.omega_cutoff_freq << std::endl;
    os << "rotor_cutoff_freq: " << p.rotor_cutoff_freq << std::endl;
    os << "ctrl_omega_xy_cutoff_freq: " << p.ctrl_omega_xy_cutoff_freq << std::endl;
    os << "ctrl_omega_z_cutoff_freq: " << p.ctrl_omega_z_cutoff_freq << std::endl;
    os << "indi_acc_cutoff_freq: " << p.indi_acc_cutoff_freq << std::endl;
    os << "indi_alpha_cutoff_freq: " << p.indi_alpha_cutoff_freq << std::endl;

    os << "k_pos: " << p.k_pos.transpose() << std::endl;
    os << "k_vel: " << p.k_vel.transpose() << std::endl;
    os << "k_att: " << p.k_att.transpose() << std::endl;
    os << "k_omega: " << p.k_omega.transpose() << std::endl;
    os << "kd_omega: " << p.kd_omega.transpose() << std::endl;
    os << "ki_omega: " << p.ki_omega.transpose() << std::endl;
    os << "ki_omega_max: " << p.ki_omega_max.transpose() << std::endl;

    os << "pos_err_bound: " << p.pos_err_bound.transpose() << std::endl;
    os << "vel_err_bound: " << p.vel_err_bound.transpose() << std::endl;

    os << "L1_force_bound: " << p.L1_force_bound.transpose() << std::endl;
    os << "INDI_force_bound: " << p.INDI_force_bound.transpose() << std::endl;
    os << "L1_torque_bound: " << p.L1_torque_bound.transpose() << std::endl;
    os << "INDI_torque_bound: " << p.INDI_torque_bound.transpose() << std::endl;

    os << "acc_compensation: " << CompensationMethodToString(p.acc_compensation) << std::endl;
    os << "alpha_compensation: " << CompensationMethodToString(p.alpha_compensation) << std::endl;
    os << "feedback_linearization: " << (p.feedback_linearization ? "True" : "False") << std::endl;
    os << "thrust_projection: " << (p.thrust_projection ? "True" : "False") << std::endl;
    os << "tilt_prioritization: " << (p.tilt_prioritization ? "True" : "False") << std::endl;

    os << "As_v: " << p.As_v << std::endl;
    os << "As_omega: " << p.As_omega << std::endl;
    os << "thrust_flt1_cutoff_freq: " << p.thrust_flt1_cutoff_freq << std::endl;
    os << "torque_flt1_cutoff_freq: " << p.torque_flt1_cutoff_freq << std::endl;
    os << "torque_flt2_cutoff_freq: " << p.torque_flt2_cutoff_freq << std::endl;
    os.precision();
    os.unsetf(std::ios::scientific);
    return os;
  }
};

class GeometricControllerParameterConfig:
    public DynamicParameterConfig<GeometricControllerParameter,
                                  RosGeometricControllerParameterConfig> {
 public:
  GeometricControllerParameterConfig() = default;
  virtual ~GeometricControllerParameterConfig() = default;

  virtual bool initParams(ros::NodeHandle& pnh,
                                 GeometricControllerParameter& params) override{
    // config from ROS parameters
    if (!getParam("acc_cutoff_freq", params.acc_cutoff_freq, pnh)) return false;
    if (!getParam("omega_cutoff_freq", params.omega_cutoff_freq, pnh)) return false;
    if (!getParam("rotor_cutoff_freq", params.rotor_cutoff_freq, pnh)) return false;
    if (!getParam("ctrl_omega_xy_cutoff_freq", params.ctrl_omega_xy_cutoff_freq, pnh)) return false;;
    if (!getParam("ctrl_omega_z_cutoff_freq", params.ctrl_omega_z_cutoff_freq, pnh)) return false;;
    if (!getParam("indi_acc_cutoff_freq", params.indi_acc_cutoff_freq, pnh)) return false;
    if (!getParam("indi_alpha_cutoff_freq", params.indi_alpha_cutoff_freq, pnh)) return false;

    if (!getParam("k_pos", params.k_pos, pnh)) return false;
    if (!getParam("k_vel", params.k_vel, pnh)) return false;
    if (!getParam("k_att", params.k_att, pnh)) return false;
    if (!getParam("k_omega", params.k_omega, pnh)) return false;
    if (!getParam("kd_omega", params.kd_omega, pnh)) return false;
    if (!getParam("ki_omega", params.ki_omega, pnh)) return false;
    if (!getParam("ki_omega_max", params.ki_omega_max, pnh)) return false;

    if (!getParam("pos_err_bound", params.pos_err_bound, pnh)) return false;
    if (!getParam("vel_err_bound", params.vel_err_bound, pnh)) return false;

    if (!getParam("L1_force_bound", params.L1_force_bound, pnh)) return false;
    if (!getParam("INDI_force_bound", params.INDI_force_bound, pnh)) return false;
    if (!getParam("L1_torque_bound", params.L1_torque_bound, pnh)) return false;
    if (!getParam("INDI_torque_bound", params.INDI_torque_bound, pnh)) return false;

    int acc_compensation, alpha_compensation;
    if (!getParam("acc_compensation", acc_compensation, pnh)) return false;
    if (!getParam("alpha_compensation", alpha_compensation, pnh)) return false;
    params.acc_compensation = static_cast<CompensationMethod>(acc_compensation);
    params.alpha_compensation = static_cast<CompensationMethod>(alpha_compensation);

    if (!getParam("feedback_linearization", params.feedback_linearization, pnh)) return false;
    if (!getParam("thrust_projection", params.thrust_projection, pnh)) return false;
    if (!getParam("tilt_prioritization", params.tilt_prioritization, pnh)) return false;

    if (!getParam("As_v", params.As_v, pnh)) return false;
    if (!getParam("As_omega", params.As_omega, pnh)) return false;
    if (!getParam("thrust_flt1_cutoff_freq", params.thrust_flt1_cutoff_freq, pnh)) return false;
    if (!getParam("torque_flt1_cutoff_freq", params.torque_flt1_cutoff_freq, pnh)) return false;
    if (!getParam("torque_flt2_cutoff_freq", params.torque_flt2_cutoff_freq, pnh)) return false;
    //TODO: debug
    if (!getParam("magic_number", params.magic_number, pnh)) return false;
    if (!getParam("ff_omega", params.ff_omega,pnh)) return false;

    return true;
  }

  void dynamicParams2Config(const GeometricControllerParameter& params,
                            RosGeometricControllerParameterConfig& config) override{
    // convert params to dynamic reconfig
    config.acc_cutoff_freq = params.acc_cutoff_freq;
    config.omega_cutoff_freq = params.omega_cutoff_freq;
    config.rotor_cutoff_freq = params.rotor_cutoff_freq;
    config.ctrl_omega_xy_cutoff_freq = params.ctrl_omega_xy_cutoff_freq;
    config.ctrl_omega_z_cutoff_freq = params.ctrl_omega_z_cutoff_freq;
    config.indi_acc_cutoff_freq = params.indi_acc_cutoff_freq;
    config.indi_alpha_cutoff_freq = params.indi_alpha_cutoff_freq;

    config.k_pos_x = params.k_pos.x();
    config.k_pos_y = params.k_pos.y();
    config.k_pos_z = params.k_pos.z();

    config.k_vel_x = params.k_vel.x();
    config.k_vel_y = params.k_vel.y();
    config.k_vel_z = params.k_vel.z();

    config.k_att_x = params.k_att.x();
    config.k_att_y = params.k_att.y();
    config.k_att_z = params.k_att.z();

    config.k_omega_x = params.k_omega.x();
    config.k_omega_y = params.k_omega.y();
    config.k_omega_z = params.k_omega.z();

    config.kd_omega_x = params.kd_omega.x();
    config.kd_omega_y = params.kd_omega.y();
    config.kd_omega_z = params.kd_omega.z();

    config.ki_omega_x = params.ki_omega.x();
    config.ki_omega_y = params.ki_omega.y();
    config.ki_omega_z = params.ki_omega.z();
    config.ki_omega_max_x = params.ki_omega_max.x();
    config.ki_omega_max_y = params.ki_omega_max.y();
    config.ki_omega_max_z = params.ki_omega_max.z();

    config.pos_err_bound_x = params.pos_err_bound.x();
    config.pos_err_bound_y = params.pos_err_bound.y();
    config.pos_err_bound_z = params.pos_err_bound.z();

    config.vel_err_bound_x = params.vel_err_bound.x();
    config.vel_err_bound_y = params.vel_err_bound.y();
    config.vel_err_bound_z = params.vel_err_bound.z();

    config.L1_force_bound_x = params.L1_force_bound.x();
    config.L1_force_bound_y = params.L1_force_bound.y();
    config.L1_force_bound_z = params.L1_force_bound.z();

    config.INDI_force_bound_x = params.INDI_force_bound.x();
    config.INDI_force_bound_y = params.INDI_force_bound.y();
    config.INDI_force_bound_z = params.INDI_force_bound.z();

    config.L1_torque_bound_x = params.L1_torque_bound.x();
    config.L1_torque_bound_y = params.L1_torque_bound.y();
    config.L1_torque_bound_z = params.L1_torque_bound.z();

    config.INDI_torque_bound_x = params.INDI_torque_bound.x();
    config.INDI_torque_bound_y = params.INDI_torque_bound.y();
    config.INDI_torque_bound_z = params.INDI_torque_bound.z();

    config.acc_compensation = static_cast<int>(params.acc_compensation);
    config.alpha_compensation = static_cast<int>(params.alpha_compensation);
    config.feedback_linearization = params.feedback_linearization;
    config.thrust_projection = params.thrust_projection;
    config.tilt_prioritization = params.tilt_prioritization;

    config.As_v = params.As_v;
    config.As_omega = params.As_omega;
    config.thrust_flt1_cutoff_freq = params.thrust_flt1_cutoff_freq;
    config.torque_flt1_cutoff_freq = params.torque_flt1_cutoff_freq;
    config.torque_flt2_cutoff_freq = params.torque_flt2_cutoff_freq;
    //TODO: debug
    config.magic_number = params.magic_number;
  }

  void config2DynamicParams(const RosGeometricControllerParameterConfig& config,
                            GeometricControllerParameter& params) override{
    // convert dynamic reconfig to params
    params.acc_cutoff_freq = config.acc_cutoff_freq;
    params.omega_cutoff_freq = config.omega_cutoff_freq;
    params.rotor_cutoff_freq = config.rotor_cutoff_freq;
    params.ctrl_omega_xy_cutoff_freq = config.ctrl_omega_xy_cutoff_freq;
    params.ctrl_omega_z_cutoff_freq = config.ctrl_omega_z_cutoff_freq;
    params.indi_acc_cutoff_freq = config.indi_acc_cutoff_freq;
    params.indi_alpha_cutoff_freq = config.indi_alpha_cutoff_freq;

    params.k_pos.x() = config.k_pos_x;
    params.k_pos.y() = config.k_pos_y;
    params.k_pos.z() = config.k_pos_z;

    params.k_vel.x() = config.k_vel_x;
    params.k_vel.y() = config.k_vel_y;
    params.k_vel.z() = config.k_vel_z;

    params.k_att.x() = config.k_att_x;
    params.k_att.y() = config.k_att_y;
    params.k_att.z() = config.k_att_z;

    params.k_omega.x() = config.k_omega_x;
    params.k_omega.y() = config.k_omega_y;
    params.k_omega.z() = config.k_omega_z;

    params.kd_omega.x() = config.kd_omega_x;
    params.kd_omega.y() = config.kd_omega_y;
    params.kd_omega.z() = config.kd_omega_z;

    params.ki_omega.x() = config.ki_omega_x;
    params.ki_omega.y() = config.ki_omega_y ;
    params.ki_omega.z() = config.ki_omega_z;
    params.ki_omega_max.x() = config.ki_omega_max_x;
    params.ki_omega_max.y() = config.ki_omega_max_y;
    params.ki_omega_max.z() = config.ki_omega_max_z;

    params.pos_err_bound.x() = config.pos_err_bound_x;
    params.pos_err_bound.y() = config.pos_err_bound_y;
    params.pos_err_bound.z() = config.pos_err_bound_z;

    params.vel_err_bound.x() = config.vel_err_bound_x;
    params.vel_err_bound.y() = config.vel_err_bound_y;
    params.vel_err_bound.z() = config.vel_err_bound_z;

    params.L1_force_bound.x() = config.L1_force_bound_x;
    params.L1_force_bound.y() = config.L1_force_bound_y;
    params.L1_force_bound.z() = config.L1_force_bound_z;

    params.INDI_force_bound.x() = config.INDI_force_bound_x;
    params.INDI_force_bound.y() = config.INDI_force_bound_y;
    params.INDI_force_bound.z() = config.INDI_force_bound_z;

    params.L1_torque_bound.x() = config.L1_torque_bound_x;
    params.L1_torque_bound.y() = config.L1_torque_bound_y;
    params.L1_torque_bound.z() = config.L1_torque_bound_z;

    params.INDI_torque_bound.x() = config.INDI_torque_bound_x;
    params.INDI_torque_bound.y() = config.INDI_torque_bound_y;
    params.INDI_torque_bound.z() = config.INDI_torque_bound_z;

    params.acc_compensation = static_cast<CompensationMethod>(config.acc_compensation);
    params.alpha_compensation = static_cast<CompensationMethod>(config.alpha_compensation);
    params.feedback_linearization = config.feedback_linearization;
    params.thrust_projection = config.thrust_projection;
    params.tilt_prioritization = config.tilt_prioritization;

    params.As_v = config.As_v;
    params.As_omega = config.As_omega;
    params.thrust_flt1_cutoff_freq = config.thrust_flt1_cutoff_freq;
    params.torque_flt1_cutoff_freq = config.torque_flt1_cutoff_freq;
    params.torque_flt2_cutoff_freq = config.torque_flt2_cutoff_freq;
    //TODO: debug
    params.magic_number = config.magic_number;
  }
};
}

