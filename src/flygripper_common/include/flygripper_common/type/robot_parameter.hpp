#pragma once
#include <memory>
#include <Eigen/Dense>
#include "flygripper_common/type/base.hpp"
#include "flygripper_common/type/robot_command.hpp"
#include "flygripper_common/type/robot_state.hpp"
#include "flygripper_common/type/reference.hpp"

#include "flygripper_common/ros/dynamic_parameter_config.hpp"
#include "flygripper_common/ros/parameters.hpp"
#include "flygripper_common/RosRobotParameterConfig.h"

namespace flygripper_common{

class RobotParameter{
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  using Ptr = std::shared_ptr<RobotParameter>;
  // dynamic parameters
  /** Inertial properties **/
  Scalar mass{INF};
  Matrix<3, 3> J{Vector<3>::Constant(INF).asDiagonal()};
  Matrix<3, 3> J_inv{Vector<3>::Constant(INF).asDiagonal()};

  /** Rotor coefficients **/
  Scalar thrust_coeff{INF}; // c_t
  Scalar moment_coeff{INF}; // c_q

  /** Dynamic modelling of mixing matrix **/
  bool dynamic_mixing{false};
  /** Limit of bodyrates and attitude mode **/
  Scalar collective_thrust_min{0.0};
  Scalar collective_thrust_max{INF};
  Vector<3> torque_max{Vector<3>::Constant(INF)};
  Vector<3> bodyrates_max{Vector<3>::Constant(INF)};
  Scalar tilt_angle_max{INF}; // maximum of tilt angle
  /** Limit of velocity and acceleration mode **/
  Scalar velocity_max{INF};
  Scalar acceleration_max{INF};

  // static parameters
  /**  Limit of single rotor mode **/
  Vector<4> rotor_thrusts_min{Vector<4>::Zero()};
  Vector<4> rotor_thrusts_max{Vector<4>::Constant(INF)};

  /** Coefficient of mapping from dshot command to rotor rpm **/
  int dshot_rpm_map_method;
  // static parameters
  //! method 0
  Vector<3> dshot_rpm_map_coeff{Vector<3>::Constant(INF)};
  //! method 1
  Vector<4> dshot_rpm_volt_map_coeff{Vector<4>::Constant(INF)};
  //! method 2
  std::string dshot_rpm_volt_map_file;

  /** Coefficient of mapping from thrust command to thrust acc **/
  Scalar hover_percentage{INF};

  // initial rotor position
  Matrix<2, 4> rotor_position{Matrix<2, 4>::Constant(INF)};

  RobotParameter() = default;
  ~RobotParameter() = default;

  Matrix<4,4> getControlAllocation(ConstRef<Matrix<2,4>> rot_pos){
    Matrix<4,4> G;
    Scalar moment_thrust_ratio = moment_coeff / thrust_coeff;
    G << 1.0, 1.0, 1.0, 1.0,
        rot_pos(1,0), rot_pos(1,1), rot_pos(1,2), rot_pos(1,3),
        -rot_pos(0,0), -rot_pos(0,1), -rot_pos(0,2), -rot_pos(0,3),
        -moment_thrust_ratio, -moment_thrust_ratio, moment_thrust_ratio, moment_thrust_ratio;
    return G;
  }

  inline bool valid() const{
    bool ret = true;
    // mass and inertia
    ret &= mass > 0.0 && mass < 10.0;
    ret &= J.allFinite() && J_inv.allFinite();

    // rotor coefficients
    ret &= std::isfinite(thrust_coeff);
    ret &= std::isfinite(moment_coeff);

    // collective thrust limit
    ret &= collective_thrust_min >= 0.0;
    ret &= collective_thrust_max > collective_thrust_min;

    // body rates limit
    ret &= (torque_max.array() > 0.0).all();
    ret &= (bodyrates_max.array() > 0.0).all();
    ret &= tilt_angle_max > 0.0;

    // rotor thrusts limit
    ret &= (rotor_thrusts_min.array() >= 0.0).all();
    ret &= (rotor_thrusts_max.array() > rotor_thrusts_min.array()).all();

    // rpm mapping
    ret &= dshot_rpm_map_method >= 0 && dshot_rpm_map_method <= 2;
    ret &= (dshot_rpm_map_method == 0 && dshot_rpm_map_coeff.allFinite()) || (dshot_rpm_map_method == 1 && dshot_rpm_volt_map_coeff.allFinite());

    // thrust mapping
    ret &= hover_percentage >= 0.1 && hover_percentage <= 0.8;
    // rotor position
    ret &= rotor_position.allFinite();
    return ret;
  }

  // Print
  friend std::ostream& operator<<(std::ostream& os, const RobotParameter& robot){
    os.precision(3);
    os << std::scientific;
    os << "mass:         [ " << robot.mass << " ]" <<std::endl;
    os << "J:            [ " << robot.J.row(0) << " ]" << std::endl;
    os << "              [ " << robot.J.row(1) << " ]" << std::endl;
    os << "              [ " << robot.J.row(2) << " ]" << std::endl;
    os << "thrust_coeff: [ " << robot.thrust_coeff << " ]" << std::endl;
    os << "moment_coeff: [ " << robot.moment_coeff << " ]" << std::endl;
    os << "velocity_max: [ " << robot.velocity_max << " ]" << std::endl;
    os << "acceleration_max: [ " << robot.acceleration_max << " ]" << std::endl;
    os << "collective_thrust_max: [ " << robot.collective_thrust_max << " ]" << std::endl;
    os << "collective_thrust_min: [ " << robot.collective_thrust_min << " ]" << std::endl;
    os << "bodyrates_max: [ " << robot.bodyrates_max.transpose() << " ]" << std::endl;
    os << "torque_max: [ " << robot.torque_max.transpose() << " ]" << std::endl;
    os << "tilt_angle_max: [ " << robot.tilt_angle_max << " ]" << std::endl;
    os << "rotor_thrusts_max: [ " << robot.rotor_thrusts_max.transpose() << " ]" << std::endl;
    os << "rotor_thrusts_min: [ " << robot.rotor_thrusts_min.transpose() << " ]" << std::endl;

    os << "dynamic_mixing: " << robot.dynamic_mixing << std::endl;
    if (robot.dshot_rpm_map_method == 0){
      os << "dshot_rpm_map_coeff: [ " << robot.dshot_rpm_map_coeff << " ]" << std::endl;
    } else if (robot.dshot_rpm_map_method == 1){
      os << "dshot_rpm_volt_map_coeff: [ " << robot.dshot_rpm_volt_map_coeff << " ]" << std::endl;
    } else if (robot.dshot_rpm_map_method == 2){
      os << "dshot_rpm_volt_map_file: [ " << robot.dshot_rpm_volt_map_file << " ]" << std::endl;
    }

    os << "hover_percentage: [" << robot.hover_percentage << "]" << std::endl;

    os << "rotor_position 0: [ " << robot.rotor_position.col(0).transpose() << " ]" << std::endl;
    os << "               1: [ " << robot.rotor_position.col(1).transpose() << " ]" << std::endl;
    os << "               2: [ " << robot.rotor_position.col(2).transpose() << " ]" << std::endl;
    os << "               3: [ " << robot.rotor_position.col(3).transpose() << " ]" << std::endl;
    os.precision();
    os.unsetf(std::ios::scientific);
    return os;
  }
};

class RobotParameterConfig:
    public DynamicParameterConfig<RobotParameter,
                                  RosRobotParameterConfig> {
 public:
  RobotParameterConfig() = default;
  virtual ~RobotParameterConfig() = default;

  virtual bool initParams(ros::NodeHandle& pnh,
                          RobotParameter& params) override{
    // config from ROS dynamic parameters
    if (!getParam("mass", params.mass, pnh)) return false;

    Vector<3> J_diag;
    if (!getParam("inertia", J_diag, pnh)) return false;
    params.J = J_diag.asDiagonal();
    params.J_inv = params.J.inverse();

    if (!getParam("thrust_coeff", params.thrust_coeff, pnh)) return false;
    if (!getParam("moment_coeff", params.moment_coeff, pnh)) return false;

    if (!getParam("collective_thrust_min", params.collective_thrust_min, pnh)) return false;
    if (!getParam("collective_thrust_max", params.collective_thrust_max, pnh)) return false;

    if (!getParam("velocity_max", params.velocity_max, pnh)) return false;
    if (!getParam("acceleration_max", params.acceleration_max, pnh)) return false;
    if (!getParam("torque_max", params.torque_max, pnh)) return false;
    if (!getParam("bodyrates_max", params.bodyrates_max, pnh)) return false;
    if (!getParam("tilt_angle_max", params.tilt_angle_max, pnh)) return false;
    if (!getParam("dynamic_mixing", params.dynamic_mixing, pnh)) return false;

    // config from ROS static parameters
    std::vector<Scalar> rotor_thrusts_min_vec, rotor_thrusts_max_vec;
    if (!getParam("rotor_thrusts_min", rotor_thrusts_min_vec, pnh)) return false;
    if (!getParam("rotor_thrusts_max", rotor_thrusts_max_vec, pnh)) return false;
    for(size_t i=0; i<rotor_thrusts_min_vec.size(); i++){
      params.rotor_thrusts_min(i) = rotor_thrusts_min_vec[i];
      params.rotor_thrusts_max(i) = rotor_thrusts_max_vec[i];
    }


    // config from ROS dynamic parameters
    if (!getParam("dshot_rpm_map_method", params.dshot_rpm_map_method, pnh)) return false;

    // config from ROS static parameters
    std::vector<Scalar> dshot_rpm_map_coeff_vec;
    if (!getParam("dshot_rpm_map_coeff", dshot_rpm_map_coeff_vec, pnh)) return false;
    for(size_t i=0; i<dshot_rpm_map_coeff_vec.size(); i++){
      params.dshot_rpm_map_coeff(i) = dshot_rpm_map_coeff_vec[i];
    }

    std::vector<Scalar> dshot_rpm_volt_map_coeff_vec;
    if (!getParam("dshot_rpm_volt_map_coeff", dshot_rpm_volt_map_coeff_vec, pnh)) return false;
    for(size_t i=0; i<dshot_rpm_volt_map_coeff_vec.size(); i++){
      params.dshot_rpm_volt_map_coeff(i) = dshot_rpm_volt_map_coeff_vec[i];
    }

    if (!getParam("dshot_rpm_volt_map_file", params.dshot_rpm_volt_map_file, pnh)) return false;

    if (!getParam("hover_percentage", params.hover_percentage, pnh)) return false;

    Vector<2> rotor_position_tmp;
    for (int i = 0; i < 4; i++){
      if (!getParam("rotor" + std::to_string(i) + "_position",
                    rotor_position_tmp , pnh)) return false;
      params.rotor_position.block<2,1>(0,i) = rotor_position_tmp;
    }

    // check all parameters validity from yaml
    if (!params.valid()) return false;
    return true;
  }

  void dynamicParams2Config(const RobotParameter& params,
                            RosRobotParameterConfig& config) override{
    // convert params to dynamic reconfig
    config.mass = params.mass;
    config.inertia_x = params.J(0,0);
    config.inertia_y = params.J(1,1);
    config.inertia_z = params.J(2,2);

    config.thrust_coeff = params.thrust_coeff;
    config.moment_coeff = params.moment_coeff;

    config.collective_thrust_min = params.collective_thrust_min;
    config.collective_thrust_max = params.collective_thrust_max;

    config.velocity_max = params.velocity_max;
    config.acceleration_max = params.acceleration_max;
    config.tilt_angle_max = params.tilt_angle_max;

    config.bodyrates_max_x = params.bodyrates_max.x();
    config.bodyrates_max_y = params.bodyrates_max.y();
    config.bodyrates_max_z = params.bodyrates_max.z();

    config.torque_max_x = params.torque_max.x();
    config.torque_max_y = params.torque_max.y();
    config.torque_max_z = params.torque_max.z();

    config.dynamic_mixing = params.dynamic_mixing;

    config.dshot_rpm_map_method = params.dshot_rpm_map_method;
    config.hover_percentage = params.hover_percentage;
  }

  void config2DynamicParams(const RosRobotParameterConfig& config,
                            RobotParameter& params) override{
    // convert dynamic reconfig to params
    params.mass = config.mass;
    params.J(0,0) = config.inertia_x;
    params.J(1,1) = config.inertia_y;
    params.J(2,2) = config.inertia_z;
    params.J_inv = params.J.inverse();
    params.thrust_coeff = config.thrust_coeff;
    params.moment_coeff = config.moment_coeff;
    params.collective_thrust_min = config.collective_thrust_min;
    params.collective_thrust_max = config.collective_thrust_max;
    params.velocity_max = config.velocity_max;
    params.acceleration_max = config.acceleration_max;
    params.tilt_angle_max = config.tilt_angle_max;
    params.torque_max.x() = config.torque_max_x;
    params.torque_max.y() = config.torque_max_y;
    params.torque_max.z() = config.torque_max_z;
    params.bodyrates_max.x() = config.bodyrates_max_x;
    params.bodyrates_max.y() = config.bodyrates_max_y;
    params.bodyrates_max.z() = config.bodyrates_max_z;
    params.dynamic_mixing = config.dynamic_mixing;
    params.dshot_rpm_map_method = config.dshot_rpm_map_method;
    params.hover_percentage = config.hover_percentage;
  }
};

}
