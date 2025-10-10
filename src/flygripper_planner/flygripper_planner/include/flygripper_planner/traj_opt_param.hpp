#pragma once

#include "flygripper_common/ros/dynamic_parameter_config.hpp"
#include "flygripper_common/ros/parameters.hpp"

#include "flygripper_planner/RosTrajOptParameterConfig.h"

namespace flygripper_planner{

using namespace flygripper_common;
struct TrajOptParameter{
  using Ptr=std::shared_ptr<TrajOptParameter>;
  // dynamic parameters
  int K;
  int M;
  int max_iteration;
  int min_piece_num;

  Scalar rhoT;
  Scalar rhoV;
  Scalar rhoA;
  Scalar rhoDYaw;
  Scalar rhoPAlignmentDistance;
  Scalar rhoPAlignmentAngle;
  Scalar rhoYawAlignmentAngle;

  bool pause_debug;
  int magic_number;
  bool fix_p;

  friend std::ostream& operator<<(std::ostream& os, const TrajOptParameter& p) {
    os.precision(3);
    os << std::scientific;
    os << "K: " << p.K << std::endl;
    os << "M: " << p.M << std::endl;

    os << "max_iteration: " << p.max_iteration << std::endl;
    os << "min_piece_num: " << p.min_piece_num << std::endl;

    os << "rhoT: " << p.rhoT << std::endl;
    os << "rhoV: " << p.rhoV << std::endl;
    os << "rhoA: " << p.rhoA << std::endl;
    os << "rhoDYaw: " << p.rhoDYaw << std::endl;

    os << "rhoPAlignmentDistance: " << p.rhoPAlignmentDistance << std::endl;
    os << "rhoPAlignmentAngle: " << p.rhoPAlignmentAngle << std::endl;
    os << "rhoYawAlignmentAngle: " << p.rhoYawAlignmentAngle << std::endl;

    os << "pause_debug: " << p.pause_debug << std::endl;
    os << "fixed_p: " << p.fix_p << std::endl;
    os << "magic_number: " << p.magic_number << std::endl;
    os.precision();
    os.unsetf(std::ios::scientific);
    return os;
  }
};

class TrajOptParameterConfig:
    public DynamicParameterConfig<TrajOptParameter, RosTrajOptParameterConfig> {
 public:
  TrajOptParameterConfig() = default;
  virtual ~TrajOptParameterConfig() = default;

  virtual bool initParams(ros::NodeHandle& pnh, TrajOptParameter& params) override{
    // config from ROS parameters [static is marked]
    if (!getParam("K", params.K, pnh)) return false; //dynamic

    if (!getParam("max_iteration", params.max_iteration, pnh)) return false; //dynamic

    if (!getParam("rhoT", params.rhoT, pnh)) return false; //dynamic
    if (!getParam("rhoV", params.rhoV, pnh)) return false; //dynamic
    if (!getParam("rhoA", params.rhoA, pnh)) return false; //dynamic
    if (!getParam("rhoDYaw", params.rhoDYaw, pnh)) return false; //dynamic

    if (!getParam("pause_debug", params.pause_debug, pnh)) return false; //dynamic
    if (!getParam("fix_p", params.fix_p, pnh)) return false; //dynamic

    // NOT USED FOR REAL
    if (!getParam("M", params.M, pnh)) return false; //dynamic
    if (!getParam("min_piece_num", params.min_piece_num, pnh)) return false; //dynamic

    if (!getParam("rhoPAlignmentDistance", params.rhoPAlignmentDistance, pnh)) return false; //dynamic
    if (!getParam("rhoPAlignmentAngle", params.rhoPAlignmentAngle, pnh)) return false; //dynamic
    if (!getParam("rhoYawAlignmentAngle", params.rhoYawAlignmentAngle, pnh)) return false; //dynamic
    if (!getParam("magic_number", params.magic_number, pnh)) return false; //dynamic
    return true;
  }

  void dynamicParams2Config(const TrajOptParameter& params, RosTrajOptParameterConfig& config) override{
    // convert params to dynamic reconfig
    config.K = params.K;
    config.M = params.M;

    config.max_iteration = params.max_iteration;
    config.min_piece_num = params.min_piece_num;

    config.rhoT = params.rhoT;
    config.rhoV = params.rhoV;
    config.rhoA = params.rhoA;
    config.rhoDYaw = params.rhoDYaw;

    config.rhoPAlignmentDistance = params.rhoPAlignmentDistance;
    config.rhoPAlignmentAngle = params.rhoPAlignmentAngle;
    config.rhoYawAlignmentAngle = params.rhoYawAlignmentAngle;

    config.pause_debug = params.pause_debug;
    config.fix_p = params.fix_p;
    config.magic_number = params.magic_number;
  }

  void config2DynamicParams(const RosTrajOptParameterConfig& config, TrajOptParameter& params) override{
    // convert dynamic reconfig to params
    params.K = config.K;
    params.M = config.M;

    params.max_iteration = config.max_iteration;
    params.min_piece_num = config.min_piece_num;

    params.rhoT = config.rhoT;
    params.rhoV = config.rhoV;
    params.rhoA = config.rhoA;
    params.rhoDYaw = config.rhoDYaw;

    params.rhoPAlignmentDistance = config.rhoPAlignmentDistance;
    params.rhoPAlignmentAngle = config.rhoPAlignmentAngle;
    params.rhoYawAlignmentAngle = config.rhoYawAlignmentAngle;

    params.pause_debug = config.pause_debug;
    params.fix_p = config.fix_p;
    params.magic_number = config.magic_number;
  }
};
}

