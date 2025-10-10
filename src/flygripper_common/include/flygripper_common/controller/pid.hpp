#pragma once
#include <iostream>
#include <algorithm>
#include <memory>
#include <cmath>

namespace flygripper_common {

struct PidParams {
  double k_p, k_i, k_d;

  double integral_zone;
  double integral_min;
  double integral_max;

  double dead_zone;
  double command_min;
  double command_max;

  friend std::ostream& operator<<(std::ostream& os, const PidParams& p) {
    os.precision(3);
    os << std::scientific;
    os << "k_p: " << p.k_p << std::endl;
    os << "k_i: " << p.k_i << std::endl;
    os << "k_d: " << p.k_d << std::endl;

    os << "integral_zone: " << p.integral_zone << std::endl;
    os << "integral_min: " << p.integral_min << std::endl;
    os << "integral_max: " << p.integral_max << std::endl;

    os << "dead_zone: " << p.dead_zone << std::endl;
    os << "command_min: " << p.command_min << std::endl;
    os << "command_max: " << p.command_max << std::endl;
    os.precision();
    os.unsetf(std::ios::scientific);
    return os;
  }
};

class Pid {
 public:
  using Ptr = std::shared_ptr<Pid>;
  Pid(): param_init_flag_(false), error_last_flag_(false){
    reset();
  }

  explicit Pid(const PidParams& rparams): Pid(){
    setParams(rparams);
    reset();
  }

  void setParams(const PidParams& rparams){
    params_ = rparams;
    param_init_flag_ = true;
  }

  ~Pid() = default;

  void reset(){
    error_ = 0.0;
    error_dot_ = 0.0;
    error_last_ = 0.0;
    integral_ = 0.0;
    cmd_ = 0.0;
  }

  double update(double error, int64_t dt){
    if (!param_init_flag_ || dt <= 0 || std::isnan(error) || std::isinf(error)) {
      return 0.0;
    }
    if (!error_last_flag_) {
      error_last_ = error;
    }
    error_dot_ = (error - error_last_) / (dt / 1e9);

    return update(error, error_dot_, dt);

  }

  double update(double error, double error_dot, int64_t dt) {

    if (!param_init_flag_ || dt <= 0 || std::isnan(error) || std::isinf(error) || std::isnan(error_dot) ||
        std::isinf(error_dot)) {
      return 0.0;
    }
    error_ = error;
    error_dot_ = error_dot;

    // 设定闭环死区
    if ((error_ > -params_.dead_zone) && (error_ < params_.dead_zone)) {
      error_ = 0;
      error_dot_ = 0;
      integral_ = 0;
      error_last_ = 0;
    }

    // 积分项，积分分离，偏差较大时去掉积分作用
    if (error_ > -params_.integral_zone
        && error_ < params_.integral_zone) {
      integral_ += (dt / 1e9) * error_;
      // 积分范围限定，防止积分饱和*
      integral_ = std::clamp(integral_, params_.integral_min, params_.integral_max);
    }

    /*PID算法实现*/
    cmd_ = params_.k_p * error_ +
        params_.k_i * integral_ +
        params_.k_d * error_dot_;

    cmd_ = std::clamp(cmd_, params_.command_min, params_.command_max);

    /*误差传递*/
    error_last_ = error_;
    error_last_flag_ = true;

    /*返回当前实际值*/
    return cmd_;
  }

  double update(double error, double error_dot, double ff, int64_t dt) {

    if (!param_init_flag_ || dt <= 0 || std::isnan(error) || std::isinf(error) || std::isnan(error_dot) ||
        std::isinf(error_dot)) {
      return 0.0;
    }
    error_ = error;
    error_dot_ = error_dot;

    // 设定闭环死区
    if ((error_ > -params_.dead_zone) && (error_ < params_.dead_zone)) {
      error_ = 0;
      error_dot_ = 0;
      integral_ = 0;
      error_last_ = 0;
    }

    // 积分项，积分分离，偏差较大时去掉积分作用
    if (error_ > -params_.integral_zone
        && error_ < params_.integral_zone) {
      integral_ += (dt / 1e9) * error_;
      // 积分范围限定，防止积分饱和*
      integral_ = std::clamp(integral_, params_.integral_min, params_.integral_max);
    }

    /*PID算法实现*/
    cmd_ = params_.k_p * error_ +
        params_.k_i * integral_ +
        params_.k_d * error_dot_ +
        ff;

    cmd_ = std::clamp(cmd_, params_.command_min, params_.command_max);

    /*误差传递*/
    error_last_ = error_;
    error_last_flag_ = true;

    /*返回当前实际值*/
    return cmd_;
  }

 private:
  double error_;
  bool error_last_flag_;
  double error_last_;
  double error_dot_;
  double integral_;
  double cmd_;

  bool param_init_flag_;
  PidParams params_;
};

}