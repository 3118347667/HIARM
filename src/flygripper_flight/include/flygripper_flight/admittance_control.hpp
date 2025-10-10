#include <iostream>
#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/WrenchStamped.h>

#include "flygripper_common/intergrator/integrator_rk4.h"
#include "flygripper_common/ros/conversions.hpp"
#include "flygripper_common/type/const.hpp"

namespace flygripper_flight
{
using namespace flygripper_common;
class AdmittanceControlSystem
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
 public:
  enum StateIndex : int {
    POS = 0,
    POS_X = 0,
    POS_Y = 1,
    POS_Z = 2,
    N_POS = 3,
    POS_YAW = 3,

    VEL = 4,
    VEL_X = 4,
    VEL_Y = 5,
    VEL_Z = 6,
    N_VEL = 3,
    VEL_YAW = 7,
    N_STATE = 8,

//    ACC = 8,
//    ACC_X = 8,
//    ACC_Y = 9,
//    ACC_Z = 10,
//    N_ACC = 3,
//    ACC_YAW = 11,
//
//    N_STATE = 12,

  };

  struct State {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    State(const Ref<const Vector<N_STATE>> init_state = Vector<N_STATE>::Zero())
        : data(init_state)
    {
    }
    State(const Ref<const Vector<N_POS>> init_pos, const Scalar& init_yaw,
          const Ref<const Vector<N_VEL>> init_vel, const Scalar& init_yaw_rate)
//          const Ref<const Vector<N_ACC>> init_acc, const Scalar& init_yaw_acc)
    {
      data << init_pos, init_yaw, init_vel, init_yaw_rate;
//      data << init_pos, init_yaw, init_vel, init_yaw_rate, init_acc, init_yaw_acc;
    }

    State(const State& other) { data = other.data; }

    State& operator=(const State&& other)
    {
      data = other.data;
      return *this;
    }

    State& operator=(const State& other)
    {
      data = other.data;
      return *this;
    }

    Vector<N_STATE> data = Vector<N_STATE>::Zero();
    Ref<Vector<N_POS>> pos = data.segment(POS, N_POS);
    Ref<Vector<N_VEL>> vel = data.segment(VEL, N_VEL);
//    Ref<Vector<N_ACC>> acc = data.segment(ACC, N_ACC);
    Scalar& yaw = data(POS_YAW);
    Scalar& yaw_rate = data(VEL_YAW);
//    Scalar& yaw_acc = data(ACC_YAW);
  };

  enum InputIndex : int {
    EXT_FORCE = 0,
    EXT_FORCE_X = 0,
    EXT_FORCE_Y = 1,
    EXT_FORCE_Z = 2,
    N_EXT_FORCE = 3,

    EXT_TORQUE_YAW = 3,
    N_INPUT = 4,
  };

  struct Input {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Input(const Ref<const Vector<N_INPUT>> init_input = Vector<N_INPUT>::Zero())
        : data(init_input)
    {
    }

    Input(const Ref<const Vector<N_EXT_FORCE>> init_ext_force,
          const Scalar init_ext_torque_yaw)
    {
      ext_force = init_ext_force;
      ext_torque_yaw = init_ext_torque_yaw;
    }

    Input& operator=(const Input&& other)
    {
      data = other.data;
      return *this;
    }

    Input& operator=(const Input& other)
    {
      data = other.data;
      return *this;
    }

    Vector<N_INPUT> data = Vector<N_INPUT>::Zero();
    Ref<Vector<N_EXT_FORCE>> ext_force = data.segment(EXT_FORCE, N_EXT_FORCE);
    Scalar& ext_torque_yaw = data(EXT_TORQUE_YAW);
  };

  AdmittanceControlSystem()
  {
    integrator_ = std::make_shared<IntegratorRK4>(kDt);
  }

  void updateParameter(const Ref<const Vector<4>> coeff_M = Vector<4>::Constant(0.5),
                       const Ref<const Vector<4>> coeff_D = Vector<4>::Constant(0.5),
                       const Ref<const Vector<4>> coeff_K = Vector<4>::Constant(0.5),
                       const Ref<const Vector<3>> desired_pos = Vector<3>::Zero(),
                       const Scalar& desired_yaw = 0.0,
                       const Ref<const Vector<3>> desired_vel = Vector<3>::Zero(),
                       const Scalar& desired_yaw_rate = 0.0,
                       const Ref<const Vector<3>> desired_acc = Vector<3>::Zero(),
                       const Scalar& desired_yaw_acc = 0.0,
                       const Ref<const Vector<4>> max_general_vel = Vector<4>::Constant(MAX),
                       const Ref<const Vector<4>> max_general_acc = Vector<4>::Constant(MAX)){
    coeff_M_ = coeff_M;
    coeff_D_ = coeff_D;
    coeff_K_ = coeff_K;
    desired_general_pos_ << desired_pos, desired_yaw;
    desired_general_vel_ << desired_vel, desired_yaw_rate;
    desired_general_acc_ << desired_acc, desired_yaw_acc;
    max_general_vel_ << max_general_vel;
    max_general_acc_ << max_general_acc;
  }

  bool valid(){
    return coeff_M_.allFinite() &&
           coeff_D_.allFinite() &&
           coeff_K_.allFinite() &&
           desired_general_pos_.allFinite() &&
           desired_general_vel_.allFinite() &&
           desired_general_acc_.allFinite() &&
           max_general_vel_.allFinite() &&
           max_general_acc_.allFinite();
  }

  State update(const State& state, const Input& input, const Scalar duration)
  {
    State update_state = state;
    if (!valid()){
      ROS_ERROR("Invalid parameters for admittance control system, "
          "please initialize the parameters first before update the system");
      return update_state;
    }
    bool ret = integrator_->do_step(
        std::bind(&AdmittanceControlSystem::dynamics, this,
                  std::placeholders::_1, input.data, std::placeholders::_2),
        state.data, update_state.data, duration);
    if (!ret) {
      ROS_WARN("Integrator failed");
    }
    //! constrain velocity explictly after integration
    update_state.data.segment(VEL, N_VEL + 1) =
        update_state.data.segment(VEL, N_VEL + 1).cwiseMin(max_general_vel_).cwiseMax(-max_general_vel_);
    return update_state;
  }

  bool dynamics(const Ref<const Vector<N_STATE>> state_data,
                const Ref<const Vector<N_INPUT>> input_data,
                Ref<Vector<N_STATE>> state_dot_data)
  {
    // without limitation
//    state_dot_data.segment(POS, N_POS + 1) = state_data.segment(VEL, N_VEL + 1);
//
//    state_dot_data.segment(VEL, N_VEL + 1) = state_data.segment(ACC, N_ACC + 1);
//
//    state_dot_data.segment(ACC, N_ACC + 1) =
//        desired_general_acc_ +
//            (input_data +
//                coeff_D_.cwiseProduct(desired_general_vel_ -
//                    state_data.segment(VEL, N_VEL + 1)) +
//                coeff_K_.cwiseProduct(desired_general_pos_ -
//                    state_data.segment(POS, N_POS + 1)))
//                .cwiseProduct(coeff_M_.cwiseInverse());

    // with limitation
    state_dot_data.segment(POS, N_POS + 1) = state_data.segment(VEL, N_VEL + 1)
        .cwiseMin(max_general_vel_).cwiseMax(-max_general_vel_);
    state_dot_data.segment(VEL, N_VEL + 1) = (desired_general_acc_ +
        (input_data +
            coeff_D_.cwiseProduct(desired_general_vel_ -
                state_data.segment(VEL, N_VEL + 1)) +
            coeff_K_.cwiseProduct(desired_general_pos_ -
                state_data.segment(POS, N_POS + 1)))
            .cwiseProduct(coeff_M_.cwiseInverse()))
        .cwiseMin(max_general_acc_).cwiseMax(-max_general_acc_);
    ROS_INFO_STREAM_THROTTLE(1.0, "dot_v: "<< state_dot_data.segment(VEL, N_VEL + 1).transpose()
                                           << "\ndot_a: "<< state_dot_data.segment(POS, N_POS + 1).transpose());
    return true;
  }

  Vector<4> getAcc(const State& state,
                   const Input& input){
    //! constrain acc explictly after integration
    if (valid()){
      return Vector<4>(desired_general_acc_ +
                       (input.data +
                        coeff_D_.cwiseProduct(desired_general_vel_ -
                                              state.data.segment(VEL, N_VEL + 1)) +
                        coeff_K_.cwiseProduct(desired_general_pos_ -
                                              state.data.segment(POS, N_POS + 1)))
                           .cwiseProduct(coeff_M_.cwiseInverse()))
          .cwiseMin(max_general_acc_).cwiseMax(-max_general_acc_);
    } else{
      ROS_ERROR("Invalid parameters for admittance control system, "
          "please initialize the parameters first before get the acc");
      return Vector<4>::Zero();
    }
  }

 private:
  static constexpr Scalar kDt = 0.001;  // 1 ms
  Vector<4> coeff_M_{Vector<4>::Constant(INF)};
  Vector<4> coeff_D_{Vector<4>::Constant(INF)};
  Vector<4> coeff_K_{Vector<4>::Constant(INF)};

  Vector<4> desired_general_pos_{Vector<4>::Constant(INF)};
  Vector<4> desired_general_vel_{Vector<4>::Constant(INF)};
  Vector<4> desired_general_acc_{Vector<4>::Constant(INF)};

  Vector<4> max_general_vel_{Vector<4>::Constant(INF)};
  Vector<4> max_general_acc_{Vector<4>::Constant(INF)};

  IntegratorBase::Ptr integrator_;
};

inline void toRosMsg(const AdmittanceControlSystem::Input& external_wrench,
                     geometry_msgs::WrenchStamped& msg){

  msg.header.stamp = ros::Time::now();
  msg.header.frame_id = kFixedFrame;

  toRosVec3(external_wrench.ext_force, msg.wrench.force);
  toRosVec3((Vector<3>() << 0,0,external_wrench.ext_torque_yaw).finished(), msg.wrench.torque);
}

}