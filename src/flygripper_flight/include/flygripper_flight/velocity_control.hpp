#include <iostream>
#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/TwistStamped.h>

#include "flygripper_common/intergrator/integrator_rk4.h"
#include "flygripper_common/ros/conversions.hpp"

using namespace flygripper_common;
class VelocityControlSystem{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
 public:
  enum StateIndex: int {
    POS   = 0,
    POS_X = 0,
    POS_Y = 1,
    POS_Z = 2,
    N_POS  = 3,

    YAW   = 3,

    VEL   = 4,
    VEL_X = 4,
    VEL_Y = 5,
    VEL_Z = 6,
    N_VEL  = 3,

    YAW_RATE = 7,

    N_STATE = 8,
  };

  struct State{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    State(const Ref<const Vector<N_STATE>> init_state=Vector<N_STATE>::Zero())
        : data(init_state) {}
    State(const Ref<const Vector<N_POS>> init_pos, const Scalar init_yaw,
          const Ref<const Vector<N_VEL>> init_vel, const Scalar init_yaw_rate)
    {
      pos = init_pos;
      yaw = init_yaw;
      vel = init_vel;
      yaw_rate = init_yaw_rate;
    }

    State(const State& other){
      data = other.data;
    }

    State& operator=(const State&& other){
      data = other.data;
      return *this;
    }

    State& operator=(const State& other){
      data = other.data;
      return *this;
    }

    Vector<N_STATE> data = Vector<N_STATE>::Zero();
    Ref<Vector<N_POS>> pos = data.segment(POS, N_POS);
    Scalar& yaw = data(YAW);
    Ref<Vector<N_VEL>> vel = data.segment(VEL, N_VEL);
    Scalar& yaw_rate = data(YAW_RATE);
  };

  enum InputIndex: int {
    DES_VEL   = 0,
    DES_VEL_X = 0,
    DES_VEL_Y = 1 ,
    DES_VEL_Z = 2,
    N_DES_VEL = 3,

    DES_YAW_RATE = 3,

    N_INPUT = 4,
  };

  struct Input{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Input(const Ref<const Vector<N_INPUT>> init_input=Vector<N_INPUT>::Zero())
        : data(init_input) {}

    Input(const Ref<const Vector<N_DES_VEL>> init_des_vel, const Scalar init_des_yaw_rate)
    {
      des_vel = init_des_vel;
      des_yaw_rate = init_des_yaw_rate;
    }

    Input& operator=(const Input&& other){
      data = other.data;
      return *this;
    }

    Input& operator=(const Input& other){
      data = other.data;
      return *this;
    }

    Vector<N_INPUT> data = Vector<N_INPUT>::Zero();
    Ref<Vector<N_DES_VEL>> des_vel = data.segment(DES_VEL, N_DES_VEL);
    Scalar& des_yaw_rate = data(DES_YAW_RATE);
  };

  VelocityControlSystem()
  {
    integrator_ = std::make_shared<IntegratorRK4>(kDt);
  }

  State update(State& state, const Input& input, const Scalar duration,
               bool world_frame=true,
               Scalar tau_linear_vel=0.5,
               Scalar tau_angular_rate=0.5) {

    world_frame_ = world_frame;
    tau_linear_vel_ = tau_linear_vel;
    tau_angular_rate_ = tau_angular_rate;

    State final_state;
    bool ret = integrator_->do_step(std::bind(&VelocityControlSystem::dynamics, this,
                                              std::placeholders::_1, input.data, std::placeholders::_2),
                                    state.data,
                                    final_state.data,
                                    duration);
    if (!ret){
      ROS_WARN("Integrator failed");
    }
    return final_state;
  }

  bool dynamics(const Ref<const Vector<N_STATE>> state_data,
                const Ref<const Vector<N_INPUT>> input_data,
                Ref<Vector<N_STATE>> state_dot_data) {

    Quaternion q;
    if (world_frame_){
      q = Quaternion::Identity();
    }
    else{
      q = Eigen::AngleAxisd(state_data(YAW),Vector<3>::UnitZ());
    }
    state_dot_data.segment(POS, N_POS) =
        state_data.segment(VEL, N_VEL);
    state_dot_data(YAW) =
        state_data(YAW_RATE);
    state_dot_data.segment(VEL, N_VEL) = 1.0 / tau_linear_vel_ *
        (q * input_data.segment(DES_VEL, N_DES_VEL) - state_data.segment(VEL, N_VEL));
    state_dot_data(YAW_RATE) = 1.0 / tau_angular_rate_ *
        (input_data(DES_YAW_RATE) - state_data(YAW_RATE));
    return true;
  }

 private:
  static constexpr Scalar kDt = 0.05; // 50 ms
  Scalar tau_linear_vel_;
  Scalar tau_angular_rate_;
  IntegratorBase::Ptr integrator_;
  bool world_frame_;

};

inline geometry_msgs::TwistStamped toRosMsg(const VelocityControlSystem::Input& input){
  geometry_msgs::TwistStamped msg;
  msg.header.frame_id = kFixedFrame;
  msg.header.stamp = ros::Time::now();
  toRosVec3(input.des_vel,msg.twist.linear);
  msg.twist.angular.z = input.des_yaw_rate;

  return msg;
}

inline nav_msgs::Odometry toRosMsg(const VelocityControlSystem::State& state){
  nav_msgs::Odometry msg;
  msg.header.frame_id = kFixedFrame;
  msg.header.stamp = ros::Time::now();
  toRosVec3(state.pos,msg.pose.pose.position);
  Quaternion q(Eigen::AngleAxisd(state.yaw, Eigen::Vector3d::UnitZ()));
  toRosQuat(q, msg.pose.pose.orientation);
  toRosVec3(state.vel,msg.twist.twist.linear);
  msg.twist.twist.angular.z = state.yaw_rate;
  return msg;
}
