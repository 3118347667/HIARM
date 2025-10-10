#pragma once
#include "flygripper_common/filter/butterworth.hpp"
#include "flygripper_common/type/robot_parameter.hpp"

#include "flygripper_controller/controller_base.hpp"
#include "flygripper_controller/geometric_controller_param.hpp"
#include "flygripper_msgs/GeometricControllerData.h"
#include "geometry_msgs/WrenchStamped.h"

namespace flygripper_controller{
using namespace flygripper_common;
struct GeometricControllerData{
  //! position
  Vector<3> ref_pos;
  Vector<3> cur_pos;
  //! velocity
  Vector<3> ref_vel;
  Vector<3> cur_vel;

  //! acceleration
  Vector<3> ref_acc;
  Vector<3> cur_acc;
  Vector<3> cur_acc_f;

  Vector<3> pos_err;
  Vector<3> vel_err;
  Vector<3> des_acc;

  //! thrust
  Vector<3> des_thrust_acc;
  Scalar des_thrust;

  //! attitude
  Quaternion des_q;
  Quaternion cur_q;
  Vector<3> angle_axis_err_axis;
  Scalar angle_axis_err_angle;
  Vector<3> q_feedback;

  //! angular velocity
  Vector<3> ref_omega;
  Vector<3> des_omega;
  Vector<3> cur_omega;
  Vector<3> cur_omega_f;
  Vector<3> omega_feedback;

  //! angular acceleration

  Vector<3> cur_alpha_f;
  Vector<3> ref_alpha;
  Vector<3> des_alpha;

  //! torque
  Vector<3> des_torque;

  //! rotor velocities
  Vector<4> cur_rotor_velocities;
  Vector<4> cur_rotor_velocities_f;
  Vector<4> des_rotor_velocities;

  //! indi estimated external wrench
  Scalar cur_thrust_f;
  Vector<3> cur_torque_f;
  Vector<3> ext_force_indi = Vector<3>::Zero();
  Vector<3> ext_torque_indi = Vector<3>::Zero();

  //! l1 estimated external wrench
  Vector<3> ext_force_l1;
  Vector<3> ext_torque_l1;

  flygripper_msgs::GeometricControllerData toRosMsg();

  geometry_msgs::WrenchStamped getExternalWrenchRosMsg(std::string estimate_method);
};

class GeometricController: public ControllerBase{
 public:
  GeometricController(RobotParameter::Ptr robot_parameter,
                      Scalar control_frequency);

  ~GeometricController();

  void prepare(const RobotState& state) override;

  void getCommand(const RobotState& state,
                  const ReferenceVector& references,
                  RobotCommand& command,
                  const Scalar& now) override;

  Vector<3> getOmegaIntegrationTerm(const Vector<3>& error, bool limit=true);

  void limitTiltAngle(Ref<Vector<3>> des_thrust_acc);

  Vector<3> tiltPrioritizedAttitudeControl(const Quaternion& q_e);

  Vector<3> AttitudeControl(const Quaternion& q_e);

  Vector<3> L1AugmentationForce(ConstRef<Vector<3>> vel,
                                const Quaternion& q,
                                const Scalar & u_b,
                                bool init);

  Vector<3> L1AugmentationTorque(ConstRef<Vector<3>> omega,
                                 ConstRef<Vector<3>> u_b,
                                 bool init);

  flygripper_msgs::GeometricControllerData getControllerMsg();

  geometry_msgs::WrenchStamped getExternalWrenchMsg(std::string estimate_method);

  Vector<6> getExternalWrench(std::string estimate_method="l1");

  void updateCompensationMethod();

 private:
  static constexpr double kAlmostZeroValueThreshold = 1e-6;
  RobotParameter::Ptr robot_parameter_;
  GeometricControllerParameter::Ptr ctrl_parameter_;
  GeometricControllerParameterConfig ctrl_parameter_cfg_;
  GeometricControllerData data_;
  Scalar control_frequency_;
  Scalar last_ctrl_time_;

  ButterworthLPF<3>::Ptr acc_lpf_, omega_lpf_, ctrl_omega_lpf_;
  ButterworthLPF<4>::Ptr rotor_speed_lpf_;
  ButterworthLPF<3>::Ptr indi_alpha_lpf_, indi_acc_lpf_;

  CompensationMethod acc_compensation_{CompensationMethod::NONE};
  CompensationMethod alpha_compensation_{CompensationMethod::NONE};
};
}
