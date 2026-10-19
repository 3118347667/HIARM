#pragma once
#include "flygripper_common/type/base.hpp"
#include "flygripper_common/type/const.hpp"
#include "flygripper_common/type/robot_command.hpp"
#include "flygripper_common/type/robot_parameter.hpp"
#include "flygripper_common/type/robot_state.hpp"

namespace flygripper_common {
class Flatness {
public:
  Flatness(RobotParameter::Ptr robot_parameter)
      : robot_parameter_(robot_parameter) {}
  ~Flatness() = default;

  void
  feed(ConstRef<Matrix<3, 5>> output, RobotState &state, RobotCommand &input,
       ConstRef<Vector<3>> heading = Vector<3>::Zero() // Defined in WORLD frame
                                                       // actuator_reference
  ) {
    // TODO: add yaw and morphing reference
    // Unpack flatness output
    auto &W_p = output.col(0);
    auto &W_v = output.col(1);
    auto &W_a = output.col(2);
    auto &W_j = output.col(3);
    auto &W_s = output.col(4);

    // Compute thrust: W_a = W_thrust + W_g
    // To be noted, W_thrust is normalized by mass
    const Vector<3> W_thrust = W_a - kGravityVector;
    const Scalar thrust = W_thrust.norm();

    // Tilt rotation (fixed axis/ extrinsic rotation)
    // W_q_Tilt = z_W (e_z) -> W_thrust
    Quaternion W_q_Tilt;
    if (thrust > 1e-3) {
      W_q_Tilt =
          Quaternion::FromTwoVectors(Vector<3>::UnitZ(), W_thrust.normalized());
    } else {
      W_q_Tilt = last_W_q_Tilt_;
    }
    last_W_q_Tilt_ = W_q_Tilt;

    // Yaw rotation euler angle
    const Quaternion Tilt_q_B =
        Quaternion(Eigen::AngleAxis<Scalar>(heading(0), Vector<3>::UnitZ()));

    // Compute W_q_B
    const Quaternion W_q_B = W_q_Tilt * Tilt_q_B;

    // Compute omega_B
    const Vector<3> B_x = W_q_B * Vector<3>::UnitX();
    const Vector<3> B_y = W_q_B * Vector<3>::UnitY();
    const Vector<3> B_z = W_q_B * Vector<3>::UnitZ();
    const Scalar thrust_dot = W_j.dot(B_z);
    const Vector<3> H_omega = 1.0 / thrust * W_j;
    // 2nd term is simplified:
    // - thrust_dot / thrust * B_z;
    const Vector<3> B_omega(-H_omega.dot(B_y), H_omega.dot(B_x),
                            (heading(1) * Vector<3>::UnitZ()).dot(B_z));
    // Compute alpha_B
    const Vector<3> W_omega = W_q_B * B_omega;
    const Vector<3> B_z_dot = W_omega.cross(B_z);
    const Vector<3> H_alpha_1 = 1.0 / thrust * W_s;
    const Vector<3> H_alpha_2 = -W_omega.cross(B_z_dot);
    const Vector<3> H_alpha_3 = -2 * thrust_dot / thrust * B_z_dot;
    // 4th term is simplified:
    // - thrust_ddot / thrust * B_z;
    // where thrust_ddot = W_s.dot(B_z) + B_z_dot.dot(W_j);
    const Vector<3> H_alpha = H_alpha_1 + H_alpha_2 + H_alpha_3;
    const Vector<3> B_alpha(-H_alpha.dot(B_y), H_alpha.dot(B_x),
                            (heading(2) * Vector<3>::UnitZ()).dot(B_z));

    state.pos = W_p;
    state.vel = W_v;
    state.acc = W_a;
    state.jerk = W_j;
    state.snap = W_s;
    state.setQ(W_q_B);
    state.omega = B_omega;
    state.alpha = B_alpha;

    input.collective_thrust = thrust;
    input.attitude = W_q_B;
    input.bodyrates = B_omega;
    input.torque = robot_parameter_->J * B_alpha +
                   B_omega.cross(robot_parameter_->J * B_omega);
    const Vector<4> thrust_and_torque(thrust, input.torque.x(),
                                      input.torque.y(), input.torque.z());
    const Matrix<4, 4> G =
        robot_parameter_->getControlAllocation(state.rot_pos);
    input.rotor_thrusts = G.inverse() * thrust_and_torque;
    input.rotor_velocities =
        (input.rotor_thrusts / robot_parameter_->thrust_coeff).cwiseSqrt();
  }

private:
  RobotParameter::Ptr robot_parameter_;
  Quaternion last_W_q_Tilt_{Quaternion::Identity()};
};

} // namespace flygripper_common
