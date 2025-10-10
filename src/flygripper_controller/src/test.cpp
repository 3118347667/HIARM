#include "flygripper_common/type/base.hpp"
#include "flygripper_common/utils/math.hpp"
#include "ros/ros.h"
using namespace flygripper_common;

Vector<3> tiltPrioritizedControl(const Quaternion& q_e,
                                 ConstRef<Vector<3>> k_att)
{
  // 2018 Tilt-prioritized quadrocopter attitude control
  Matrix<3, 3> k_att_mat = Vector<3>(k_att.x(), k_att.y(), k_att.z()).asDiagonal();

  Vector<3> q_err = 1.0 / std::sqrt(q_e.w() * q_e.w() + q_e.z() * q_e.z()) *
      Vector<3>(q_e.w() * q_e.x() - q_e.y() * q_e.z(),
                q_e.w() * q_e.y() + q_e.x() * q_e.z(),
                q_e.z());
  ROS_INFO_STREAM("rotor q_err: " << q_err.transpose() << "\n");
  if (q_e.w() <= 0)
    q_err(2) *= -1.0;
  const Vector<3> q_feedback = 2.0 * k_att_mat * q_err;

  return q_feedback;
}

Vector<3> axisAngleControl(const Quaternion& q_e,
                           ConstRef<Vector<3>> k_att){
  // Compute desired body rates from control error

  Matrix<3, 3> k_att_mat = Vector<3>(k_att.x(), k_att.y(), k_att.z()).asDiagonal();

  Vector<3> q_err = Vector<3>(q_e.x(), q_e.y(), q_e.z());
  ROS_INFO_STREAM("wang q_err: " << q_err.transpose() << "\n");
  if (q_e.w() <= 0)
    q_err *= -1.0;

  const Vector<3> q_feedback = 2.0 * k_att_mat * q_err;
  return q_feedback;
}

bool flatnessTransport(const Eigen::Vector3d &acc,
                       const Eigen::Vector3d &jer,
                       const Eigen::Vector3d &snp,
                       const Eigen::Vector3d &indi,
                       const Eigen::Vector3d &q,
                       const Eigen::Vector3d &dq,
                       const Eigen::Vector3d &d2q,
                       const double &F,
                       const double &dF,
                       const double &d2F,
                       const double &psi,
                       const double &dpsi,
                       const double &ddpsi,
                       double &thr,
                       Eigen::Quaterniond &quat,
                       Eigen::Vector3d &omg,
                       Eigen::Vector3d &domg,
                       const double &massQ,
                       const double &grav)
{
  const double almost_zero = 1.0e-6;

  double dFq0, dFq1, dFq2, dzu_norm, zu_sqr_norm, zu_norm;
  double d2Fq0, d2Fq1, d2Fq2;
  double f_term0, f_term1, f_term2;
  double c_half_psi, s_half_psi, c_psi, s_psi;
  double dng00, dng01, dng02, dng11, dng12, dng22, ng_den, ng_den4;
  double dz_term0, dz_term1, dz_term2, d2z_term0, d2z_term1, d2z_term2;
  double z0, z1, z2, dz0, dz1, dz2, d2z0, d2z1, d2z2, zu0, zu1, zu2, zu_sqr0, zu_sqr1, zu_sqr2, zu01, zu12, zu02;
  double ng00, ng01, ng02, ng11, ng12, ng22;
  double omg_den, omg_term;
  double tilt0, tilt1, tilt2, tilt_den;
  double massFrac = 1.0 / massQ;

  zu0 = acc(0) - indi(0);
  zu1 = acc(1) - indi(1);
  zu2 = acc(2) - indi(2) + grav;

  zu_sqr0 = zu0 * zu0;
  zu_sqr1 = zu1 * zu1;
  zu_sqr2 = zu2 * zu2;
  zu01 = zu0 * zu1;
  zu12 = zu1 * zu2;
  zu02 = zu0 * zu2;
  zu_sqr_norm = zu_sqr0 + zu_sqr1 + zu_sqr2;
  zu_norm = sqrt(zu_sqr_norm);

  if (zu_norm < almost_zero)
  {
    return false;
  }

  z0 = zu0 / zu_norm;
  z1 = zu1 / zu_norm;
  z2 = zu2 / zu_norm;
  ng_den = zu_sqr_norm * zu_norm;
  ng00 = (zu_sqr1 + zu_sqr2) / ng_den;
  ng01 = -zu01 / ng_den;
  ng02 = -zu02 / ng_den;
  ng11 = (zu_sqr0 + zu_sqr2) / ng_den;
  ng12 = -zu12 / ng_den;
  ng22 = (zu_sqr0 + zu_sqr1) / ng_den;
  dFq0 = massFrac * (F * dq(0) + dF * q(0));
  dFq1 = massFrac * (F * dq(1) + dF * q(1));
  dFq2 = massFrac * (F * dq(2) + dF * q(2));
  dz_term0 = jer(0) + dFq0;
  dz_term1 = jer(1) + dFq1;
  dz_term2 = jer(2) + dFq2;
  dz0 = ng00 * dz_term0 + ng01 * dz_term1 + ng02 * dz_term2;
  dz1 = ng01 * dz_term0 + ng11 * dz_term1 + ng12 * dz_term2;
  dz2 = ng02 * dz_term0 + ng12 * dz_term1 + ng22 * dz_term2;

  f_term0 = massQ * (acc.x() - indi.x());
  f_term1 = massQ * (acc.y() - indi.y());
  f_term2 = massQ * ((acc.z() + grav) - indi.z());
  thr = z0 * f_term0 + z1 * f_term1 + z2 * f_term2;

  // for domg
  ng_den4 = zu_sqr_norm * zu_sqr_norm;
  dzu_norm = (zu0 * dz_term0 + zu1 * dz_term1 + zu2 * dz_term2) / zu_norm;
  dng00 = (2 * zu_norm * (zu1 * dz_term1 + zu2 * dz_term2) - 3 * dzu_norm * (zu_sqr1 + zu_sqr2)) / ng_den4;
  dng01 = -((dz_term0 * zu1 + dz_term1 * zu0) * zu_norm - 3 * dzu_norm * zu0 * zu1) / ng_den4;
  dng02 = -((dz_term0 * zu2 + dz_term2 * zu0) * zu_norm - 3 * dzu_norm * zu0 * zu2) / ng_den4;
  dng11 = (2 * zu_norm * (zu0 * dz_term0 + zu2 * dz_term2) - 3 * dzu_norm * (zu_sqr0 + zu_sqr2)) / ng_den4;
  dng12 = -((dz_term1 * zu2 + dz_term2 * zu1) * zu_norm - 3 * dzu_norm * zu1 * zu2) / ng_den4;
  dng22 = (2 * zu_norm * (zu0 * dz_term0 + zu1 * dz_term1) - 3 * dzu_norm * (zu_sqr0 + zu_sqr1)) / ng_den4;

  d2Fq0 = massFrac * (F * d2q(0) + 2 * dF * dq(0) + d2F * q(0));
  d2Fq1 = massFrac * (F * d2q(1) + 2 * dF * dq(1) + d2F * q(1));
  d2Fq2 = massFrac * (F * d2q(2) + 2 * dF * dq(2) + d2F * q(2));

  d2z_term0 = snp(0) + d2Fq0;
  d2z_term1 = snp(1) + d2Fq1;
  d2z_term2 = snp(2) + d2Fq2;
  d2z0 = dng00 * dz_term0 + dng01 * dz_term1 + dng02 * dz_term2 + ng00 * d2z_term0 + ng01 * d2z_term1 + ng02 * d2z_term2;
  d2z1 = dng01 * dz_term0 + dng11 * dz_term1 + dng12 * dz_term2 + ng01 * d2z_term0 + ng11 * d2z_term1 + ng12 * d2z_term2;
  d2z2 = dng02 * dz_term0 + dng12 * dz_term1 + dng22 * dz_term2 + ng02 * d2z_term0 + ng12 * d2z_term1 + ng22 * d2z_term2;

  if (1.0 + z2 < almost_zero)
  {
    return false;
  }

  tilt_den = sqrt(2.0 * (1.0 + z2));
  tilt0 = 0.5 * tilt_den;
  tilt1 = -z1 / tilt_den;
  tilt2 = z0 / tilt_den;
  c_half_psi = cos(0.5 * psi);
  s_half_psi = sin(0.5 * psi);
  quat.w() = tilt0 * c_half_psi;
  quat.x() = tilt1 * c_half_psi + tilt2 * s_half_psi;
  quat.y() = tilt2 * c_half_psi - tilt1 * s_half_psi;
  quat.z() = tilt0 * s_half_psi;
  c_psi = cos(psi);
  s_psi = sin(psi);
  omg_den = z2 + 1.0;
  omg_term = dz2 / omg_den;
  omg(0) = dz0 * s_psi - dz1 * c_psi -
      (z0 * s_psi - z1 * c_psi) * omg_term;
  omg(1) = dz0 * c_psi + dz1 * s_psi -
      (z0 * c_psi + z1 * s_psi) * omg_term;
  omg(2) = (z1 * dz0 - z0 * dz1) / omg_den + dpsi;
  domg(0) = s_psi * d2z0 - c_psi * d2z1 - (s_psi * z0 - c_psi * z1) * d2z2 / omg_den + c_psi * dz0 * dpsi + s_psi * dz1 * dpsi - dz2 * (c_psi * z0 * dpsi + s_psi * z1 * dpsi + s_psi * dz0 - c_psi * dz1) / omg_den + dz2 * dz2 * (s_psi * z0 - c_psi * z1) / pow(omg_den, 2);
  domg(1) = c_psi * d2z0 + s_psi * d2z1 - (c_psi * z0 + s_psi * z1) * d2z2 / omg_den + c_psi * dz1 * dpsi - s_psi * dz0 * dpsi - dz2 * (-s_psi * z0 * dpsi + c_psi * z1 * dpsi + c_psi * dz0 + s_psi * dz1) / omg_den + dz2 * dz2 * (c_psi * z0 + s_psi * z1) / pow(omg_den, 2);
  domg(2) = (z1 * d2z0 - z0 * d2z1 - dz1 * dz0 + dz0 * dz1) / omg_den - dz2 * (-dz1 * z0 + dz0 * z1) / pow(omg_den, 2) + ddpsi;

  return true;
}

void minimumSingularityFlat(const double mass,
                            const double grav,
                            const Eigen::Vector3d &vel,
                            const Eigen::Vector3d &acc,
                            const Eigen::Vector3d &jer,
                            const Eigen::Vector3d &snp,
                            const double &yaw,
                            const double &yawd,
                            const double &yawd2,
                            const Eigen::Quaterniond &att_est,
                            Eigen::Quaterniond &att,
                            Eigen::Vector3d &omg,
                            Eigen::Vector3d &domg,
                            double &thrust)
{

  static Eigen::Vector3d omg_old(0.0, 0.0, 0.0);
  static Eigen::Vector3d domg_old(0.0, 0.0, 0.0);
  static double thrust_old = mass * (acc + grav * Eigen::Vector3d::UnitZ()).norm();

  Eigen::Quaterniond quat;
  if (flatnessTransport(acc, jer, snp, Eigen::Vector3d::Zero(),
                        Eigen::Vector3d::Zero(),
                        Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), 0.0, 0.0, 0.0, yaw, yawd, yawd2,
                        thrust, quat, omg, domg, mass, grav))
  {
    att = quat;
    omg_old = omg;
    domg_old = domg;
    thrust_old = thrust;
  }
  else
  {
    ROS_WARN("Conor case: 1. Eactly inverted flight or 2. Unactuated falling");
    att = att_est;
    omg = omg_old;
    domg = domg_old;
    thrust = thrust_old;
  }

  return;
}

void test(){

  Vector<3> ref_pos = Vector<3>(1.0, 1.0, 0.1);
  Vector<3> cur_pos = Vector<3>(0.8, 0.9, 0.0);

  Vector<3> ref_vel = Vector<3>(0.0, 0.0, 0.0);
  Vector<3> cur_vel = Vector<3>(0.1, 0.12, 0.1);

  Vector<3> ref_acc = Vector<3>(0.0, 0.0, 0.0);
  Vector<3> ref_jerk = Vector<3>(0.03, 0.03, 0.01);
  Vector<3> ref_snap = Vector<3>(0.01, 0.01, 0.01);

  Scalar ref_yaw_angle = 0.3; // TODO: should not be updated as state every time
  Scalar ref_yaw_rate = 0.0;
  Scalar ref_yaw_acc = 0.0;

  Quaternion cur_q = Quaternion::Identity();
  Vector<3> cur_omega = Vector<3>::Zero();

  /** parameters**/
  Vector<3> k_pos = Vector<3>(3.5,3.5,3.5);
  Vector<3> k_vel = Vector<3>(4.0,4.0,4.0);
  Vector<3> k_att_1 = Vector<3>(10.0, 10.0, 8.0);
  Vector<3> k_att_2 = Vector<3>(10.0, 10.0, 8.0);
  Vector<3> k_omega = Vector<3>(12.0, 12.0, 8.0);
  Scalar kAlmostZeroValueThreshold = 1e-6;
  Scalar gravity = 9.794;
  const Vector<3> kGravityVector = Vector<3>(0, 0, -gravity);
  Scalar mass = 0.559;
  /** outer loop**/
  Vector<3> pos_err = clipVector(ref_pos - cur_pos, Vector<3>(1.0,1.0,1.0));
  Vector<3> des_vel = k_pos.cwiseProduct(pos_err) + ref_vel;
  Vector<3> vel_err = clipVector(des_vel - cur_vel, Vector<3>(6.0,6.0,6.0));
  Vector<3> des_acc = k_vel.cwiseProduct(vel_err) + ref_acc;

  /** flatness **/
  Quaternion des_q;
  Vector<3> ref_omega;
  Vector<3>  ref_alpha;

  Scalar des_thrust_val;
  Scalar des_thrust_acc_val;
  Vector<3> des_thrust;
  Vector<3> des_thrust_acc;

  {
    minimumSingularityFlat(mass, gravity,
                           ref_vel, des_acc, ref_jerk, ref_snap,
                           ref_yaw_angle, ref_yaw_rate, ref_yaw_acc, cur_q,
                           des_q, ref_omega, ref_alpha, des_thrust_val);
    des_thrust = des_q * (des_thrust_val * Eigen::Vector3d::UnitZ());
    des_thrust_acc = des_thrust / mass;
    ROS_INFO_STREAM("************ wang *********"
                        << "\ndes_thrust_acc:\n"<< des_thrust_acc
                        << "\n des_thrust_val:\n" << des_thrust_val
                        << "\n des_q:\n" << des_q.coeffs());
    ROS_INFO_STREAM("ref_omega:\n"<<ref_omega);
    ROS_INFO_STREAM("ref_alpha:\n"<<ref_alpha);
  }
//  else
  {
    des_thrust_acc = des_acc - kGravityVector;
    des_thrust_acc_val = des_thrust_acc.norm();

    Vector<3> des_body_z_axis = des_thrust_acc.normalized();
    Quaternion des_q_tilt = Quaternion::FromTwoVectors(Vector<3>::UnitZ(), des_body_z_axis);
    Quaternion des_q_yaw(Eigen::AngleAxis<Scalar>(ref_yaw_angle, Vector<3>::UnitZ()));
    des_q = des_q_tilt * des_q_yaw;

    ROS_INFO_STREAM("************ rotor *********"
                        << "\ndes_thrust_acc:\n"<< des_thrust_acc
                        << "\n des_thrust_val:\n" << des_thrust_val
                        << "\n des_q:\n" << des_q.coeffs());

    // Feedback linearization vs feedforward linearization,
    Quaternion q = des_q;;
    Vector<3> omega = ref_omega;
    Scalar norm_thrust = des_thrust_acc_val;
    // NOTE: B_x, B_y, B_z and norm_thrust are computed
    //       using the current state instead of reference state
    //       (feedback linearization vs feedforward linearization)
    const Vector<3> B_x = q * Vector<3>::UnitX();
    const Vector<3> B_y = q * Vector<3>::UnitY();
    const Vector<3> B_z = q * Vector<3>::UnitZ();
    const Scalar thrust_dot = ref_jerk.dot(B_z);
    Vector<3> H_omega = ref_jerk;

    // Avoid singularity when thrust is zero
    // using sensor measured thrust acceleration value (feedback linearization)
    if ( norm_thrust > kAlmostZeroValueThreshold){
      H_omega *= 1.0/norm_thrust;
    }

    Vector<3> ref_omega;
    ref_omega = Vector<3>(- H_omega.dot(B_y),
                          H_omega.dot(B_x),
                          (ref_yaw_rate * Vector<3>::UnitZ()).dot(B_z));
    ROS_INFO_STREAM("ref_omega:\n"<<ref_omega);
    const Vector<3> W_omega = q * omega; //TODO: feedback
    const Vector<3> B_z_dot = W_omega.cross(B_z);
    Vector<3> H_alpha_1 =  ref_snap; // 1.0 / norm_thrust_f * ref.state.snap;
    Vector<3> H_alpha_2 = - W_omega.cross(B_z_dot);
    Vector<3> H_alpha_3 =  - 2 * thrust_dot * B_z_dot;// - 2 * thrust_dot / norm_thrust_f * B_z_dot;
    // Avoid singularity when thrust is zero
    // using sensor measured thrust acceleration value (feedback linearization)
    if (norm_thrust > kAlmostZeroValueThreshold){
      H_alpha_1 *= 1.0/norm_thrust;
      H_alpha_3 *= 1.0/norm_thrust;
    }
    const Vector<3> H_alpha = H_alpha_1 + H_alpha_2 + H_alpha_3;
    ref_alpha = Vector<3>(- H_alpha.dot(B_y),
                          H_alpha.dot(B_x),
                          (ref_yaw_acc * Vector<3>::UnitZ()).dot(B_z));

    ROS_INFO_STREAM("ref_alpha:\n"<<ref_alpha);


  }


  //! Calculate thrust acceleration value
  //! use its projection on current thrust orientation:
  ROS_INFO_STREAM("cur_q:\n"<<cur_q.coeffs());
  Eigen::Vector3d cur_body_z_axis = cur_q * Eigen::Vector3d::UnitZ();
  des_thrust_acc_val = des_thrust_acc.dot(cur_body_z_axis);
  ROS_INFO_STREAM("************ wang *********"
                      << "des_thrust_acc_val:\n" << des_thrust_acc_val);
  //! use its norm directly:
  des_thrust_acc_val = des_thrust_acc.norm();
  ROS_INFO_STREAM("************ rotor *********"
                      << "des_thrust_acc_val:\n" << des_thrust_acc_val);

  des_thrust_val = des_thrust_acc_val * mass;

  //! Calculate q_feedback value
  Quaternion q_e = cur_q.inverse()*des_q;
  Eigen::AngleAxisd rotation_vector(q_e); // debug
  Vector<3> axis = rotation_vector.axis();
  ROS_INFO_STREAM("q_e: \n" << q_e.coeffs());
  ROS_INFO("err_axis_angle_x: %f,  "
           "err_axis_angle_y: %f, "
           "err_axis_angle_z: %f, "
           "err_axis_angle_ang: %f",
           axis(0), axis(1), axis(2),
           rotation_vector.angle() * 180 / M_PI);

  Vector<3> q_feedback;
  //! use tilt prioritized control:
  q_feedback = axisAngleControl(q_e, k_att_1);
  ROS_INFO_STREAM("************ wang *********"
                      << "q_feedback:\n" << q_feedback);
  //! use axis angle control:
  q_feedback = tiltPrioritizedControl(q_e, k_att_2);
  ROS_INFO_STREAM("************ rotor *********"
                      << "q_feedback:\n" << q_feedback);

  Vector<3> des_omega = ref_omega + q_feedback;
  Vector<3> omega_feedback = k_omega.cwiseProduct(des_omega - cur_omega);
  Vector<3> des_alpha = ref_alpha + omega_feedback;

}

int main(int argc, char** argv){
  ros::init(argc, argv, "test_node");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  test();
  return 0;
}
