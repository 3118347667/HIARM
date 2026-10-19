#include "flygripper_controller/geometric_controller.hpp"

#include "flygripper_common/ros/conversions.hpp"
#include "flygripper_common/type/const.hpp"
#include "flygripper_common/utils/math.hpp"
#include "flygripper_common/utils/rotation.hpp"
#include "flygripper_controller/geometric_controller_param.hpp"
#include "flygripper_msgs/GraspState.h"

namespace flygripper_controller {

flygripper_msgs::GeometricControllerData GeometricControllerData::toRosMsg() {
  //    static Quat2RPY cur_q_converter, des_q_converter;
  flygripper_msgs::GeometricControllerData msg;

  toRosVec3(ref_pos, msg.position_reference);
  toRosVec3(cur_pos, msg.position_current);
  toRosVec3(ref_vel, msg.velocity_reference);
  toRosVec3(cur_vel, msg.velocity_current);
  toRosVec3(ref_acc, msg.acceleration_reference);
  toRosVec3(cur_acc, msg.acceleration_current);
  toRosVec3(cur_acc_f, msg.acceleration_current_filtered);

  toRosVec3(pos_err, msg.position_error);
  toRosVec3(vel_err, msg.velocity_error);
  toRosVec3(des_acc, msg.acceleration_desired);
  toRosVec3(des_thrust_acc, msg.thrust_acceleration_desired);
  msg.thrust_desired = des_thrust;

  toRosQuat(des_q, msg.attitude_desired);
  toRosQuat(cur_q, msg.attitude_current);

  // Noted: transform quaternion to eular angles RPY
  static Quat2RPY des_q_converter, cur_q_converter;
  toRosVec3(des_q_converter.convert(des_q), msg.attitude_desired_rpy);
  toRosVec3(cur_q_converter.convert(cur_q), msg.attitude_current_rpy);
  toRosVec3(angle_axis_err_axis, msg.attitude_error_axis);
  msg.attitude_error_angle = angle_axis_err_angle * RAD2DEG;

  toRosVec3(q_feedback * RAD2DEG, msg.attitude_feedback);
  toRosVec3(ref_omega * RAD2DEG, msg.omega_reference);
  toRosVec3(des_omega * RAD2DEG, msg.omega_desired);
  toRosVec3(cur_omega * RAD2DEG, msg.omega_current);
  toRosVec3(cur_omega_f * RAD2DEG, msg.omega_current_filtered);
  toRosVec3(omega_feedback * RAD2DEG, msg.omega_feedback);

  toRosVec3(cur_alpha_f * RAD2DEG, msg.alpha_current_filtered);
  toRosVec3(ref_alpha * RAD2DEG, msg.alpha_reference);
  toRosVec3(des_alpha * RAD2DEG, msg.alpha_desired);
  toRosVec3(des_torque, msg.torque_desired);

  msg.thrust_current_filtered = cur_thrust_f;
  toRosVec3(cur_torque_f, msg.torque_current_filtered);

  toRosVec3(cur_q * ext_force_indi, msg.external_force_indi);
  toRosVec3(ext_torque_indi, msg.external_torque_indi);
  toRosVec3(cur_q * ext_force_l1, msg.external_force_l1);
  toRosVec3(ext_torque_l1, msg.external_torque_l1);
  toRosVec3(cur_q * ext_force_tact, msg.external_force_tact);

  msg.rotor_velocities_current.resize(4);
  msg.rotor_velocities_current_filtered.resize(4);
  msg.rotor_velocities_desired.resize(4);

  for (int i = 0; i < 4; ++i) {
    msg.rotor_velocities_current[i] = cur_rotor_velocities(i);
    msg.rotor_velocities_current_filtered[i] = cur_rotor_velocities_f(i);
    msg.rotor_velocities_desired[i] = des_rotor_velocities(i);
  }
  return msg;
}

geometry_msgs::WrenchStamped GeometricControllerData::getExternalWrenchRosMsg(
    std::string estimate_method) {
  const std::string robot_namespace =
      ros::this_node::getNamespace().substr(1).c_str();
  geometry_msgs::WrenchStamped msg;
  msg.header.stamp = ros::Time::now();
  msg.header.frame_id = robot_namespace;
  if (estimate_method == "indi") {
    toRosVec3(ext_force_indi, msg.wrench.force);
    toRosVec3(ext_torque_indi, msg.wrench.torque);
  } else if (estimate_method == "l1") {
    toRosVec3(ext_force_l1, msg.wrench.force);
    toRosVec3(ext_torque_l1, msg.wrench.torque);
  } else if (estimate_method == "tact") {
    toRosVec3(ext_force_tact, msg.wrench.force);
    toRosVec3(Vector<3>::Zero(), msg.wrench.torque);
  } else {
    ROS_WARN(
        "Argument estimate_mode '%s' of getExternalWrenchRosMsg "
        "is invalid, it should be 'indi' or 'l1'.",
        estimate_method.c_str());
  }
  return msg;
}

GeometricController::GeometricController(RobotParameter::Ptr robot_parameter,
                                         Scalar control_frequency)
    : robot_parameter_(robot_parameter),
      control_frequency_(control_frequency),
      last_ctrl_time_(-1.0) {
  ros::NodeHandle cnh("geometric_controller");
  if (!ctrl_parameter_cfg_.init(cnh)) {
    throw ros::Exception("Failed to initialize geometric controller parameter");
  }
  ctrl_parameter_ = std::make_shared<GeometricControllerParameter>();
  ctrl_parameter_cfg_.getParams(*ctrl_parameter_);

  acc_lpf_ = std::make_shared<ButterworthLPF<3>>(
      ctrl_parameter_->acc_cutoff_freq, control_frequency_);
  omega_lpf_ = std::make_shared<ButterworthLPF<3>>(
      ctrl_parameter_->omega_cutoff_freq, control_frequency_);
  rotor_speed_lpf_ = std::make_shared<ButterworthLPF<4>>(
      ctrl_parameter_->rotor_cutoff_freq, control_frequency_);
  ctrl_omega_lpf_ = std::make_shared<ButterworthLPF<3>>(
      Vector<3>(ctrl_parameter_->ctrl_omega_xy_cutoff_freq,
                ctrl_parameter_->ctrl_omega_xy_cutoff_freq,
                ctrl_parameter_->ctrl_omega_z_cutoff_freq),
      Vector<3>::Ones() * control_frequency_);
  indi_acc_lpf_ = std::make_shared<ButterworthLPF<3>>(
      ctrl_parameter_->indi_acc_cutoff_freq, control_frequency_);
  indi_alpha_lpf_ = std::make_shared<ButterworthLPF<3>>(
      ctrl_parameter_->indi_alpha_cutoff_freq, control_frequency_);
  grasp_state_sub_ = cnh.subscribe<flygripper_msgs::GraspState>(
      "/grasp_state", 10, &GeometricController::grasp_state_callback, this,
      ros::TransportHints().tcpNoDelay());
  grasp_force_shear_ = 0.0;
}

GeometricController::~GeometricController() {}

void GeometricController::grasp_state_callback(
    const flygripper_msgs::GraspStateConstPtr& msg) {
  if (msg->force_shear > 0) {
    grasp_force_shear_ = 0.0;
  } else {
    grasp_force_shear_ = msg->force_shear;
  }
}

void GeometricController::prepare(const RobotState& state) {
  // Update controller parameter
  ctrl_parameter_cfg_.getParams(*ctrl_parameter_);
  updateCompensationMethod();
  //    ROS_INFO_STREAM("Geometric controller parameter:" << *ctrl_parameter_);
  //    ROS_INFO_STREAM("Robot parameter: " << *robot_parameter_);

  if (std::abs(ctrl_parameter_->acc_cutoff_freq -
               acc_lpf_->getCutoffFrequency(0)) > 1e-3) {
    ROS_INFO("acc_cutoff_freq changed from %f to %f",
             acc_lpf_->getCutoffFrequency(0), ctrl_parameter_->acc_cutoff_freq);
    acc_lpf_ = std::make_shared<ButterworthLPF<3>>(
        ctrl_parameter_->acc_cutoff_freq, control_frequency_, state.acc);
  } else {
    acc_lpf_->apply(state.acc);
  }

  if (std::abs(ctrl_parameter_->omega_cutoff_freq -
               omega_lpf_->getCutoffFrequency(0)) > 1e-3) {
    ROS_INFO("omega_cutoff_freq changed from %f to %f",
             omega_lpf_->getCutoffFrequency(0),
             ctrl_parameter_->omega_cutoff_freq);
    omega_lpf_ = std::make_shared<ButterworthLPF<3>>(
        ctrl_parameter_->omega_cutoff_freq, control_frequency_, state.omega);
  } else {
    omega_lpf_->apply(state.omega);
  }

  if (std::abs(ctrl_parameter_->rotor_cutoff_freq -
               rotor_speed_lpf_->getCutoffFrequency(0)) > 1e-3) {
    ROS_INFO("rotor_cutoff_freq changed from %f to %f",
             rotor_speed_lpf_->getCutoffFrequency(0),
             ctrl_parameter_->rotor_cutoff_freq);
    rotor_speed_lpf_ = std::make_shared<ButterworthLPF<4>>(
        ctrl_parameter_->rotor_cutoff_freq, control_frequency_,
        state.rot_speed);
  } else {
    rotor_speed_lpf_->apply(state.rot_speed);
  }

  if ((std::abs(ctrl_parameter_->ctrl_omega_xy_cutoff_freq -
                ctrl_omega_lpf_->getCutoffFrequency(0)) > 1e-3) ||
      (std::abs(ctrl_parameter_->ctrl_omega_z_cutoff_freq -
                ctrl_omega_lpf_->getCutoffFrequency(2)) > 1e-3)) {
    ROS_INFO(
        "ctrl_omega_cutoff_freq changed from (xy: %f, z: %f) to (xy: %f, z: "
        "%f)",
        ctrl_omega_lpf_->getCutoffFrequency(0),
        ctrl_omega_lpf_->getCutoffFrequency(2),
        ctrl_parameter_->ctrl_omega_xy_cutoff_freq,
        ctrl_parameter_->ctrl_omega_z_cutoff_freq);
    ctrl_omega_lpf_ = std::make_shared<ButterworthLPF<3>>(
        Vector<3>(ctrl_parameter_->ctrl_omega_xy_cutoff_freq,
                  ctrl_parameter_->ctrl_omega_xy_cutoff_freq,
                  ctrl_parameter_->ctrl_omega_z_cutoff_freq),
        Vector<3>::Ones() * control_frequency_, state.omega);
  } else {
    ctrl_omega_lpf_->apply(state.omega);
  }
}

void GeometricController::getCommand(const RobotState& state,
                                     const ReferenceVector& references,
                                     RobotCommand& command, const Scalar& now) {
  //! Step -1: Check if it is first time in autonomous ctrl
  bool init = (now - last_ctrl_time_ > 0.3);  // TODO: can load from param
  if (init) {
    ROS_WARN("Reset geometric controller.");
  }
  last_ctrl_time_ = now;
  //! Step 0: Get current estimated thrust and torque from rotor speed
  auto& ref = references[0];
  // Update control mixing matrix and its inverse matrix
  Matrix<4, 4> G;
  static Matrix<4, 4> static_G =
      robot_parameter_->getControlAllocation(state.rot_pos);
  // TODO: use static G or rot_pos based G
  if (robot_parameter_->dynamic_mixing) {
    G = robot_parameter_->getControlAllocation(state.rot_pos);
  } else {
    G = static_G;
  }
  Matrix<4, 4> G_inv = G.inverse();

  // INDI or feedback linearization of flatness using only
  Vector<3> acc_f = acc_lpf_->get();
  Vector<3> omega_f = omega_lpf_->get();
  Vector<3> alpha_f = omega_lpf_->getDerivative();
  Vector<4> rotor_speed_f = rotor_speed_lpf_->get();
  Vector<3> ctrl_omega_f = ctrl_omega_lpf_->get();

  // omega D only
  Vector<3> ctrl_alpha_f = ctrl_omega_lpf_->getDerivative();

  Vector<4> rotor_thrusts_f = robot_parameter_->thrust_coeff *
                              rotor_speed_f.asDiagonal() * rotor_speed_f;
  Vector<4> thrust_and_torque_f = G * rotor_thrusts_f;
  Scalar thrust_acc_norm_f =
      1.0 / robot_parameter_->mass * thrust_and_torque_f(0);
  Vector<3> torque_f = thrust_and_torque_f.tail(3);
  Quaternion cur_q = state.getQ();

  //! Step 1: Compute desired acc
  Vector<3> pos_err =
      clipVector(ref.state.pos - state.pos, ctrl_parameter_->pos_err_bound);
  Vector<3> des_vel =
      ctrl_parameter_->k_pos.cwiseProduct(pos_err) + ref.state.vel;
  Vector<3> vel_err =
      clipVector(des_vel - state.vel, ctrl_parameter_->vel_err_bound);
  Vector<3> des_acc =
      ctrl_parameter_->k_vel.cwiseProduct(vel_err) + ref.state.acc;

  //! Step 2: Compute desired thrust and its magnitude
  Vector<3> des_thrust_acc = des_acc - kGravityVector;
  limitTiltAngle(des_thrust_acc);

  Scalar des_thrust_acc_norm, des_thrust_norm;
  if (ctrl_parameter_->thrust_projection) {
    // project to current attitude
    Vector<3> cur_body_z_axis = cur_q * Vector<3>::UnitZ();
    des_thrust_acc_norm = des_thrust_acc.dot(cur_body_z_axis);
  } else {
    // directly use its norm
    des_thrust_acc_norm = des_thrust_acc.norm();
  }
  des_thrust_norm = des_thrust_acc_norm * robot_parameter_->mass;

  // External force defined body frame and compensation
  // L1 here estimate the external [force] in [body] frame
  data_.ext_force_l1 =
      clipVector(L1AugmentationForce(state.vel, cur_q, des_thrust_norm, init),
                 ctrl_parameter_->L1_force_bound);
  data_.ext_force_indi = clipVector(
      cur_q.inverse() *
          (robot_parameter_->mass *
           (acc_f - cur_q * (thrust_acc_norm_f * Vector<3>::UnitZ()) -
            kGravityVector)),
      ctrl_parameter_->INDI_force_bound);
  data_.ext_force_tact = clipVector(getTactForce(state.acc, cur_q),
                                    ctrl_parameter_->tact_force_bound);

  if (acc_compensation_ == CompensationMethod::INDI) {
    //      des_thrust_acc = cur_q * (thrust_acc_norm_f * Vector<3>::UnitZ())
    //          + des_acc - acc_f;
    des_thrust_acc -= indi_acc_lpf_->apply(cur_q * data_.ext_force_indi /
                                           robot_parameter_->mass);
  } else if (acc_compensation_ == CompensationMethod::L1) {
    des_thrust_acc -= cur_q * data_.ext_force_l1 / robot_parameter_->mass;
  } else if (acc_compensation_ == CompensationMethod::TACT) {
    des_thrust_acc -= cur_q * data_.ext_force_tact / robot_parameter_->mass;
  }

  // Again to limit tilt angle after compensation
  limitTiltAngle(des_thrust_acc);
  if (ctrl_parameter_->thrust_projection) {
    // project to current attitude
    Vector<3> cur_body_z_axis = cur_q * Vector<3>::UnitZ();
    des_thrust_acc_norm = des_thrust_acc.dot(cur_body_z_axis);
  } else {
    des_thrust_acc_norm = des_thrust_acc.norm();
  }
  des_thrust_norm = des_thrust_acc_norm * robot_parameter_->mass;

  //! Step 3: Compute desired q (attitude)
  Quaternion des_q;
  Vector<3> des_body_z_axis = des_thrust_acc.normalized();
  Quaternion des_q_tilt =
      Quaternion::FromTwoVectors(Vector<3>::UnitZ(), des_body_z_axis);
  Scalar ref_yaw_angle = ref.state.getYaw(state.getYaw(0));
  Quaternion des_q_yaw(
      Eigen::AngleAxis<Scalar>(ref_yaw_angle, Vector<3>::UnitZ()));
  des_q = des_q_tilt * des_q_yaw;

  //! Step 4: Compute ref_omega and ref_alpha from flatness

  //! 4.1 Calculate reference omega from flatness
  Vector<3> ref_omega;
  // Feedback linearization vs feedforward linearization,
  Quaternion q;
  Scalar thrust_acc_norm;
  if (ctrl_parameter_->feedback_linearization) {
    // Feedback linearization
    q = cur_q;
    thrust_acc_norm = thrust_acc_norm_f;
  } else {
    // Feedforward linearization
    q = des_q;
    thrust_acc_norm = des_thrust_acc_norm;
  }
  const Vector<3> body_x_axis = q * Vector<3>::UnitX();
  const Vector<3> body_y_axis = q * Vector<3>::UnitY();
  const Vector<3> body_z_axis = q * Vector<3>::UnitZ();
  const Scalar thrust_dot = ref.state.jerk.dot(body_z_axis);
  Vector<3> H_omega = ref.state.jerk;
  // Avoid singularity when thrust is zero
  // using sensor measured thrust acceleration value (feedback linearization)
  if (thrust_acc_norm > kAlmostZeroValueThreshold) {
    H_omega *= 1.0 / thrust_acc_norm;
  }
  ref_omega =
      Vector<3>(-H_omega.dot(body_y_axis), H_omega.dot(body_x_axis),
                (ref.state.omega.z() * Vector<3>::UnitZ()).dot(body_z_axis));

  //! 4.2 Calculate reference alpha from flatness
  Vector<3> ref_alpha;
  Vector<3> omega;
  if (ctrl_parameter_->feedback_linearization) {
    omega = state.omega;
  } else {
    omega = ref_omega;
  }
  const Vector<3> W_omega = q * omega;
  const Vector<3> body_z_axis_dot = W_omega.cross(body_z_axis);
  Vector<3> H_alpha_1 =
      ref.state.snap;  // 1.0 / thrust_acc_norm_f * ref.state.snap;
  Vector<3> H_alpha_2 = -W_omega.cross(body_z_axis_dot);
  Vector<3> H_alpha_3 =
      -2 * thrust_dot *
      body_z_axis_dot;  // - 2 * thrust_dot / thrust_acc_norm_f * B_z_dot;
                        // Avoid singularity when thrust is zero
  // using sensor measured thrust acceleration value (feedback linearization)
  if (thrust_acc_norm > kAlmostZeroValueThreshold) {
    H_alpha_1 *= 1.0 / thrust_acc_norm;
    H_alpha_3 *= 1.0 / thrust_acc_norm;
  }
  const Vector<3> H_alpha = H_alpha_1 + H_alpha_2 + H_alpha_3;
  ref_alpha =
      Vector<3>(-H_alpha.dot(body_y_axis), H_alpha.dot(body_x_axis),
                (ref.state.alpha.z() * Vector<3>::UnitZ()).dot(body_z_axis));
  //! Step 5: Compute desired omega (angular velocity) from q_feedback and
  //! ref_omega
  Quaternion q_err = cur_q.inverse() * des_q;
  Vector<3> q_feedback;
  if (ctrl_parameter_->tilt_prioritization) {
    q_feedback = tiltPrioritizedAttitudeControl(q_err);
  } else {
    q_feedback = AttitudeControl(q_err);
  }
  Vector<3> des_omega =
      clipVector(q_feedback + ref_omega, robot_parameter_->bodyrates_max);

  //! Step 6: Compute desired alpha (angular acceleration) from omega_feedback
  //! and ref_alpha
  Vector<3> omega_err = q_err * des_omega - ctrl_omega_f;
  Vector<3> omega_feedback = ctrl_parameter_->k_omega.cwiseProduct(omega_err);
  Vector<3> des_alpha = omega_feedback -
                        ctrl_parameter_->kd_omega.cwiseProduct(ctrl_alpha_f) +
                        getOmegaIntegrationTerm(omega_err);  //+ TODO: ref_alpha

  //! Step 7: Computed desired torque
  Vector<3> des_torque = robot_parameter_->J * des_alpha +
                         ctrl_omega_f.cross(robot_parameter_->J * ctrl_omega_f);

  data_.ext_torque_indi =
      clipVector(robot_parameter_->J * alpha_f - torque_f +
                     omega_f.cross(robot_parameter_->J * omega_f),
                 ctrl_parameter_->INDI_torque_bound);
  data_.ext_torque_l1 = clipVector(
      L1AugmentationTorque(ctrl_omega_f, des_torque, init),
      ctrl_parameter_->L1_torque_bound);  // TODO: state.omega, des_torque);

  if (alpha_compensation_ == CompensationMethod::INDI) {
    // NOTE:use ndi instead of indi for yaw acc to prevent yaw osscilation.
    // des_torque.segment<2>(0) =(torque_f + robot_parameter_->J * (des_alpha -
    // alpha_f)).segment<2>(0);
    des_torque.segment<2>(0) =
        (robot_parameter_->J * (des_alpha) +
         indi_alpha_lpf_->apply(torque_f - robot_parameter_->J * alpha_f))
            .segment<2>(0);
  } else if (alpha_compensation_ == CompensationMethod::L1) {
    // NOTE:use ndi instead of L1 for yaw acc to prevent yaw osscilation ?
    des_torque.segment<2>(0) -= data_.ext_torque_l1.segment<2>(0);
    //    des_torque -= data_.ext_torque_l1;
  } else {
    des_torque -= ctrl_parameter_->ff_omega;
  }

  //! Step 7: Computed desired single rotor thrusts or velocities
  Vector<4> des_thrust_and_torque;
  des_thrust_and_torque(0) =
      std::clamp(des_thrust_norm, robot_parameter_->collective_thrust_min,
                 robot_parameter_->collective_thrust_max);

  des_thrust_and_torque.tail(3) =
      clipVector(des_torque, robot_parameter_->torque_max);

  //    //! [Optional] Apply L1 adaptive control for both thrust and torque
  //    des_thrust_and_torque += L1Augmentation(state.vel, state.omega, cur_q,
  //                                            des_thrust_and_torque);

  // Ensure all thrusts are positive
  Vector<4> des_rotor_thrusts = clipVector(G_inv * des_thrust_and_torque,
                                           robot_parameter_->rotor_thrusts_min,
                                           robot_parameter_->rotor_thrusts_max);
  Vector<4> des_rotor_velocities =
      (des_rotor_thrusts / robot_parameter_->thrust_coeff).cwiseSqrt();

  // control mode by if-else
  command.stamp = now;
  command.collective_thrust = des_thrust_and_torque(0);
  command.attitude = des_q;
  command.bodyrates = des_omega;
  command.torque = des_torque;
  command.rotor_thrusts = des_rotor_thrusts;
  command.rotor_velocities = des_rotor_velocities;
  command.delta_mass = 0.0;  //-(cur_q * data_.ext_force_indi).z() / kGravity;
  command.compensation_method = static_cast<int>(alpha_compensation_);

  // debug info
  data_.cur_pos = state.pos;
  data_.ref_pos = ref.state.pos;
  data_.cur_vel = state.vel;
  data_.ref_vel = ref.state.vel;
  data_.cur_acc = state.acc;
  data_.cur_acc_f = acc_f;
  data_.ref_acc = ref.state.acc;

  data_.pos_err = pos_err;
  data_.vel_err = vel_err;
  data_.des_acc = des_acc;
  data_.des_thrust_acc = des_thrust_acc;

  data_.cur_q = cur_q;
  data_.des_q = des_q;

  Eigen::AngleAxisd rotation_vector(q_err);
  data_.angle_axis_err_axis = rotation_vector.axis();
  data_.angle_axis_err_angle = rotation_vector.angle();
  data_.q_feedback = q_feedback;
  data_.ref_omega = ref_omega;
  data_.des_omega = des_omega;

  data_.cur_omega = ctrl_omega_f;  // state.omega;
  data_.cur_omega_f = omega_f;
  data_.omega_feedback = omega_feedback;

  data_.cur_alpha_f = alpha_f;
  data_.ref_alpha = ref_alpha;
  data_.des_alpha = des_alpha;

  data_.des_thrust = des_thrust_norm;
  data_.des_torque = des_torque;
  data_.des_rotor_velocities = des_rotor_velocities;
  data_.cur_rotor_velocities = state.rot_speed;
  data_.cur_rotor_velocities_f = rotor_speed_f;

  data_.cur_torque_f = torque_f;
  data_.cur_thrust_f = thrust_and_torque_f(0);
  ;

  if (!command.valid()) {
    ROS_INFO("-------------------------------------");
    ROS_WARN_STREAM("Command is invalid!"
                    << command << "\n"
                    << "des_alpha: " << des_alpha.transpose() << "\n"
                    << "rot_pos: " << state.rot_pos.transpose());
  }
}

Vector<3> GeometricController::getOmegaIntegrationTerm(const Vector<3>& error,
                                                       bool limit) {
  static Vector<3> integral_term = Vector<3>::Zero();
  for (int i = 0; i < 3; ++i) {
    if (!isZero(ctrl_parameter_->ki_omega(i))) {
      // TODO: APM
      //        // Ensure that integrator can only be reduced if the output is
      //        saturated if (!limit || ((integral_term(i) > 0.0 && error(i) <
      //        0.0) || (integral_term(i) < 0.0 && error(i) > 0.0))) {
      //          integral_term(i) += (float)error(i) *
      //          ctrl_parameter_->ki_omega(i); integral_term(i) =
      //          std::clamp(integral_term(i),
      //                                        -ctrl_parameter_->ki_omega_max(i),
      //                                        ctrl_parameter_->ki_omega_max(i));
      //        }

      integral_term(i) += std::clamp(error(i), -1.0, 1.0);
      integral_term(i) =
          std::clamp(integral_term(i), -ctrl_parameter_->ki_omega_max(i),
                     ctrl_parameter_->ki_omega_max(i));
    } else {
      integral_term(i) = 0.0f;
    }
  }
  return ctrl_parameter_->ki_omega.cwiseProduct(integral_term);
}

void GeometricController::limitTiltAngle(Ref<Vector<3>> des_thrust_acc) {
  Vector<3> des_B_z = des_thrust_acc.normalized();
  Scalar des_thrust_acc_z_magnitude =
      des_thrust_acc.dot(Eigen::Vector3d::UnitZ());
  Eigen::Vector3d tilt_rotation_axis = Eigen::Vector3d::UnitZ().cross(des_B_z);
  Scalar tilt_angle = std::acos(Eigen::Vector3d::UnitZ().dot(des_B_z));
  if (robot_parameter_->tilt_angle_max > 0 &&
      tilt_angle > robot_parameter_->tilt_angle_max) {
    ROS_WARN_THROTTLE(0.1,
                      "Exceed the tilt angle limit, tilt angle: %f, limit: %f",
                      tilt_angle, robot_parameter_->tilt_angle_max);
    Eigen::Vector3d limited_z_B =
        Eigen::AngleAxisd(robot_parameter_->tilt_angle_max,
                          tilt_rotation_axis) *
        Eigen::Vector3d::UnitZ();
    // Update the desired thrust acceleration and its normalized vector
    des_thrust_acc = des_thrust_acc_z_magnitude /
                     std::cos(robot_parameter_->tilt_angle_max) * limited_z_B;
  }
}

Vector<3> GeometricController::tiltPrioritizedAttitudeControl(
    const Quaternion& q_e) {
  // 2018 Tilt-prioritized quadrocopter attitude control
  Matrix<3, 3> k_att_mat = ctrl_parameter_->k_att.asDiagonal();
  Vector<3> q_err = 1.0 / std::sqrt(q_e.w() * q_e.w() + q_e.z() * q_e.z()) *
                    Vector<3>(q_e.w() * q_e.x() - q_e.y() * q_e.z(),
                              q_e.w() * q_e.y() + q_e.x() * q_e.z(), q_e.z());
  if (q_e.w() <= 0) {
    q_err(2) *= -1.0;
  }
  const Vector<3> q_feedback = 2.0 * k_att_mat * q_err;
  return q_feedback;
}

Vector<3> GeometricController::AttitudeControl(const Quaternion& q_e) {
  Matrix<3, 3> k_att_mat = ctrl_parameter_->k_att.asDiagonal();
  Vector<3> q_err = Vector<3>(q_e.x(), q_e.y(), q_e.z());

  if (q_e.w() <= 0) {
    q_err *= -1.0;
  }
  const Vector<3> q_feedback = 2.0 * k_att_mat * q_err;
  return q_feedback;
}

Vector<3> GeometricController::L1AugmentationForce(ConstRef<Vector<3>> vel,
                                                   const Quaternion& q,
                                                   const Scalar& u_b,
                                                   bool init) {
  Matrix<3> As_v = Vector<3>::Constant(ctrl_parameter_->As_v).asDiagonal();
  Scalar dt = 1.0 / control_frequency_;
  Scalar thrust_flt1_coeff =
      std::exp(-ctrl_parameter_->thrust_flt1_cutoff_freq * dt);
  bool l1_augmentation =
      ctrl_parameter_->acc_compensation == CompensationMethod::L1;

  static Vector<3> prev_vel = vel;  // previous velocity v(k-1)
  static Quaternion prev_q = q;     // previous orientation q(k-1)
  static Vector<3> prev_vel_pred =
      vel;                        // previous predicted observation z_hat(k-1)
  static Scalar prev_u_b = u_b;   // previous nominal control input u_b(k-1)
  static Scalar prev_u_ad = 0.0;  // previous adaptive control input u_ad(k-1)
  static Scalar prev_signma_est_m =
      0.0;  // previous estimated matched uncertainty signma_hat_m(k-1)
  static Vector<2> prev_signma_est_um =
      Vector<2>::Zero();  // previous estimated unmatched uncertainty
                          // signma_hat_um(k-1)
  // previous filter data
  static Scalar prev_lpf1 = 0.0;
  static Vector<2> prev_lpf1_um = Vector<2>::Zero();

  if (init) {
    prev_vel = vel;
    prev_q = q;
    prev_vel_pred = vel;
    prev_u_b = u_b;
    prev_u_ad = 0.0;
    prev_signma_est_m = 0.0;
    prev_signma_est_um = Vector<2>::Zero();
    prev_lpf1 = 0.0;
    prev_lpf1_um = Vector<2>::Zero();
  }

  Vector<3> prev_vel_error =
      prev_vel_pred - prev_vel;  // previous prediction error z_tild(k-1)

  //! Matrix definition
  Scalar mass_inv = 1.0 / robot_parameter_->mass;
  Vector<3> prev_f = kGravityVector;

  Vector<3> prev_B = mass_inv * (prev_q * Vector<3>::UnitZ());
  Matrix<3, 2> prev_B_u = Matrix<3, 2>::Zero();
  prev_B_u.block<3, 1>(0, 0) = mass_inv * (prev_q * Vector<3>::UnitX());
  prev_B_u.block<3, 1>(0, 1) = mass_inv * (prev_q * Vector<3>::UnitY());

  Matrix<3> B_all = Matrix<3>::Zero();
  Ref<Vector<3>> B = B_all.block<3, 1>(0, 0);
  B = mass_inv * (q * Vector<3>::UnitZ());

  Ref<Matrix<3, 2>> B_u = B_all.block<3, 2>(0, 1);
  B_u.block<3, 1>(0, 0) = mass_inv * (q * Vector<3>::UnitX());
  B_u.block<3, 1>(0, 1) = mass_inv * (q * Vector<3>::UnitY());

  //! Step 1: State prediction
  Vector<3> vel_pred =
      prev_vel_pred +
      (prev_f + prev_B * (prev_u_b + prev_u_ad + prev_signma_est_m) +
       prev_B_u * prev_signma_est_um + As_v * prev_vel_error) *
          dt;

  //! Step 2: Update the prediciton error
  Vector<3> vel_error = vel_pred - vel;

  //! Step 3:  Estimate the uncertainty
  Scalar exp_As_v_dt = std::exp(ctrl_parameter_->As_v * dt);
  Scalar phi_inv_v = 1.0 / (exp_As_v_dt - 1) * ctrl_parameter_->As_v;
  Matrix<3> phi_inv = Vector<3>::Constant(phi_inv_v).asDiagonal();
  Vector<3> mu = Vector<3>::Constant(exp_As_v_dt).asDiagonal() * vel_error;
  Vector<3> sigma_est = -B_all.inverse() * phi_inv * mu;
  Scalar& sigma_est_m = sigma_est(0);
  Ref<Vector<2>> sigma_est_um = sigma_est.block<2, 1>(1, 0);

  //! Step 4: LowPass filter the matched uncertainties to calculate adaptive
  //! control u_ad
  Scalar sigma_est_m_lpf1 =
      thrust_flt1_coeff * prev_lpf1 + (1 - thrust_flt1_coeff) * sigma_est_m;
  prev_lpf1 = sigma_est_m_lpf1;

  // Just filter the unmatched uncertainties
  Vector<2> sigma_est_um_lpf1;
  sigma_est_um_lpf1 =
      thrust_flt1_coeff * prev_lpf1_um + (1 - thrust_flt1_coeff) * sigma_est_um;
  prev_lpf1_um = sigma_est_um_lpf1;

  // Get u_ad
  Scalar u_ad = -sigma_est_m_lpf1;

  //! Store the previous variable for next step
  prev_vel = vel;            // previous velocity v(k-1)
  prev_q = q;                // previous orientation q(k-1)
  prev_vel_pred = vel_pred;  // previous predicted observation z_hat(k-1)
  prev_u_b = u_b;            // previous nominal control input u_b(k-1)
  prev_signma_est_m =
      sigma_est_m;  // previous estimated matched uncertainty signma_hat_m(k-1)
  prev_signma_est_um = sigma_est_um;  // previous estimated unmatched
                                      // uncertainty signma_hat_um(k-1)
  // To be noted, if l1 is not enabled, set to zeros
  prev_u_ad = l1_augmentation
                  ? u_ad
                  : 0.0;  // previous adaptive control input u_ad(k-1)
  return Vector<3>(sigma_est_um_lpf1.x(), sigma_est_um_lpf1.y(),
                   sigma_est_m_lpf1);
}

Vector<3> GeometricController::L1AugmentationTorque(ConstRef<Vector<3>> omega,
                                                    ConstRef<Vector<3>> u_b,
                                                    bool init) {
  Matrix<3> As_omega =
      Vector<3>::Constant(ctrl_parameter_->As_omega).asDiagonal();
  Scalar dt = 1.0 / control_frequency_;
  Scalar torque_flt1_coeff =
      std::exp(-ctrl_parameter_->torque_flt1_cutoff_freq * dt);
  Scalar torque_flt2_coeff =
      std::exp(-ctrl_parameter_->torque_flt2_cutoff_freq * dt);
  bool l1_augmentation =
      ctrl_parameter_->alpha_compensation == CompensationMethod::L1;

  static Vector<3> prev_omega = omega;  // previous velocity v(k-1)
  static Vector<3> prev_omega_pred =
      omega;                        // previous predicted observation z_hat(k-1)
  static Vector<3> prev_u_b = u_b;  // previous nominal control input u_b(k-1)
  static Vector<3> prev_u_ad =
      Vector<3>::Zero();  // previous adaptive control input u_ad(k-1)
  static Vector<3> prev_signma_est_m =
      Vector<3>::Zero();  // previous estimated matched uncertainty
                          // signma_hat_m(k-1)

  // previous filter data
  static Vector<3> prev_lpf1 = Vector<3>::Zero();
  static Vector<3> prev_lpf2 = Vector<3>::Zero();

  if (init) {
    prev_omega = omega;
    prev_omega_pred = omega;
    prev_u_b = u_b;
    prev_u_ad = Vector<3>::Zero();
    prev_signma_est_m = Vector<3>::Zero();

    prev_lpf1 = Vector<3>::Zero();
    prev_lpf2 = Vector<3>::Zero();
  }

  Vector<3> prev_omega_error =
      prev_omega_pred - prev_omega;  // previous prediction error z_tild(k-1)

  //! Matrix definition
  Scalar mass_inv = 1.0 / robot_parameter_->mass;
  Vector<3> prev_f = -robot_parameter_->J_inv *
                     prev_omega.cross(robot_parameter_->J * prev_omega);
  Matrix<3> prev_B = robot_parameter_->J_inv;
  Matrix<3> B = robot_parameter_->J_inv;

  //! Step 1: State prediction
  Vector<3> omega_pred =
      prev_omega_pred +
      (prev_f + prev_B * (prev_u_b + prev_u_ad + prev_signma_est_m) +
       As_omega * prev_omega_error) *
          dt;

  //! Step 2: Update the prediciton error
  Vector<3> omega_error = omega_pred - omega;

  //! Step 3:  Estimate the uncertainty
  Scalar exp_As_omega_dt = std::exp(ctrl_parameter_->As_omega * dt);
  Scalar phi_inv_omega =
      1.0 / (exp_As_omega_dt - 1) * ctrl_parameter_->As_omega;
  Matrix<3> phi_inv = Vector<3>::Constant(phi_inv_omega).asDiagonal();
  Vector<3> mu =
      Vector<3>::Constant(exp_As_omega_dt).asDiagonal() * omega_error;
  Vector<3> sigma_est_m = -B.inverse() * phi_inv * mu;

  //! Step 4: LowPass filter the matched uncertainties to calculate adaptive
  //! control u_ad
  Vector<3> sigma_est_m_lpf1 =
      torque_flt1_coeff * prev_lpf1 + (1 - torque_flt1_coeff) * sigma_est_m;
  prev_lpf1 = sigma_est_m_lpf1;
  Vector<3> sigma_est_m_lpf2 = torque_flt2_coeff * prev_lpf2 +
                               (1 - torque_flt2_coeff) * sigma_est_m_lpf1;
  prev_lpf2 = sigma_est_m_lpf2;

  // Get u_ad
  Vector<3> u_ad = -sigma_est_m_lpf2;

  //! Store the previous variable for next step
  prev_omega = omega;            // previous velocity v(k-1)
  prev_omega_pred = omega_pred;  // previous predicted observation z_hat(k-1)
  prev_u_b = u_b;                // previous nominal control input u_b(k-1)
  prev_signma_est_m =
      sigma_est_m;  // previous estimated matched uncertainty signma_hat_m(k-1)
  // To be noted, if l1 is not enabled, set to zeros
  prev_u_ad =
      l1_augmentation
          ? u_ad
          : Vector<3>::Zero();  // previous adaptive control input u_ad(k-1)
  return sigma_est_m_lpf2;
}

Vector<3> GeometricController::getTactForce(ConstRef<Vector<3>> acc,
                                            const Quaternion& q) {
  return Vector<3>(0, 0, grasp_force_shear_);
}

flygripper_msgs::GeometricControllerData
GeometricController::getControllerMsg() {
  return data_.toRosMsg();
}

Vector<6> GeometricController::getExternalWrench(std::string estimate_method) {
  Vector<6> external_wrench;
  if (estimate_method == "l1") {
    external_wrench << data_.cur_q * data_.ext_force_l1, data_.ext_torque_l1;
  } else if (estimate_method == "indi") {
    external_wrench << data_.cur_q * data_.ext_force_indi,
        data_.ext_torque_indi;
  } else if (estimate_method == "tact") {
    external_wrench << data_.cur_q * data_.ext_force_tact, Vector<3>::Zero();
  } else {
    external_wrench = Vector<6>::Zero();
  }
  return external_wrench;
}

geometry_msgs::WrenchStamped GeometricController::getExternalWrenchMsg(
    std::string estimate_method) {
  return data_.getExternalWrenchRosMsg(estimate_method);
}

void GeometricController::updateCompensationMethod() {
  if (ctrl_parameter_->acc_compensation != acc_compensation_) {
    ROS_INFO(
        "\033[36mLinear acceleration compensation: %s => %s\033[0m",
        CompensationMethodToString(acc_compensation_).c_str(),
        CompensationMethodToString(ctrl_parameter_->acc_compensation).c_str());
    acc_compensation_ = ctrl_parameter_->acc_compensation;
  }
  if (ctrl_parameter_->alpha_compensation != alpha_compensation_) {
    ROS_INFO("\033[36mAngular acceleration compensation: %s => %s\033[0m",
             CompensationMethodToString(alpha_compensation_).c_str(),
             CompensationMethodToString(ctrl_parameter_->alpha_compensation)
                 .c_str());
    alpha_compensation_ = ctrl_parameter_->alpha_compensation;
  }
}

}  // namespace flygripper_controller
