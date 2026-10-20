#pragma once
#include "flygripper_common/quadrotor/flatness.hpp"
#include "flygripper_common/type/reference.hpp"
#include "flygripper_common/type/robot_parameter.hpp"
#include "flygripper_common/type/trajectory.hpp"

#include "flygripper_flight/admittance_control.hpp"
#include "flygripper_flight/fsm_parm.hpp"
#include "flygripper_flight/velocity_control.hpp"
using namespace flygripper_common;
namespace flygripper_flight {
class Sampler {
public:
  using Ptr = std::shared_ptr<Sampler>;
  Sampler(RobotParameter::Ptr robot_parameter, FsmParameter::Ptr fsm_parameter,
          int horizon_len = 20, Scalar horizon_dt = 0.05,
          Scalar position_error_thres = MAX)
      : robot_parameter_(robot_parameter), fsm_parameter_(fsm_parameter),
        flatness_(robot_parameter), horizon_len_(horizon_len),
        horizon_dt_(horizon_dt), position_error_thres_(position_error_thres) {}
  ~Sampler() = default;

  void getHoverReference(const RobotState &robot_state,
                         ReferenceVector &references,
                         const Scalar &current_stamp,
                         const RobotCommandMode &command_mode,
                         bool init = false) {
    static RobotState hover_robot_state = RobotState(current_stamp);
    if (init) {
      hover_robot_state = robot_state.getHoverState();
    }
    getPoseReference(hover_robot_state.pos, hover_robot_state.getYaw(),
                     references, current_stamp, command_mode);
  }

  void getPoseReference(ConstRef<Vector<3>> position, const Scalar &heading,
                        ReferenceVector &references,
                        const Scalar &current_stamp,
                        const RobotCommandMode &command_mode) {
    references.resize(horizon_len_);
    Scalar stamp = current_stamp; // robot_state.stamp;
    for (auto &reference : references) {
      //! Reference State
      reference.state = RobotState(stamp);
      reference.state.pos = position;
      reference.state.setQ(heading);

      //! Reference Input
      reference.input = RobotCommand(stamp, command_mode);
      stamp += horizon_dt_;

      // Hover input reference
      reference.input.collective_thrust = robot_parameter_->mass * kGravity;
      switch (command_mode) {
      case RobotCommandMode::MODE_SINGLE_ROTOR: {
        reference.input.rotor_thrusts =
            Vector<4>::Constant(1.0 / 4 * reference.input.collective_thrust);
        reference.input.rotor_velocities =
            (1 / robot_parameter_->thrust_coeff * reference.input.rotor_thrusts)
                .cwiseSqrt();
        break;
      }
      case RobotCommandMode::MODE_BODY_RATE: {
        reference.input.bodyrates = Vector<3>::Zero();
        break;
      }
      case RobotCommandMode::MODE_ATTITUDE: {
        reference.input.setAttitude(reference.state.getYaw());
        break;
      }
      default:
        break;
      }
    }
  }

  void getTakeoffReference(const TakeoffPlan &plan,
                           ReferenceVector &references,
                           const Scalar &current_stamp,
                           const RobotCommandMode &command_mode) {
    references.resize(horizon_len_);
    Scalar stamp = current_stamp; // robot_state.stamp;
    for (auto &reference : references) {
      //! Reference State
      const auto sample = plan.sample(stamp);
      reference.state = RobotState(stamp);
      for (int axis = 0; axis < 3; ++axis)
        reference.state.pos(axis) = sample.position[axis];
      reference.state.setQ(sample.yaw);
      reference.state.vel.z() = sample.velocity;
      reference.state.acc.z() = sample.acceleration;
      reference.state.jerk.z() = sample.jerk;
      reference.state.snap.z() = sample.snap;
      //! Reference Input
      reference.input = RobotCommand(stamp, command_mode);
      stamp += horizon_dt_;

      // Hover input reference
      reference.input.collective_thrust = robot_parameter_->mass * kGravity;
      switch (command_mode) {
      case RobotCommandMode::MODE_SINGLE_ROTOR: {
        reference.input.rotor_thrusts =
            Vector<4>::Constant(1.0 / 4 * reference.input.collective_thrust);
        reference.input.rotor_velocities =
            (1 / robot_parameter_->thrust_coeff * reference.input.rotor_thrusts)
                .cwiseSqrt();
        break;
      }
      case RobotCommandMode::MODE_BODY_RATE: {
        reference.input.bodyrates = Vector<3>::Zero();
        break;
      }
      case RobotCommandMode::MODE_ATTITUDE: {
        reference.input.setAttitude(reference.state.getYaw());
        break;
      }
      default:
        break;
      }
    }
  }

  void getVelocityReference(const RobotState &robot_state,
                            const VelocityControlSystem::Input &velocity_input,
                            ReferenceVector &references,
                            const Scalar &current_stamp,
                            const RobotCommandMode &command_mode,
                            bool init = false, bool world_frame = true) {
    static VelocityControlSystem sys;
    static VelocityControlSystem::State init_state;
    static Scalar last_stamp = current_stamp;
    if (init) {
      init_state =
          VelocityControlSystem::State(robot_state.pos, robot_state.getYaw(),
                                       robot_state.vel, robot_state.omega.z());
    } else {
      init_state = sys.update(init_state, velocity_input,
                              current_stamp - last_stamp, world_frame);
    }
    last_stamp = current_stamp;
    VelocityControlSystem::State ref_state = init_state;
    references.resize(horizon_len_);
    Scalar stamp = current_stamp; // robot_state.stamp;
    for (auto &reference : references) {
      //! Reference State
      reference.state = RobotState(stamp);
      reference.state.pos = ref_state.pos;
      reference.state.vel = Vector<3>::Zero(); // TODO: ref_state.vel;
      reference.state.setQ(ref_state.yaw);
      reference.state.omega.z() = 0; // TODO: ref_state.yaw_rate;
      ref_state =
          sys.update(ref_state, velocity_input, horizon_dt_, world_frame);

      //! Reference Input
      reference.input = RobotCommand(stamp, command_mode);
      stamp += horizon_dt_;

      // Hover input reference
      reference.input.collective_thrust = robot_parameter_->mass * kGravity;
      switch (command_mode) {
      case RobotCommandMode::MODE_SINGLE_ROTOR: {
        reference.input.rotor_thrusts =
            Vector<4>::Constant(1.0 / 4 * reference.input.collective_thrust);
        reference.input.rotor_velocities =
            (1 / robot_parameter_->thrust_coeff * reference.input.rotor_thrusts)
                .cwiseSqrt();
        break;
      }
      case RobotCommandMode::MODE_BODY_RATE: {
        reference.input.bodyrates = Vector<3>::Zero();
        break;
      }
      case RobotCommandMode::MODE_ATTITUDE: {
        reference.input.setAttitude(reference.state.getYaw());
        break;
      }
      default:
        break;
      }
    }
  }

  bool getTrajectoryReference(const RobotState &robot_state,
                              const Trajectory &trajectory,
                              ReferenceVector &references,
                              const Scalar &current_stamp,
                              const RobotCommandMode &command_mode) {
    static Scalar last_start_stamp = -1.0;
    references.resize(horizon_len_);
    // Check if trajectory is timeout or for future at current stamp
    Scalar stamp = current_stamp;

    Scalar start_stamp = trajectory.getStartTime();
    Scalar end_stamp = trajectory.getEndTime();

    // TODO: check if this is first time for this trajectory
    bool init = !isApprox(start_stamp, last_start_stamp);
    last_start_stamp = start_stamp;

    // Check if position error is really large
    // TODO: search closest point and refine the trajectory time adaptively?

    auto current_traj_pos =
        trajectory.xyz_mode == Trajectory::XYZMode::XYZ_MIN_JERK
            ? trajectory.xyz_min_jerk_poly.getPos(stamp - start_stamp)
            : trajectory.xyz_min_snap_poly.getPos(stamp - start_stamp);
    auto position_error = (current_traj_pos - robot_state.pos).norm();
    if (position_error > position_error_thres_) {
      ROS_WARN("Trajectory is too far away from current position. "
               "Position error is %f, larger than given threshold %f."
               "Current position is %f, %f, %f, "
               "and trajectory position is %f, %f, %f",
               position_error, position_error_thres_, robot_state.pos(0),
               robot_state.pos(1), robot_state.pos(2), current_traj_pos(0),
               current_traj_pos(1), current_traj_pos(2));
      return false;
    }

    // NOTE: first state is current state or trajectory state(*)?
    // S=3 minimum snap trajectory
    Matrix<3, 5> flatness_output;
    Vector<3> heading_vec = Vector<3>::Zero();
    Scalar clamp_stamp;

    for (int i = 0; i < horizon_len_; ++i) {
      clamp_stamp = std::clamp(stamp, start_stamp, end_stamp);
      references[i].state = RobotState(clamp_stamp);
      references[i].state.rot_pos = robot_state.rot_pos;
      references[i].input = RobotCommand(clamp_stamp, command_mode);
      flatness_output =
          trajectory.xyz_mode == Trajectory::XYZMode::XYZ_MIN_JERK
              ? trajectory.xyz_min_jerk_poly.getPVAJS(clamp_stamp - start_stamp)
              : trajectory.xyz_min_snap_poly.getPVAJS(clamp_stamp -
                                                      start_stamp);
      // differential flatness mapping for quadrotor system
      if (trajectory.heading_mode == Trajectory::HeadingMode::HEADING_FORWARD ||
          trajectory.heading_mode ==
              Trajectory::HeadingMode::HEADING_BACKWARD) {
        heading_vec = Vector<3>::Zero();
        if (i == 0) {
          // TODO: leave this unimplemented here for horizon-based controller
          calculateHeading(flatness_output, robot_state,
                           1.0 / fsm_parameter_->loop_frequency, heading_vec,
                           trajectory.heading_mode ==
                               Trajectory::HeadingMode::HEADING_FORWARD,
                           init);
          heading_vec(1) = 0.0;
          ROS_INFO_THROTTLE(1.0,
                            "Heading: %f, heading_rate: %f, heading_mode: %s ",
                            heading_vec(0), heading_vec(1),
                            trajectory.heading_mode ==
                                    Trajectory::HeadingMode::HEADING_FORWARD
                                ? "HEADING_FORWARD"
                                : "HEADING_BACKWARD");
        }

        flatness_.feed(flatness_output, references[i].state,
                       references[i].input, heading_vec);
      } else if (trajectory.heading_mode ==
                 Trajectory::HeadingMode::HEADING_POLY) {
        heading_vec = Vector<3>::Zero();
        heading_vec(0) =
            trajectory.heading_poly.getPos(clamp_stamp - start_stamp)(0);
        heading_vec(1) =
            trajectory.heading_poly.getVel(clamp_stamp - start_stamp)(0);
        ROS_INFO_THROTTLE(1.0,
                          "Heading: %f, heading_rate: %f, heading_mode: %s",
                          heading_vec(0), heading_vec(1), "HEADING_POLY");
        flatness_.feed(flatness_output, references[i].state,
                       references[i].input, heading_vec);
      } else {
        flatness_.feed(flatness_output, references[i].state,
                       references[i].input);
      }
      stamp += horizon_dt_;
    }
    return true;
  }

  void getAdmittanceReference(
      const RobotState &robot_state,
      const AdmittanceControlSystem::Input &admittance_force,
      ReferenceVector &references, const Scalar &current_stamp,
      const RobotCommandMode &command_mode, bool init = false) {
    static AdmittanceControlSystem sys;
    static AdmittanceControlSystem::State init_state;
    static Scalar last_stamp = current_stamp;
    static RobotState hover_robot_state = RobotState(current_stamp);

    sys.updateParameter(
        fsm_parameter_->coeff_M, fsm_parameter_->coeff_D,
        fsm_parameter_->admittance_mode == AdmittanceMode::ZERO_FORCE
            ? Vector<4>::Zero()
            : fsm_parameter_->coeff_K,
        fsm_parameter_->admittance_mode == AdmittanceMode::ZERO_FORCE
            ? robot_state.pos
            : hover_robot_state.pos,
        fsm_parameter_->admittance_mode == AdmittanceMode::ZERO_FORCE
            ? robot_state.getYaw()
            : hover_robot_state.getYaw(), // current desired
        Vector<3>::Zero(), 0.0, Vector<3>::Zero(), 0.0,
        fsm_parameter_->admittance_max_vel, fsm_parameter_->admittance_max_acc);

    if (init) {
      hover_robot_state = robot_state.getHoverState();
      init_state = AdmittanceControlSystem::State(
          robot_state.pos, robot_state.getYaw(), Vector<3>::Zero(), 0.0);
    } else {
      init_state =
          sys.update(init_state, admittance_force, current_stamp - last_stamp);
    }
    last_stamp = current_stamp;
    AdmittanceControlSystem::State ref_state = init_state;
    references.resize(horizon_len_);
    Scalar stamp = current_stamp;
    ROS_INFO_STREAM_THROTTLE(
        1.0, "[admittance]"
                 << "\n  input: " << admittance_force.data.transpose()
                 << "\n  reference position: " << ref_state.pos.transpose()
                 << ", yaw: " << ref_state.yaw
                 << "\n  reference velocity: " << ref_state.vel.transpose()
                 << ", yaw_rate: " << ref_state.yaw_rate
                 << "\n  reference acc: "
                 << sys.getAcc(ref_state, admittance_force).transpose());
    Vector<4> general_acc;
    for (auto &reference : references) {
      //! Reference State
      general_acc = sys.getAcc(ref_state, admittance_force);
      reference.state = RobotState(stamp);
      reference.state.pos = ref_state.pos;
      reference.state.vel = ref_state.vel;
      reference.state.acc = general_acc.segment(0, 3);
      reference.state.setQ(ref_state.yaw);
      reference.state.omega.z() = ref_state.yaw_rate;
      reference.state.alpha.z() = general_acc(3);

      ref_state = sys.update(ref_state, admittance_force, horizon_dt_);

      //! Reference Input
      reference.input = RobotCommand(stamp, command_mode);
      stamp += horizon_dt_;

      // Hover input reference
      reference.input.collective_thrust = robot_parameter_->mass * kGravity;
      switch (command_mode) {
      case RobotCommandMode::MODE_SINGLE_ROTOR: {
        reference.input.rotor_thrusts =
            Vector<4>::Constant(1.0 / 4 * reference.input.collective_thrust);
        reference.input.rotor_velocities =
            (1 / robot_parameter_->thrust_coeff * reference.input.rotor_thrusts)
                .cwiseSqrt();
        break;
      }
      case RobotCommandMode::MODE_BODY_RATE: {
        reference.input.bodyrates = Vector<3>::Zero();
        break;
      }
      case RobotCommandMode::MODE_ATTITUDE: {
        reference.input.setAttitude(reference.state.getYaw());
        break;
      }
      default:
        break;
      }
    }
  }

private:
  void calculateHeading(ConstRef<Matrix<3, 5>> output,
                        const RobotState &robot_state, const Scalar dt,
                        Ref<Vector<3>> heading_vec, bool heading_forward,
                        bool init) {
    static Scalar last_heading = 0.0;
    static Scalar last_heading_rate = 0.0;
    static Scalar last_heading_acc = 0.0;

    auto &heading = heading_vec(0);
    auto &heading_rate = heading_vec(1);
    auto &heading_acc = heading_vec(2);

    if (init) {
      last_heading = robot_state.getYaw(0);
    }

    auto &W_p = output.col(0);
    auto &W_v = output.col(1);
    auto &W_a = output.col(2);

    // Calculate heading
    Scalar HEADING_RATE_MAX = robot_parameter_->bodyrates_max.z();
    Scalar heading_tmp;
    if ((W_v.x() * W_v.x() + W_v.y() * W_v.y()) > 1e-6) {
      heading_tmp = heading_forward ? std::atan2(W_v.y(), W_v.x())
                                    : std::atan2(-W_v.y(), -W_v.x());
    } else {
      heading_tmp = last_heading;
    }

    Scalar max_yaw_change = HEADING_RATE_MAX * dt;
    if (heading_tmp - last_heading > M_PI) {
      if (heading_tmp - last_heading - 2 * M_PI < -max_yaw_change) {
        heading = last_heading - max_yaw_change;
        if (heading < -M_PI)
          heading += 2 * M_PI;

        heading_rate = -HEADING_RATE_MAX;
      } else {
        heading = heading_tmp;
        if (heading - last_heading > M_PI)
          heading_rate = -HEADING_RATE_MAX;
        else
          heading_rate = (heading_rate - last_heading) / dt;
      }
    } else if (heading_tmp - last_heading < -M_PI) {
      if (heading_tmp - last_heading + 2 * M_PI > max_yaw_change) {
        heading = last_heading + max_yaw_change;
        if (heading > M_PI)
          heading -= 2 * M_PI;

        heading_rate = HEADING_RATE_MAX;
      } else {
        heading = heading_tmp;
        if (heading - last_heading < -M_PI)
          heading_rate = HEADING_RATE_MAX;
        else
          heading_rate = (heading_tmp - last_heading) / dt;
      }
    } else {
      if (heading_tmp - last_heading < -max_yaw_change) {
        heading = last_heading - max_yaw_change;
        if (heading < -M_PI)
          heading += 2 * M_PI;

        heading_rate = -HEADING_RATE_MAX;
      } else if (heading_tmp - last_heading > max_yaw_change) {
        heading = last_heading + max_yaw_change;
        if (heading > M_PI)
          heading -= 2 * M_PI;

        heading_rate = HEADING_RATE_MAX;
      } else {
        heading = heading_tmp;
        if (heading - last_heading > M_PI)
          heading_rate = -HEADING_RATE_MAX;
        else if (heading - last_heading < -M_PI)
          heading_rate = HEADING_RATE_MAX;
        else
          heading_rate = (heading_tmp - last_heading) / dt;
      }
    }

    if (fabs(heading - last_heading) <= max_yaw_change)
      heading = 0.5 * last_heading + 0.5 * heading; //
    heading_rate = 0.5 * last_heading_rate + 0.5 * heading_rate;
    heading_acc = (heading_rate - last_heading_rate) / dt;

    last_heading = heading;
    last_heading_rate = heading_rate;
    last_heading_acc = heading_acc;
  }

  RobotParameter::Ptr robot_parameter_;
  FsmParameter::Ptr fsm_parameter_;
  Flatness flatness_;
  int horizon_len_;
  Scalar horizon_dt_;
  Scalar position_error_thres_;
};
} // namespace flygripper_flight
