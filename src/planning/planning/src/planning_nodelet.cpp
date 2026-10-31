#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <env/free_space_grid.hpp>
#include <nav_msgs/Odometry.h>
#include <nodelet/nodelet.h>
#include <quadrotor_msgs/GraspState.h>
#include <quadrotor_msgs/PolyTraj.h>
#include <quadrotor_msgs/ReplanState.h>
#include <ros/package.h>
#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <traj_opt/traj_opt.h>

#include <Eigen/Core>
#include <atomic>
#include <cmath>
#include <env/env.hpp>
#include <prediction/prediction.hpp>
#include <planning/moving_grasp.hpp>
#include <chrono>
#include <thread>
#include <mutex>
#include <visualization/visualization.hpp>
#include <wr_msg/wr_msg.hpp>

// flygripper
#include "flygripper_msgs/PolynomialTrajectory.h"
#include "ros/console.h"
#include "ros/publisher.h"
#include "ros/subscriber.h"
#include "ros/time.h"
#include "ros/transport_hints.h"
#include "traj_opt/poly_traj_utils.hpp"
#include <flygripper_common/trajectory/polynomial_trajectory.hpp>
#include <flygripper_common/type/base.hpp>
#include <flygripper_common/type/trajectory.hpp>
#include <flygripper_common/utils/math.hpp>
#include <flygripper_common/utils/runtime_profiler.hpp>

#include <flygripper_msgs/ActuatorCommand.h>
#include <flygripper_msgs/FlightTrajectoryStatus.h>
#include <deque>

namespace planning {

Eigen::IOFormat CommaInitFmt(Eigen::StreamPrecision, Eigen::DontAlignCols, ", ",
                             ", ", "", "", " << ", ";");

class Nodelet : public nodelet::Nodelet {
private:
  std::thread initThread_;
  ros::Subscriber odom_sub_, target_sub_, target_twist_sub_, triger_sub_,
      land_triger_sub_, flight_status_sub_;
  ros::Timer plan_timer_;

  ros::Publisher traj_pub_, heartbeat_pub_, replanState_pub_, graspState_pub_,
      actuator_command_pub_, grasp_phase_pub_, pregrasp_target_pub_, grasp_alignment_pub_,
      grasp_point_mode_pub_;

  flygripper_common::RuntimeProfiler planning_total_profiler_{"planning_total",
                                                               50};
  flygripper_common::RuntimeProfiler prediction_profiler_{"target_prediction",
                                                           50};
  flygripper_common::RuntimeProfiler path_search_profiler_{"grasp_path_search",
                                                            50};
  flygripper_common::RuntimeProfiler visibility_profiler_{
      "visibility_region_construction", 50};
  flygripper_common::RuntimeProfiler corridor_profiler_{"corridor_generation",
                                                         50};
  flygripper_common::RuntimeProfiler optimization_profiler_{
      "trajectory_optimization", 50};

  std::shared_ptr<env::FreeSpaceGrid> gridmapPtr_;
  std::shared_ptr<env::Env> envPtr_;
  std::shared_ptr<visualization::Visualization> visPtr_;
  std::shared_ptr<traj_opt::TrajOpt> trajOptPtr_;
  std::shared_ptr<prediction::Predict> prePtr_;

  // NOTE planning or fake target
  bool fake_ = false;
  Eigen::Vector3d goal_;
  Eigen::Vector3d land_p_;
  Eigen::Quaterniond land_q_;

  // NOTE just for debug
  bool debug_ = false;
  quadrotor_msgs::ReplanState replanStateMsg_;
  quadrotor_msgs::GraspState graspStateMsg_;
  ros::Publisher flygripper_traj_pub_;

  double tracking_dur_, tracking_dist_, tolerance_d_, grasp_tole_dis_,
      grasp_start_dis_, grasp_position_, grasp_yaw_;

  Trajectory traj_poly_;
  ros::Time replan_stamp_;
  int traj_id_ = 0;
  bool wait_hover_ = true;
  bool landing_hold_active_ = false;
  Eigen::Vector3d landing_hold_position_ = Eigen::Vector3d::Zero();
  double landing_hold_yaw_ = 0.0;
  bool force_hover_ = true;
  int grasp_state_ = 0;
  double grasp_degree_ = 0;
  double pos_dx, pos_dy, pos_dz;
  double target_yaw_rate_ = 0.0;
  double target_yaw_filter_tau_ = 0.1;
  double target_yaw_rate_limit_ = 3.0;
  bool target_yaw_rate_valid_ = false;
  double grasp_vx_, grasp_vy_, grasp_vz_;
  double replan_head_time_;
  bool hold_yaw_on_trigger_ = true;
  bool hold_yaw_valid_ = false;
  double hold_yaw_ = 0.0;
  double post_grasp_lift_ = 0.0;
  double grasp_max_relative_speed_ = 0.10;
  moving_grasp::Sequence grasp_sequence_;
  moving_grasp::Direction car_heading_;
  bool pregrasp_enabled_ = false, pregrasp_heading_initialized_ = false;
  bool require_flight_status_ = true;
  bool grasp_point_mode_ = false, grasp_point_trajectory_published_ = false;
  double pregrasp_distance_ = 0.25;
  moving_grasp::State current_state_;
  double current_yaw_ = 0.0;
  std::vector<moving_grasp::YawPiece> heading_pieces_;
  ros::Time heading_stamp_;
  Eigen::Vector3d stop_target_ = Eigen::Vector3d::Zero();
  bool tracking_started_ = false;
  double target_receive_time_ = 0.0, odom_receive_time_ = 0.0;
  double target_twist_receive_time_ = 0.0;
  ros::Time heading_twist_stamp_;
  double grasp_close_duration_ = 0.85, grasp_gate_duration_ = 0.15;
  double grasp_open_duration_ = 0.85, grasp_open_position_ = 0.0;
  double flight_status_timeout_ = 0.20;
  double grasp_max_yaw_error_ = 0.174533;
  double grasp_abort_yaw_error_ = 0.785398;
  double grasp_abort_distance_ = 0.20, grasp_lift_abort_distance_ = 0.10;
  double grasp_abort_relative_speed_ = 0.50;
  double target_timeout_ = 0.20, odom_timeout_ = 0.20;
  double target_twist_timeout_ = 0.20, target_pose_twist_max_skew_ = 0.15;
  bool target_twist_check_header_age_ = false;
  bool velocity_from_pose_ = false;
  double target_velocity_cutoff_hz_ = 3.0;
  moving_grasp::VelocityFilter target_velocity_filter_; // protected by target_lock_
  double grasp_lift_duration_ = 2.0, grasp_follow_horizon_ = 0.50;
  double grasp_yaw_rate_max_ = 0.60, grasp_yaw_acc_max_ = 1.20;
  double grasp_yaw_speed_low_ = 0.05, grasp_yaw_speed_high_ = 0.08;
  double grasp_yaw_velocity_filter_hz_ = 2.0 / moving_grasp::kPi, grasp_yaw_direction_dwell_ = 0.25;
  double grasp_heading_offset_ = 0.0;
  double grasp_vmax_ = 0.40, grasp_amax_ = 0.40;
  bool velocity_heading_ = true;

  static double steadySeconds() {
    return std::chrono::duration<double>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
  }
  std::mutex planning_mutex_, dispatch_mutex_, flight_status_mutex_, trajectory_history_mutex_;
  std::atomic<uint64_t> trigger_generation_{0};
  uint64_t mission_generation_ = 0;
  bool opening_ = false, opening_hold_valid_ = false;
  bool stop_hold_enabled_ = false;
  bool mission_control_started_ = false;
  ros::Time bootstrap_end_;
  double opening_start_ = 0.0;
  ros::Time opening_brake_end_;
  std::atomic<bool> flight_required_{false}, flight_loss_seen_{false};
  flygripper_msgs::FlightTrajectoryStatus flight_status_;
  double flight_status_received_ = 0.0;
  bool flight_status_received_valid_ = false;
  struct SentTrajectory { uint32_t id; ros::Time start, end; };
  std::deque<SentTrajectory> mission_trajectories_;

  bool statusMatchesMission(const flygripper_msgs::FlightTrajectoryStatus& s,
                            const ros::Time& now) {
    std::lock_guard<std::mutex> history_lock(trajectory_history_mutex_);
    for (const auto& sent : mission_trajectories_)
      if (s.trajectory_id == sent.id && s.trajectory_start == sent.start &&
          s.header.stamp >= sent.start && s.header.stamp <= sent.end && now <= sent.end)
        return true;
    return false;
  }
  void clearMissionTrajectories() {
    std::lock_guard<std::mutex> history_lock(trajectory_history_mutex_);
    mission_trajectories_.clear();
  }
  void recordMissionTrajectory(uint32_t id, const ros::Time& start, double duration) {
    std::lock_guard<std::mutex> history_lock(trajectory_history_mutex_);
    mission_trajectories_.push_back({id, start, start + ros::Duration(duration)});
    while (mission_trajectories_.size() > 64) mission_trajectories_.pop_front();
  }
  void flightStatusCallback(const flygripper_msgs::FlightTrajectoryStatusConstPtr& msg) {
    std::lock_guard<std::mutex> lock(flight_status_mutex_);
    const double received = steadySeconds();
    const ros::Time now = ros::Time::now();
    const double stamp_age = (now - msg->header.stamp).toSec();
    if (flight_required_ && (!flight_status_received_valid_ ||
        received - flight_status_received_ > flight_status_timeout_ ||
        msg->header.stamp.isZero() || stamp_age < 0 || stamp_age > flight_status_timeout_ ||
        !msg->trajectory_active || !msg->fcu_state_fresh || !msg->fcu_armed ||
        !msg->fcu_guided_nogps || msg->fsm_state != msg->FSM_TRAJ ||
        msg->reference_type != msg->REF_TRAJ || !statusMatchesMission(*msg, now)))
      flight_loss_seen_ = true;
    flight_status_ = *msg;
    flight_status_received_ = received;
    flight_status_received_valid_ = true;
  }

  // Caller holds flight_status_mutex_; only trajectories sent in this task count.
  bool flightActiveLocked() {
    const auto& s = flight_status_;
    const ros::Time now = ros::Time::now();
    const double receive_age = steadySeconds() - flight_status_received_;
    const double stamp_age = (now - s.header.stamp).toSec();
    return flight_status_received_valid_ && !flight_loss_seen_ && s.trajectory_active &&
        s.fcu_state_fresh && s.fcu_armed && s.fcu_guided_nogps &&
        s.fsm_state == s.FSM_TRAJ && s.reference_type == s.REF_TRAJ &&
        !s.header.stamp.isZero() && receive_age >= 0 && receive_age <= flight_status_timeout_ &&
        stamp_age >= 0 && stamp_age <= flight_status_timeout_ && statusMatchesMission(s, now);
  }
  bool flightActive() {
    std::lock_guard<std::mutex> lock(flight_status_mutex_);
    return flightActiveLocked();
  }
  bool flightModeReady() {
    std::lock_guard<std::mutex> lock(flight_status_mutex_);
    const auto& s = flight_status_;
    const double received_age = steadySeconds() - flight_status_received_;
    const double stamp_age = (ros::Time::now() - s.header.stamp).toSec();
    return flight_status_received_valid_ && s.fcu_state_fresh && s.fcu_armed &&
        s.fcu_guided_nogps && s.fsm_state == s.FSM_TRAJ && !s.header.stamp.isZero() &&
        received_age >= 0 && received_age <= flight_status_timeout_ &&
        stamp_age >= 0 && stamp_age <= flight_status_timeout_;
  }
  bool publishMissionMorphing(double position, bool require_flight = false) {
    std::lock_guard<std::mutex> flight_lock(flight_status_mutex_);
    if (require_flight && !flightActiveLocked()) return false;
    std::lock_guard<std::mutex> dispatch_lock(dispatch_mutex_);
    if (trigger_generation_ != mission_generation_) return false;
    publishMorphing(position);
    if (require_flight) flight_required_ = true;
    return true;
  }

  nav_msgs::Odometry odom_msg_, target_msg_, target_body_msg_;
  geometry_msgs::TwistStamped target_twist_msg_;
  std::atomic_flag odom_lock_ = ATOMIC_FLAG_INIT;
  std::atomic_flag target_lock_ = ATOMIC_FLAG_INIT;
  std::atomic_bool odom_received_ = ATOMIC_VAR_INIT(false);
  std::atomic_bool triger_received_ = ATOMIC_VAR_INIT(false);
  std::atomic_bool target_received_ = ATOMIC_VAR_INIT(false);
  bool target_twist_received_ = false; // protected by target_lock_
  std::atomic_bool land_triger_received_ = ATOMIC_VAR_INIT(false);

  bool pub_hover_p(const Eigen::Vector3d &hover_p, const ros::Time &stamp) {
    std::lock_guard<std::mutex> lock(dispatch_mutex_);
    if (trigger_generation_ != mission_generation_) return false;
    quadrotor_msgs::PolyTraj traj_msg;
    Trajectory traj;
    traj_msg.hover = true;
    traj_msg.hover_p.resize(3);
    for (int i = 0; i < 3; ++i) {
      traj_msg.hover_p[i] = hover_p[i];
    }
    traj_msg.start_time = stamp;
    traj_msg.traj_id = traj_id_++;
    traj_msg.yaw = hold_yaw_;
    // The aircraft subscribes to /flygripper/trajectory, not position_cmd.
    // Send the hold to both consumers using the same position and heading.
    CoefficientMat coeff = CoefficientMat::Zero();
    coeff.col(5) = hover_p;
    Trajectory hold(std::vector<double>{1.0},
                    std::vector<CoefficientMat>{coeff});
    recordMissionTrajectory(static_cast<uint32_t>(traj_msg.traj_id), stamp,
                            hold.getTotalDuration());
    traj_pub_.publish(traj_msg);
    convertToFlygripperMsgs(hold, hold_yaw_, stamp, traj_msg);
    return true;
  }

  bool pub_traj(const Trajectory &traj, const double &yaw,
                const ros::Time &stamp,
                const std::vector<moving_grasp::YawPiece>* heading = nullptr) {
    std::lock_guard<std::mutex> lock(dispatch_mutex_);
    if (trigger_generation_ != mission_generation_) return false;
    quadrotor_msgs::PolyTraj traj_msg;
    traj_msg.hover = false;
    traj_msg.order = 5;
    Eigen::VectorXd durs = traj.getDurations();
    int piece_num = traj.getPieceNum();
    traj_msg.duration.resize(piece_num);
    traj_msg.coef_x.resize(6 * piece_num);
    traj_msg.coef_y.resize(6 * piece_num);
    traj_msg.coef_z.resize(6 * piece_num);
    for (int i = 0; i < piece_num; ++i) {
      traj_msg.duration[i] = durs(i);
      CoefficientMat cMat = traj[i].getCoeffMat();
      int i6 = i * 6;
      for (int j = 0; j < 6; j++) {
        traj_msg.coef_x[i6 + j] = cMat(0, j);
        traj_msg.coef_y[i6 + j] = cMat(1, j);
        traj_msg.coef_z[i6 + j] = cMat(2, j);
      }
    }
    traj_msg.start_time = stamp;
    traj_msg.traj_id = traj_id_++;
    // NOTE yaw
    traj_msg.yaw = heading ? yaw : (hold_yaw_on_trigger_ && hold_yaw_valid_ ? hold_yaw_ : yaw);
    recordMissionTrajectory(static_cast<uint32_t>(traj_msg.traj_id), stamp,
                            traj.getTotalDuration());
    traj_pub_.publish(traj_msg);
    convertToFlygripperMsgs(traj, traj_msg.yaw, stamp, traj_msg, heading);
    return true;
  }

  // todo@yinloonga:
  // 转换quadrotor_msgs::PolyTraj为flygripper_msgs::PolynomialTrajectory
  void convertToFlygripperMsgs(const Trajectory &traj, const double &yaw,
                               const ros::Time &stamp,
                               const quadrotor_msgs::PolyTraj &tracker_msg,
                               const std::vector<moving_grasp::YawPiece>* yaw_pieces = nullptr) {
    flygripper_msgs::PolynomialTrajectory flygripper_msg; // need to publish

    flygripper_msg.header.stamp = stamp;
    flygripper_msg.header.frame_id = "world";
    flygripper_msg.id = tracker_msg.traj_id;

    // tracker_msg是通过Min_Jerk规划的轨迹
    // ! xyz polynomial
    int xyz_pice_num = traj.getPieceNum();
    for (int i = 0; i < xyz_pice_num; ++i) {
      flygripper_msgs::Polynomial xyz_piece;
      xyz_piece.dim = traj[i].getDim();
      xyz_piece.order = 5;
      xyz_piece.duration = traj[i].getDuration();
      CoefficientMat cMat = traj[i].getCoeffMat();
      xyz_piece.coeff.assign(
          cMat.data(),
          cMat.data() +
              cMat.rows() * cMat.cols()); // ??@yinloonga: 注意矩阵的维度
      flygripper_msg.xyz_pieces.emplace_back(xyz_piece);
    }

    // ! heading
    if (yaw_pieces) {
      flygripper_msg.heading_mode = static_cast<int>(
          flygripper_common::Trajectory::HeadingMode::HEADING_POLY);
      for (const auto& piece : *yaw_pieces) {
        flygripper_msgs::Polynomial msg;
        msg.dim = 1; msg.order = 3; msg.duration = piece.duration;
        msg.coeff.assign(piece.c.data(), piece.c.data()+4);
        flygripper_msg.heading_pieces.push_back(msg);
      }
    } else if (hold_yaw_on_trigger_ || tracker_msg.hover) {
      flygripper_msg.heading_mode = static_cast<int>(
          flygripper_common::Trajectory::HeadingMode::HEADING_POLY);
      flygripper_msgs::Polynomial heading;
      heading.dim = 1;
      heading.order = 3;
      heading.duration = traj.getTotalDuration();
      heading.coeff = {0.0, 0.0, 0.0, yaw};
      flygripper_msg.heading_pieces.push_back(heading);
    } else {
      // Explicit opt-in to the legacy velocity-following heading.
      flygripper_msg.heading_mode = static_cast<int>(
          flygripper_common::Trajectory::HeadingMode::HEADING_FORWARD);
    }

    // 发布信息
    flygripper_traj_pub_.publish(flygripper_msg);

    // print flygripper_msg
    // std::cerr << "================> quadrotor_msgs::PolyTraj To
    // flygripper_msgs::PolynomialTrajectory<================" << std::endl;
    // std::cerr << "msg.id: " << flygripper_msg.id << std::endl;
    // std::cerr << "msg.header.stamp: " << flygripper_msg.header.stamp <<
    // std::endl; std::cerr << "\tpieces: " << flygripper_msg.xyz_pieces.size()
    // << std::endl;

    // // print xyz_pieces msgs
    // std::cerr << "msg.xyz_pieces[0]: " << std::endl;
    // std::cerr << "\tdim: " << flygripper_msg.xyz_pieces[0].dim << std::endl;
    // std::cerr << "\torder: " << flygripper_msg.xyz_pieces[0].order <<
    // std::endl; std::cerr << "\tduration: " <<
    // flygripper_msg.xyz_pieces[0].duration << std::endl; std::cerr <<
    // "\tCoeffMat: \n" << traj[0].getCoeffMat() << std::endl; std::cerr <<
    // "\tcoeff: "; for (const auto& coeff : flygripper_msg.xyz_pieces[0].coeff)
    // {
    //   std::cerr << coeff << " ";
    // }
    // std::cerr << std::endl;

    // // print heading_pieces
    // if (flygripper_msg.heading_mode ==
    // static_cast<int>(flygripper_common::Trajectory::HeadingMode::HEADING_POLY))
    // {
    //   std::cerr << "msh.heading_pieces: " << std::endl;
    //   std::cerr << "\tsize: " << flygripper_msg.heading_pieces.size() <<
    //   std::endl; std::cerr << "\tdim: " <<
    //   flygripper_msg.heading_pieces[0].dim << std::endl; std::cerr <<
    //   "\torer: " << flygripper_msg.heading_pieces[0].order << std::endl;
    //   std::cerr << "\tduration: " <<
    //   flygripper_msg.heading_pieces[0].duration << std::endl; std::cerr <<
    //   "\tcoeff: "; for (const auto &coeff :
    //   flygripper_msg.heading_pieces[0].coeff) {
    //     std::cerr << coeff << " ";
    //   }
    //   std::cerr << std::endl;
    // }
  }

  void triger_callback(const geometry_msgs::PoseStampedConstPtr &msgPtr) {
    uint64_t generation;
    double opened_at = 0.0;
    {
      // Do not wait for a potentially long optimization before opening.
      std::lock_guard<std::mutex> dispatch_lock(dispatch_mutex_);
      generation = ++trigger_generation_;
      if (generation > 1) {
        publishMorphing(grasp_open_position_);
        opened_at = steadySeconds();
      }
    }
    std::lock_guard<std::mutex> guard(planning_mutex_);
    if (generation != trigger_generation_) return;
    mission_generation_ = generation;
    mission_control_started_ = false;
    bootstrap_end_ = ros::Time();
    {
      std::lock_guard<std::mutex> status_lock(flight_status_mutex_);
      flight_required_ = false;
      flight_loss_seen_ = false;
      clearMissionTrajectories();
    }
    goal_ << msgPtr->pose.position.x, msgPtr->pose.position.y, 0.9;
    if (generation > 1) {
      const bool was_opening = opening_;
      opening_ = true;
      opening_start_ = opened_at;
      if (!was_opening) {
        const auto phase = grasp_sequence_.phase();
        if (phase == moving_grasp::Phase::BRAKING) {
          opening_brake_end_ = replan_stamp_ + ros::Duration(traj_poly_.getTotalDuration());
          opening_hold_valid_ = true;
        } else if (phase == moving_grasp::Phase::HOLD || phase == moving_grasp::Phase::ABORTED) {
          opening_brake_end_ = ros::Time::now();
          // The pilot may have left this old stop point in GUIDED mode.
          // Let the flight node retain its current reference while opening.
          opening_hold_valid_ = false;
        } else if (tracking_started_ && !force_hover_ && traj_poly_.getPieceNum() > 0) {
          startBraking(false, "Retrigger: open gripper and brake before restart");
          opening_brake_end_ = grasp_sequence_.phase() == moving_grasp::Phase::BRAKING ?
              replan_stamp_ + ros::Duration(traj_poly_.getTotalDuration()) : ros::Time::now();
          opening_hold_valid_ = true;
        } else {
          opening_brake_end_ = ros::Time::now();
          opening_hold_valid_ = false;
        }
      }
    } else {
      opening_ = false;
      force_hover_ = true;
      heading_pieces_.clear();
      hold_yaw_valid_ = false;
    }
    wait_hover_ = false;
    // Landing belongs to the previous mission. A new trigger must approach
    // the grasp point and allow the closing gate again.
    land_triger_received_ = false;
    landing_hold_active_ = false;
    grasp_state_ = 0;
    grasp_sequence_.reset();
    tracking_started_ = false;
    grasp_point_mode_ = false;
    grasp_point_trajectory_published_ = false;
    if (pregrasp_enabled_) {
      std_msgs::Bool mode;
      mode.data = false;
      grasp_point_mode_pub_.publish(mode);
    }
    pregrasp_heading_initialized_ = false;
    if (pregrasp_enabled_) car_heading_.resetEvidence();
    heading_twist_stamp_ = ros::Time();
    triger_received_ = true;
    publishGraspPhase();
  }

  void land_triger_callback(const geometry_msgs::PoseStampedConstPtr &msgPtr) {
    std::lock_guard<std::mutex> guard(planning_mutex_);
    if (grasp_sequence_.phase() != moving_grasp::Phase::APPROACH) return;
    land_p_.x() = msgPtr->pose.position.x;
    land_p_.y() = msgPtr->pose.position.y;
    land_p_.z() = msgPtr->pose.position.z;
    land_q_.w() = msgPtr->pose.orientation.w;
    land_q_.x() = msgPtr->pose.orientation.x;
    land_q_.y() = msgPtr->pose.orientation.y;
    land_q_.z() = msgPtr->pose.orientation.z;
    landing_hold_active_ = false;
    land_triger_received_ = true;
  }

  void odom_callback(const nav_msgs::Odometry::ConstPtr &msgPtr) {
    while (odom_lock_.test_and_set())
      ;
    odom_msg_ = *msgPtr;
    odom_receive_time_ = steadySeconds();
    odom_received_ = true;
    odom_lock_.clear();
  }

  void target_callback(const nav_msgs::Odometry::ConstPtr &msgPtr) {
    const auto &p = msgPtr->pose.pose.position;
    const auto &orientation = msgPtr->pose.pose.orientation;
    Eigen::Quaterniond target_q(orientation.w, orientation.x, orientation.y,
                                orientation.z);
    if (msgPtr->header.stamp.isZero() ||
        !Eigen::Vector3d(p.x, p.y, p.z).allFinite() ||
        !target_q.coeffs().allFinite() || target_q.squaredNorm() < 1e-12) {
      return;
    }
    target_q.normalize();
    const Eigen::Vector3d reference_p =
        Eigen::Vector3d(p.x, p.y, p.z) +
        target_q * Eigen::Vector3d(pos_dx, pos_dy, pos_dz);
    if (!reference_p.allFinite()) {
      return;
    }
    // Store pose and finite-difference velocity for the same grasp point.
    // Keep the tracked car orientation for direction-dependent planning.
    nav_msgs::Odometry reference_msg = *msgPtr;
    auto &reference_pose = reference_msg.pose.pose;
    reference_pose.position.x = reference_p.x();
    reference_pose.position.y = reference_p.y();
    reference_pose.position.z = reference_p.z();
    reference_pose.orientation.w = target_q.w();
    reference_pose.orientation.x = target_q.x();
    reference_pose.orientation.y = target_q.y();
    reference_pose.orientation.z = target_q.z();

    while (target_lock_.test_and_set())
      ;
    const double dt =
        (msgPtr->header.stamp - target_body_msg_.header.stamp).toSec();
    if (target_body_msg_.header.stamp.isZero() || dt > 0.5 || dt < 0.0 ||
        msgPtr->header.frame_id != target_body_msg_.header.frame_id) {
      // Need two consecutive grasp-reference positions before predicting motion.
      target_msg_ = reference_msg;
      target_body_msg_ = *msgPtr;
      target_yaw_rate_ = 0.0;
      target_yaw_rate_valid_ = false;
      target_received_ = false;
      if (velocity_from_pose_) {
        target_velocity_filter_.reset();
        target_twist_received_ = false;
      }
    } else if (dt > 1e-6) {
      // The pose stream supplies orientation/yaw rate. Linear velocity is
      // read independently from the target_twist topic below.
      const auto previous_orientation = target_msg_.pose.pose.orientation;
      const auto previous_origin = target_body_msg_.pose.pose.position;
      target_msg_ = reference_msg;
      target_body_msg_ = *msgPtr;
      if (velocity_from_pose_) {
        // Compatibility only for generic simulation launch files. The real
        // grasping.launch explicitly selects the independent twist source.
        geometry_msgs::TwistStamped derived;
        derived.header = msgPtr->header;
        derived.twist.linear.x = (p.x-previous_origin.x)/dt;
        derived.twist.linear.y = (p.y-previous_origin.y)/dt;
        derived.twist.linear.z = (p.z-previous_origin.z)/dt;
        acceptTargetTwist(derived, steadySeconds());
      }

      const Eigen::Quaterniond previous_q(
          previous_orientation.w, previous_orientation.x,
          previous_orientation.y, previous_orientation.z);
      const Eigen::Vector3d forward = target_q * Eigen::Vector3d::UnitX();
      const Eigen::Vector3d previous_forward =
          previous_q * Eigen::Vector3d::UnitX();
      const double yaw = std::atan2(forward.y(), forward.x());
      const double previous_yaw =
          std::atan2(previous_forward.y(), previous_forward.x());
      const double measured_yaw_rate =
          std::remainder(yaw - previous_yaw, 2.0 * M_PI) / dt;
      if (std::isfinite(measured_yaw_rate) &&
          std::abs(measured_yaw_rate) <= target_yaw_rate_limit_) {
        const double alpha = dt / (target_yaw_filter_tau_ + dt);
        target_yaw_rate_ = target_yaw_rate_valid_
                               ? target_yaw_rate_ +
                                     alpha * (measured_yaw_rate - target_yaw_rate_)
                               : measured_yaw_rate;
        target_yaw_rate_valid_ = true;
      } else {
        target_yaw_rate_ = 0.0;
        target_yaw_rate_valid_ = false;
      }
      target_received_ = true;
      target_receive_time_ = steadySeconds();
    }
    target_lock_.clear();
  }

  // Caller holds target_lock_; both real twist and simulated pose differences
  // enter here. Keep the unfiltered message for freshness and safety gates.
  void acceptTargetTwist(const geometry_msgs::TwistStamped& msg, double receipt) {
    const auto& v = msg.twist.linear;
    if (msg.header.stamp.isZero() || !Eigen::Vector3d(v.x,v.y,v.z).allFinite()) return;
    // A repeated, out-of-order, or invalid timestamp must not refresh age.
    if (target_twist_received_ && msg.header.stamp <= target_twist_msg_.header.stamp) return;
    if (msg.header.frame_id != target_twist_msg_.header.frame_id) target_velocity_filter_.reset();
    if (target_velocity_filter_.update({v.x,v.y,v.z}, msg.header.stamp.toSec(), receipt,
                                      target_velocity_cutoff_hz_, target_twist_timeout_)) {
      target_twist_msg_ = msg;
      target_twist_receive_time_ = receipt;
      target_twist_received_ = true;
    }
  }

  void target_twist_callback(const geometry_msgs::TwistStamped::ConstPtr &msg) {
    if (velocity_from_pose_) return;
    const double receipt = steadySeconds();
    while (target_lock_.test_and_set()) {}
    acceptTargetTwist(*msg, receipt);
    target_lock_.clear();
  }

  void publishMorphing(double position) {
    flygripper_msgs::ActuatorCommand actuator_command_msg;
    actuator_command_msg.header.stamp = ros::Time::now();
    actuator_command_msg.header.frame_id = "robot";
    actuator_command_msg.mode = flygripper_msgs::ActuatorCommand::MODE_POSITION;
    actuator_command_msg.reference_velocity = 0.0;
    actuator_command_msg.reference_effort = 0.0;
    actuator_command_msg.reference_position = position;
    actuator_command_pub_.publish(actuator_command_msg);
  }

  void publishGraspPhase() {
    std_msgs::String phase;
    phase.data = opening_ ? "opening_assumed" : moving_grasp::phaseName(grasp_sequence_.phase());
    grasp_phase_pub_.publish(phase);
    // Compatibility output for visualization; it is never a control input.
    grasp_state_ = grasp_sequence_.closeSent() ? 1 : 0;
    graspStateMsg_.header.stamp = ros::Time::now();
    graspStateMsg_.header.frame_id = "world";
    graspStateMsg_.replan_stamp = replan_stamp_;
    graspStateMsg_.graspState = grasp_state_;
    graspStateMsg_.graspDegree = grasp_sequence_.closeSent() ? 1.0 : 0.0;
    graspState_pub_.publish(graspStateMsg_);
  }

  moving_grasp::State referenceAt(const ros::Time& stamp) const {
    moving_grasp::State state = current_state_;
    if (!force_hover_ && traj_poly_.getPieceNum() > 0) {
      const double t = (stamp-replan_stamp_).toSec();
      if (t >= 0.0 && t <= traj_poly_.getTotalDuration()) {
        state.p = traj_poly_.getPos(t);
        state.v = traj_poly_.getVel(t);
        state.a = traj_poly_.getAcc(t);
      }
    }
    return state;
  }

  void publishMotion(const Trajectory& traj, const ros::Time& stamp,
                     double desired_yaw, double desired_rate) {
    Eigen::Vector2d initial(current_yaw_, 0.0);
    if (!heading_pieces_.empty()) {
      initial = moving_grasp::sampleHeading(heading_pieces_, (stamp-heading_stamp_).toSec());
    }
    auto heading = moving_grasp::heading(initial[0], initial[1], desired_yaw,
        desired_rate, traj.getTotalDuration(), grasp_yaw_rate_max_, grasp_yaw_acc_max_);
    const double terminal_yaw = moving_grasp::sampleHeading(heading, traj.getTotalDuration())[0];
    if (!pub_traj(traj, terminal_yaw, stamp, &heading)) return;
    heading_pieces_ = std::move(heading);
    heading_stamp_ = stamp;
    traj_poly_ = traj;
    replan_stamp_ = stamp;
    force_hover_ = false;
    if (grasp_point_mode_ && !land_triger_received_ &&
        grasp_sequence_.phase() == moving_grasp::Phase::APPROACH)
      grasp_point_trajectory_published_ = true;
    replanStateMsg_.header.stamp = ros::Time::now();
    replanStateMsg_.state = grasp_sequence_.phase() == moving_grasp::Phase::BRAKING ? 2 : 0;
    replanStateMsg_.iniState.resize(9);
    Eigen::Map<Eigen::Matrix<double,3,3>> initial_state(replanStateMsg_.iniState.data());
    initial_state.col(0)=traj.getPos(0.0);
    initial_state.col(1)=traj.getVel(0.0);
    initial_state.col(2)=traj.getAcc(0.0);
    replanState_pub_.publish(replanStateMsg_);
  }

  bool motionWithinLimits(const Trajectory& traj, double vmax, double amax) const {
    for (int i = 0; i < traj.getPieceNum(); ++i) {
      if (!traj[i].getCoeffMat().allFinite() ||
          traj[i].getMaxVelRate() > vmax+1e-5 ||
          traj[i].getMaxAccRate() > amax+1e-5) return false;
    }
    return traj.getPieceNum() > 0;
  }

  bool predictAt(const nav_msgs::Odometry& body, double turn_rate,
                 const ros::Time& stamp, moving_grasp::State& state) const {
    const auto& p = body.pose.pose.position;
    const auto& v = body.twist.twist.linear;
    const auto& q = body.pose.pose.orientation;
    const Eigen::Vector3d offset(pos_dx, pos_dy, pos_dz);
    std::vector<prediction::TargetState> prediction;
    if (!prediction::Predict::predictRigidBodyModel({p.x,p.y,p.z}, {v.x,v.y,v.z},
          Eigen::Quaterniond(q.w,q.x,q.y,q.z), turn_rate, offset,
          std::max(0.0, (stamp-body.header.stamp).toSec()), 0.01, 0.01, 4.0, prediction)) return false;
    const auto& sample = prediction.front();
    state.p = sample.p; state.v = sample.v;
    const Eigen::Vector3d omega(0,0,turn_rate);
    // Under the constant-turn-rate model, a = omega x v at the grasp point.
    state.a = omega.cross(sample.v);
    return true;
  }

  bool followTrajectory(const nav_msgs::Odometry& body, double turn_rate,
                        const ros::Time& stamp, double monotonic_now,
                        Trajectory& result) const {
    const auto initial = referenceAt(stamp);
    for (double horizon = grasp_follow_horizon_; horizon <= std::max(1.5, grasp_follow_horizon_)+1e-6;
         horizon += 0.25) {
      moving_grasp::State end;
      if (!predictAt(body, turn_rate, stamp+ros::Duration(horizon), end)) return false;
      if (grasp_sequence_.phase() == moving_grasp::Phase::LIFTING) {
        const auto dz = moving_grasp::lift(grasp_sequence_.liftElapsed(monotonic_now) +
            std::max(0.0, (stamp-ros::Time::now()).toSec()) + horizon,
            grasp_lift_duration_, post_grasp_lift_);
        end.p.z() += dz[0]; end.v.z() += dz[1]; end.a.z() += dz[2];
      }
      result = Trajectory({horizon}, {moving_grasp::quintic(initial, end, horizon)});
      if (motionWithinLimits(result, grasp_vmax_, grasp_amax_)) return true;
    }
    return false;
  }

  void startBraking(bool abort, const char* reason) {
    const auto phase = grasp_sequence_.phase();
    if (phase == moving_grasp::Phase::BRAKING || phase == moving_grasp::Phase::HOLD ||
        phase == moving_grasp::Phase::ABORTED) return;
    grasp_sequence_.brake(abort);
    stop_hold_enabled_ = true;
    ROS_WARN("[planner] %s; controlled brake then %s (new trigger required)",
             reason, abort ? "abort hold" : "hold");
    const ros::Time stamp = ros::Time::now()+ros::Duration(replan_head_time_);
    const auto initial = referenceAt(stamp);
    Eigen::Vector2d initial_heading(current_yaw_,0.0);
    if (!heading_pieces_.empty())
      initial_heading = moving_grasp::sampleHeading(heading_pieces_, (stamp-heading_stamp_).toSec());
    double duration = std::max({1.0, 2.0*initial.v.norm()/grasp_amax_,
                               4.0*std::abs(initial_heading[1])/grasp_yaw_acc_max_});
    Trajectory brake;
    bool valid = false;
    for (int attempt=0; attempt<12; ++attempt, duration*=1.3) {
      moving_grasp::State end;
      end.p = initial.p + initial.v*duration/2.0 + initial.a*duration*duration/12.0;
      brake = Trajectory({duration}, {moving_grasp::quintic(initial, end, duration)});
      // Do not demand an instantaneous reduction if the existing reference is
      // already outside the limits; braking starts at the same p/v/a.
      if (motionWithinLimits(brake, std::max(grasp_vmax_,initial.v.norm()+1e-4),
                                   std::max(grasp_amax_,initial.a.norm()+1e-4))) {
        stop_target_ = end.p; valid = true; break;
      }
    }
    if (!valid) {
      ROS_ERROR("[planner] No feasible braking polynomial; requesting position hold");
      stop_target_ = initial.p;
      hold_yaw_ = current_yaw_;
      pub_hover_p(stop_target_, stamp);
      grasp_sequence_.stopped();
    } else {
      publishMotion(brake, stamp, initial_heading[0], 0.0);
    }
    publishGraspPhase();
  }

  bool maintainStop() {
    const auto phase = grasp_sequence_.phase();
    if (phase != moving_grasp::Phase::BRAKING && phase != moving_grasp::Phase::HOLD &&
        phase != moving_grasp::Phase::ABORTED) return false;
    if (grasp_sequence_.closeSent()) publishMissionMorphing(grasp_position_);
    const bool flight_mode_ready = (!require_flight_status_ && !flight_required_) || flightModeReady();
    // Once trajectory control is left, never resume an old task's stop point
    // solely because TRAJ becomes active again. A new trigger must reset it.
    if (!flight_mode_ready) stop_hold_enabled_ = false;
    if (phase == moving_grasp::Phase::BRAKING &&
        (ros::Time::now()-replan_stamp_).toSec() >= traj_poly_.getTotalDuration()) {
      hold_yaw_ = moving_grasp::sampleHeading(heading_pieces_, traj_poly_.getTotalDuration())[0];
      grasp_sequence_.stopped();
    }
    if (grasp_sequence_.phase() != moving_grasp::Phase::BRAKING &&
        stop_hold_enabled_ && flight_mode_ready)
      pub_hover_p(stop_target_, ros::Time::now()+ros::Duration(replan_head_time_));
    publishGraspPhase();
    return true;
  }

  void plan_timer_callback(const ros::TimerEvent&) {
    std::unique_lock<std::mutex> guard(planning_mutex_, std::try_to_lock);
    if (!guard.owns_lock()) return;
    heartbeat_pub_.publish(std_msgs::Empty());
    if (!triger_received_ || trigger_generation_ != mission_generation_) return;
    if (opening_) {
      publishMissionMorphing(grasp_open_position_);
      const ros::Time now = ros::Time::now();
      if (opening_hold_valid_ && now >= opening_brake_end_) {
        if (!heading_pieces_.empty() && traj_poly_.getPieceNum() > 0)
          hold_yaw_ = moving_grasp::sampleHeading(heading_pieces_, traj_poly_.getTotalDuration())[0];
        pub_hover_p(stop_target_, now + ros::Duration(replan_head_time_));
      }
      if (steadySeconds() - opening_start_ < grasp_open_duration_ || now < opening_brake_end_) {
        publishGraspPhase();
        return;
      }
      opening_ = false;
      clearMissionTrajectories();
      force_hover_ = true;
      heading_pieces_.clear();
      hold_yaw_valid_ = false;
      publishGraspPhase();
    }
    // A latched abort cannot be cancelled by a late status message or fresh target.
    if (maintainStop()) return;
    const double monotonic_now = steadySeconds();
    const ros::Time now = ros::Time::now();
    while (odom_lock_.test_and_set()) {}
    const auto odom = odom_msg_;
    const double odom_receipt = odom_receive_time_;
    odom_lock_.clear();
    const auto& op = odom.pose.pose.position;
    const auto& ov = odom.twist.twist.linear;
    const auto& oq = odom.pose.pose.orientation;
    const Eigen::Vector3d odom_p(op.x,op.y,op.z), odom_v(ov.x,ov.y,ov.z);
    Eigen::Quaterniond odom_q(oq.w,oq.x,oq.y,oq.z);
    if (!odom_received_ || !odom_p.allFinite() || !odom_v.allFinite() ||
        !odom_q.coeffs().allFinite() || odom_q.norm()<1e-6 ||
        !moving_grasp::fresh(odom.header.stamp.toSec(), now.toSec(), steadySeconds()-odom_receipt, odom_timeout_)) {
      if (tracking_started_) startBraking(true, "Odometry invalid or stale");
      return;
    }
    current_state_.p = odom_p; current_state_.v = odom_v;
    current_state_.a.setZero();
    const auto forward = odom_q.normalized()*Eigen::Vector3d::UnitX();
    current_yaw_ = std::atan2(forward.y(), forward.x());
    if (!hold_yaw_valid_) {
      hold_yaw_ = current_yaw_; hold_yaw_valid_ = true;
      if (!pregrasp_enabled_) car_heading_.reset(current_yaw_);
    }

    while (target_lock_.test_and_set()) {}
    auto body = target_body_msg_;
    const auto twist = target_twist_msg_;
    const Eigen::Vector3d filtered_twist_v = target_velocity_filter_.value();
    replanStateMsg_.target = target_msg_;
    const double turn_rate = target_yaw_rate_;
    const double target_receipt = target_receive_time_;
    const double twist_receipt = target_twist_receive_time_;
    const bool received = target_received_;
    const bool twist_received = target_twist_received_;
    target_lock_.clear();
    auto twist_fresh = [&](double wall, double steady) {
      const double age = steady - twist_receipt;
      return twist_received && age >= 0.0 && age <= target_twist_timeout_ &&
          (!target_twist_check_header_age_ ||
           moving_grasp::fresh(twist.header.stamp.toSec(), wall, age, target_twist_timeout_));
    };
    const bool compatible_frames = body.header.frame_id.empty() || twist.header.frame_id.empty() ||
                                   body.header.frame_id == twist.header.frame_id;
    if (!received || !moving_grasp::fresh(body.header.stamp.toSec(), now.toSec(),
                                         steadySeconds()-target_receipt, target_timeout_) ||
        !twist_fresh(now.toSec(), steadySeconds()) || !compatible_frames ||
        std::abs(target_receipt-twist_receipt) > target_pose_twist_max_skew_) {
      grasp_sequence_.ready(monotonic_now, false, grasp_gate_duration_);
      if (tracking_started_) startBraking(true, "Target pose/velocity invalid, stale, or unsynchronized");
      return;
    }
    // The installed VRPN driver may use different clock offsets for pose and
    // twist headers. Approximate their relative sample time by local receipt
    // time, then rotate velocity back to the pose sample under the same
    // constant-turn model used by the predictor. Both streams remain bounded
    // by independent freshness and receipt-skew checks.
    const Eigen::AngleAxisd align_velocity(
        -turn_rate*(twist_receipt-target_receipt), Eigen::Vector3d::UnitZ());
    const Eigen::Vector3d pose_v = align_velocity * filtered_twist_v;
    body.twist.twist.linear.x = pose_v.x();
    body.twist.twist.linear.y = pose_v.y();
    body.twist.twist.linear.z = pose_v.z();
    auto snapshot_current = [&]() {
      const double wall = ros::Time::now().toSec(), steady = steadySeconds();
      return moving_grasp::fresh(body.header.stamp.toSec(),wall,steady-target_receipt,target_timeout_) &&
             twist_fresh(wall,steady) &&
             moving_grasp::fresh(odom.header.stamp.toSec(),wall,steady-odom_receipt,odom_timeout_);
    };
    moving_grasp::State target;
    // Use the same filtered velocity for prediction and all grasp checks.
    if (!predictAt(body, turn_rate, now, target)) {
      if (mission_control_started_) startBraking(true, "Invalid target prediction");
      return;
    }
    replanStateMsg_.target.twist.twist.linear.x = target.v.x();
    replanStateMsg_.target.twist.twist.linear.y = target.v.y();
    replanStateMsg_.target.twist.twist.linear.z = target.v.z();
    const double distance = (target.p-odom_p).norm();
    const double relative_speed = (target.v-odom_v).norm();
    if (pregrasp_enabled_ && velocity_heading_ && !pregrasp_heading_initialized_) {
      // Keep a previously confirmed motion direction across retriggers. If no
      // motion has been confirmed, initialize from aircraft to grasp point.
      if (!car_heading_.hasConfirmedDirection())
        car_heading_.reset(moving_grasp::initialApproachYaw(odom_p, target.p, current_yaw_));
      pregrasp_heading_initialized_ = true;
    }
    if (distance>5.0) {
      if (mission_control_started_) startBraking(true,"Target position jump/distance exceeds 5 m");
      return;
    }
    if (!mission_control_started_) {
      if (require_flight_status_) {
        // For real car grasp, establish a trajectory from the current state
        // and wait until the flight node confirms this mission's trajectory.
        if (!flightModeReady()) return;
        {
          std::lock_guard<std::mutex> lock(flight_status_mutex_);
          if (flightActiveLocked()) {
            flight_required_ = true;
            mission_control_started_ = true;
          }
        }
        if (!mission_control_started_) {
          if (now >= bootstrap_end_ && odom_v.norm() <= grasp_vmax_) {
            const ros::Time stamp = now + ros::Duration(replan_head_time_);
            CoefficientMat coeff = CoefficientMat::Zero();
            coeff.col(5) = odom_p + odom_v * replan_head_time_;
            coeff.col(4) = odom_v;
            Trajectory bootstrap({0.5}, {coeff});
            publishMotion(bootstrap, stamp, current_yaw_, 0.0);
            bootstrap_end_ = stamp + ros::Duration(bootstrap.getTotalDuration());
          }
          return;
        }
      } else {
        // Generic planning has no flight-status publisher; trajectory planning
        // can proceed, while the closing gate below still requires an ACK.
        mission_control_started_ = true;
      }
    } else if (require_flight_status_ && !flightActive()) {
      startBraking(true, "Flight trajectory control lost during approach");
      return;
    }

    tracking_started_ = true;
    if (pregrasp_enabled_ && !land_triger_received_ && !grasp_point_mode_ && distance <= 0.30) {
      grasp_point_mode_ = true;
      std_msgs::Bool mode;
      mode.data = true;
      grasp_point_mode_pub_.publish(mode);
      ROS_INFO("[planner] Grasp point mode latched at %.3f m until next trigger", distance);
    }
    // Heading uses the 3 Hz filtered world-frame origin velocity. The car
    // body's yaw rate remains part of the position predictor only.
    if (velocity_heading_ && twist.header.stamp != heading_twist_stamp_) {
      car_heading_.update(filtered_twist_v.x(),filtered_twist_v.y(),grasp_yaw_speed_low_,
                          grasp_yaw_speed_high_,grasp_heading_offset_,twist_receipt,
                          grasp_yaw_velocity_filter_hz_,grasp_yaw_direction_dwell_,
                          grasp_yaw_rate_max_);
      heading_twist_stamp_ = twist.header.stamp;
    } else if (!velocity_heading_) car_heading_.reset(hold_yaw_);
    const bool heading_ready = !velocity_heading_ ||
        car_heading_.readyForGrasp(filtered_twist_v.head<2>().norm(),grasp_yaw_speed_low_,grasp_yaw_speed_high_);
    const double yaw_error = std::abs(moving_grasp::angleError(car_heading_.yaw(),current_yaw_));
    if (pregrasp_enabled_) {
      geometry_msgs::Vector3Stamped alignment;
      alignment.header.stamp = now;
      alignment.header.frame_id = "world";
      alignment.vector.x = distance;
      alignment.vector.y = relative_speed;
      alignment.vector.z = yaw_error;
      grasp_alignment_pub_.publish(alignment);
    }
    const auto phase = grasp_sequence_.phase();
    if (phase == moving_grasp::Phase::CLOSING || phase == moving_grasp::Phase::LIFTING) {
      if (!flightActive()) { startBraking(true, "Flight trajectory control lost"); return; }
      Eigen::Vector3d expected = target.p;
      if (phase == moving_grasp::Phase::LIFTING)
        expected.z() += moving_grasp::lift(grasp_sequence_.liftElapsed(monotonic_now),
                                           grasp_lift_duration_,post_grasp_lift_)[0];
      // Lifting intentionally has vertical relative motion; check horizontal speed then.
      const double speed_error = phase == moving_grasp::Phase::CLOSING ? relative_speed :
                                  (target.v-odom_v).head<2>().norm();
      const double position_limit = phase == moving_grasp::Phase::LIFTING ?
                                    grasp_lift_abort_distance_ : grasp_abort_distance_;
      if ((expected-odom_p).norm()>position_limit || speed_error>grasp_abort_relative_speed_ ||
          yaw_error>grasp_abort_yaw_error_) {
        startBraking(true,"Tracking error during closing/lift"); return;
      }
      const auto old_phase = grasp_sequence_.phase();
      if (grasp_sequence_.update(monotonic_now,grasp_close_duration_,grasp_lift_duration_)) {
        startBraking(false,"Timed lift complete (grasp assumed)"); return;
      }
      if (old_phase != grasp_sequence_.phase())
        ROS_INFO("[planner] Close timer complete; lifting while following the car");
      const ros::Time stamp = ros::Time::now()+ros::Duration(replan_head_time_);
      Trajectory follow;
      if (!followTrajectory(body,turn_rate,stamp,monotonic_now,follow) || !snapshot_current()) {
        startBraking(true,"Follow trajectory exceeds motion limits"); return;
      }
      if (!publishMissionMorphing(grasp_position_, true)) {
        if (trigger_generation_ == mission_generation_) startBraking(true, "Flight trajectory control lost");
        return;
      } // Does not restart the close timer.
      publishMotion(follow,stamp,car_heading_.yaw()+car_heading_.rate()*(stamp-now).toSec(),car_heading_.rate());
      publishGraspPhase();
      return;
    }

    const bool gates = !land_triger_received_ && flightActive() && heading_ready && distance<=grasp_start_dis_ &&
                       relative_speed<=grasp_max_relative_speed_ && yaw_error<=grasp_max_yaw_error_;
    if (grasp_sequence_.ready(monotonic_now,gates,grasp_gate_duration_)) {
      const ros::Time stamp = ros::Time::now()+ros::Duration(replan_head_time_);
      Trajectory follow;
      if (!followTrajectory(body,turn_rate,stamp,monotonic_now,follow) || !snapshot_current()) {
        startBraking(true,"Cannot start a feasible closing trajectory"); return;
      }
      if (!publishMissionMorphing(grasp_position_, true)) {
        grasp_sequence_.ready(steadySeconds(), false, grasp_gate_duration_);
        return;
      }
      grasp_sequence_.close(steadySeconds()); // First actual close publication only.
      publishMotion(follow,stamp,car_heading_.yaw()+car_heading_.rate()*(stamp-now).toSec(),car_heading_.rate());
      publishGraspPhase();
      ROS_INFO("[planner] Closing for %.3f s while tracking; no gripper feedback required",grasp_close_duration_);
      return;
    }
    publishGraspPhase();
    // Near the grasp point, maintain velocity matching while the yaw/dwell
    // gates settle. Avoid forcing a new corridor search for a near-zero path.
    if (!land_triger_received_ && (pregrasp_enabled_ ? grasp_point_mode_ :
                                    distance <= std::max(0.30,grasp_abort_distance_))) {
      const ros::Time stamp = ros::Time::now()+ros::Duration(replan_head_time_);
      Trajectory follow;
      if (followTrajectory(body,turn_rate,stamp,monotonic_now,follow) && snapshot_current()) {
        publishMotion(follow,stamp,car_heading_.yaw()+car_heading_.rate()*(stamp-now).toSec(),car_heading_.rate());
        return;
      }
    }
    planApproach(body,turn_rate,odom,now,twist,twist_receipt);
  }

  void planApproach(const nav_msgs::Odometry& body, double turn_rate,
                    const nav_msgs::Odometry& odom, const ros::Time& observation_time,
                    const geometry_msgs::TwistStamped& twist, double twist_receipt) {
    flygripper_common::ScopedRuntimeSample profile(planning_total_profiler_);
    const ros::Time stamp = ros::Time::now()+ros::Duration(replan_head_time_);
    const auto initial = referenceAt(stamp);
    Eigen::MatrixXd iniState(3,3);
    iniState.col(0)=initial.p; iniState.col(1)=initial.v; iniState.col(2)=initial.a;
    const auto& p = body.pose.pose.position;
    const auto& v = body.twist.twist.linear;
    const auto& q = body.pose.pose.orientation;
    Eigen::Vector3d offset(pos_dx,pos_dy,pos_dz);
    if (land_triger_received_) offset += land_p_;
    std::vector<prediction::TargetState> states;
    std::vector<Eigen::Vector3d> targets, way_pts, path, visible, grasp_ps;
    std::vector<Eigen::Quaterniond> orientations;
    std::vector<double> thetas;
    Eigen::Vector3d terminal_target = Eigen::Vector3d::Zero();
    bool ok;
    {
      flygripper_common::ScopedRuntimeSample sample(prediction_profiler_);
      ok = prePtr_->predictRigidBody({p.x,p.y,p.z},{v.x,v.y,v.z},
          Eigen::Quaterniond(q.w,q.x,q.y,q.z),turn_rate,offset,
          std::max(0.0,(stamp-body.header.stamp).toSec()),states);
    }
    if (ok) {
      for (const auto& state:states) {
        Eigen::Vector3d point = state.p;
        if (pregrasp_enabled_ && !grasp_point_mode_ && !land_triger_received_) {
          // Heading includes the optional aircraft yaw offset; the circle is
          // aligned with velocity itself, not that offset.
          const double direction = car_heading_.yaw() -
              (car_heading_.hasConfirmedDirection() ? grasp_heading_offset_ : 0.0) +
              car_heading_.rate() * ((stamp-observation_time).toSec() + state.t);
          point = moving_grasp::pregraspPoint(state.p, direction, pregrasp_distance_);
        }
        targets.push_back(point);
        orientations.push_back(state.q);
      }
      if (!targets.empty()) terminal_target = targets.back();
      if (pregrasp_enabled_ && !grasp_point_mode_ && !land_triger_received_ && !states.empty()) {
        const auto& end = states.back();
        const double yaw = car_heading_.yaw() + car_heading_.rate() *
            ((stamp-observation_time).toSec() + end.t);
        nav_msgs::Odometry goal;
        goal.header.frame_id = "world";
        goal.header.stamp = stamp + ros::Duration(end.t);
        goal.pose.pose.position.x = targets.back().x();
        goal.pose.pose.position.y = targets.back().y();
        goal.pose.pose.position.z = targets.back().z();
        goal.pose.pose.orientation.z = std::sin(yaw/2.0);
        goal.pose.pose.orientation.w = std::cos(yaw/2.0);
        goal.twist.twist.linear.x = end.v.x();
        goal.twist.twist.linear.y = end.v.y();
        goal.twist.twist.linear.z = end.v.z();
        goal.twist.twist.angular.z = car_heading_.rate();
        pregrasp_target_pub_.publish(goal);
        std::vector<Eigen::Vector3d> circle;
        for (int i=0; i<=40; ++i) {
          const double a = 2.0 * moving_grasp::kPi * i / 40.0;
          circle.push_back(end.p + pregrasp_distance_ * Eigen::Vector3d(std::cos(a),std::sin(a),0.0));
        }
        const std::vector<Eigen::Vector3d> diameter{targets.back(), 2.0*end.p-targets.back()};
        visPtr_->visualize_path(circle, "pregrasp_circle");
        visPtr_->visualize_path(diameter, "pregrasp_direction");
      }
      if (land_triger_received_ && (targets.front()-current_state_.p).norm()<0.1 &&
          current_state_.v.norm()<0.1 && states.front().v.norm()<0.2) {
        if (!landing_hold_active_) {
          landing_hold_position_ = current_state_.p;
          // Capture the heading of the executing reference at the switch,
          // rather than the heading from the original mission trigger.
          landing_hold_yaw_ = heading_pieces_.empty() ? current_yaw_ :
              moving_grasp::sampleHeading(heading_pieces_, (stamp-heading_stamp_).toSec())[0];
          landing_hold_active_ = true;
        }
        // A hold lasts one second on the flight node. Refresh it while at the
        // landing point. Keep the heading polynomial so both yaw and yaw rate
        // are continuous across entry, refreshes, and later replanning/abort.
        // A nonzero entry yaw rate settles to zero under the normal limits.
        CoefficientMat coeff = CoefficientMat::Zero();
        coeff.col(5) = landing_hold_position_;
        publishMotion(Trajectory({1.0}, {coeff}), stamp, landing_hold_yaw_, 0.0);
        wait_hover_ = true;
        return;
      }
      landing_hold_active_ = false;
      wait_hover_=false;
      flygripper_common::ScopedRuntimeSample sample(path_search_profiler_);
      ok = land_triger_received_ ? envPtr_->short_astar(initial.p,targets.front(),path) :
                                  envPtr_->findVisiblePath(initial.p,targets,way_pts,path);
    }
    Trajectory traj;
    if (ok && !path.empty() && states.size()>1) {
      visPtr_->visualize_path(targets,"car_predict");
      if (land_triger_received_) path.insert(path.end(),targets.begin(),targets.end());
      else if (!way_pts.empty()) {
        flygripper_common::ScopedRuntimeSample sample(visibility_profiler_);
        targets.pop_back(); orientations.pop_back(); way_pts.pop_back();
        envPtr_->generate_visible_regions(targets,way_pts,visible,thetas,orientations,grasp_ps,grasp_yaw_);
        way_pts.insert(way_pts.begin(),initial.p);
        envPtr_->pts2path(way_pts,path);
      } else ok=false;
      if (ok && !path.empty()) {
        std::vector<Eigen::MatrixXd> hPolys;
        std::vector<std::pair<Eigen::Vector3d,Eigen::Vector3d>> keyPts;
        {
          flygripper_common::ScopedRuntimeSample sample(corridor_profiler_);
          envPtr_->generateSFC(path,2.0,hPolys,keyPts);
        }
        Eigen::MatrixXd finState=Eigen::MatrixXd::Zero(3,3);
        finState.col(0) = (land_triger_received_ || (states.front().p-current_state_.p).norm()<=2.0)
                         ? terminal_target : path.back();
        finState.col(1) = states.back().v+Eigen::Vector3d(grasp_vx_,grasp_vy_,grasp_vz_);
        flygripper_common::ScopedRuntimeSample sample(optimization_profiler_);
        ok = land_triger_received_ ? trajOptPtr_->generate_traj(iniState,finState,targets,hPolys,traj) :
             trajOptPtr_->generate_traj(iniState,finState,targets,visible,thetas,grasp_ps,hPolys,traj);
      }
    }
    // Optimization can take time. Never close or publish from an expired snapshot.
    const double twist_age = steadySeconds()-twist_receipt;
    if ((ros::Time::now()-body.header.stamp).toSec()>target_timeout_ ||
        (ros::Time::now()-odom.header.stamp).toSec()>odom_timeout_ ||
        twist_age<0.0 || twist_age>target_twist_timeout_ ||
        (target_twist_check_header_age_ && !moving_grasp::fresh(
          twist.header.stamp.toSec(),ros::Time::now().toSec(),twist_age,target_twist_timeout_))) {
      startBraking(true,"Planning snapshot expired during optimization"); return;
    }
    if (ok && validcheck(traj,stamp)) {
      publishMotion(traj,stamp,car_heading_.yaw()+car_heading_.rate()*(stamp-observation_time).toSec(),car_heading_.rate());
      visPtr_->visualize_traj(traj,"traj");
    } else {
      // A failed approach must not count toward the continuous closing gate.
      grasp_sequence_.ready(steadySeconds(),false,grasp_gate_duration_);
      if ((grasp_point_mode_ && !land_triger_received_ && !grasp_point_trajectory_published_) ||
          !validcheck(traj_poly_,replan_stamp_))
        startBraking(true,"No valid grasp-point or approach trajectory");
      else { replanStateMsg_.state=3; replanState_pub_.publish(replanStateMsg_); }
    }
  }

  void fake_timer_callback(const ros::TimerEvent &event) {
    heartbeat_pub_.publish(std_msgs::Empty());
    if (!odom_received_) {
      return;
    }

    // obtain state of odom
    while (odom_lock_.test_and_set())
      ;
    auto odom_msg = odom_msg_;
    odom_lock_.clear();
    Eigen::Vector3d odom_p(odom_msg.pose.pose.position.x,
                           odom_msg.pose.pose.position.y,
                           odom_msg.pose.pose.position.z);
    Eigen::Vector3d odom_v(odom_msg.twist.twist.linear.x,
                           odom_msg.twist.twist.linear.y,
                           odom_msg.twist.twist.linear.z);
    if (!triger_received_) {
      return;
    }

    ROS_INFO("grasp_state_: %d", grasp_state_);
    if (grasp_state_ == 1) {
      Eigen::MatrixXd iniState;
      iniState.setZero(3, 3);
      ros::Time replan_stamp = ros::Time::now() + ros::Duration(0.03);
      force_hover_ = true;
      ROS_WARN("[planner] FORCE HOVERING...");
      replanStateMsg_.state = 2;
      replanState_pub_.publish(replanStateMsg_);
      iniState.col(0) = odom_p;
      iniState.col(0)(2) = 2.0;
      // fake target hover
      pub_hover_p(iniState.col(0), replan_stamp);
      return;
    } else {
      force_hover_ = false;
    }

    // NOTE force-hover: waiting for the speed of drone small enough
    if (force_hover_ && odom_v.norm() > 0.1) {
      return;
    }

    // NOTE local goal
    Eigen::Vector3d local_goal;
    Eigen::Vector3d delta = goal_ - odom_p;
    if (delta.norm() < 15) {
      local_goal = goal_;
    } else {
      local_goal = delta.normalized() * 15 + odom_p;
    }

    // NOTE determin whether to replan
    bool no_need_replan = false;
    if (!force_hover_ && !wait_hover_) {
      double last_traj_t_rest = traj_poly_.getTotalDuration() -
                                (ros::Time::now() - replan_stamp_).toSec();
      bool new_goal =
          (local_goal - traj_poly_.getPos(traj_poly_.getTotalDuration()))
              .norm() > tracking_dist_;
      if (!new_goal) {
        if (last_traj_t_rest < 1.0) {
          ROS_WARN("[planner] NEAR GOAL...");
          no_need_replan = true;
        } else if (validcheck(traj_poly_, replan_stamp_, last_traj_t_rest)) {
          ROS_WARN("[planner] NO NEED REPLAN...");
          double t_delta = traj_poly_.getTotalDuration() < 1.0
                               ? traj_poly_.getTotalDuration()
                               : 1.0;
          double t_yaw = (ros::Time::now() - replan_stamp_).toSec() + t_delta;
          Eigen::Vector3d un_known_p = traj_poly_.getPos(t_yaw);
          Eigen::Vector3d dp = un_known_p - odom_p;
          double yaw = std::atan2(dp.y(), dp.x());
          pub_traj(traj_poly_, yaw, replan_stamp_);
          no_need_replan = true;
        }
      }
    }
    // // NOTE determin whether to pub hover
    // if ((goal_ - odom_p).norm() < tracking_dist_ + tolerance_d_ &&
    //     odom_v.norm() < 0.1) {
    //   if (!wait_hover_) {
    //     pub_hover_p(odom_p, ros::Time::now());
    //     wait_hover_ = true;
    //   }
    //   ROS_WARN("[planner] HOVERING...");
    //   replanStateMsg_.state = -1;
    //   replanState_pub_.publish(replanStateMsg_);
    //   return;
    // } else {
    //   wait_hover_ = false;
    // }
    // if (no_need_replan) {
    //   return;
    // }

    // NOTE replan state
    Eigen::MatrixXd iniState;
    iniState.setZero(3, 3);
    ros::Time replan_stamp = ros::Time::now() + ros::Duration(0.03);
    double replan_t = (replan_stamp - replan_stamp_).toSec();

    if (force_hover_ || replan_t > traj_poly_.getTotalDuration()) {
      // should replan from the hover state
      iniState.col(0) = odom_p;
      iniState.col(1) = odom_v;
    } else {
      // should replan from the last trajectory
      iniState.col(0) = traj_poly_.getPos(replan_t);
      iniState.col(1) = traj_poly_.getVel(replan_t);
      iniState.col(2) = traj_poly_.getAcc(replan_t);
    }
    replanStateMsg_.header.stamp = ros::Time::now();
    replanStateMsg_.iniState.resize(9);
    Eigen::Map<Eigen::MatrixXd>(replanStateMsg_.iniState.data(), 3, 3) =
        iniState;

    // NOTE generate an extra corridor
    Eigen::Vector3d p_start = iniState.col(0);
    bool need_extra_corridor = iniState.col(1).norm() > 1.0;
    Eigen::MatrixXd hPoly;
    std::pair<Eigen::Vector3d, Eigen::Vector3d> line;
    if (need_extra_corridor) {
      Eigen::Vector3d v_norm = iniState.col(1).normalized();
      line.first = p_start;
      double step = 0.1;
      for (double dx = step; dx < 1.0; dx += step) {
        p_start += step * v_norm;
        if (gridmapPtr_->isOccupied(p_start)) {
          p_start -= step * v_norm;
          break;
        }
      }
      line.second = p_start;
      envPtr_->generateOneCorridor(line, 2.0, hPoly);
    }
    // NOTE path searching
    std::vector<Eigen::Vector3d> path;
    bool generate_new_traj_success =
        envPtr_->astar_search(p_start, local_goal, path);
    Trajectory traj;
    if (generate_new_traj_success) {
      visPtr_->visualize_path(path, "astar");
      // NOTE corridor generating
      std::vector<Eigen::MatrixXd> hPolys;
      std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> keyPts;
      envPtr_->generateSFC(path, 2.0, hPolys, keyPts);
      if (need_extra_corridor) {
        hPolys.insert(hPolys.begin(), hPoly);
        keyPts.insert(keyPts.begin(), line);
      }
      envPtr_->visCorridor(hPolys);
      visPtr_->visualize_pairline(keyPts, "keyPts");

      // NOTE trajectory optimization
      Eigen::MatrixXd finState;
      finState.setZero(3, 3);
      finState.col(0) = path.back();
      // return;
      generate_new_traj_success =
          trajOptPtr_->generate_traj(iniState, finState, hPolys, traj);
      visPtr_->visualize_traj(traj, "traj");
    }

    // NOTE collision check
    bool valid = false;
    if (generate_new_traj_success) {
      valid = validcheck(traj, replan_stamp);
    } else {
      replanStateMsg_.state = -2;
      replanState_pub_.publish(replanStateMsg_);
    }
    if (valid) {
      force_hover_ = false;
      ROS_WARN("[planner] REPLAN SUCCESS");
      replanStateMsg_.state = 0;
      replanState_pub_.publish(replanStateMsg_);
      // NOTE : if the trajectory is known, watch that direction
      Eigen::Vector3d un_known_p = traj.getPos(
          traj.getTotalDuration() < 1.0 ? traj.getTotalDuration() : 1.0);
      Eigen::Vector3d dp = un_known_p - odom_p;
      double yaw = std::atan2(dp.y(), dp.x());
      pub_traj(traj, yaw, replan_stamp);
      traj_poly_ = traj;
      replan_stamp_ = replan_stamp;
    } else if (force_hover_) {
      ROS_ERROR("[planner] REPLAN FAILED, HOVERING...");
      replanStateMsg_.state = 1;
      replanState_pub_.publish(replanStateMsg_);
      return;
    } else if (!validcheck(traj_poly_, replan_stamp_)) {
      force_hover_ = true;
      ROS_FATAL("[planner] EMERGENCY STOP!!!");
      replanStateMsg_.state = 2;
      replanState_pub_.publish(replanStateMsg_);
      pub_hover_p(iniState.col(0), replan_stamp);
      return;
    } else {
      ROS_ERROR("[planner] REPLAN FAILED, EXECUTE LAST TRAJ...");
      replanStateMsg_.state = 3;
      replanState_pub_.publish(replanStateMsg_);
      return; // current generated traj invalid but last is valid
    }
    visPtr_->visualize_traj(traj, "traj");
  }

  void debug_timer_callback(const ros::TimerEvent &event) {
    Eigen::MatrixXd iniState;
    iniState.setZero(3, 3);
    ros::Time replan_stamp = ros::Time::now() + ros::Duration(0.03);

    iniState =
        Eigen::Map<Eigen::MatrixXd>(replanStateMsg_.iniState.data(), 3, 3);
    Eigen::Vector3d target_p(replanStateMsg_.target.pose.pose.position.x,
                             replanStateMsg_.target.pose.pose.position.y,
                             replanStateMsg_.target.pose.pose.position.z);
    Eigen::Vector3d target_v(replanStateMsg_.target.twist.twist.linear.x,
                             replanStateMsg_.target.twist.twist.linear.y,
                             replanStateMsg_.target.twist.twist.linear.z);
    // std::cout << "target_p: " << target_p.transpose() << std::endl;
    // std::cout << "target_v: " << target_v.transpose() << std::endl;

    // visualize the target and the drone velocity
    visPtr_->visualize_arrow(iniState.col(0), iniState.col(0) + iniState.col(1),
                             "drone_vel");
    visPtr_->visualize_arrow(target_p, target_p + target_v, "target_vel");

    // visualize the ray from drone to target
    if (envPtr_->checkRayValid(iniState.col(0), target_p)) {
      visPtr_->visualize_arrow(iniState.col(0), target_p, "ray",
                               visualization::yellow);
    } else {
      visPtr_->visualize_arrow(iniState.col(0), target_p, "ray",
                               visualization::red);
    }

    // NOTE prediction
    std::vector<Eigen::Vector3d> target_predcit;
    if (gridmapPtr_->isOccupied(target_p)) {
      std::cout << "target is invalid!" << std::endl;
      assert(false);
    }
    bool generate_new_traj_success =
        prePtr_->predict(target_p, target_v, target_predcit);

    if (generate_new_traj_success) {
      Eigen::Vector3d observable_p = target_predcit.back();
      visPtr_->visualize_path(target_predcit, "car_predict");
      std::vector<Eigen::Vector3d> observable_margin;
      for (double theta = 0; theta <= 2 * M_PI; theta += 0.01) {
        observable_margin.emplace_back(
            observable_p +
            tracking_dist_ * Eigen::Vector3d(cos(theta), sin(theta), 0));
      }
      visPtr_->visualize_path(observable_margin, "observable_margin");
    }

    // NOTE path searching
    Eigen::Vector3d p_start = iniState.col(0);
    std::vector<Eigen::Vector3d> path, way_pts;
    if (generate_new_traj_success) {
      generate_new_traj_success =
          envPtr_->findVisiblePath(p_start, target_predcit, way_pts, path);
    }

    std::vector<Eigen::Vector3d> visible_ps;
    std::vector<double> thetas;
    Trajectory traj;
    if (generate_new_traj_success) {
      visPtr_->visualize_path(path, "astar");
      // NOTE generate visible regions
      target_predcit.pop_back();
      way_pts.pop_back();
      envPtr_->generate_visible_regions(target_predcit, way_pts, visible_ps,
                                        thetas);
      visPtr_->visualize_pointcloud(visible_ps, "visible_ps");
      visPtr_->visualize_fan_shape_meshes(target_predcit, visible_ps, thetas,
                                          "visible_region");
      // NOTE corridor generating
      std::vector<Eigen::MatrixXd> hPolys;
      std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> keyPts;
      // TODO change the final state
      std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> rays;
      for (int i = 0; i < (int)way_pts.size(); ++i) {
        rays.emplace_back(target_predcit[i], way_pts[i]);
      }
      visPtr_->visualize_pointcloud(way_pts, "way_pts");
      way_pts.insert(way_pts.begin(), p_start);
      envPtr_->pts2path(way_pts, path);
      visPtr_->visualize_path(path, "corridor_path");
      envPtr_->generateSFC(path, 2.0, hPolys, keyPts);
      envPtr_->visCorridor(hPolys);
      visPtr_->visualize_pairline(keyPts, "keyPts");

      // NOTE trajectory optimization
      Eigen::MatrixXd finState;
      finState.setZero(3, 3);
      finState.col(0) = path.back();
      finState.col(1) = target_v;

      generate_new_traj_success = trajOptPtr_->generate_traj(
          iniState, finState, target_predcit, visible_ps, thetas, hPolys, traj);
      visPtr_->visualize_traj(traj, "traj");
    }
    if (!generate_new_traj_success) {
      return;
      // assert(false);
    }
    // check
    bool valid = true;
    std::vector<Eigen::Vector3d> check_pts, invalid_pts;
    double t0 = (ros::Time::now() - replan_stamp).toSec();
    t0 = t0 > 0.0 ? t0 : 0.0;
    double check_dur = 1.0;
    double delta_t = check_dur < traj.getTotalDuration()
                         ? check_dur
                         : traj.getTotalDuration();
    for (double t = t0; t < t0 + delta_t; t += 0.1) {
      Eigen::Vector3d p = traj.getPos(t);
      check_pts.push_back(p);
      if (gridmapPtr_->isOccupied(p)) {
        invalid_pts.push_back(p);
      }
    }
    visPtr_->visualize_path(invalid_pts, "invalid_pts");
    visPtr_->visualize_path(check_pts, "check_pts");
    valid = validcheck(traj, replan_stamp);
    if (!valid) {
      std::cout << "invalid!" << std::endl;
    }
  }

  bool validcheck(const Trajectory &traj, const ros::Time &t_start,
                  const double &check_dur = 1.0) {
    if (traj.getPieceNum() == 0 || !std::isfinite(traj.getTotalDuration())) {
      return false;
    }
    double t0 = (ros::Time::now() - t_start).toSec();
    t0 = t0 > 0.0 ? t0 : 0.0;
    if (t0 >= traj.getTotalDuration()) {
      return false;
    }
    double delta_t = std::min(check_dur, traj.getTotalDuration() - t0);
    for (double t = t0; t < t0 + delta_t; t += 0.01) {
      Eigen::Vector3d p = traj.getPos(t);
      if (gridmapPtr_->isOccupied(p)) {
        return false;
      }
    }
    return true;
  }

  void init(ros::NodeHandle &nh) {
    // set parameters of planning
    int plan_hz = 10;
    nh.getParam("plan_hz", plan_hz);
    nh.getParam("tracking_dur", tracking_dur_);
    nh.getParam("tracking_dist", tracking_dist_);
    nh.getParam("tolerance_d", tolerance_d_);
    nh.getParam("debug", debug_);
    nh.getParam("fake", fake_);
    nh.getParam("grasp_tole_dis", grasp_tole_dis_);
    nh.getParam("grasp_start_dis", grasp_start_dis_);
    nh.getParam("grasp_position", grasp_position_);
    nh.getParam("grasp_yaw", grasp_yaw_);
    nh.getParam("pos_dx", pos_dx);
    nh.getParam("pos_dy", pos_dy);
    nh.getParam("pos_dz", pos_dz);
    nh.param("target_yaw_filter_tau", target_yaw_filter_tau_, 0.1);
    nh.param("target_yaw_rate_limit", target_yaw_rate_limit_, 3.0);
    nh.getParam("grasp_vx", grasp_vx_);
    nh.getParam("grasp_vy", grasp_vy_);
    nh.getParam("grasp_vz", grasp_vz_);
    nh.getParam("replan_head_time", replan_head_time_);
    nh.param("hold_yaw_on_trigger", hold_yaw_on_trigger_, true);
    nh.param("post_grasp_lift", post_grasp_lift_, 0.0);
    nh.param("grasp_max_relative_speed", grasp_max_relative_speed_, 0.10);
    nh.param("velocity_heading", velocity_heading_, true);
    nh.param("pregrasp_enabled", pregrasp_enabled_, false);
    nh.param("require_flight_status", require_flight_status_, true);
    nh.param("pregrasp_distance", pregrasp_distance_, 0.25);
    if (!std::isfinite(pregrasp_distance_) || pregrasp_distance_ <= 0.0) {
      ROS_WARN("[planner] Invalid pregrasp_distance; using 0.25 m");
      pregrasp_distance_ = 0.25;
    }
    std::string velocity_source;
    nh.param<std::string>("target_velocity_source", velocity_source, "twist");
    if (velocity_source == "pose_difference") velocity_from_pose_ = true;
    else if (velocity_source != "twist")
      ROS_WARN("[planner] Unknown target_velocity_source; requiring twist input");
    auto positive = [&](const char* key, double& value, double fallback) {
      nh.param(key, value, fallback);
      if (!std::isfinite(value) || value <= 0.0) {
        ROS_WARN("[planner] Invalid %s; using %.3f", key, fallback);
        value = fallback;
      }
    };
    positive("grasp_close_duration", grasp_close_duration_, 0.85);
    positive("grasp_open_duration", grasp_open_duration_, 0.85);
    positive("flight_status_timeout", flight_status_timeout_, 0.20);
    nh.param("grasp_open_position", grasp_open_position_, 0.0);
    if (!std::isfinite(grasp_open_position_)) grasp_open_position_ = 0.0;
    positive("grasp_gate_duration", grasp_gate_duration_, 0.15);
    positive("grasp_max_yaw_error", grasp_max_yaw_error_, 0.174533);
    positive("grasp_abort_yaw_error", grasp_abort_yaw_error_, 0.785398);
    positive("grasp_abort_distance", grasp_abort_distance_, 0.20);
    positive("grasp_lift_abort_distance", grasp_lift_abort_distance_, 0.10);
    positive("grasp_abort_relative_speed", grasp_abort_relative_speed_, 0.50);
    positive("target_timeout", target_timeout_, 0.20);
    positive("target_twist_timeout", target_twist_timeout_, 0.20);
    nh.param("target_velocity_cutoff_hz", target_velocity_cutoff_hz_, 3.0);
    if (!std::isfinite(target_velocity_cutoff_hz_) || target_velocity_cutoff_hz_ < 0.0) {
      ROS_WARN("[planner] Invalid target_velocity_cutoff_hz; using 3 Hz");
      target_velocity_cutoff_hz_ = 3.0;
    }
    positive("target_pose_twist_max_skew", target_pose_twist_max_skew_, 0.15);
    nh.param("target_twist_check_header_age", target_twist_check_header_age_, false);
    positive("odom_timeout", odom_timeout_, 0.20);
    positive("grasp_lift_duration", grasp_lift_duration_, 2.0);
    positive("grasp_follow_horizon", grasp_follow_horizon_, 0.50);
    positive("grasp_yaw_rate_max", grasp_yaw_rate_max_, 0.60);
    positive("grasp_yaw_acc_max", grasp_yaw_acc_max_, 1.20);
    positive("grasp_yaw_speed_low", grasp_yaw_speed_low_, 0.05);
    positive("grasp_yaw_speed_high", grasp_yaw_speed_high_, 0.08);
    positive("grasp_yaw_velocity_filter_hz", grasp_yaw_velocity_filter_hz_, 2.0 / moving_grasp::kPi);
    positive("grasp_yaw_direction_dwell", grasp_yaw_direction_dwell_, 0.25);
    positive("vmax", grasp_vmax_, 0.40);
    positive("amax", grasp_amax_, 0.40);
    positive("grasp_start_dis", grasp_start_dis_, 0.05);
    positive("grasp_max_relative_speed", grasp_max_relative_speed_, 0.10);
    nh.param("grasp_heading_offset", grasp_heading_offset_, 0.0);
    if (!std::isfinite(grasp_heading_offset_)) grasp_heading_offset_ = 0.0;
    if (!std::isfinite(post_grasp_lift_) || post_grasp_lift_ < 0.0) post_grasp_lift_ = 0.0;
    grasp_yaw_speed_high_ = std::max(grasp_yaw_speed_high_, grasp_yaw_speed_low_);
    grasp_abort_distance_ = std::max(grasp_abort_distance_, grasp_start_dis_);
    grasp_abort_relative_speed_ = std::max(grasp_abort_relative_speed_, grasp_max_relative_speed_);
    grasp_abort_yaw_error_ = std::max(grasp_abort_yaw_error_, grasp_max_yaw_error_);


    double grid_resolution = 0.1;
    nh.param("grid_resolution", grid_resolution, 0.1);
    gridmapPtr_ = std::make_shared<env::FreeSpaceGrid>(grid_resolution);
    envPtr_ = std::make_shared<env::Env>(nh, gridmapPtr_);
    visPtr_ = std::make_shared<visualization::Visualization>(nh);
    trajOptPtr_ = std::make_shared<traj_opt::TrajOpt>(nh);
    prePtr_ = std::make_shared<prediction::Predict>(nh);

    heartbeat_pub_ = nh.advertise<std_msgs::Empty>("heartbeat", 10);
    traj_pub_ = nh.advertise<quadrotor_msgs::PolyTraj>("trajectory", 1);
    flygripper_traj_pub_ = nh.advertise<flygripper_msgs::PolynomialTrajectory>(
        "/flygripper/trajectory", 1, this);
    replanState_pub_ =
        nh.advertise<quadrotor_msgs::ReplanState>("replanState", 1);
    graspState_pub_ =
        nh.advertise<quadrotor_msgs::GraspState>("/graspState", 1);
    grasp_phase_pub_ = nh.advertise<std_msgs::String>("grasp_phase", 1, true);
    pregrasp_target_pub_ = nh.advertise<nav_msgs::Odometry>("pregrasp_target", 1);
    grasp_alignment_pub_ = nh.advertise<geometry_msgs::Vector3Stamped>("grasp_alignment", 1);
    grasp_point_mode_pub_ = nh.advertise<std_msgs::Bool>("grasp_point_mode", 1, true);
    publishGraspPhase();
    actuator_command_pub_ = nh.advertise<flygripper_msgs::ActuatorCommand>(
        "/flygripper/actuator_command", 10);

    if (debug_) {
      plan_timer_ = nh.createTimer(ros::Duration(1.0 / plan_hz),
                                   &Nodelet::debug_timer_callback, this);
      // TODO read debug data from files
      wr_msg::readMsg(replanStateMsg_, ros::package::getPath("planning") +
                                           "/../../../debug/replan_state.bin");
      std::cout << "plan state: " << replanStateMsg_.state << std::endl;
    } else if (fake_) {
      plan_timer_ = nh.createTimer(ros::Duration(1.0 / plan_hz),
                                   &Nodelet::fake_timer_callback, this);
    } else {
      plan_timer_ = nh.createTimer(ros::Duration(1.0 / plan_hz),
                                   &Nodelet::plan_timer_callback, this);
    }
    odom_sub_ = nh.subscribe<nav_msgs::Odometry>(
        "odom", 10, &Nodelet::odom_callback, this,
        ros::TransportHints().tcpNoDelay());
    target_sub_ = nh.subscribe<nav_msgs::Odometry>(
        "target", 10, &Nodelet::target_callback, this,
        ros::TransportHints().tcpNoDelay());
    target_twist_sub_ = nh.subscribe<geometry_msgs::TwistStamped>(
        "target_twist", 10, &Nodelet::target_twist_callback, this,
        ros::TransportHints().tcpNoDelay());
    flight_status_sub_ = nh.subscribe<flygripper_msgs::FlightTrajectoryStatus>(
        "/flygripper/flight_trajectory_status", 10, &Nodelet::flightStatusCallback, this,
        ros::TransportHints().tcpNoDelay());
    triger_sub_ = nh.subscribe<geometry_msgs::PoseStamped>(
        "triger", 10, &Nodelet::triger_callback, this,
        ros::TransportHints().tcpNoDelay());
    land_triger_sub_ = nh.subscribe<geometry_msgs::PoseStamped>(
        "land_triger", 10, &Nodelet::land_triger_callback, this,
        ros::TransportHints().tcpNoDelay());
    ROS_WARN("Mocap planning initialized: no occupancy map or obstacle avoidance.");
  }

public:
  ~Nodelet() override {
    if (initThread_.joinable()) {
      initThread_.join();
    }
  }

  void onInit(void) {
    ros::NodeHandle nh(getMTPrivateNodeHandle());
    initThread_ = std::thread(std::bind(&Nodelet::init, this, nh));
  }
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

} // namespace planning

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(planning::Nodelet, nodelet::Nodelet);
