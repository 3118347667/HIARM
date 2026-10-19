#include <geometry_msgs/PoseStamped.h>
#include <env/free_space_grid.hpp>
#include <nav_msgs/Odometry.h>
#include <nodelet/nodelet.h>
#include <quadrotor_msgs/GraspState.h>
#include <quadrotor_msgs/PolyTraj.h>
#include <quadrotor_msgs/ReplanState.h>
#include <ros/package.h>
#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <traj_opt/traj_opt.h>

#include <Eigen/Core>
#include <atomic>
#include <cmath>
#include <env/env.hpp>
#include <prediction/prediction.hpp>
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

namespace planning {

Eigen::IOFormat CommaInitFmt(Eigen::StreamPrecision, Eigen::DontAlignCols, ", ",
                             ", ", "", "", " << ", ";");

class Nodelet : public nodelet::Nodelet {
private:
  std::thread initThread_;
  ros::Subscriber odom_sub_, target_sub_, triger_sub_,
      land_triger_sub_, grasp_sub_;
  ros::Timer plan_timer_;

  ros::Publisher traj_pub_, heartbeat_pub_, replanState_pub_, graspState_pub_,
      actuator_command_pub_;

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
  bool grasp_hold_valid_ = false;
  Eigen::Vector3d grasp_hold_p_ = Eigen::Vector3d::Zero();
  std::mutex planning_mutex_;

  nav_msgs::Odometry odom_msg_, target_msg_, target_body_msg_;
  std::atomic_flag odom_lock_ = ATOMIC_FLAG_INIT;
  std::atomic_flag target_lock_ = ATOMIC_FLAG_INIT;
  std::atomic_bool odom_received_ = ATOMIC_VAR_INIT(false);
  std::atomic_bool triger_received_ = ATOMIC_VAR_INIT(false);
  std::atomic_bool target_received_ = ATOMIC_VAR_INIT(false);
  std::atomic_bool land_triger_received_ = ATOMIC_VAR_INIT(false);

  void pub_hover_p(const Eigen::Vector3d &hover_p, const ros::Time &stamp) {
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
    traj_pub_.publish(traj_msg);
    // The aircraft subscribes to /flygripper/trajectory, not position_cmd.
    // Send the hold to both consumers using the same position and heading.
    CoefficientMat coeff = CoefficientMat::Zero();
    coeff.col(5) = hover_p;
    Trajectory hold(std::vector<double>{1.0},
                    std::vector<CoefficientMat>{coeff});
    convertToFlygripperMsgs(hold, hold_yaw_, stamp, traj_msg);
  }

  void pub_traj(const Trajectory &traj, const double &yaw,
                const ros::Time &stamp) {
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
    traj_msg.yaw = hold_yaw_on_trigger_ && hold_yaw_valid_ ? hold_yaw_ : yaw;
    traj_pub_.publish(traj_msg);

    convertToFlygripperMsgs(traj, traj_msg.yaw, stamp, traj_msg);
  }

  // todo@yinloonga:
  // 转换quadrotor_msgs::PolyTraj为flygripper_msgs::PolynomialTrajectory
  void convertToFlygripperMsgs(const Trajectory &traj, const double &yaw,
                               const ros::Time &stamp,
                               const quadrotor_msgs::PolyTraj &tracker_msg) {
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
    if (hold_yaw_on_trigger_ || tracker_msg.hover) {
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
    std::lock_guard<std::mutex> guard(planning_mutex_);
    goal_ << msgPtr->pose.position.x, msgPtr->pose.position.y, 0.9;
    force_hover_ = true;
    wait_hover_ = false;
    grasp_state_ = 0;
    grasp_hold_valid_ = false;
    hold_yaw_valid_ = false;
    triger_received_ = true;
  }

  void land_triger_callback(const geometry_msgs::PoseStampedConstPtr &msgPtr) {
    land_p_.x() = msgPtr->pose.position.x;
    land_p_.y() = msgPtr->pose.position.y;
    land_p_.z() = msgPtr->pose.position.z;
    land_q_.w() = msgPtr->pose.orientation.w;
    land_q_.x() = msgPtr->pose.orientation.x;
    land_q_.y() = msgPtr->pose.orientation.y;
    land_q_.z() = msgPtr->pose.orientation.z;
    land_triger_received_ = true;
  }

  void grasp_callback(const quadrotor_msgs::GraspStateConstPtr &msgPtr) {
    std::lock_guard<std::mutex> guard(planning_mutex_);
    grasp_state_ = msgPtr->graspState;
  }

  void odom_callback(const nav_msgs::Odometry::ConstPtr &msgPtr) {
    while (odom_lock_.test_and_set())
      ;
    odom_msg_ = *msgPtr;
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
    } else if (dt > 1e-6) {
      // This workspace's VRPN driver publishes Odometry with an empty twist.
      const auto previous_reference = target_msg_.pose.pose.position;
      const auto previous_origin = target_body_msg_.pose.pose.position;
      const auto previous_orientation = target_msg_.pose.pose.orientation;
      target_msg_ = reference_msg;
      target_body_msg_ = *msgPtr;
      auto &reference_v = target_msg_.twist.twist.linear;
      reference_v.x = (reference_p.x() - previous_reference.x) / dt;
      reference_v.y = (reference_p.y() - previous_reference.y) / dt;
      reference_v.z = (reference_p.z() - previous_reference.z) / dt;
      auto &origin_v = target_body_msg_.twist.twist.linear;
      origin_v.x = (p.x - previous_origin.x) / dt;
      origin_v.y = (p.y - previous_origin.y) / dt;
      origin_v.z = (p.z - previous_origin.z) / dt;

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
    }
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

  // NOTE main callback
  void plan_timer_callback(const ros::TimerEvent &event) {
    // A multi-threaded nodelet manager may schedule overlapping timer calls.
    std::unique_lock<std::mutex> guard(planning_mutex_, std::try_to_lock);
    if (!guard.owns_lock()) return;
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
    Eigen::Quaterniond odom_q(
        odom_msg.pose.pose.orientation.w, odom_msg.pose.pose.orientation.x,
        odom_msg.pose.pose.orientation.y, odom_msg.pose.pose.orientation.z);

    if (triger_received_ && !hold_yaw_valid_) {
      const Eigen::Vector3d forward = odom_q * Eigen::Vector3d::UnitX();
      hold_yaw_ = std::atan2(forward.y(), forward.x());
      hold_yaw_valid_ = true;
    }

    if (grasp_state_ == 1) {
      ROS_WARN("[planner] Grasp Done !!! FORCE HOVERING...");
      replanStateMsg_.state = 2;
      replanState_pub_.publish(replanStateMsg_);
      if (!grasp_hold_valid_) {
        grasp_hold_p_ = odom_p;
        grasp_hold_p_.z() += post_grasp_lift_;
        grasp_hold_valid_ = true;
      }
      ros::Time replan_stamp =
          ros::Time::now() + ros::Duration(replan_head_time_);
      publishMorphing(grasp_position_);
      pub_hover_p(grasp_hold_p_, replan_stamp);
      return;
    }

    if (!triger_received_) {
      return;
    }
    if (!target_received_) {
      return;
    }
    flygripper_common::ScopedRuntimeSample planning_profile_sample(
        planning_total_profiler_);

    // NOTE obtain state of target
    while (target_lock_.test_and_set())
      ;
    replanStateMsg_.target = target_msg_;
    const nav_msgs::Odometry target_body_msg = target_body_msg_;
    const double target_yaw_rate = target_yaw_rate_;
    target_lock_.clear();
    Eigen::Vector3d target_p(replanStateMsg_.target.pose.pose.position.x,
                             replanStateMsg_.target.pose.pose.position.y,
                             replanStateMsg_.target.pose.pose.position.z);
    Eigen::Vector3d target_v(replanStateMsg_.target.twist.twist.linear.x,
                             replanStateMsg_.target.twist.twist.linear.y,
                             replanStateMsg_.target.twist.twist.linear.z);
    Eigen::Quaterniond target_q;
    target_q.w() = replanStateMsg_.target.pose.pose.orientation.w;
    target_q.x() = replanStateMsg_.target.pose.pose.orientation.x;
    target_q.y() = replanStateMsg_.target.pose.pose.orientation.y;
    target_q.z() = replanStateMsg_.target.pose.pose.orientation.z;

    if ((target_p - odom_p).norm() > 5.0) {
      replanStateMsg_.state = 2;
      replanState_pub_.publish(replanStateMsg_);
      ROS_WARN("[planner] Perception might be wrong! Distance too large");
      ros::Time replan_stamp =
          ros::Time::now() + ros::Duration(replan_head_time_);
      pub_hover_p(odom_p, replan_stamp);
      return;
    }

    // NOTE force-hover: waiting for the speed of drone small enough
    if (force_hover_ && odom_v.norm() > 0.1) {
      return;
    }

    // NOTE just for landing on the car!
    if (land_triger_received_) {
      if (std::fabs((target_p - odom_p).norm() < 0.1 && odom_v.norm() < 0.1 &&
                    target_v.norm() < 0.2)) {
        if (!wait_hover_) {
          pub_hover_p(odom_p, ros::Time::now());
          wait_hover_ = true;
        }
        ROS_WARN("[planner] HOVERING...");
        return;
      }
      // TODO get the orientation fo target and calculate the pose of landing
      // point
      target_p = target_p + target_q * land_p_;
      wait_hover_ = false;
    } else {
      // target_p.z() += 1.0;
      // NOTE
      // determin whether to replan target_p.z() -= 0.4;
      Eigen::Vector3d dp = target_p - odom_p;
      std::cout << "dist : " << dp.norm() << std::endl;
      std::cout << "dist x: " << dp.x() << std::endl;
      double desired_yaw = std::atan2(dp.y(), dp.x());
      Eigen::Vector3d project_yaw =
          odom_q.toRotationMatrix().col(0); // NOTE ZYX
      double now_yaw = std::atan2(project_yaw.y(), project_yaw.x());
      // Do not stop on the old 10 cm viewing ring before reaching the
      // configured grasp point. Completion is checked below using distance
      // and relative velocity, independent of the held aircraft heading.
      wait_hover_ = false;
    }

    // Mocap mode: the search grid treats the environment as free space.
    ros::Time replan_stamp =
        ros::Time::now() + ros::Duration(replan_head_time_);

    // visualize the ray from drone to target
    if (envPtr_->checkRayValid(odom_p, target_p)) {
      visPtr_->visualize_arrow(odom_p, target_p, "ray", visualization::yellow);
    } else {
      visPtr_->visualize_arrow(odom_p, target_p, "ray", visualization::red);
    }

    // NOTE prediction
    std::vector<Eigen::Vector3d> target_predcit;
    std::vector<Eigen::Quaterniond> target_predicted_qs;
    std::vector<prediction::TargetState> target_predicted_states;
    bool generate_new_traj_success;
    {
      flygripper_common::ScopedRuntimeSample profile_sample(
          prediction_profiler_);
      const auto &origin = target_body_msg.pose.pose.position;
      const auto &origin_velocity = target_body_msg.twist.twist.linear;
      Eigen::Vector3d target_offset(pos_dx, pos_dy, pos_dz);
      if (land_triger_received_) {
        target_offset += land_p_;
      }
      const double start_delay = std::max(
          0.0, (replan_stamp - target_body_msg.header.stamp).toSec());
      generate_new_traj_success = prePtr_->predictRigidBody(
          Eigen::Vector3d(origin.x, origin.y, origin.z),
          Eigen::Vector3d(origin_velocity.x, origin_velocity.y,
                          origin_velocity.z),
          target_q, target_yaw_rate, target_offset, start_delay,
          target_predicted_states);
      if (generate_new_traj_success) {
        target_predcit.reserve(target_predicted_states.size());
        target_predicted_qs.reserve(target_predicted_states.size());
        for (const auto &state : target_predicted_states) {
          target_predcit.push_back(state.p);
          target_predicted_qs.push_back(state.q);
        }
        target_p = target_predicted_states.front().p;
        target_v = target_predicted_states.front().v;
        target_q = target_predicted_states.front().q;
      }
    }
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

    // NOTE replan state
    Eigen::MatrixXd iniState;
    iniState.setZero(3, 3);
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

    // NOTE path searching
    Eigen::Vector3d p_start = iniState.col(0);
    std::vector<Eigen::Vector3d> path, way_pts;

    // NOTE calculate time of path searching, corridor generation and
    // optimization

    // static double t_path_ = 0;
    // static double t_corridor_ = 0;
    // static double t_optimization_ = 0;
    // static int times_path_ = 0;
    // static int times_corridor_ = 0;
    // static int times_optimization_ = 0;
    // double t_path = 0;

    if (generate_new_traj_success) {
      flygripper_common::ScopedRuntimeSample profile_sample(
          path_search_profiler_);
      if (land_triger_received_) {
        generate_new_traj_success =
            envPtr_->short_astar(p_start, target_p, path);
      } else {
        generate_new_traj_success =
            envPtr_->findVisiblePath(p_start, target_predcit, way_pts, path);
      }
    }

    std::vector<Eigen::Vector3d> visible_ps;
    std::vector<double> thetas;
    std::vector<Eigen::Vector3d> grasp_ps;
    Trajectory traj;
    if (generate_new_traj_success) {
      visPtr_->visualize_path(path, "astar");
      if (land_triger_received_) {
        for (const auto &p : target_predcit) {
          path.push_back(p);
        }
      } else {
        flygripper_common::ScopedRuntimeSample profile_sample(
            visibility_profiler_);
        // NOTE generate visible regions
        target_predcit.pop_back();
        target_predicted_qs.pop_back();
        way_pts.pop_back();
        // ros::Time t_front1 = ros::Time::now();
        Eigen::Vector3d dp = target_p - odom_p;

        // if(dp.norm() > 2.0) {
        //   envPtr_->generate_visible_regions(target_predcit, way_pts,
        //   visible_ps, thetas);
        // } else {
        //   envPtr_->generate_visible_regions(target_predcit, way_pts,
        //   visible_ps, thetas, target_q, thetas_grasp);
        // }

        envPtr_->generate_visible_regions(target_predcit, way_pts, visible_ps,
                                          thetas, target_predicted_qs, grasp_ps,
                                          grasp_yaw_);

        // envPtr_->generate_visible_regions(target_predcit, way_pts,
        // visible_ps, thetas);

        // ros::Time t_end1 = ros::Time::now();
        // t_path += (t_end1 - t_front1).toSec() * 1e3;
        visPtr_->visualize_pointcloud(visible_ps, "visible_ps");
        visPtr_->visualize_fan_shape_meshes(target_predcit, visible_ps, thetas,
                                            "visible_region");

        // TODO change the final state
        std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> rays;
        for (int i = 0; i < (int)way_pts.size(); ++i) {
          rays.emplace_back(target_predcit[i], way_pts[i]);
        }
        visPtr_->visualize_pointcloud(way_pts, "way_pts");
        way_pts.insert(way_pts.begin(), p_start);
        // ros::Time t_front2 = ros::Time::now();
        envPtr_->pts2path(way_pts, path);
        // ros::Time t_end2 = ros::Time::now();
        // t_path += (t_end2 - t_front2).toSec() * 1e3;

        // if(dp.norm() < 1.0)
        // {
        //   for (const auto& p : target_predcit) {
        //     path.push_back(p);
        //   }
        // }
        // else
        // {
        //   // if(dp.norm() > 2.0) {
        //   //   envPtr_->generate_visible_regions(target_predcit, way_pts,
        //   visible_ps, thetas);
        //   // } else {
        //   //   envPtr_->generate_visible_regions(target_predcit, way_pts,
        //   visible_ps, thetas, target_q, thetas_grasp);
        //   // }

        //   envPtr_->generate_visible_regions(target_predcit, way_pts,
        //   visible_ps, thetas, target_q, grasp_ps);

        //   // envPtr_->generate_visible_regions(target_predcit, way_pts,
        //   //                                   visible_ps, thetas);

        //   // ros::Time t_end1 = ros::Time::now();
        //   // t_path += (t_end1 - t_front1).toSec() * 1e3;
        //   visPtr_->visualize_pointcloud(visible_ps, "visible_ps");
        //   visPtr_->visualize_fan_shape_meshes(target_predcit, visible_ps,
        //   thetas, "visible_region");

        //   // TODO change the final state
        //   std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> rays;
        //   for (int i = 0; i < (int)way_pts.size(); ++i) {
        //     rays.emplace_back(target_predcit[i], way_pts[i]);
        //   }
        //   visPtr_->visualize_pointcloud(way_pts, "way_pts");
        //   way_pts.insert(way_pts.begin(), p_start);
        //   // ros::Time t_front2 = ros::Time::now();
        //   envPtr_->pts2path(way_pts, path);
        //   // ros::Time t_end2 = ros::Time::now();
        //   // t_path += (t_end2 - t_front2).toSec() * 1e3;
        // }
      }
      // NOTE corridor generating
      std::vector<Eigen::MatrixXd> hPolys;
      std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> keyPts;

      {
        flygripper_common::ScopedRuntimeSample profile_sample(
            corridor_profiler_);
        envPtr_->generateSFC(path, 2.0, hPolys, keyPts);
      }

      envPtr_->visCorridor(hPolys);
      visPtr_->visualize_pairline(keyPts, "keyPts");

      Eigen::Vector3d project_target_yaw =
          target_predicted_states.back().q.toRotationMatrix().col(0);
      double target_yaw =
          std::atan2(project_target_yaw.y(), project_target_yaw.x());
      // double grasp_target_yaw = target_yaw + M_PI / 4;
      double grasp_target_yaw = target_yaw;

      // NOTE trajectory optimization
      Eigen::MatrixXd finState;
      finState.setZero(3, 3);
      finState.col(0) = path.back();
      finState.col(1) =
          target_predicted_states.back().v +
          Eigen::Vector3d(grasp_vx_, grasp_vy_, grasp_vz_) +
          0.0 * target_v.norm() *
              Eigen::Vector3d(cos(grasp_target_yaw), sin(grasp_target_yaw), 0);
      {
        flygripper_common::ScopedRuntimeSample profile_sample(
            optimization_profiler_);
        if (land_triger_received_) {
          finState.col(0) = target_predicted_states.back().p;
          generate_new_traj_success = trajOptPtr_->generate_traj(
              iniState, finState, target_predcit, hPolys, traj);
        } else {
        // generate_new_traj_success = trajOptPtr_->generate_traj(iniState,
        // finState,
        //                                                        target_predcit,
        //                                                        visible_ps,
        //                                                        thetas,
        //                                                        hPolys, traj);
        // finState.col(0) = target_predcit.back();

        if ((target_p - odom_p).norm() <= 2.0) {

          finState.col(0) = target_predicted_states.back().p;
          const bool grasp_reached =
              (target_p - odom_p).norm() <= grasp_start_dis_ &&
              (odom_v - target_v).norm() <= grasp_max_relative_speed_;
          if (!grasp_reached && grasp_state_ == 0) {
            ROS_ERROR("(target_p - odom_p).norm(), %f",
                      (target_p - odom_p).norm());
            ROS_ERROR("(target_p - odom_p).x(), %f", (target_p - odom_p).x());
            ROS_ERROR("odom_v.norm(), %f", odom_v.norm());
            grasp_degree_ =
                std::max(0.0, std::min(1.0, (1.0 + grasp_tole_dis_ -
                                             (target_p - odom_p).norm()) /
                                                1.0));
            graspStateMsg_.graspState = grasp_state_;
            graspStateMsg_.graspDegree = grasp_degree_;
            graspState_pub_.publish(graspStateMsg_);
            tracking_dist_ = 0.001;
            tolerance_d_ = 0.0001;
            generate_new_traj_success = trajOptPtr_->generate_traj(
                iniState, finState, target_predcit, visible_ps, thetas,
                grasp_ps, hPolys, traj);

            if ((target_p - odom_p).norm() <= grasp_start_dis_) {
              ROS_INFO("[planner] START GRASPING...");
              publishMorphing(grasp_position_);
            }

            // generate_new_traj_success = trajOptPtr_->generate_traj(iniState,
            // finState,
            //                                                         target_predcit,
            //                                                         visible_ps,
            //                                                         thetas,
            //                                                         hPolys,
            //                                                         traj);
          } else {
            force_hover_ = true;
            ROS_WARN("[planner] FORCE HOVERING...");
            grasp_state_ = 1;
            grasp_degree_ = 1.0;
            graspStateMsg_.graspState = grasp_state_;
            graspStateMsg_.graspDegree = grasp_degree_;
            graspState_pub_.publish(graspStateMsg_);
            replanStateMsg_.state = 2;
            replanState_pub_.publish(replanStateMsg_);
            grasp_hold_p_ = odom_p;
            grasp_hold_p_.z() += post_grasp_lift_;
            grasp_hold_valid_ = true;
            pub_hover_p(grasp_hold_p_, replan_stamp);
            publishMorphing(grasp_position_);
            return;
          }

        } else {
          graspStateMsg_.graspState = grasp_state_;
          graspState_pub_.publish(graspStateMsg_);
          finState.col(0) = path.back();
          ROS_ERROR("(target_p - odom_p).norm(), %f",
                    (target_p - odom_p).norm());
          // std::cout << "finState: \n" << finState << std::endl;

          generate_new_traj_success = trajOptPtr_->generate_traj(
              iniState, finState, target_predcit, visible_ps, thetas, grasp_ps,
              hPolys, traj);

          // generate_new_traj_success = trajOptPtr_->generate_traj(iniState,
          // finState,
          //                                                         target_predcit,
          //                                                         visible_ps,
          //                                                         thetas,
          //                                                         hPolys,
          //                                                         traj);
        }
      }
      }

      // NOTE average calculating time of path searching, corridor generation
      // and optimization

      // t_path_ = (t_path_ * times_path_ + t_path) / (++times_path_);
      // t_corridor_ = (t_corridor_ * times_corridor_ + t_corridor) /
      // (++times_corridor_); t_optimization_ = (t_optimization_ *
      // times_optimization_ + t_optimization) / (++times_optimization_);

      // std::cout << "t_path_: " << t_path_ << " ms" << std::endl;
      // std::cout << "t_corridor_: " << t_corridor_ << " ms" << std::endl;
      // std::cout << "t_optimization_: " << t_optimization_ << " ms" <<
      // std::endl;

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
      Eigen::Vector3d dp =
          target_p + target_v * replan_head_time_ - iniState.col(0);
      // NOTE : if the drone is going to unknown areas, watch that direction
      // Eigen::Vector3d un_known_p = traj.getPos(1.0);
      // if (gridmapPtr_->isUnKnown(un_known_p)) {
      //   dp = un_known_p - odom_p;
      // }
      double yaw = std::atan2(dp.y(), dp.x());
      if (land_triger_received_) {
        const Eigen::Quaterniond &future_q = target_predicted_states.back().q;
        yaw = 2 * std::atan2(future_q.z(), future_q.w());
      }
      // static int replan_twice = 2;
      // if (replan_twice != 0) {
      //   pub_traj(traj, yaw, replan_stamp);
      //   replan_twice--;
      // }
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
    triger_sub_ = nh.subscribe<geometry_msgs::PoseStamped>(
        "triger", 10, &Nodelet::triger_callback, this,
        ros::TransportHints().tcpNoDelay());
    land_triger_sub_ = nh.subscribe<geometry_msgs::PoseStamped>(
        "land_triger", 10, &Nodelet::land_triger_callback, this,
        ros::TransportHints().tcpNoDelay());
    grasp_sub_ = nh.subscribe<quadrotor_msgs::GraspState>(
        "/graspState", 10, &Nodelet::grasp_callback, this,
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
