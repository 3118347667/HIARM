#include <chrono>
#include <cmath>
#include <cstddef>
#include <geometry_msgs/PoseStamped.h>
#include <iostream>
#include <mapping/mapping.h>
#include <nav_msgs/Odometry.h>
#include <nodelet/nodelet.h>
#include <quadrotor_msgs/OccMap3d.h>
#include <quadrotor_msgs/PolyTraj.h>
#include <quadrotor_msgs/ReplanState.h>
#include <ros/package.h>
#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Bool.h>
#include <traj_opt/traj_opt.h>

#include <Eigen/Core>
#include <atomic>
#include <env/env.hpp>
#include <prediction/prediction.hpp>
#include <thread>
#include <vector>
#include <visualization/visualization.hpp>
#include <wr_msg/wr_msg.hpp>

// flygripper
// #include "flygripper_msgs/PolynomialTrajectory.h"
// #include "flygripper_msgs/ActuatorCommand.h"
// #include "flygripper_common/trajectory/polynomial_trajectory.hpp"
// #include "flygripper_common/type/base.hpp"
// #include "flygripper_common/type/trajectory.hpp"
// #include "flygripper_common/utils/math.hpp"
#include "ros/console.h"
#include "ros/publisher.h"
#include "ros/subscriber.h"
#include "ros/time.h"
#include "ros/transport_hints.h"
#include "traj_opt/poly_traj_utils.hpp"

namespace planning {

Eigen::IOFormat CommaInitFmt(Eigen::StreamPrecision, Eigen::DontAlignCols, ", ", ", ", "", "", " << ", ";");

class Nodelet : public nodelet::Nodelet {
 private:
  std::thread initThread_;
  ros::Subscriber gridmap_sub_, odom_sub_, target_sub_, triger_sub_, land_triger_sub_;
  ros::Timer plan_timer_;

  ros::Publisher traj_pub_, heartbeat_pub_, replanState_pub_;
  bool enable_flygripper_ = true;
  ros::Publisher flygripper_traj_pub_;

  std::shared_ptr<mapping::OccGridMap> gridmapPtr_;
  std::shared_ptr<env::Env> envPtr_;
  std::shared_ptr<visualization::Visualization> visPtr_;
  std::shared_ptr<traj_opt::TrajOpt> trajOptPtr_;
  std::shared_ptr<prediction::Predict> prePtr_;

  // NOTE planning or fake target
  bool fake_ = false;
  Eigen::Vector3d goal_;  // 静态目标点
  Eigen::Vector3d land_p_;  // 相对于目标的着陆点位置
  Eigen::Quaterniond land_q_;  // 相对于目标的着陆点姿态

  // NOTE just for debug
  bool debug_ = false;  // 是否启用调试模式
  quadrotor_msgs::ReplanState replanStateMsg_;  // 存储当前的重规划状态信息
  ros::Publisher gridmap_pub_, inflate_gridmap_pub_;  // debug模式下用于发布地图
  quadrotor_msgs::OccMap3d occmap_msg_;  // debug模式下用于发布地图

  double tracking_dur_, tracking_dist_, tolerance_d_;  // 跟踪参数(预测时长、期望跟踪距离、距离容忍度)

  Trajectory traj_poly_;  // 存储当前正在执行(或上次成功规划)的多项式轨迹
  ros::Time replan_stamp_;  // 上次成功轨迹规划的起始时间戳
  int traj_id_ = 0;  // 发布轨迹的唯一ID
  bool wait_hover_ = true;  // 标志是否应该发布悬停指令(例如, 到达目标附近时)
  bool force_hover_ = true;  // 标志是否强制无人机悬停(例如, 刚启动或紧急情况)

  nav_msgs::Odometry odom_msg_, target_msg_;  // 存储接收到的最新消息
  quadrotor_msgs::OccMap3d map_msg_;  // 同上
  std::atomic_flag odom_lock_ = ATOMIC_FLAG_INIT;  // 标志锁,用于保护消息缓冲区的线程安全访问(防止回调函数和规划线程同时读写消息)
  std::atomic_flag target_lock_ = ATOMIC_FLAG_INIT;  // 同上
  std::atomic_flag gridmap_lock_ = ATOMIC_FLAG_INIT;  // 同上
  std::atomic_bool odom_received_ = ATOMIC_VAR_INIT(false);  // 标记是否接收到相应类型的消息
  std::atomic_bool map_received_ = ATOMIC_VAR_INIT(false);  // 同上
  std::atomic_bool triger_received_ = ATOMIC_VAR_INIT(false);  // 同上
  std::atomic_bool target_received_ = ATOMIC_VAR_INIT(false);  // 同上
  std::atomic_bool land_triger_received_ = ATOMIC_VAR_INIT(false);  // 同上

  // fix@yinloonga: disable map
  bool enable_map_ = false;
  bool enable_vis_ = false;
  // track_done
  ros::Publisher track_done_pub_;
  bool wait_track_done_ = false;  // 标志是否track的第二阶段完成
  double track_min_dist_ = 10.0f, track_dist_diff_ = 0.1f;
  ros::Time last_warn_time_;
  // graps_done
  ros::Subscriber grasp_done_sub_;
  bool wait_grasp_done_ = false;  // 标志是否抓取完成
  // Trigger 无人机起飞触发
  ros::Subscriber rviz_trigger_sub_;
  std::atomic_bool launch_trigger_ = ATOMIC_VAR_INIT(false);  // 标志是否触发起飞
  int cnt_launch_trigger_ = -1;

  // 统计各个阶段时间开销
  double predict_total_time_ = 0.0f, search_total_time_ = 0.0f, 
          visible_total_time_= 0.0f, corridor_total_time_ = 0.0f, traj_opt_total_time_ = 0.0f;
  std::size_t cnt_predict_ = 0, cnt_search_ = 0, cnt_visible_ = 0, 
            cnt_corridor_ = 0, cnt_traj_opt_ = 0;
  ros::Time last_print_time_;
  double print_interval_ = 1.0f;

  // 偏移
  double track_x_offset_= 0.0, track_y_offset_ = 0, track_z_offset_ = 0.0;

  // fix@yinloonga(0509): 添加抓取
  ros::Publisher actuator_pub_;
  double grasp_degree_ = 0.0f;
  int grasp_state_ = 0;

  double grasp_vmax_ = 0.0f, grasp_amx_ = 0.0f;
  double vmax_ = 0.0f, amax_ =0.0f;
  double track_dist_ = 0.0f;
  double grasp_degree_init_ = 0.0f;
  double grasp_th_1_ = 0.0f, grasp_th_2_ = 0.0f;



  bool is_debug_ = false;  // 是否启用调试模式


  void rvizTriggerCallback(const geometry_msgs::PoseStamped::ConstPtr &msg) {
    if (cnt_launch_trigger_ != -1) {
        ROS_ERROR("All stage is not finished!!!, Currently total trigger: %d", cnt_launch_trigger_);
    } else {
      launch_trigger_ = true;
      ROS_INFO("Trigger from Rviz!");
    }
    cnt_launch_trigger_ += 1;
  }

  void pub_hover_p(const Eigen::Vector3d& hover_p, const ros::Time& stamp) {  // 发布一个悬停指令，指定悬停点和时间戳
    quadrotor_msgs::PolyTraj traj_msg;
    traj_msg.hover = true;
    traj_msg.hover_p.resize(3);
    for (int i = 0; i < 3; ++i) {
      traj_msg.hover_p[i] = hover_p[i];
    }
    traj_msg.start_time = stamp;
    traj_msg.traj_id = traj_id_++;
    traj_pub_.publish(traj_msg);
  }

  // 将计算出的Trajectory对象转换为quadrotor_msgs::PolyTraj消息并发布，包含轨迹的阶数, 分段时长, 系数以及期望的偏航角(yaw)
  void pub_traj(const Trajectory& traj, const double& yaw, const ros::Time& stamp) {
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
    traj_msg.yaw = yaw;
    traj_pub_.publish(traj_msg);

    // print traj
    // std::cerr << "================> Elastric Tracker Trajectory <================" << std::endl;
    // std::cerr << "traj[0].CoeffMat.size(): " << traj[0].getCoeffMat().size() << std::endl;  // !!@yinloonga: should be 18 (3*6)
    // std::cerr << "msg.id: " << traj_msg.traj_id << std::endl;
    // std::cerr << "msg.start_time: " << traj_msg.start_time << std::endl;
    // std::cerr << "msg.size: " << traj_msg.duration.size() << std::endl;
    // std::cerr << "msg.order: " << static_cast<int>(traj_msg.order) << std::endl;
    // std::cerr << "msg.hover: " << static_cast<int>(traj_msg.hover) << std::endl;
    // std::cerr << "msg.yaw: " << traj_msg.yaw << std::endl;
    // // print hover
    // std::cerr << "msg.hover_p: ";
    // for (const auto& hover : traj_msg.hover_p) {
    //   std::cerr << hover << " ";
    // }
    // std::cerr << std::endl;

    // // print xyz_pieces
    // std::cerr << "msg.xyz_pieces: " << std::endl;
    // std::cerr << "\tduration: " ;
    // for (const auto& time : traj_msg.duration) {
    //   std::cerr << time << " ";
    // }
    // std::cerr << std::endl;
    // std::cerr << "\tcoef_x: ";
    // for (const auto& coef : traj_msg.coef_x) {
    //   std::cerr << coef << " ";
    // }
    // std::cerr << std::endl;
    // std::cerr << "\tcoef_y: ";
    // for (const auto& coef: traj_msg.coef_y) {
    //   std::cerr << coef << " ";
    // }
    // std::cerr << std::endl;
    // std::cerr << "\tcoef_z: ";
    // for (const auto& coef : traj_msg.coef_z) {
    //   std::cerr << coef << " ";
    // }
    // std::cerr << std::endl;


    // if (enable_flygripper_) {
      // convertToFlygripperMsgs(traj, yaw, stamp, traj_msg);
    // }
  }
  
  // todo@yinloonga: 转换quadrotor_msgs::PolyTraj为flygripper_msgs::PolynomialTrajectory
  // void convertToFlygripperMsgs(const Trajectory& traj, const double& yaw, const ros::Time& stamp, const quadrotor_msgs::PolyTraj& tracker_msg) {
  //   flygripper_msgs::PolynomialTrajectory flygripper_msg;  // need to publish

  //   flygripper_msg.header.stamp = stamp;
  //   flygripper_msg.id = tracker_msg.traj_id;

  //   // tracker_msg是通过Min_Jerk规划的轨迹
  //   // ! xyz polynomial
  //   int xyz_pice_num = traj.getPieceNum();
  //   for (int i = 0; i < xyz_pice_num; ++i) {
  //     flygripper_msgs::Polynomial xyz_piece;
  //     xyz_piece.dim = traj[i].getDim();
  //     xyz_piece.order = 5;
  //     xyz_piece.duration = traj[i].getDuration();
  //     CoefficientMat cMat = traj[i].getCoeffMat();
  //     xyz_piece.coeff.assign(cMat.data(), cMat.data() + cMat.rows() * cMat.cols());  // ??@yinloonga: 注意矩阵的维度
  //     flygripper_msg.xyz_pieces.emplace_back(xyz_piece);
  //   }

  //   // ! heading
  //   flygripper_msg.heading_mode = static_cast<int>(flygripper_common::Trajectory::HeadingMode::HEADING_NONE);


  //   // 发布信息
  //   if (enable_flygripper_) flygripper_traj_pub_.publish(flygripper_msg);

  //   // 是否启动终端打印信息
  //   bool enable_print = true;
  //   if (!enable_print) return;

  //   // print flygripper_msg
  //   // std::cerr << "================> quadrotor_msgs::PolyTraj To flygripper_msgs::PolynomialTrajectory<================" << std::endl;
  //   // std::cerr << "msg.id: " << flygripper_msg.id << std::endl;
  //   // std::cerr << "msg.header.stamp: " << flygripper_msg.header.stamp << std::endl;
  //   // std::cerr << "\tpieces: " << flygripper_msg.xyz_pieces.size() << std::endl;

  //   // // print xyz_pieces msgs
  //   // std::cerr << "msg.xyz_pieces[0]: " << std::endl;
  //   // std::cerr << "\tdim: " << flygripper_msg.xyz_pieces[0].dim << std::endl;
  //   // std::cerr << "\torder: " << flygripper_msg.xyz_pieces[0].order << std::endl;
  //   // std::cerr << "\tduration: " << flygripper_msg.xyz_pieces[0].duration << std::endl;
  //   // std::cerr << "\tCoeffMat: \n" << traj[0].getCoeffMat() << std::endl;
  //   // std::cerr << "\tcoeff: ";
  //   // for (const auto& coeff : flygripper_msg.xyz_pieces[0].coeff) {
  //   //   std::cerr << coeff << " ";
  //   // }
  //   // std::cerr << std::endl;

  //   // // print heading_pieces
  //   // if (flygripper_msg.heading_mode == static_cast<int>(flygripper_common::Trajectory::HeadingMode::HEADING_POLY)) {
  //   //   std::cerr << "msh.heading_pieces: " << std::endl;
  //   //   std::cerr << "\tsize: " << flygripper_msg.heading_pieces.size() << std::endl;
  //   //   std::cerr << "\tdim: " << flygripper_msg.heading_pieces[0].dim << std::endl;
  //   //   std::cerr << "\torer: " << flygripper_msg.heading_pieces[0].order << std::endl;
  //   //   std::cerr << "\tduration: " << flygripper_msg.heading_pieces[0].duration << std::endl;
  //   //   std::cerr << "\tcoeff: ";
  //   //   for (const auto &coeff : flygripper_msg.heading_pieces[0].coeff) {
  //   //     std::cerr << coeff << " ";
  //   //   }
  //   //   std::cerr << std::endl;
  //   // }

  // }

  void triger_callback(const geometry_msgs::PoseStampedConstPtr& msgPtr) {
    goal_ << msgPtr->pose.position.x, msgPtr->pose.position.y, 0.9;
    triger_received_ = true;
  }

  void land_triger_callback(const geometry_msgs::PoseStampedConstPtr& msgPtr) {
    land_p_.x() = msgPtr->pose.position.x;
    land_p_.y() = msgPtr->pose.position.y;
    land_p_.z() = msgPtr->pose.position.z;
    land_q_.w() = msgPtr->pose.orientation.w;
    land_q_.x() = msgPtr->pose.orientation.x;
    land_q_.y() = msgPtr->pose.orientation.y;
    land_q_.z() = msgPtr->pose.orientation.z;
    land_triger_received_ = true;
  }

  void odom_callback(const nav_msgs::Odometry::ConstPtr& msgPtr) {
    while (odom_lock_.test_and_set())
      ;
    odom_msg_ = *msgPtr;
    odom_received_ = true;
    odom_lock_.clear();
  }

  void target_callback(const nav_msgs::Odometry::ConstPtr& msgPtr) {
    while (target_lock_.test_and_set())
      ;
    target_msg_ = *msgPtr;

    // 根据yaw角计算偏置
    Eigen::Quaterniond target_q;
    target_q.w() = target_msg_.pose.pose.orientation.w;
    target_q.x() = target_msg_.pose.pose.orientation.x;
    target_q.y() = target_msg_.pose.pose.orientation.y;
    target_q.z() = target_msg_.pose.pose.orientation.z;
    // 四元素转为yaw pitch ,roll
    Eigen::Matrix3d R = target_q.toRotationMatrix();
    double yaw = std::atan2(R(1, 0), R(0,0));
    yaw = yaw * 180 / M_PI;

    bool use_yaw = false;
    if (use_yaw) {  // bug@yinloonga: 实验发现 yaw角的计算存在问题
      target_msg_.pose.pose.position.x += target_msg_.pose.pose.position.x * std::cos(yaw) - target_msg_.pose.pose.position.y * std::sin(yaw);
      target_msg_.pose.pose.position.y += target_msg_.pose.pose.position.x * std::sin(yaw) + target_msg_.pose.pose.position.y * std::cos(yaw);
    } else {
      target_msg_.pose.pose.position.x += track_x_offset_;
      target_msg_.pose.pose.position.y += track_y_offset_;
    }

    target_msg_.pose.pose.position.z += track_z_offset_;
    target_received_ = true;
    target_lock_.clear();
  }

  void gridmap_callback(const quadrotor_msgs::OccMap3dConstPtr& msgPtr) {
    if (!enable_map_) return;
    while (gridmap_lock_.test_and_set())
      ;
    map_msg_ = *msgPtr;
    map_received_ = true;
    gridmap_lock_.clear();
  }

  // NOTE main callback
  // 核心规划逻辑(动态目标跟踪/着陆)
  void plan_timer_callback(const ros::TimerEvent& event) {
    if (wait_grasp_done_ || wait_track_done_) return;  // 如果抓取完成 不再执行路径规划

    heartbeat_pub_.publish(std_msgs::Empty());  // 发布心跳
    if (!enable_map_) map_received_ = true;
    if (!odom_received_ || !map_received_) {  // 检查是否收到Odom和Map
      ROS_WARN_THROTTLE(1.0, "NO Odom Message Received!!!!!!!");
      return;  // 没有则返回
    }
    // obtain state of odom
    while (odom_lock_.test_and_set())
      ;
    auto odom_msg = odom_msg_;  // 使用odom_lock安全地复制最新的无人机Odom
    odom_lock_.clear();
    Eigen::Vector3d odom_p(odom_msg.pose.pose.position.x,
                           odom_msg.pose.pose.position.y,
                           odom_msg.pose.pose.position.z);  // 提取位置odom_p
    Eigen::Vector3d odom_v(odom_msg.twist.twist.linear.x,
                           odom_msg.twist.twist.linear.y,
                           odom_msg.twist.twist.linear.z);  // 提取速度odom_v
    Eigen::Quaterniond odom_q(odom_msg.pose.pose.orientation.w,
                              odom_msg.pose.pose.orientation.x,
                              odom_msg.pose.pose.orientation.y,
                              odom_msg.pose.pose.orientation.z);  // 提取姿态odom_q
    // ROS_INFO("odom_p (x, y, z): (%f, %f, %f)", odom_p.x(), odom_p.y(), odom_p.z());
    if (!triger_received_) {  // 如果没有收到触发消息，则返回
      // return;
    }
    // fix@yinloonga: 等待手柄触发
    // if (!launch_trigger_) {
    //   ROS_WARN_THROTTLE(1.0, "Wait Launch Trigger!!!!!!!!!!!!");
    //   return;
    // }
    if (!target_received_) {  // 如果没有收到目标消息，则返回
      ROS_WARN_THROTTLE(1.0, "No target Message Received!!!!!!!!!!!!");
      return;
    }
    // NOTE obtain state of target
    while (target_lock_.test_and_set())
      ;
    replanStateMsg_.target = target_msg_;  // 使用target_lock安全地复制最新的目标Odom, 然后存入replanStateMsg
    target_lock_.clear();
    Eigen::Vector3d target_p(replanStateMsg_.target.pose.pose.position.x,
                             replanStateMsg_.target.pose.pose.position.y,
                             replanStateMsg_.target.pose.pose.position.z);  // 提取目标位置target_p
    Eigen::Vector3d target_v(replanStateMsg_.target.twist.twist.linear.x,
                             replanStateMsg_.target.twist.twist.linear.y,
                             replanStateMsg_.target.twist.twist.linear.z);  // 提取目标速度target_v
    Eigen::Quaterniond target_q;
    target_q.w() = replanStateMsg_.target.pose.pose.orientation.w;
    target_q.x() = replanStateMsg_.target.pose.pose.orientation.x;
    target_q.y() = replanStateMsg_.target.pose.pose.orientation.y;
    target_q.z() = replanStateMsg_.target.pose.pose.orientation.z;  // 提取目标速度target_q

    // NOTE force-hover: waiting for the speed of drone small enough
    if (force_hover_ && odom_v.norm() > 0.1) {  // 如果force_hover_为true且无人机速度odom_v > 0.1(m/s), 则返回等待无人机减速
      return;
    }

    // NOTE just for landing on the car!
    if (land_triger_received_) {  // 判断是否收到着陆触发
      double dist_diff = std::fabs((target_p - odom_p).norm());
      double dist_diff_x = std::fabs(target_p.x() - odom_p.x());
      track_min_dist_ = std::min(track_min_dist_, dist_diff_x);
      
      ros::Time now = ros::Time::now();
      if ((now - last_warn_time_).toSec() >= 1.0) {
        // 每隔1s输出
        ROS_WARN("Minimum Distance (x) and (len) between target_p and odom_p: %f, %f", track_min_dist_, dist_diff);
        last_warn_time_ = now;

        // print target_p and odom_p x, y, z
        if (is_debug_) ROS_INFO("Drone (x, y, z): (%f, %f, %f)", odom_p.x(), odom_p.y(), odom_p.z());
        if (is_debug_) ROS_INFO("Target (x, y, z): (%f, %f, %f)", target_p.x(), target_p.y(), target_p.z());
      }

      if (dist_diff_x < track_dist_diff_) {  // if无人机和目标距离近(<0.1m), 无人机速度(odom_v)和目标速度(target_v)都小(0.1m/s, 0.2m/s)
        // if (!wait_hover_) {  // 满足着陆模式，则发布悬停指令
        //   pub_hover_p(odom_p, ros::Time::now());
        //   wait_hover_ = true;
        // }
        // ROS_WARN("[planner] HOVERING...");

        // hack@yinloonga: 第二阶段land完成
        if (!wait_track_done_) {
            std_msgs::Bool msg;
            msg.data = true;
            track_done_pub_.publish(msg);
            wait_track_done_ = true;
          ROS_INFO("===========================> Track And Land Done! <===========================");
        }

        return;
      }
      // TODO get the orientation fo target and calculate the pose of landing point
      target_p = target_p + target_q * land_p_;  // 不满足悬停条件，根据目标的当前姿态(target_q)和相对着陆点(land_p_)计算出世界坐标下的实际着陆目标点；更新target_p，并设置wait_hover_为false
      wait_hover_ = false;
      wait_track_done_ = false;
    } else {  // 跟踪模式(非着陆)
      // hack@yinloonga: 调整跟踪高度 
      // target_p.z() += 0.1;  // 将目标点的z坐标抬高1.0m, 确保无人机在目标上方跟踪
      if (is_debug_) ROS_INFO("Drone (x, y, z): (%f, %f, %f)", odom_p.x(), odom_p.y(), odom_p.z());
      if (is_debug_) ROS_INFO("Target (x, y, z): (%f, %f, %f)", target_p.x(), target_p.y(), target_p.z());
      // junjie
      // Eigen::Vector3d project_target_yaw = target_q.toRotationMatrix().col(0);
      // double target_yaw = std::atan2(project_target_yaw.y(), project_target_yaw.x());
      // double grasp_target_yaw = target_yaw + M_PI / 4;
      // double grasp_distance = 0.10;
      // target_p = target_p + 0.12 * Eigen::Vector3d(cos(grasp_target_yaw), sin(grasp_target_yaw), 0) + 0.12 * Eigen::Vector3d(cos(target_yaw), sin(target_yaw), 0);

      // NOTE determin whether to replan
      Eigen::Vector3d dp = target_p - odom_p;  // 计算无人机和目标点之间的距离
      // std::cout << "dist : " << dp.norm() << std::endl;
      double desired_yaw = std::atan2(dp.y(), dp.x());  // 计算无人机和目标点之间的期望偏航角
      Eigen::Vector3d project_yaw = odom_q.toRotationMatrix().col(0);  // NOTE ZYX
      double now_yaw = std::atan2(project_yaw.y(), project_yaw.x());  // 获取无人机当前的偏航角
      if (std::fabs((target_p - odom_p).norm() - tracking_dist_) < tolerance_d_ && /* 无人机与目标距离约等于tracking_dist_(在tolerance_d_范围内) */
          odom_v.norm() < 0.1 && target_v.norm() < 0.2 && /* 无人机和目标的速度够小 */
          std::fabs(desired_yaw - now_yaw) < 0.5 /* 无人机和目标的偏航角误差够小 */) {
        if (!wait_hover_) {  // 满足悬停条件，则发布悬停指令
          pub_hover_p(odom_p, ros::Time::now());
          wait_hover_ = true;
        }
        ROS_WARN("[planner] HOVERING...");
        replanStateMsg_.state = -1;
        replanState_pub_.publish(replanStateMsg_);
        return;
      } else {  // 不满足悬停条件
        wait_hover_ = false;
      }
    }

    if (enable_map_) {
      // NOTE obtain map
      while (gridmap_lock_.test_and_set())
        ;
      gridmapPtr_->from_msg(map_msg_);  // 使用gridmap+lock安全地复制最新的地图消息map_msg_
      replanStateMsg_.occmap = map_msg_;  // 将地图存入replanStateMsg_
      gridmap_lock_.clear();
      prePtr_->setMap(*gridmapPtr_);  // 将更新后的地图设置给预测模块
    }

    // visualize the ray from drone to target
    // 可视化无人机到目标的射线, 根据是否碰撞用不同颜色表示
    if (envPtr_->checkRayValid(odom_p, target_p)) {
      if (enable_vis_) visPtr_->visualize_arrow(odom_p, target_p, "ray", visualization::yellow);
    } else {
      if (enable_vis_) visPtr_->visualize_arrow(odom_p, target_p, "ray", visualization::red);
    }

    // NOTE prediction
    std::vector<Eigen::Vector3d> target_predcit;
    // ros::Time t_start = ros::Time::now();
    // 调用预测模块, 根据目标当前状态target_p和target_v预测其未来一段时间内(由tracking_dur_等参数决定)的轨迹点target_predict
    auto predict_start_tt = std::chrono::high_resolution_clock::now();
    bool generate_new_traj_success = prePtr_->predict(target_p, target_v, target_predcit);
    auto predict_ent_tt = std::chrono::high_resolution_clock::now();
    auto predict_duration = std::chrono::duration_cast<std::chrono::microseconds>(predict_ent_tt - predict_start_tt).count();
    predict_total_time_ += predict_duration;
    cnt_predict_ += 1;
    // ros::Time t_stop = ros::Time::now();
    // std::cout << "predict costs: " << (t_stop - t_start).toSec() * 1e3 << "ms" << std::endl;
    if (generate_new_traj_success) {  // 如果预测成功, 可视化预测出的目标轨迹target_predice和围绕着最终预测点的"可观测边界"(一个圆形区域)
      Eigen::Vector3d observable_p = target_predcit.back();
      if (enable_vis_) visPtr_->visualize_path(target_predcit, "car_predict");
      std::vector<Eigen::Vector3d> observable_margin;
      for (double theta = 0; theta <= 2 * M_PI; theta += 0.01) {
        observable_margin.emplace_back(observable_p + tracking_dist_ * Eigen::Vector3d(cos(theta), sin(theta), 0));
      }
      if (enable_vis_) visPtr_->visualize_path(observable_margin, "observable_margin");
    }

    // NOTE replan state
    Eigen::MatrixXd iniState;
    iniState.setZero(3, 3);
    ros::Time replan_stamp = ros::Time::now() + ros::Duration(0.03);
    double replan_t = (replan_stamp - replan_stamp_).toSec();  // 计算距离上次重规划的时间replan_t
    if (force_hover_ || replan_t > traj_poly_.getTotalDuration()) {  // 如果force_hover_为true, 或者上次规划的轨迹traj_ploly已经执行完毕
      // should replan from the hover state
      iniState.col(0) = odom_p;  // 重置规划的初始状态, 直接使用当前无人机的odom_p
      iniState.col(1) = odom_v;  // 重置规划的初始状态, 直接使用当前无人机的odom_v
    } else {  // 否则, 说明上次的轨迹还在执行中, 为了平滑过渡, 从上次的轨迹traj_poly_在replan_t时刻采样得到位置, 速度和加速度作为重规划的初始状态
      // should replan from the last trajectory
      iniState.col(0) = traj_poly_.getPos(replan_t);
      iniState.col(1) = traj_poly_.getVel(replan_t);
      iniState.col(2) = traj_poly_.getAcc(replan_t);
    }
    replanStateMsg_.header.stamp = ros::Time::now();
    replanStateMsg_.iniState.resize(9);
    Eigen::Map<Eigen::MatrixXd>(replanStateMsg_.iniState.data(), 3, 3) = iniState;

    // NOTE path searching
    Eigen::Vector3d p_start = iniState.col(0);
    std::vector<Eigen::Vector3d> path, way_pts;

    // NOTE calculate time of path searching, corridor generation and optimization

    // static double t_path_ = 0;
    // static double t_corridor_ = 0;
    // static double t_optimization_ = 0;
    // static int times_path_ = 0;
    // static int times_corridor_ = 0;
    // static int times_optimization_ = 0;
    // double t_path = 0;

    if (generate_new_traj_success) {  // 轨迹预测成功
      // ros::Time t_front0 = ros::Time::now();
      auto search_start_tt = std::chrono::high_resolution_clock::now();
      if (land_triger_received_) {  // 着陆模式
        generate_new_traj_success = envPtr_->short_astar(p_start, target_p, path);  // 使用short_astar搜索从p_start(重规划初始位置)到着陆点的最短路径path
      } else {  // 跟踪模式
        // 使用findVisiblePath搜索一条路径path和关键点路径way_pts，使得无人机能沿着该路径移动, 并尽量保持对目标预测轨迹target_predict的可见性
        generate_new_traj_success = envPtr_->findVisiblePath(p_start, target_predcit, way_pts, path);
      }
      auto search_end_tt = std::chrono::high_resolution_clock::now();
      auto search_duraion = std::chrono::duration_cast<std::chrono::microseconds>(search_end_tt - search_start_tt).count();
      search_total_time_ += search_duraion;
      cnt_search_ += 1;
      // ros::Time t_end0 = ros::Time::now();
      // t_path += (t_end0 - t_front0).toSec() * 1e3;
    }

    std::vector<Eigen::Vector3d> visible_ps;
    std::vector<double> thetas;
    std::vector<Eigen::Vector3d> grasp_ps;  // junjie
    Trajectory traj;
    if (generate_new_traj_success) {  // 路径搜索成功
      if (enable_vis_) visPtr_->visualize_path(path, "astar");  // 可视化A*路径path
      if (land_triger_received_) {  // 着陆模式: 将预测的目标点追加到A*路径末尾
        for (const auto& p : target_predcit) {
          path.push_back(p);
        }
      } else {  // 跟踪模式
        // NOTE generate visible regions
        target_predcit.pop_back();  // 移除预测轨迹的最后一个点
        way_pts.pop_back();  // 移除路径点的最后一个点
        Eigen::Vector3d dp = target_p - odom_p;  // junjie
        // ros::Time t_front1 = ros::Time::now();
        // 调用envPtr_的generate_visible_regions函数, 计算出可见区域一系列可见点visible_ps和对应的角度thetas(视场约束)
        auto visible_start_tt = std::chrono::high_resolution_clock::now();
        envPtr_->generate_visible_regions(target_predcit, way_pts,
                                          visible_ps, thetas, target_q, grasp_ps);
        auto visible_end_tt = std::chrono::high_resolution_clock::now();
        auto visible_duration = std::chrono::duration_cast<std::chrono::microseconds>(visible_end_tt - visible_start_tt).count();
        visible_total_time_ += visible_duration;
        cnt_visible_ += 1;

        // ros::Time t_end1 = ros::Time::now();
        // t_path += (t_end1 - t_front1).toSec() * 1e3;
        if (enable_vis_) visPtr_->visualize_pointcloud(visible_ps, "visible_ps");
        if (enable_vis_) visPtr_->visualize_fan_shape_meshes(target_predcit, visible_ps, thetas, "visible_region");

        // TODO change the final state
        std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> rays;
        for (int i = 0; i < (int)way_pts.size(); ++i) {
          rays.emplace_back(target_predcit[i], way_pts[i]);
        }
        if (enable_vis_) visPtr_->visualize_pointcloud(way_pts, "way_pts");
        way_pts.insert(way_pts.begin(), p_start);
        // ros::Time t_front2 = ros::Time::now();
        envPtr_->pts2path(way_pts, path);  // 将路径点way_pts转换回平滑的路径path
        // ros::Time t_end2 = ros::Time::now();
        // t_path += (t_end2 - t_front2).toSec() * 1e3;
      }
      // NOTE corridor generating
      std::vector<Eigen::MatrixXd> hPolys;
      std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> keyPts;

      // ros::Time t_front3 = ros::Time::now();
      // 生成安全飞行走廊: 围绕路径path生成一系列凸多边形(H-多胞体)表示的安全通道hPolys; 可视化走廊hPolys和走廊的关键点keyPts
      auto corridor_start_tt = std::chrono::high_resolution_clock::now();
      envPtr_->generateSFC(path, 2.0, hPolys, keyPts);
      auto corridor_end_tt = std::chrono::high_resolution_clock::now();
      auto corridor_duration = std::chrono::duration_cast<std::chrono::microseconds>(corridor_end_tt - corridor_start_tt).count();
      corridor_total_time_ += corridor_duration;
      cnt_corridor_ += 1;

      // ros::Time t_end3 = ros::Time::now();
      // double t_corridor = (t_end3 - t_front3).toSec() * 1e3;

      envPtr_->visCorridor(hPolys);
      if (enable_vis_) visPtr_->visualize_pairline(keyPts, "keyPts");

      // NOTE trajectory optimization
      // 定义轨迹末端状态: 通常是路径的终点path.back, 目标速度target_v, 零加速度
      Eigen::MatrixXd finState;
      finState.setZero(3, 3);
      finState.col(0) = path.back();
      finState.col(1) = target_v;
      // ros::Time t_front4 = ros::Time::now();
      // 调用轨迹优化器, 求解一条在iniState和finState之间, 穿过hPolys安全走廊, 满足动力学约束
      auto traj_opt_start_tt = std::chrono::high_resolution_clock::now();
      if (land_triger_received_) {
        finState.col(0) = target_predcit.back();
        generate_new_traj_success = trajOptPtr_->generate_traj(iniState, finState, target_predcit, hPolys, traj);
      } else {
        if (is_debug_) ROS_INFO("Drone22222 (x, y, z): (%f, %f, %f)", odom_p.x(), odom_p.y(), odom_p.z());
        if (is_debug_) ROS_INFO("Target22222 (x, y, z): (%f, %f, %f)", target_p.x(), target_p.y(), target_p.z());
        // junjie
        if((target_p - odom_p).norm() <= this->track_dist_ /* track dist */) {

          finState.col(0) = target_predcit.back();
          if((target_p - odom_p).norm() >= grasp_th_1_ + grasp_th_2_ && grasp_state_ == 0) { 
            ROS_ERROR("(target_p - odom_p).norm() [<= 2.0], %f", (target_p - odom_p).norm());
            grasp_degree_ = this->grasp_degree_init_ * std::max(0.0, std::min(1.0, (2.0 - (target_p - odom_p).norm()) / (2.0 - 0.22)));

            // publishMorphing(grasp_degree_);

            tracking_dist_ = 0.001;
            tolerance_d_ = 0.0001;

            trajOptPtr_->vmax_ = this->grasp_vmax_;
            trajOptPtr_->amax_ = this->grasp_amx_;
            // ROS_WARN("11111111111111111111111111");
            
            generate_new_traj_success = trajOptPtr_->generate_traj(iniState, finState,
                                                                    target_predcit, visible_ps, thetas, grasp_ps,
                                                                    hPolys, traj);
          }
          else {
            force_hover_ = true;
            ROS_WARN("[planner] FORCE HOVERING...");

            grasp_degree_ = this->grasp_degree_init_;
            grasp_state_ = 1;
            // publishMorphing(grasp_degree_);

            // ROS_INFO("222222222222222222");

            replanStateMsg_.state = 2;
            replanState_pub_.publish(replanStateMsg_);
            // iniState.col(0)(2) = 2.0;
            pub_hover_p(iniState.col(0), replan_stamp);
            return;
          }

        } else {
          grasp_degree_ = 0.0;
          grasp_state_ = 0;
          // publishMorphing(grasp_degree_);

          finState.col(0) = path.back();
          ROS_ERROR("(target_p - odom_p).norm() [> 2.0], %f", (target_p - odom_p).norm());
          std::cout << "finState: \n" << finState << std::endl;

          trajOptPtr_->vmax_ = this->vmax_;
          trajOptPtr_->amax_ = this->amax_;
          // ROS_ERROR("3333333333333333333333333333");
          generate_new_traj_success = trajOptPtr_->generate_traj(iniState, finState,
                                                                  target_predcit, visible_ps, thetas, grasp_ps,
                                                                  hPolys, traj);
        }  
      }
      //   // 除了满足上述条件, 还要满足视场约束(target_predict, visible_ps, thetas)的最优多项式轨迹traj
      //   generate_new_traj_success = trajOptPtr_->generate_traj(iniState, finState,
      //                                                          target_predcit, visible_ps, thetas,
      //                                                          hPolys, traj);
      // }
      auto traj_opt_end_tt = std::chrono::high_resolution_clock::now();
      auto traj_opt_duration = std::chrono::duration_cast<std::chrono::microseconds>(traj_opt_end_tt - traj_opt_start_tt).count();  
      traj_opt_total_time_ += traj_opt_duration;
      cnt_traj_opt_ += 1;

      // ros::Time t_end4 = ros::Time::now();
      // double t_optimization = (t_end4 - t_front4).toSec() * 1e3;

      // NOTE average calculating time of path searching, corridor generation and optimization

      // t_path_ = (t_path_ * times_path_ + t_path) / (++times_path_);
      // t_corridor_ = (t_corridor_ * times_corridor_ + t_corridor) / (++times_corridor_);
      // t_optimization_ = (t_optimization_ * times_optimization_ + t_optimization) / (++times_optimization_);

      // std::cout << "t_path_: " << t_path_ << " ms" << std::endl;
      // std::cout << "t_corridor_: " << t_corridor_ << " ms" << std::endl;
      // std::cout << "t_optimization_: " << t_optimization_ << " ms" << std::endl;

      // if (enable_vis_) 
      visPtr_->visualize_traj(traj, "traj");
    }

    // 打印时间统计信息
    ros::Time now = ros::Time::now();
    if ((now - last_print_time_).toSec() >= print_interval_ ) {
      last_print_time_ = now;
      // 每隔2s输出一次
      std::cerr << "\n"
          << std::fixed << std::setprecision(3)
          << "================ Trajectory Tracking Result ================\n"
          << "  Prediction Time:         : "<< predict_total_time_ / cnt_predict_ << " us\n"
          << "  Path Searching Time:     : " << search_total_time_ / cnt_search_ << " us\n"
          << "  Generate Visible Region  : " << visible_total_time_ / cnt_visible_ << " us\n"
          << "  Generate SFC             : " << corridor_total_time_ / cnt_corridor_ << " us\n"
          << "  Trajectory Optimization  : " << corridor_total_time_ / cnt_corridor_ << " us\n"
          << "------------------------------------------------------------\n"
          << "  Status                : \033[32mTrajectory tracking succeeded.\033[0m\n"
          << "============================================================\n"
          << std::endl;
    }

    // NOTE collision check
    bool valid = false;
    if (generate_new_traj_success) {
      valid = validcheck(traj, replan_stamp);  // 碰撞检查:检查生成的轨迹traj在未来一段时间(如1s)内是否与障碍物发生碰撞
    } else {
      replanStateMsg_.state = -2;
      replanState_pub_.publish(replanStateMsg_);
    }
    if (valid) {  // 如果新规划的轨迹有效valid
      force_hover_ = false;
      // hack@yinloonga: 终端频繁输出 建议关闭
      // ROS_WARN("[planner] REPLAN SUCCESS");
      replanStateMsg_.state = 0;
      replanState_pub_.publish(replanStateMsg_);
      Eigen::Vector3d dp = target_p + target_v * 0.03 - iniState.col(0);
      // NOTE : if the drone is going to unknown areas, watch that direction
      // Eigen::Vector3d un_known_p = traj.getPos(1.0);
      // if (gridmapPtr_->isUnKnown(un_known_p)) {
      //   dp = un_known_p - odom_p;
      // }
      double yaw = std::atan2(dp.y(), dp.x());  // 计算期望偏航角yaw
      if (land_triger_received_) {
        yaw = 2 * std::atan2(target_q.z(), target_q.w());
      }
      pub_traj(traj, yaw, replan_stamp);  // 发布新轨迹
      traj_poly_ = traj; // 更新当前执行的轨迹
      replan_stamp_ = replan_stamp;  // 更新起始时间
    } else if (force_hover_) {  // 轨迹无效且force_hover_为true
      // 重规划失败，且当前处于强制悬停恢复状态 
      // bug@yinloonga: 在land阶段时 会触发当前代码
      ROS_ERROR("[planner] REPLAN FAILED, HOVERING...");
      replanStateMsg_.state = 1;
      replanState_pub_.publish(replanStateMsg_);  // 发布失败状态, 继续保持悬停
      return;
    } else if (validcheck(traj_poly_, replan_stamp_)) {  // 重规划失败, 且上次规划的轨迹traj_poly_也变得不安全了
      force_hover_ = true;
      ROS_FATAL("[planner] EMERGENCY STOP!!!");
      replanStateMsg_.state = 2;
      replanState_pub_.publish(replanStateMsg_);
      pub_hover_p(iniState.col(0), replan_stamp);  // 发布一个立即在当前无人机位置(重规划的起始点)悬停的指令
      return;
    } else {  // 重规划失败, 且上次规划的轨迹traj_poly_仍然有效
      ROS_ERROR("[planner] REPLAN FAILED, EXECUTE LAST TRAJ...");
      replanStateMsg_.state = 3;
      replanState_pub_.publish(replanStateMsg_);  // 不发布新轨迹, 让无人机继续执行上一条轨迹traj_poly_
      return;  // current generated traj invalid but last is valid
    }
    // if (enable_vis_) 
    visPtr_->visualize_traj(traj, "traj");  // 可视化轨迹

    // fix@yinloonga: check是否满足降落条件
    if (std::fabs((target_p - odom_p).norm() - tracking_dist_) < tolerance_d_ && !land_triger_received_) {
      ROS_WARN("=========================Track Met And Ready Grasp (Old Not publish land_triger_received_=True)======================");
      // land_triger_received_ = true;

      // fix@yinloonga: 借用下wait_track_done_
      // if (!wait_track_done_ && false) {
      //   std_msgs::Bool msg;
      //   msg.data = true;
      //   // track_done_pub_.publish(msg);  // 发布抓取完成的消息
      //   wait_track_done_ = true;  // 借用下wait_track_done_
      // }
    }
  }

  // 核心规划逻辑(静态目标) 
  void fake_timer_callback(const ros::TimerEvent& event) {
    heartbeat_pub_.publish(std_msgs::Empty());  // 发布一个心跳消息, 表明节点仍在运行
    if (!enable_map_) map_received_ = true;
    if (!odom_received_ || !map_received_) {  // 检查无人机是否收到必要的数据(无人机里程计和地图)
      return;  // 未收到 则返回
    }
    // obtain state of odom
    while (odom_lock_.test_and_set())
      ;
    auto odom_msg = odom_msg_;  // 使用odom_lock_原子锁安全地复制最新的无人机Odom消息
    odom_lock_.clear();
    Eigen::Vector3d odom_p(odom_msg.pose.pose.position.x,
                           odom_msg.pose.pose.position.y,
                           odom_msg.pose.pose.position.z);  // 提取位置odom_p
    Eigen::Vector3d odom_v(odom_msg.twist.twist.linear.x,
                           odom_msg.twist.twist.linear.y,
                           odom_msg.twist.twist.linear.z);  // 提取速度odom_v
    if (!triger_received_) {  // 检查是否收到了初始的触发消息(通常用于设置静态目标点goal_)
      return;  // 如果没收到, 则返回
    }
    // NOTE force-hover: waiting for the speed of drone small enough
    if (force_hover_ && odom_v.norm() > 0.1) { // 悬停检查, 或者无人机仍以较快速度移动(大于0.1m/s); 防止无人机在应该停止但仍在移动的情况下进行规划
      return;
    }

    // NOTE local goal 局部目标点计算
    Eigen::Vector3d local_goal;  // 声明一个变量, 用于存储本次迭代规划所使用的中间目标点
    Eigen::Vector3d delta = goal_ - odom_p;  // 计算无人机从当前位置指向目标点的向量delta
    if (delta.norm() < 15) {  // 如果最终目标点相对较近(距离小于15m)
      local_goal = goal_;  // 那么本次规划的目标就是最终的goal_
    } else {  // 如果目标点较远
      local_goal = delta.normalized() * 15 + odom_p;  // 将local_goal_设置为沿着指向最终goal_的方向, 距离无人机当前位置15m的点
    }

    if (enable_map_) {
        // NOTE obtain map  获取地图
        while (gridmap_lock_.test_and_set())
          ;
        gridmapPtr_->from_msg(map_msg_);  // 通过原子锁安全地复制最新的map_msg_
        replanStateMsg_.occmap = map_msg_;  // 存储地图信息
        gridmap_lock_.clear();
    }

    // NOTE determin whether to replan
    bool no_need_replan = false;
    if (!force_hover_ && !wait_hover_) {
      double last_traj_t_rest = traj_poly_.getTotalDuration() - (ros::Time::now() - replan_stamp_).toSec();  // 计算上一条轨迹的剩余时间
      bool new_goal = (local_goal - traj_poly_.getPos(traj_poly_.getTotalDuration())).norm() > tracking_dist_;  // 检查当前的目标点local_goal是否与上次规划的轨迹末端点之间的距离大于tracking_dist_，如果是，则说明需要重新规划
      if (!new_goal) {  // 如果局部目标点没有显著变化
        if (last_traj_t_rest < 1.0) {  // 如果上一条轨迹快要执行完了(剩余时间小于1s), 则认为接近目标, 暂时不重新规划
          ROS_WARN("[planner] NEAR GOAL...");
          no_need_replan = true;
        } else if (validcheck(traj_poly_, replan_stamp_, last_traj_t_rest)) {  // 如果上一条轨迹还有较长的执行时间, 则调用validcheck检查这条旧轨迹在其剩余时间内是否仍然是无碰撞的
          ROS_WARN("[planner] NO NEED REPLAN...");
          double t_delta = traj_poly_.getTotalDuration() < 1.0 ? traj_poly_.getTotalDuration() : 1.0;  // 如果旧轨迹有效, 就重新发布一条旧轨迹(traj_poly_)
          // 但更新其期望偏航角
          double t_yaw = (ros::Time::now() - replan_stamp_).toSec() + t_delta;
          Eigen::Vector3d un_known_p = traj_poly_.getPos(t_yaw);
          Eigen::Vector3d dp = un_known_p - odom_p;
          double yaw = std::atan2(dp.y(), dp.x());
          pub_traj(traj_poly_, yaw, replan_stamp_);
          no_need_replan = true;
        }
      }
    }
    // NOTE determin whether to pub hover 判断是否需要悬停(到达目标)
    // 如果无人机到达目标点(距离小于tracking_dist_ + tolerance_d_)且速度小于0.1m/s, 则发布悬停指令
    if ((goal_ - odom_p).norm() < tracking_dist_ + tolerance_d_ && odom_v.norm() < 0.1) {
      if (!wait_hover_) {  // 如果无人机之前没有命令它悬停
        pub_hover_p(odom_p, ros::Time::now());  // 发布一个在当前位置odom_p悬停的指令
        wait_hover_ = true;  // 设置标志 表示无人机现在应该处于悬停
      }
      ROS_WARN("[planner] HOVERING...");
      replanStateMsg_.state = -1;
      replanState_pub_.publish(replanStateMsg_);
      return;
    } else {  // 如果距离目标还远或速度较快
      wait_hover_ = false;
    }
    if (no_need_replan) {  // 如果之前的优化步骤决定本次不需要重规划 则直接返回
      return;
    }

    // NOTE replan state
    Eigen::MatrixXd iniState;  // 声明一个矩阵变量iniState, 用于存储重规划的初始状态: 位置 速度 加速度
    iniState.setZero(3, 3);
    ros::Time replan_stamp = ros::Time::now() + ros::Duration(0.03);  // 设置新轨迹的预期开始时间 通常是给当前时间加上一个小的延迟(0.03s) 给计算留出时间
    double replan_t = (replan_stamp - replan_stamp_).toSec();  // 计算自上一次轨迹开始执行以来经过的时间
    if (force_hover_ || replan_t > traj_poly_.getTotalDuration()) {  // 如果处于强制悬停状态 或者上一次的轨迹已经完全执行完毕
      // should replan from the hover state
      iniState.col(0) = odom_p;  // 使用当前的里程计信息作为新规划的初始状态
      iniState.col(1) = odom_v;
    } else {  // 如果上一次的轨迹仍在执行中
      // should replan from the last trajectory
      // 从上一次的轨迹(traj_poly_)中采样replan_t时刻的位置 速度 加速度; 确保新旧轨迹之间的平滑过渡(位置 速度 加速度连续)
      iniState.col(0) = traj_poly_.getPos(replan_t);
      iniState.col(1) = traj_poly_.getVel(replan_t);
      iniState.col(2) = traj_poly_.getAcc(replan_t);
    }
    replanStateMsg_.header.stamp = ros::Time::now();
    replanStateMsg_.iniState.resize(9);
    Eigen::Map<Eigen::MatrixXd>(replanStateMsg_.iniState.data(), 3, 3) = iniState;

    // NOTE generate an extra corridor 生成额外的走廊
    Eigen::Vector3d p_start = iniState.col(0);
    bool need_extra_corridor = iniState.col(1).norm() > 1.0;  // 检查本次重规划的初始速度是否较高 大于1.0m/s
    Eigen::MatrixXd hPoly;
    std::pair<Eigen::Vector3d, Eigen::Vector3d> line;
    if (need_extra_corridor) {
      // 沿着初始速度方向生成一个短的 直线的安全走廊(hPoly) 这个走廊从p_start开始 向前延申最多1m 或者直到碰到障碍物为止
      Eigen::Vector3d v_norm = iniState.col(1).normalized();
      line.first = p_start;
      double step = 0.1;
      for (double dx = step; dx < 1.0; dx += step) {
        p_start += step * v_norm;
        if (enable_map_) {
          if (gridmapPtr_->isOccupied(p_start)) {
            p_start -= step * v_norm;
            break;
          }
        }
      }
      line.second = p_start;
      envPtr_->generateOneCorridor(line, 2.0, hPoly);
    }
    // NOTE path searching  路径搜索(A*)
    std::vector<Eigen::Vector3d> path;  // 用于存储搜索到的路径点
    bool generate_new_traj_success = envPtr_->astar_search(p_start, local_goal, path);  // 调用evnPtr_模块提供的A*算法 在地图中搜索一条从起始点p_start到计算出的local_goal的无碰撞路径
    Trajectory traj;
    if (generate_new_traj_success) {  // 如果路径搜索成功
      if (enable_vis_) visPtr_->visualize_path(path, "astar");  // 可视化路径
      // NOTE corridor generating
      std::vector<Eigen::MatrixXd> hPolys;
      std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> keyPts;
      // 围绕A*路径生成一系列安全飞行走廊(safe flight corridor) 通常表示为凸多面体hPolys 
      envPtr_->generateSFC(path, 2.0, hPolys, keyPts);
      if (need_extra_corridor) {  // 如果之前生成了额外的走廊 将其插入到hPolys列表的开头
        hPolys.insert(hPolys.begin(), hPoly);
        keyPts.insert(keyPts.begin(), line);
      }
      envPtr_->visCorridor(hPolys);  // 可视化生成的安全走廊
      if (enable_vis_) visPtr_->visualize_pairline(keyPts, "keyPts");

      // NOTE trajectory optimization
      Eigen::MatrixXd finState;  // 定义轨迹期望的末端状态
      finState.setZero(3, 3);
      // 末端位置设置为A*路径的最后一个点(也就是local_goal) 末端速度和加速度通常设为零 表示希望在local_goal平稳停止
      finState.col(0) = path.back();
      // return;
      // 调用轨迹优化器trajOptrPtr 寻找一条平滑且动力学可行的多项式轨迹
      // 轨迹需要满足: 从iniState出发 到达finState 并且全程保持在生成到的安全走廊hPolys内
      generate_new_traj_success = trajOptPtr_->generate_traj(iniState, finState, hPolys, traj);
      // if (enable_vis_) 
      visPtr_->visualize_traj(traj, "traj");  // 可视化得到的轨迹
    }

    // NOTE collision check 碰撞检查与发布/处理失败
    bool valid = false;
    if (generate_new_traj_success) {  // 用于标记新生成的轨迹是否有效
      valid = validcheck(traj, replan_stamp);  // 对新生成的轨迹traj进行最终的 更细致的碰撞检查 通常检查未来一段时间内(如1s)内的路径点
    } else { // 如果路径搜索或优化在前面步骤失败了
      replanStateMsg_.state = -2;
      replanState_pub_.publish(replanStateMsg_);
    }
    if (valid) {  // 如果新生成的轨迹traj是好的 无碰撞的
      force_hover_ = false;  // 清楚强制悬停标志
      ROS_WARN("[planner] REPLAN SUCCESS");
      replanStateMsg_.state = 0;
      replanState_pub_.publish(replanStateMsg_);
      // NOTE : if the trajectory is known, watch that direction
      // 计算期望的偏航角yaw
      Eigen::Vector3d un_known_p = traj.getPos(traj.getTotalDuration() < 1.0 ? traj.getTotalDuration() : 1.0);
      Eigen::Vector3d dp = un_known_p - odom_p;
      double yaw = std::atan2(dp.y(), dp.x());
      pub_traj(traj, yaw, replan_stamp);  // 发布新轨迹
      // 将新轨迹及其开始时间保存为当前正在执行的轨迹
      traj_poly_ = traj;
      replan_stamp_ = replan_stamp;
    } else if (force_hover_) {  // 新轨迹无效 并且当前处于强制悬停状态
      ROS_ERROR("[planner] REPLAN FAILED, HOVERING...");
      replanStateMsg_.state = 1;
      replanState_pub_.publish(replanStateMsg_);
      return;
    } else if (!validcheck(traj_poly_, replan_stamp_)) {  // 新规及无效 并且检查上一条轨迹traj_poly_也变得无效了（由于地图更新导致旧路径上出现障碍物)
      force_hover_ = true;
      ROS_FATAL("[planner] EMERGENCY STOP!!!");
      replanStateMsg_.state = 2;
      replanState_pub_.publish(replanStateMsg_);
      pub_hover_p(iniState.col(0), replan_stamp);  // 发布一个立即在重规划起始位置(iniState.col(0))悬停的指令
      return;
    } else { // 新轨迹无效 但是检查发现上一条轨迹traj_poly_仍然是有效的
      ROS_ERROR("[planner] REPLAN FAILED, EXECUTE LAST TRAJ...");
      replanStateMsg_.state = 3;
      replanState_pub_.publish(replanStateMsg_);
      return;  // current generated traj invalid but last is valid
    }
    // if (enable_vis_) 
    visPtr_->visualize_traj(traj, "traj");  // 可视化最终决定要执行的轨迹
  }

  // Deubg模式下的定时回调, 通常用于加载保存的状态并重现规划过程
  void debug_timer_callback(const ros::TimerEvent& event) {
    inflate_gridmap_pub_.publish(replanStateMsg_.occmap);
    Eigen::MatrixXd iniState;
    iniState.setZero(3, 3);
    ros::Time replan_stamp = ros::Time::now() + ros::Duration(0.03);

    iniState = Eigen::Map<Eigen::MatrixXd>(replanStateMsg_.iniState.data(), 3, 3);
    Eigen::Vector3d target_p(replanStateMsg_.target.pose.pose.position.x,
                             replanStateMsg_.target.pose.pose.position.y,
                             replanStateMsg_.target.pose.pose.position.z);
    Eigen::Vector3d target_v(replanStateMsg_.target.twist.twist.linear.x,
                             replanStateMsg_.target.twist.twist.linear.y,
                             replanStateMsg_.target.twist.twist.linear.z);
    // std::cout << "target_p: " << target_p.transpose() << std::endl;
    // std::cout << "target_v: " << target_v.transpose() << std::endl;

    // visualize the target and the drone velocity
    if (enable_vis_) visPtr_->visualize_arrow(iniState.col(0), iniState.col(0) + iniState.col(1), "drone_vel");
    if (enable_vis_) visPtr_->visualize_arrow(target_p, target_p + target_v, "target_vel");

    // visualize the ray from drone to target
    if (envPtr_->checkRayValid(iniState.col(0), target_p)) {
      if (enable_vis_) visPtr_->visualize_arrow(iniState.col(0), target_p, "ray", visualization::yellow);
    } else {
      if (enable_vis_) visPtr_->visualize_arrow(iniState.col(0), target_p, "ray", visualization::red);
    }

    // NOTE prediction
    std::vector<Eigen::Vector3d> target_predcit;
    if (enable_map_) {
      if (gridmapPtr_->isOccupied(target_p)) {
        std::cout << "target is invalid!" << std::endl;
        assert(false);
      }
    }
    bool generate_new_traj_success = prePtr_->predict(target_p, target_v, target_predcit);

    if (generate_new_traj_success) {
      Eigen::Vector3d observable_p = target_predcit.back();
      if (enable_vis_) visPtr_->visualize_path(target_predcit, "car_predict");
      std::vector<Eigen::Vector3d> observable_margin;
      for (double theta = 0; theta <= 2 * M_PI; theta += 0.01) {
        observable_margin.emplace_back(observable_p + tracking_dist_ * Eigen::Vector3d(cos(theta), sin(theta), 0));
      }
      if (enable_vis_) visPtr_->visualize_path(observable_margin, "observable_margin");
    }

    // NOTE path searching
    Eigen::Vector3d p_start = iniState.col(0);
    std::vector<Eigen::Vector3d> path, way_pts;
    if (generate_new_traj_success) {
      generate_new_traj_success = envPtr_->findVisiblePath(p_start, target_predcit, way_pts, path);
    }

    std::vector<Eigen::Vector3d> visible_ps;
    std::vector<double> thetas;
    Trajectory traj;
    if (generate_new_traj_success) {
      if (enable_vis_) visPtr_->visualize_path(path, "astar");
      // NOTE generate visible regions
      target_predcit.pop_back();
      way_pts.pop_back();
      envPtr_->generate_visible_regions(target_predcit, way_pts,
                                        visible_ps, thetas);
      if (enable_vis_) visPtr_->visualize_pointcloud(visible_ps, "visible_ps");
      if (enable_vis_) visPtr_->visualize_fan_shape_meshes(target_predcit, visible_ps, thetas, "visible_region");
      // NOTE corridor generating
      std::vector<Eigen::MatrixXd> hPolys;
      std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> keyPts;
      // TODO change the final state
      std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> rays;
      for (int i = 0; i < (int)way_pts.size(); ++i) {
        rays.emplace_back(target_predcit[i], way_pts[i]);
      }
      if (enable_vis_) visPtr_->visualize_pointcloud(way_pts, "way_pts");
      way_pts.insert(way_pts.begin(), p_start);
      envPtr_->pts2path(way_pts, path);
      if (enable_vis_) visPtr_->visualize_path(path, "corridor_path");
      envPtr_->generateSFC(path, 2.0, hPolys, keyPts);
      envPtr_->visCorridor(hPolys);
      if (enable_vis_) visPtr_->visualize_pairline(keyPts, "keyPts");

      // NOTE trajectory optimization
      Eigen::MatrixXd finState;
      finState.setZero(3, 3);
      finState.col(0) = path.back();
      finState.col(1) = target_v;

      generate_new_traj_success = trajOptPtr_->generate_traj(iniState, finState,
                                                             target_predcit, visible_ps, thetas,
                                                             hPolys, traj);
      // if (enable_vis_) 
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
    double delta_t = check_dur < traj.getTotalDuration() ? check_dur : traj.getTotalDuration();
    for (double t = t0; t < t0 + delta_t; t += 0.1) {
      Eigen::Vector3d p = traj.getPos(t);
      check_pts.push_back(p);
      if (enable_map_) {
        if (gridmapPtr_->isOccupied(p)) {
          invalid_pts.push_back(p);
        }
      }
    }
    if (enable_vis_) visPtr_->visualize_path(invalid_pts, "invalid_pts");
    if (enable_vis_) visPtr_->visualize_path(check_pts, "check_pts");
    valid = validcheck(traj, replan_stamp);
    if (!valid) {
      std::cout << "invalid!" << std::endl;
    }
  }

  // 检查给定轨迹在未来一段时间内(默认1s)是否与当前地图发生碰撞
  bool validcheck(const Trajectory& traj, const ros::Time& t_start, const double& check_dur = 1.0) {
    double t0 = (ros::Time::now() - t_start).toSec();
    t0 = t0 > 0.0 ? t0 : 0.0;
    double delta_t = check_dur < traj.getTotalDuration() ? check_dur : traj.getTotalDuration();
    for (double t = t0; t < t0 + delta_t; t += 0.01) {
      Eigen::Vector3d p = traj.getPos(t);
      if (enable_map_) {
        if (gridmapPtr_->isOccupied(p)) {
          return false;
        }
      }
    }
    return true;
  }

  void graspDoneCallback(const std_msgs::Bool::ConstPtr& msg) {
    ROS_WARN("graspDoneCallback: %d", msg->data);
    wait_grasp_done_ = true;
  }

  // void publishMorphing(double morph_shrink_pos_) {
  //   flygripper_msgs::ActuatorCommand actuator_command_msg;
  //   actuator_command_msg.header.stamp = ros::Time::now();
  //   actuator_command_msg.header.frame_id = "robot";
  //   actuator_command_msg.mode = flygripper_msgs::ActuatorCommand::MODE_POSITION;
  //   actuator_command_msg.reference_velocity = 0.0;
  //   actuator_command_msg.reference_effort = 0.0;

  //   actuator_command_msg.reference_position = morph_shrink_pos_;
  //   actuator_pub_.publish(actuator_command_msg);
  // }

  void init(ros::NodeHandle& nh) {
    // set parameters of planning
    int plan_hz = 10;
    nh.getParam("plan_hz", plan_hz);
    nh.getParam("tracking_dur", tracking_dur_);
    nh.getParam("tracking_dist", tracking_dist_);
    nh.getParam("tolerance_d", tolerance_d_);
    nh.getParam("debug", debug_);
    nh.getParam("fake", fake_);
    nh.getParam("track_dist_diff", track_dist_diff_);
    ROS_WARN("ROS Param (track_dist_diff): %f", track_dist_diff_);
    nh.getParam("track_x_offset", track_x_offset_);
    nh.getParam("track_y_offset", track_y_offset_);
    nh.getParam("track_z_offset", track_z_offset_);
    ROS_WARN("Track Offset (x, y, z): %f, %f, %f", track_x_offset_, track_y_offset_, track_z_offset_);

    nh.getParam("vmax", vmax_);
    nh.getParam("amax", amax_);
    ROS_WARN("Track (vmax, amax): %f, %f", vmax_, amax_);

    nh.getParam("grasp_vmax", grasp_vmax_);
    nh.getParam("grasp_amax", grasp_amx_);
    ROS_WARN("Track (grasp_vmax, grasp_amax): %f, %f", grasp_vmax_, grasp_amx_);

    nh.getParam("track_dist", track_dist_);
    ROS_WARN("Track Disttance: %f", track_dist_);

    nh.getParam("grasp_degree_init", grasp_degree_init_);
    nh.getParam("grasp_th_1", grasp_th_1_);
    nh.getParam("grasp_th_2", grasp_th_2_);
    ROS_WARN("Grasp_Degree_Init: %f, grasp_th_1: %f, grasp_th_2: %f", grasp_degree_init_, grasp_th_1_, grasp_th_2_);


    gridmapPtr_ = std::make_shared<mapping::OccGridMap>();
    envPtr_ = std::make_shared<env::Env>(nh, gridmapPtr_);
    visPtr_ = std::make_shared<visualization::Visualization>(nh);
    trajOptPtr_ = std::make_shared<traj_opt::TrajOpt>(nh);
    prePtr_ = std::make_shared<prediction::Predict>(nh);

    heartbeat_pub_ = nh.advertise<std_msgs::Empty>("heartbeat", 10);
    traj_pub_ = nh.advertise<quadrotor_msgs::PolyTraj>("trajectory", 1);
    // if (enable_flygripper_) flygripper_traj_pub_ = nh.advertise<flygripper_msgs::PolynomialTrajectory>("/flygripper/trajectory", 1);
    replanState_pub_ = nh.advertise<quadrotor_msgs::ReplanState>("replanState", 1);

    if (debug_) {
      plan_timer_ = nh.createTimer(ros::Duration(1.0 / plan_hz), &Nodelet::debug_timer_callback, this);
      // TODO read debug data from files
      wr_msg::readMsg(replanStateMsg_, ros::package::getPath("planning") + "/../../../debug/replan_state.bin");
      inflate_gridmap_pub_ = nh.advertise<quadrotor_msgs::OccMap3d>("gridmap_inflate", 10);
      gridmapPtr_->from_msg(replanStateMsg_.occmap);
      prePtr_->setMap(*gridmapPtr_);
      std::cout << "plan state: " << replanStateMsg_.state << std::endl;
    } else if (fake_) {  // ture表示使用静态目标点规划模式
      plan_timer_ = nh.createTimer(ros::Duration(1.0 / plan_hz), &Nodelet::fake_timer_callback, this);
    } else {  // false表示使用动态目标点跟踪模式
      plan_timer_ = nh.createTimer(ros::Duration(1.0 / plan_hz), &Nodelet::plan_timer_callback, this);
    }
    gridmap_sub_ = nh.subscribe<quadrotor_msgs::OccMap3d>("gridmap_inflate", 1, &Nodelet::gridmap_callback, this, ros::TransportHints().tcpNoDelay());
    odom_sub_ = nh.subscribe<nav_msgs::Odometry>("odom", 10, &Nodelet::odom_callback, this, ros::TransportHints().tcpNoDelay());
    target_sub_ = nh.subscribe<nav_msgs::Odometry>("target", 10, &Nodelet::target_callback, this, ros::TransportHints().tcpNoDelay());
    triger_sub_ = nh.subscribe<geometry_msgs::PoseStamped>("triger", 10, &Nodelet::triger_callback, this, ros::TransportHints().tcpNoDelay());
    land_triger_sub_ = nh.subscribe<geometry_msgs::PoseStamped>("land_triger", 10, &Nodelet::land_triger_callback, this, ros::TransportHints().tcpNoDelay());

    // fix@yinloonga: 和track_grasp的交互
    track_done_pub_ = nh.advertise<std_msgs::Bool>("track_done", 10);
    grasp_done_sub_ = nh.subscribe<std_msgs::Bool>(
      "/flygripper/grasp_done", 10, &Nodelet::graspDoneCallback, this,
      ros::TransportHints().tcpNoDelay());

    // fix@yinloonga: 初始化降落点信息
    land_p_ = Eigen::Vector3d(0, 0, 0);
    land_q_.w() = 1.0f, land_q_.x() = 0.0f, land_q_.y() = 0.0f, land_q_.z() = 0.0f;
    last_warn_time_ = ros::Time::now();
    rviz_trigger_sub_ = nh.subscribe<geometry_msgs::PoseStamped>("trigger", 10, &Nodelet::rvizTriggerCallback, this, ros::TransportHints().tcpNoDelay());
    nh.getParam("is_debug", is_debug_);
    nh.getParam("print_interval", print_interval_);

    last_print_time_ = ros::Time::now();
    // actuator_pub_ = nh.advertise<flygripper_msgs::ActuatorCommand>("/flygripper/actuator_command", 10);

    ROS_WARN("Planning node initialized!");
  }

 public:
  void onInit(void) {
    ros::NodeHandle nh(getMTPrivateNodeHandle());
    initThread_ = std::thread(std::bind(&Nodelet::init, this, nh));
  }
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

}  // namespace planning

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(planning::Nodelet, nodelet::Nodelet);