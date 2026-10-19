#include "flygripper_planner/traj_opt.h"
#include "flygripper_planner/visualizer.hpp"

#include "flygripper_common/ros/parameters.hpp"
#include "flygripper_common/type/odometry.hpp"
#include "flygripper_common/type/trajectory.hpp"
#include <fsto/point.h>

using namespace std;
using namespace ros;
using namespace Eigen;

MavGlobalPlanner::MavGlobalPlanner(Config &conf, NodeHandle &nh_)
    : config(conf), nh(nh_), odomInitialized(false) {
  goalSub = nh.subscribe("goal", 1, &MavGlobalPlanner::goalCallBack, this,
                         TransportHints().tcpNoDelay());
  odomSub = nh.subscribe("odom", 1, &MavGlobalPlanner::odomCallBack, this,
                         TransportHints().tcpNoDelay());
  trajPub =
      nh.advertise<flygripper_msgs::PolynomialTrajectory>("trajectory", 1);
}

void MavGlobalPlanner::goalCallBack(
    const geometry_msgs::PoseStamped::ConstPtr &msg) {
  if (!odomInitialized) {
    ROS_INFO("odom not init")
    return;
  }

  // Initialize start and goal states with zero velocity and acceleration
  Eigen::Vector3d startTrans = curOdom;
  Eigen::Vector3d goalTrans(msg->pose.position.x, msg->pose.position.y,
                            msg->pose.position.z);

  Eigen::Matrix3d iniState;
  Eigen::Matrix3d finState;
  iniState << startTrans, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero();
  finState << goalTrans, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero();

  // Create a loose global bounding box to satisfy solver constraints
  Eigen::Vector3d pMin =
      startTrans.cwiseMin(goalTrans) - Eigen::Vector3d(10.0, 10.0, 10.0);
  Eigen::Vector3d pMax =
      startTrans.cwiseMax(goalTrans) + Eigen::Vector3d(10.0, 10.0, 10.0);
  Eigen::MatrixXd hPoly(6, 6);
  hPoly << Eigen::Vector3d::UnitX(), Eigen::Vector3d::UnitY(),
      Eigen::Vector3d::UnitZ(), -Eigen::Vector3d::UnitX(),
      -Eigen::Vector3d::UnitY(), -Eigen::Vector3d::UnitZ(), pMax, pMax, pMax,
      pMin, pMin, pMin;

  std::vector<Eigen::MatrixXd> hPolys;
  hPolys.push_back(hPoly);

  // Setup optimizer parameters and solve for the trajectory
  double vmax = config.maxVelRate;
  double amax = config.maxAccRate;
  Eigen::Vector3d chi(config.chiVec[0], config.chiVec[1], config.chiVec[2]);
  double res = 2.0;
  int itg = 8;

  GCOPTER nonlinOpt;
  PolynomialTrajectory<3, 7> traj;

  if (!nonlinOpt.setup(config.weightT, 1.0, iniState, finState, hPolys, res,
                       itg, vmax, amax, chi, config.c2Diffeo)) {
    ROS_WARN("Optimization setup failed!");
    return;
  }

  nonlinOpt.optimize(traj, config.relCostTol);

  if (traj.getPieceNum() > 0) {
    flygripper_msgs::PolynomialTrajectory trajMsg;
    Time stamp = odomStamp;
    polynomialTrajConverter(traj, trajMsg, stamp, config.headingMode);
    trajPub.publish(trajMsg);
    ROS_INFO("publish traj")
  }
}

void MavGlobalPlanner::odomCallBack(const nav_msgs::Odometry::ConstPtr &msg) {
  curOdom(0) = msg->pose.pose.position.x;
  curOdom(1) = msg->pose.pose.position.y;
  curOdom(2) = msg->pose.pose.position.z;
  odomStamp = msg->header.stamp;
  odomInitialized = true;
}

void MavGlobalPlanner::polynomialTrajConverter(
    const PolynomialTrajectory<3, 7> &traj,
    flygripper_msgs::PolynomialTrajectory &msg, Time &iniStamp,
    const int &headingMode) {
  static int traj_id = 1;
  msg.id = traj_id;
  msg.header.stamp = iniStamp;
  int piece_num = traj.getPieceNum();

  for (int i = 0; i < piece_num; ++i) {
    flygripper_msgs::Polynomial piece;
    piece.dim = traj[i].getDim();
    piece.order = traj[i].getDegree();
    piece.duration = traj[i].getDuration();
    auto cMat = traj[i].getCoeffMat();
    piece.coeff.assign(cMat.data(), cMat.data() + cMat.rows() * cMat.cols());
    msg.xyz_pieces.emplace_back(piece);
  }

  if (headingMode >= 0 && headingMode <= 2) {
    msg.heading_mode = headingMode;
  } else {
    ROS_WARN("Invalid heading mode %d. Set to HEADING_NONE default",
             headingMode);
    msg.heading_mode = 0;
  }
  traj_id++;
}
