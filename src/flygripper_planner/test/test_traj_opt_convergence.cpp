#include <gtest/gtest.h>
#include <ros/ros.h>

#include <cmath>
#include <vector>

#include "flygripper_planner/traj_opt_real.h"

TEST(TrajOptConvergence, CrossingBagStatesProduceFiniteTrajectory) {
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  pnh.setParam("K", 8);
  pnh.setParam("M", 8);
  pnh.setParam("max_iteration", 300);
  pnh.setParam("min_piece_num", 2);
  pnh.setParam("rhoT", 10.0);
  pnh.setParam("rhoV", 1000.0);
  pnh.setParam("rhoA", 1000.0);
  pnh.setParam("rhoDYaw", 1000.0);
  pnh.setParam("rhoPAlignmentDistance", 1.5);
  pnh.setParam("rhoPAlignmentAngle", 1000.0);
  pnh.setParam("rhoYawAlignmentAngle", 1000.0);
  pnh.setParam("pause_debug", false);
  pnh.setParam("fix_p", true);
  pnh.setParam("magic_number", 0);

  flygripper_planner::TrajOpt optimizer(nh, pnh);
  optimizer.updateParameter(0.5, 0.5, 0.5);
  const Eigen::Vector3d goal(0.12, -0.66, 0.72);
  // The second sample previously reproduced LBFGSERR_MAXIMUMLINESEARCH
  // in ARM64; the first is from an earlier successful crossing.
  const std::vector<Eigen::Vector3d> starts{
      {-0.8583113046640841, -0.6627887090317932, 0.7144470115930895},
      {-0.85067155345110723, -0.66631407975833878, 0.71327702296346573}};
  for (const auto& start : starts) {
    Eigen::MatrixXd initial = Eigen::MatrixXd::Zero(3, 3);
    Eigen::MatrixXd terminal = Eigen::MatrixXd::Zero(3, 3);
    initial.col(0) = start;
    terminal.col(0) = goal;
    Eigen::MatrixXd yaw = Eigen::MatrixXd::Zero(1, 2);
    yaw(0, 0) = 0.030827692410510574;
    flygripper_common::PolynomialTrajectory<3, 5> xyz_traj;
    flygripper_common::PolynomialTrajectory<1, 3> yaw_traj;
    ASSERT_TRUE(optimizer.generateTraj(initial, terminal, yaw, yaw, {start, goal},
                                       {yaw(0, 0), yaw(0, 0)}, xyz_traj, yaw_traj));
    const double duration = xyz_traj.getTotalDuration();
    ASSERT_TRUE(std::isfinite(duration));
    ASSERT_GT(duration, 0.0);
    EXPECT_LT(duration, 10.0);
    EXPECT_LT((xyz_traj.getPos(0.0) - start).norm(), 1e-6);
    EXPECT_LT((xyz_traj.getPos(duration) - goal).norm(), 1e-6);
    EXPECT_LT(xyz_traj.getVel(0.0).norm(), 1e-6);
    EXPECT_LT(xyz_traj.getVel(duration).norm(), 1e-6);
    for (int i = 0; i <= 100; ++i) {
      const double time = duration * i / 100.0;
      ASSERT_TRUE(xyz_traj.getPos(time).allFinite());
      ASSERT_TRUE(xyz_traj.getVel(time).allFinite());
      ASSERT_TRUE(xyz_traj.getAcc(time).allFinite());
      // Limits are soft penalties; retain a margin for this particular case.
      EXPECT_LT(xyz_traj.getVel(time).norm(), 0.51);
      EXPECT_LT(xyz_traj.getAcc(time).norm(), 0.51);
    }
  }
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "traj_opt_convergence");
  return RUN_ALL_TESTS();
}
