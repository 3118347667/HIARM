#include "flygripper_planner/traj_opt_real.h"
#include "flygripper_planner/visualizer.hpp"

#include "flygripper_common/type/odometry.hpp"
#include "flygripper_common/type/trajectory.hpp"
#include "flygripper_common/ros/parameters.hpp"

namespace flygripper_planner {


struct Parameter
{
  double arc_radius;
  double arc_angle;
  int arc_point_num;

  Parameter(const ros::NodeHandle &pnh) {
    if (!getParam("arc_radius",             arc_radius,             pnh) || // two point mode
        !getParam("arc_angle",              arc_angle,              pnh) ||
        !getParam("arc_point_num",          arc_point_num,          pnh)) // multi point mode
    {
      throw ros::Exception("Failed to load parameters");
    }
  }

};


class TrajectoryGenerator {
 private:
  Parameter parameter_;

  ros::Subscriber trigger_sub_, odom_sub_;
  ros::Publisher traj_pub_;

  TrajOpt::Ptr traj_opt_;
  Odometry odom_;

 public:
  TrajectoryGenerator(ros::NodeHandle& nh, ros::NodeHandle& pnh) :
                                                                   parameter_(pnh){
    odom_sub_ = nh.subscribe("odom", 1,
                             &TrajectoryGenerator::odomCallBack, this,
                             ros::TransportHints().tcpNoDelay());
    traj_opt_ = std::make_shared<TrajOpt>(nh, pnh);
    trigger_sub_ = nh.subscribe("/move_base_simple/goal", 1,
                                &TrajectoryGenerator::triggerCallBack, this,
                                ros::TransportHints().tcpNoDelay());
    traj_pub_ = nh.advertise<flygripper_msgs::PolynomialTrajectory>("trajectory", 1);
  }

  void generateQuarterCircle(const Ref<const Vector<3>> ref_pos,
                                                     const double ref_yaw,
                                                     const double arc_radius,
                                                     const double arc_angle,
                                                     const int arc_point_num,
                             std::vector<Eigen::Vector3d>& xyz_path,
                             std::vector<double>& yaw_path) {
    xyz_path.clear();
    yaw_path.clear();
    // 计算每一步的角度增量
    double angle_increment = arc_angle / static_cast<double>(arc_point_num-1);
    Vector<3> origin = ref_pos + Vector<3>(sin(ref_yaw), -cos(ref_yaw), 0) * arc_radius;
    for (int i = 0; i < arc_point_num; ++i) {
      // 计算当前点的角度
      double angle = ref_yaw + angle_increment * i;

      // 计算当前点的x和y坐标
      double x = origin.x() - arc_radius * std::sin(angle);
      double y = origin.y() + arc_radius * std::cos(angle);

      // 添加点到vector中，z坐标保持不变
      xyz_path.push_back(Eigen::Vector3d(x, y, ref_pos.z()));
      yaw_path.push_back(angle);
    }

    return;
  }

  void odomCallBack(nav_msgs::OdometryConstPtr msg) {
    fromRosMsg(msg, odom_);
  }

  void triggerCallBack(const geometry_msgs::PoseStamped::ConstPtr &msg) {
    Scalar now = ros::Time::now().toSec();
    if (!odom_.valid() || !odom_.isReceived(now)){
      return;
    }

    std::vector<Vector<3>> xyz_path;
    std::vector<double> yaw_path;
    Matrix<3,3> initial_state;
    Matrix<1,2> initial_yaw;
    initial_state.resize(3,3);
    initial_yaw.resize(1,2);
    Matrix<3,3> terminal_state;
    Matrix<1,2> terminal_yaw;

    Vector<3> cur_pos = odom_.pos;
    double cur_yaw = odom_.getYaw(0.0);

    generateQuarterCircle(cur_pos, cur_yaw,
                          parameter_.arc_radius,
                          parameter_.arc_angle,
                          parameter_.arc_point_num,
                          xyz_path, yaw_path);

    initial_state << xyz_path.front(), Vector<3>::Zero(), Vector<3>::Zero();
    initial_yaw << yaw_path.front(), 0.0;

    terminal_state << xyz_path.back(), Vector<3>::Zero(), Vector<3>::Zero();
    terminal_yaw <<  yaw_path.back(), 0.0;

//    Vector<3> goal_pos;
//    fromRosVec3(msg->pose.position, goal_pos);
//    goal_pos.z() = odom_.pos.z();
//
//    Quaternion q;
//    fromRosQuat(msg->pose.orientation, q);
//
//    initial_state << odom_.pos, odom_.vel, Vector<3>::Zero();
//    initial_yaw << odom_.getYaw(0.0), (odom_.att*odom_.omega).z();
//
//    terminal_state << goal_pos, Vector<3>::Zero(), Vector<3>::Zero();
//    terminal_yaw << quat2Yaw(q), 0.0;
//
//    init_path.push_back(odom_.pos);
//    init_path.push_back(goal_pos);

    traj_opt_->visualizer_->visualizeWaypoints(xyz_path, "init_path", Color::YELLOW);

    Trajectory traj;
    ROS_INFO("Begin to generating trajectory...");
    traj_opt_->updateParameter(1.0, 1.5, 0.5);
    bool ret = traj_opt_->generateTraj(initial_state,
                                       terminal_state,
                                       initial_yaw,
                                       terminal_yaw,
                                       xyz_path,
                                       yaw_path,
                                       traj.xyz_min_jerk_poly,
                                       traj.heading_poly);

    traj.updateFromPoly(ros::Time::now().toSec(),
                        0,
                        Trajectory::HeadingMode::HEADING_POLY);

    if (ret) {
      ROS_INFO("Generate trajectory success.");
      flygripper_msgs::PolynomialTrajectory traj_msg;
      toRosMsg(traj,traj_msg);
      traj_pub_.publish(traj_msg);
    } else {
      ROS_ERROR("Generate trajectory failed.");
    }
    return;
  }
};

};

int main(int argc, char **argv) {
  ros::init(argc, argv, "door");
  ros::NodeHandle nh, pnh("~");
  flygripper_planner::TrajectoryGenerator trajectory_generator(nh, pnh);
  ros::spin();
  return 0;
}