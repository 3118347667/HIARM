#include "flygripper_planner/traj_opt.h"
#include "flygripper_planner/visualizer.hpp"

#include "flygripper_common/type/odometry.hpp"
#include "flygripper_common/type/trajectory.hpp"
#include "flygripper_common/ros/parameters.hpp"

namespace flygripper_planner {


struct Parameter
{
  double segment_per_distance;

  Parameter(const ros::NodeHandle &pnh) {
    if (!flygripper_common::getParam("segment_per_distance",    segment_per_distance,    pnh)) {
      throw ros::Exception("Failed to load parameters");
    }
  }

};


class TrajectoryGenerator {
 private:
  Parameter parameter_;

  ros::Subscriber waypoint_sub_, odom_sub_;
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
    waypoint_sub_ = nh.subscribe("/move_base_simple/goal", 1,
                                 &TrajectoryGenerator::goalCallBack, this,
                                 ros::TransportHints().tcpNoDelay());
    traj_pub_ = nh.advertise<flygripper_msgs::PolynomialTrajectory>("trajectory", 1);
  }

  std::vector<Eigen::Vector3d> generateQuarterCircle(const Eigen::Ref<const Eigen::Vector3d> ref,
                                                     const double radius,
                                                     const int num_points) {
    std::vector<Eigen::Vector3d> circle_points;

    // 计算每一步的角度增量
    double angle_range = M_PI/2;
    double angle_increment = angle_range / static_cast<double>(num_points);
    Eigen::Vector3d origin = ref - Eigen::Vector3d(0.0, radius, 0.0);
    for (int i = 0; i < num_points; ++i) {
      // 计算当前点的角度
      double angle = angle_increment * i;

      // 计算当前点的x和y坐标
      double x = origin.x() - radius * std::sin(angle);
      double y = origin.y() + radius * std::cos(angle);

      // 添加点到vector中，z坐标保持不变
      circle_points.push_back(Eigen::Vector3d(x, y, ref.z()));
    }

    return circle_points;
  }

  void interpolateWaypoints(const std::vector<Eigen::Vector3d>& points,
                            std::vector<Eigen::Vector3d>& inter_points,
                            const double dist_thresh) {
    inter_points.clear();
    int points_size = points.size();
    for (int i = 0; i < points_size - 1; ++i)
    {
      inter_points.push_back(points.at(i));

      double dist = (points.at(i + 1) - points.at(i)).norm();

      if (dist > dist_thresh)
      {
        int id_num = floor(dist / dist_thresh) + 1;

        for (int j = 1; j < id_num; ++j)
        {
          Eigen::Vector3d inter_pt =
              points.at(i) * (1.0 - double(j) / id_num) + points.at(i + 1) * double(j) / id_num;
          inter_points.push_back(inter_pt);
        }
      }
    }
    inter_points.push_back(points.back());
  }

  void odomCallBack(nav_msgs::OdometryConstPtr msg) {
    fromRosMsg(msg, odom_);
  }

  void goalCallBack(const geometry_msgs::PoseStamped::ConstPtr &msg) {
    Scalar now = ros::Time::now().toSec();
    if (!odom_.valid() || !odom_.isReceived(now)){
      return;
    }

    std::vector<Vector<3>> init_path;
    Matrix<3, 3> initial_state;
    Matrix<1, 2> initial_yaw;
    Matrix<3, 3> terminal_state;
    Matrix<1, 2> terminal_yaw;

    Vector<3> goal_pos;
    fromRosVec3(msg->pose.position, goal_pos);
    goal_pos.z() = odom_.pos.z();

    Quaternion q;
    fromRosQuat(msg->pose.orientation, q);

    initial_state << odom_.pos, odom_.vel, Vector<3>::Zero();
    initial_yaw << odom_.getYaw(0.0), (odom_.att*odom_.omega).z();

    terminal_state << goal_pos, Vector<3>::Zero(), Vector<3>::Zero();
    terminal_yaw << quat2Yaw(q), 0.0;

    init_path.push_back(odom_.pos);
    init_path.push_back(goal_pos);
    std::vector<Vector<3>> path;
    interpolateWaypoints(init_path, path, parameter_.segment_per_distance);
    ROS_INFO("init_path size: %d", path.size());
    traj_opt_->visualizer_->visualizeWaypoints(path, "initial_path");

    Trajectory traj;
    ROS_INFO("Generating trajectory...");
    int retry = 0;
    while (retry < 3) {
      bool ret = traj_opt_->generateTraj(initial_state,
                                         terminal_state,
                                         initial_yaw,
                                         terminal_yaw,
                                         parameter_.segment_per_distance,
                                         path,
                                         traj.xyz_min_jerk_poly,
                                         traj.heading_poly);
      traj_opt_->visualizer_->visualizeTrajectory(traj.xyz_min_jerk_poly, traj.heading_poly,
                                                  "debug_trajectory2", 0.1);
      traj.updateFromPoly(ros::Time::now().toSec(),
                          0,
                          Trajectory::HeadingMode::HEADING_POLY);
      flygripper_msgs::PolynomialTrajectory traj_msg;
      toRosMsg(traj,traj_msg);
      ROS_INFO("Generating trajectory.");
      if (ret) {
        ROS_INFO("publishing...");
        traj_pub_.publish(traj_msg);
        break;
      }  else{
        ROS_INFO("Retry...");
        retry++;
      }
    }


    return;
  }
};

};

int main(int argc, char **argv) {
  ros::init(argc, argv, "test");
  ros::NodeHandle nh, pnh("~");
  flygripper_planner::TrajectoryGenerator trajectory_generator(nh, pnh);
  ros::spin();
  return 0;
}