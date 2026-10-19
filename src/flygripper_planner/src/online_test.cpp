#include "flygripper_planner/minco.hpp"
#include "flygripper_planner/visualizer.hpp"

#include "flygripper_common/type/odometry.hpp"
#include "flygripper_common/type/trajectory.hpp"

#include "flygripper_common/ros/parameters.hpp"

using namespace flygripper_common;
struct Parameter
{
  Scalar waypoint_height;
  Vector<3> terminal_vel;
  Vector<3> terminal_acc;
  Vector<3> terminal_jerk;
  Scalar allocation_speed;
  Scalar allocation_acc;
  int max_piece_num;

  Parameter(const ros::NodeHandle &pnh) {
    if (!getParam("waypoint_height",  waypoint_height,  pnh) ||
        !getParam("terminal_vel",     terminal_vel,     pnh) ||
        !getParam("terminal_acc",     terminal_acc,     pnh) ||
        !getParam("terminal_jerk",    terminal_jerk,    pnh) ||
        !getParam("allocation_speed", allocation_speed, pnh) ||
        !getParam("allocation_acc",   allocation_acc,   pnh) ||
        !getParam("max_piece_num",    max_piece_num,    pnh)) {
      throw ros::Exception("Failed to load parameters");
    }
  }

};

inline Scalar timeTrapzVel(const Scalar dist,
                           const Scalar vel,
                           const Scalar acc)
{
  const Scalar t = vel / acc;
  const Scalar d = 0.5 * acc * t * t;

  if (dist < d + d)
  {
    return 2.0 * sqrt(dist / acc);
  }
  else
  {
    return 2.0 * t + (dist - 2.0 * d) / vel;
  }
}

template <int S>
class TrajectoryGenerator {
 private:
  static constexpr int Dim = 3;
  static constexpr int Order = 2*S-1;
  minco::MINCO<3, S,false> minco_;
  Trajectory traj_; // S = 4;
  Odometry odom_;

  Parameter parameter_;
  Visualizer visualizer_;
  ros::Subscriber waypoint_sub_;
  ros::Subscriber odom_sub_;
  ros::Publisher traj_pub_;

  Matrix<3,Dynamic> waypoints_;
  Vector<> times_;
  int waypoint_num_;

 public:
  TrajectoryGenerator(ros::NodeHandle& nh, ros::NodeHandle& pnh) :
      parameter_(pnh),
      visualizer_(nh),
      waypoints_(Dim, parameter_.max_piece_num),
      times_(parameter_.max_piece_num),
      waypoint_num_(0) {
    waypoint_sub_ = nh.subscribe("/move_base_simple/goal", 1,
                                 &TrajectoryGenerator::waypointCallBack, this,
                                 ros::TransportHints().tcpNoDelay());
    odom_sub_ = nh.subscribe("odom", 1,
                              &TrajectoryGenerator::odomCallBack, this,
                              ros::TransportHints().tcpNoDelay());
    traj_pub_ = nh.advertise<flygripper_msgs::PolynomialTrajectory>("trajectory", 1);
  }

  void odomCallBack(nav_msgs::OdometryConstPtr msg) {
    fromRosMsg(msg, odom_);
  }

  void waypointCallBack(geometry_msgs::PoseStampedConstPtr msg) {
    // Robot State is valid and received on time
    flygripper_common::Scalar now = ros::Time::now().toSec();
    if (!odom_.valid() || !odom_.isReceived(now)){
      return;
    }
    ++waypoint_num_;
    // If waypoint is larger than max_piece_num, replan
    if (waypoint_num_ > parameter_.max_piece_num) {
      waypoint_num_ = 1;
      traj_.xyz_poly.clear();
    }

    const int piece_num = waypoint_num_;
    const int waypoint_idx = waypoint_num_ - 1;


    // Update the waypoints
    waypoints_(0, waypoint_idx) = msg->pose.position.x;
    waypoints_(1, waypoint_idx) = msg->pose.position.y;
    waypoints_(2, waypoint_idx) = parameter_.waypoint_height;

    // Init time allocation based on speed and acc
    const Vector<3> last_pos = waypoint_idx > 0 ? waypoints_.col(waypoint_idx-1) : odom_.pos;
    const double dist = (waypoints_.col(waypoint_idx) - last_pos).norm();
    times_(waypoint_idx) = timeTrapzVel(dist,
                                     parameter_.allocation_speed,
                                     parameter_.allocation_acc);

    // --              --
    // | p_x v_x a_x ... |
    // | p_y v_y a_y ... |
    // | p_z v_z a_z ... |
    // --              --
    Matrix<Dim, S> head_state = Matrix<Dim, S>::Zero();
    Matrix<Dim, S> tail_state = Matrix<Dim, S>::Zero();

    for (int i = 0; i < S; ++i) {
      switch (i) {
        case 0:
          head_state.col(i) = odom_.pos;
          tail_state.col(i) = waypoints_.col(waypoint_idx);
          break;
        case 1:
          head_state.col(i) = odom_.vel;
          tail_state.col(i) = parameter_.terminal_vel;
          break;
        case 2:
          head_state.col(i) = Vector<3>::Zero();
          tail_state.col(i) = parameter_.terminal_acc;
          break;
        case 3:
          head_state.col(i) = Vector<3>::Zero();
          tail_state.col(i) = parameter_.terminal_jerk;
          break;
        default:
          ROS_ERROR("S can not be larger than 4, but it is %d", S);
          break;
      }
    }

    ROS_INFO_STREAM("Head State: \n" << head_state);
    ROS_INFO_STREAM("Tail State: \n" << tail_state);

    const Matrix<3, Dynamic> intermediate_pos = waypoints_.middleCols(0, piece_num - 1);
    const Eigen::VectorXd times = times_.head(piece_num);
    minco_.reset(head_state, tail_state, piece_num);
    minco_.generate(intermediate_pos, times);
    minco_.getTrajectory(traj_.xyz_poly);

    traj_.updateDurationFromPoly(ros::Time::now().toSec(), 0);
    visualizer_.visualizeWaypoints(waypoints_.leftCols(waypoint_num_));
    visualizer_.visualizeTrajectory(traj_);

    if (piece_num == parameter_.max_piece_num){
      flygripper_msgs::PolynomialTrajectory traj_msg;
      toRosMsg(traj_,traj_msg);
      traj_pub_.publish(traj_msg);
      ROS_INFO("Trajectory published");
    }
    return;
  }
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "test");
  ros::NodeHandle nh, pnh("~");
  const int S = 4; // minimum snap
  TrajectoryGenerator<S> trajectory_generator(nh, pnh);
  ros::spin();
  return 0;
}