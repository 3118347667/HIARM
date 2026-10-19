#include "ros/ros.h"
#include "sensor_msgs/Joy.h"

#include "flygripper_common/type/odometry.hpp"
#include "flygripper_common/type/trajectory.hpp"
#include "flygripper_common/ros/parameters.hpp"

#include "flygripper_planner/minco.hpp"
#include "flygripper_planner/visualizer.hpp"
#include "flygripper_planner/velocity_control.hpp"

using namespace flygripper_common;
struct Parameter
{
  Scalar max_velocity;
  Scalar update_duration;
  Parameter(const ros::NodeHandle &pnh) {
    if (!getParam("max_velocity",     max_velocity,     pnh) ||
        !getParam("update_duration",  update_duration,  pnh)){
      throw ros::Exception("Failed to load parameters");
    }
  }

};

template <int S>
class TrajectoryGenerator {
 private:
  static constexpr int Dim = 3;
  static constexpr int Order = 2*S-1;
  static constexpr int PieceNum = 1;
  minco::MINCO<3, S,false> minco_;
  Trajectory traj_; // S = 4;
  Odometry odom_;

  Parameter parameter_;
  Visualizer visualizer_;
  ros::Subscriber joy_sub_;
  ros::Subscriber odom_sub_;
  ros::Publisher traj_pub_;
  ros::Timer update_timer_;

  Vector<PieceNum> times_;
  VelocityControlSystem::Input joy_input_;
  bool joy_ctrl_;
 public:
  TrajectoryGenerator(ros::NodeHandle& nh, ros::NodeHandle& pnh) :
      parameter_(pnh),
      visualizer_(nh),
      joy_input_(Vector<3>::Zero(), 0.0),
      joy_ctrl_(false){
    joy_sub_ = nh.subscribe("joy", 1,
                            &TrajectoryGenerator::joyCallBack, this,
                            ros::TransportHints().tcpNoDelay());
    odom_sub_ = nh.subscribe("odom", 1,
                              &TrajectoryGenerator::odomCallBack, this,
                              ros::TransportHints().tcpNoDelay());
    traj_pub_ = nh.advertise<flygripper_msgs::PolynomialTrajectory>("trajectory", 1);
    update_timer_ = nh.createTimer(ros::Duration(parameter_.update_duration),
                                   &TrajectoryGenerator::timerCallBack, this);
  }

  void odomCallBack(nav_msgs::OdometryConstPtr msg) {
    fromRosMsg(msg, odom_);
  }

  void joyCallBack(sensor_msgs::JoyConstPtr msg) {
    // Robot State is valid and received on time
    //              1                      3
    //              Λ                      Λ
    //              |                      |
    //              |                      |
    //    0  <—— ——             2  <—— ——

    Scalar real_scale = parameter_.max_velocity * parameter_.update_duration;
    joy_input_ = VelocityControlSystem::Input(Vector<3>(msg->axes[3] * real_scale,
                                                        msg->axes[2] * real_scale,
                                                        msg->axes[1] * real_scale),
                                              0);
  }

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

  void timerCallBack(const ros::TimerEvent& event) {

    static VelocityControlSystem sys;
    static VelocityControlSystem::State update_state;
    flygripper_common::Scalar now = ros::Time::now().toSec();
    if (!odom_.valid() || !odom_.isReceived(now)){
      return;
    }

    if (joy_input_.data.isMuchSmallerThan(0.03)){
      joy_ctrl_ = false;
      return;
    }

    if (!joy_ctrl_) {
      joy_ctrl_ = true;
      update_state = VelocityControlSystem::State(odom_.pos, odom_.getYaw(),
                                                  odom_.vel, odom_.omega.z());
    }


    update_state = sys.update(update_state,
                              joy_input_,
                              1.0);

    traj_.xyz_poly.clear();

    // Init time allocation based on speed and acc
    times_(0) = 1.0;
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
          tail_state.col(i) = update_state.pos;
          break;
        case 1:
          head_state.col(i) = odom_.vel;
          tail_state.col(i) = Vector<3>::Zero();
          break;
        case 2:
          head_state.col(i) = Vector<3>::Zero();
          tail_state.col(i) = Vector<3>::Zero();
          break;
        case 3:
          head_state.col(i) = Vector<3>::Zero();
          tail_state.col(i) = Vector<3>::Zero();
          break;
        default:
          ROS_ERROR("S can not be larger than 4, but it is %d", S);
          break;
      }
    }

    ROS_INFO_STREAM("Head State: \n" << head_state);
    ROS_INFO_STREAM("Tail State: \n" << tail_state);

    const Matrix<3, 0> intermediate_pos = Matrix<3, 0>::Zero();
    const Eigen::VectorXd times = times_.head(PieceNum);
    minco_.reset(head_state, tail_state, PieceNum);
    minco_.generate(intermediate_pos, times);
    minco_.getTrajectory(traj_.xyz_poly);

    traj_.updateDurationFromPoly(ros::Time::now().toSec(), 0);
    visualizer_.visualizeWaypoints(update_state.pos);
    visualizer_.visualizeTrajectory(traj_);

    flygripper_msgs::PolynomialTrajectory traj_msg;
    toRosMsg(traj_,traj_msg);
    traj_pub_.publish(traj_msg);
    ROS_INFO("Trajectory published");

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