#include <ros/ros.h>
#include <tf/tf.h>
#include <iostream>

#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>

#include "flygripper_common/type/trajectory.hpp"
#include "flygripper_common/ros/parameters.hpp"
#include "flygripper_common/type/odometry.hpp"
#include "flygripper_common/type/actuator_state.hpp"

#include "flygripper_msgs/ActuatorCommand.h" // not type for convenient

#include "flygripper_planner/traj_opt.h"
#include "flygripper_planner/minco.hpp"
#include "flygripper_planner/visualizer.hpp"

#include "std_msgs/UInt16.h"

using namespace flygripper_common;
class TrajectoryGeneration{
 private:
  static constexpr int Dim = 3;

  minco::MINCO<3, 3, false> minco_s3_;
  minco::MINCO<3, 4, false> minco_s4_;
  Trajectory traj_; // S = 4;

  Visualizer visualizer_;
  Scalar piece_length_;

 public:
  TrajectoryGeneration(ros::NodeHandle& nh,
                       ros::NodeHandle& pnh):
      visualizer_(nh){
    if (!getParam("piece_length",     piece_length_,     pnh)){
      throw ros::Exception("Failed to load parameters");
    }
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

  Scalar timeTrapzVel(const Scalar dist,
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

  flygripper_msgs::PolynomialTrajectory generateTrajectory(const Vector<Dim>& head_pos,
                                                           const Vector<Dim>& tail_pos,
                                                           const std::vector<Vector<Dim>>& iner_pts,
                                                           const Scalar& allocation_speed,
                                                           const Scalar& allocation_acc,
                                                           const Trajectory::HeadingMode& heading_mode,
                                                           bool min_jerk = false,
                                                           const Vector<Dim>& head_vel = Vector<Dim>::Zero(),
                                                           const Vector<Dim>& tail_vel = Vector<Dim>::Zero())
  {
    flygripper_msgs::PolynomialTrajectory traj_msg;
    Scalar now = ros::Time::now().toSec();
    if (min_jerk){
      traj_.xyz_mode = Trajectory::XYZMode::XYZ_MIN_JERK;
      traj_.xyz_min_jerk_poly.clear();
    } else{
      traj_.xyz_mode = Trajectory::XYZMode::XYZ_MIN_SNAP;
      traj_.xyz_min_snap_poly.clear();
    }

    // --              --
    // | p_x v_x a_x ... |
    // | p_y v_y a_y ... |
    // | p_z v_z a_z ... |
    // --              --
    // Init head and tail state
    Matrix<Dim, 3> head_state_s3;
    Matrix<Dim, 3> tail_state_s3;
    head_state_s3 << head_pos, head_vel, Vector<Dim>::Zero();
    tail_state_s3 << tail_pos, tail_vel, Vector<Dim>::Zero();

    Matrix<Dim, 4> head_state_s4;
    Matrix<Dim, 4> tail_state_s4;
    head_state_s4 << head_pos, head_vel, Vector<Dim>::Zero(), Vector<Dim>::Zero();
    tail_state_s4 << tail_pos, tail_vel, Vector<Dim>::Zero(), Vector<Dim>::Zero();


    std::vector<Eigen::Vector3d> given_points, points;
    given_points.push_back(head_pos);
    for (auto pt : iner_pts){
      given_points.push_back(pt);
    }
    given_points.push_back(tail_pos);

    interpolateWaypoints(given_points, points, piece_length_);

    // Init waypoints
    int waypoints_num = points.size();
    int piece_num = waypoints_num - 1;
    Matrix<Dim, Dynamic> waypoints(Dim, waypoints_num);
    for (int i = 0; i < waypoints_num; ++i) {
      waypoints.col(i) = points.at(i);
    }

    // Init times
    Vector<> times(piece_num);
    Scalar dist;
    for (int i = 0; i < waypoints_num - 1; ++i) {
      dist = (waypoints.col(i + 1) - waypoints.col(i)).norm();
//      times(i) = dist / allocation_speed;
      times(i) = timeTrapzVel(dist, allocation_speed, allocation_acc);
    }

    const Matrix<3, Dynamic> intermediate_pos =
        waypoints.middleCols(1, waypoints_num - 1);

//    ROS_INFO_STREAM("-----------------------" << times);
//    ROS_INFO_STREAM("Head State: \n" << head_state);
//    ROS_INFO_STREAM("Tail State: \n" << tail_state);
//    ROS_INFO_STREAM("Times: \n" << times);


    if (min_jerk){
      minco_s3_.reset(head_state_s3, tail_state_s3, piece_num);
      minco_s3_.generate(intermediate_pos, times);
      minco_s3_.getTrajectory(traj_.xyz_min_jerk_poly);
    } else {
      minco_s4_.reset(head_state_s4, tail_state_s4, piece_num);
      minco_s4_.generate(intermediate_pos, times);
      minco_s4_.getTrajectory(traj_.xyz_min_snap_poly);
    }

    traj_.updateFromPoly(now, 0,  heading_mode);
    visualizer_.visualizeWaypoints(waypoints);
    if (min_jerk){
      visualizer_.visualizeTrajectory(traj_.xyz_min_jerk_poly);
    } else {
      visualizer_.visualizeTrajectory(traj_.xyz_min_snap_poly);
    }

    toRosMsg(traj_, traj_msg);

    return traj_msg;
  }


};

class GraspPlanner{
 public:
  GraspPlanner(ros::NodeHandle& nh, ros::NodeHandle& pnh):
      traj_gen_(nh, pnh){
    // odom
    odom_sub_ = nh.subscribe<nav_msgs::Odometry>("odom", 10,
                                                 &GraspPlanner::odomCallback, this,
                                                 ros::TransportHints().tcpNoDelay());
    // trigger
    rviz_trigger_sub_ = nh.subscribe<geometry_msgs::PoseStamped>("trigger", 10,
                                                                 &GraspPlanner::rvizTriggerCallback, this,
                                                                 ros::TransportHints().tcpNoDelay());

    traj_pub_ = nh.advertise<flygripper_msgs::PolynomialTrajectory>("trajectory", 1);

    // control and feedback from actuator
    actuator_pub_ = nh.advertise<flygripper_msgs::ActuatorCommand>("actuator_command", 10);
    actuator_sub_ = nh.subscribe<flygripper_msgs::ActuatorState>("actuator_state", 10,
                                                                 &GraspPlanner::actuatorStateCallback, this,
                                                                 ros::TransportHints().tcpNoDelay());

    if (!getParam("morph_shrink_pos", morph_shrink_pos_) ||
        !getParam("morph_expand_pos", morph_expand_pos_) ||
        !getParamWaypoints(pnh)){
      throw ros::Exception("Failed to load parameters");
    }

    // printing
    for (int i = 0; i < int(staged_waypoints_.size()); i++)
    {
      ROS_INFO("stage %d, vel: %f, morph action: %s, morph trigger: %s", i, staged_plan_speed_[i],
               morphingActionToString(staged_morph_action_[i]).c_str(),
               morphingTriggerToString(staged_morph_trigger_[i]).c_str());
      for (int j = 0; j < int(staged_waypoints_[i].size()); j++)
      {
        ROS_INFO("p%d: x: %f y: %f z: %f", j,
                 staged_waypoints_[i][j].x(),
                 staged_waypoints_[i][j].y(),
                 staged_waypoints_[i][j].z());
      }
    }
    resetFirstStage();
    ROS_INFO("Waiting for trigger from RViz...");
  }

  enum class MorphingAction: int{
    IDLE=0,
    EXPANDING=1,
    SHRINKING=2,
  };
  enum class MorphingTrigger: int{
    START=0,
    GOAL=1,
  };
  enum class MorphingStatus: int{
    IDLE=0,
    EXECUTING=1,
    DONE=2
  };
  enum class PlanStatus: int{
    IDLE=0,
    EXECUTING=1,
    DONE=2
  };

  std::string morphingTriggerToString(MorphingTrigger trigger){
    switch(trigger){
    case MorphingTrigger::START:
      return "START";
    case MorphingTrigger::GOAL:
      return "GOAL";
    }
    return "INVALID";
  }

  std::string morphingActionToString(MorphingAction action){
    switch(action){
    case MorphingAction::IDLE:
      return "IDLE";
    case MorphingAction::EXPANDING:
      return "EXPANDING";
    case MorphingAction::SHRINKING:
      return "SHRINKING";
    }
    return "INVALID";
  }

  std::string morphingStatusToString(MorphingStatus status){
    switch(status){
    case MorphingStatus::IDLE:
      return "IDLE";
    case MorphingStatus::EXECUTING:
      return "EXECUTING";
    case MorphingStatus::DONE:
      return "DONE";
    }
    return "INVALID";
  }

  void printProgressBar(Scalar goal_dist,
                        Scalar tolerance_dist=0.1,
                        bool init=false) {
    static Scalar origin_dist = 100.0;
    if (init){
      origin_dist = goal_dist;
    }
    Scalar k1 = 1.0 / (tolerance_dist - origin_dist);
    Scalar k0 = - k1 * origin_dist;
    Scalar percentage = std::max(std::min(k1*goal_dist+k0, 1.0), 0.0);
    const int barWidth = 50;
    int numChars = static_cast<int>(percentage * barWidth);

    std::cout << "[";
    for (int i = 0; i < barWidth; ++i) {
      if (i < numChars)
        std::cout << "=";
      else
        std::cout << " ";
    }
    std::cout << "] " << std::fixed << std::setprecision(2) << "(" << goal_dist << "m ) " << percentage * 100 << "%\r";
    std::cout.flush();
  }

  void triggerNextStage(){
    morphing_status_ = MorphingStatus::IDLE;
    plan_status_ = PlanStatus::IDLE;
    trigger_ = true;
    stage_++;
  }

  void run(){
    Scalar now = ros::Time::now().toSec();
    //! Make sure odom and actuator_state message has been received
    if (odom_.valid() && odom_.isReceived(now) &&
        actuator_state_.valid() && actuator_state_.isReceived(now) &&
        stage_ >= 0 &&
        stage_ < int(staged_waypoints_.size()))
    {
      //! Trigger to plan
      if (plan_status_ == PlanStatus::IDLE &&
          morphing_status_ == MorphingStatus::IDLE &&
          trigger_){
        ROS_INFO("\033[34m stage %d with velocity %.2f and heading mode %d planning\033[34m",
                 stage_,
                 staged_plan_speed_[stage_],
                 static_cast<int>(staged_plan_heading_mode_[stage_]));
        plan_status_ = PlanStatus::EXECUTING;
        publishTrajectory();
        //! Morphing start with planning
        if (staged_morph_trigger_[stage_] == MorphingTrigger::START &&
            morphing_status_ == MorphingStatus::IDLE){
          ROS_INFO("\033[34m stage %d with morphing %s \033[34m",
                   stage_, morphingActionToString(staged_morph_action_[stage_]).c_str());
          morphing_status_ = MorphingStatus::EXECUTING;
          publishMorphing();
        }
        trigger_ = false;
      }

      //! Plan executing
      if (plan_status_ == PlanStatus::EXECUTING) {
        //! plan completed checking
        if (isPlanCompleted()){
          plan_status_ = PlanStatus::DONE;
          ROS_INFO("\033[32m stage %d plan completed\033[32m", stage_);
          //! Morphing after planning completed
          if (staged_morph_trigger_[stage_] == MorphingTrigger::GOAL &&
              morphing_status_ == MorphingStatus::IDLE){
            ROS_INFO("\033[34m stage %d with morphing %s \033[34m",
                     stage_, morphingActionToString(staged_morph_action_[stage_]).c_str());
            morphing_status_ = MorphingStatus::EXECUTING;
            publishMorphing();
          }
        }
      }

      //! Morphing executing
      if (morphing_status_ == MorphingStatus::EXECUTING) {
        if (isMorphingCompleted()){
          ROS_INFO("\033[32m stage %d Morphing reached\033[32m", stage_);
          morphing_status_ = MorphingStatus::DONE;
        }
      }

      //! Plan and morphing done checking
      if(plan_status_ == PlanStatus::DONE &&
          morphing_status_ == MorphingStatus::DONE) {
        ROS_INFO("\033[32m stage %d Completed \033[32m", stage_);
        if (stage_ == int(staged_waypoints_.size())-1){
          ROS_INFO("-------[END] All stage reached -------");
          resetFirstStage();
          ROS_INFO("You can try again by giving trigger from RViz...");
        } else {
          triggerNextStage();
        }
      }
    }

  }

  void resetFirstStage(){
    stage_ = -1;
    morphing_status_ = MorphingStatus::IDLE;
    plan_status_ = PlanStatus::IDLE;
    trigger_ = false;
  }

  bool getParamWaypoints(ros::NodeHandle& pnh){

    std::string param_name, point_prefix, stage_prefix;
    int stage_index = 0;
    while(true){
      int point_index = 0;
      std::vector<Eigen::Vector3d> tmp_pts;
      stage_prefix = "waypoints_" + std::to_string(stage_index);
      while(true){
        //! Get p0, p1, ..., pn with format of {x, y, z}
        Eigen::Vector3d tmp_pt;
        point_prefix="/p" + std::to_string(point_index);
        param_name = stage_prefix+point_prefix;
        if(!getParam(param_name, tmp_pt)){
          break;
        }
        tmp_pts.push_back(tmp_pt);
        point_index++;
      }
      // if point index is 0, it means this stage_ is empty
      if(point_index==0){
        break;
      }
      staged_waypoints_.push_back(tmp_pts);

      // add plan speed
      double tmp_plan_speed;
      param_name = stage_prefix+ "/plan_speed";
      getParamDefault(param_name, tmp_plan_speed, 0.1);
      staged_plan_speed_.push_back(tmp_plan_speed);

      // add plan acc
      double tmp_plan_acc;
      param_name = stage_prefix+ "/plan_acc";
      getParamDefault(param_name, tmp_plan_acc, 0.1);
      staged_plan_acc_.push_back(tmp_plan_acc);

      // add plan tolerance
      double tmp_plan_tolerance;
      param_name = stage_prefix+ "/plan_tolerance";
      getParamDefault(param_name, tmp_plan_tolerance, 0.1);
      staged_plan_tolerance_.push_back(tmp_plan_tolerance);

      // add plan heading mode
      int tmp_plan_heading_mode;
      param_name = stage_prefix+ "/plan_heading_mode";
      getParamDefault(param_name, tmp_plan_heading_mode, 0);
      staged_plan_heading_mode_.push_back(static_cast<Trajectory::HeadingMode>(tmp_plan_heading_mode));

      // add morph action
      int tmp_morph_action;
      param_name = stage_prefix+ "/morph_action";
      getParamDefault(param_name, tmp_morph_action, 0);
      staged_morph_action_.push_back(static_cast<MorphingAction>(tmp_morph_action));

      // add morph trigger
      int tmp_morph_trigger;
      param_name = stage_prefix+ "/morph_trigger";
      getParamDefault(param_name, tmp_morph_trigger, 0);
      staged_morph_trigger_.push_back(static_cast<MorphingTrigger>(tmp_morph_trigger));

      // add morph tolerance
      double tmp_morph_tolerance;
      param_name = stage_prefix+ "/morph_tolerance";
      getParamDefault(param_name, tmp_morph_tolerance, 0.1);
      staged_morph_tolerance_.push_back(tmp_morph_tolerance);


      stage_index++;

    }

    return true;
  }

  void publishTrajectory(){

    std::vector<Vector<3>> iner_pts;
    Eigen::Vector3d tmp_wp;
    for (int i = 0; i < int(staged_waypoints_[stage_].size()-1); i++)
    {
      tmp_wp <<staged_waypoints_[stage_][i].x(),
          staged_waypoints_[stage_][i].y(),
          staged_waypoints_[stage_][i].z();
      iner_pts.push_back(tmp_wp);
    }
    traj_pub_.publish(traj_gen_.generateTrajectory(odom_.pos,
                                                   staged_waypoints_[stage_].back(),
                                                   iner_pts,
                                                   staged_plan_speed_[stage_],
                                                   staged_plan_acc_[stage_],
                                                   staged_plan_heading_mode_[stage_],
                                                   false,
                                                   odom_.vel));
  }

  bool isPlanCompleted(){
    if (!odom_.valid() || !odom_.isReceived(ros::Time::now().toSec()))
    {
      ROS_WARN("odom is not initialized while calling isPlanCompleted()");
      return false;
    }
    Scalar dist = (staged_waypoints_[stage_].back()-odom_.pos).norm();
    return dist <= staged_plan_tolerance_[stage_];
  }

  bool isMorphingCompleted(){
    bool completed = false;
    completed |= (staged_morph_action_[stage_] == MorphingAction::SHRINKING &&
        (actuator_state_.overload ||
            std::abs(actuator_state_.position - morph_shrink_pos_) < staged_morph_tolerance_[stage_]));
    completed |= (staged_morph_action_[stage_] == MorphingAction::EXPANDING &&
        std::abs(actuator_state_.position - morph_expand_pos_) < staged_morph_tolerance_[stage_]);
    completed |= staged_morph_action_[stage_] == MorphingAction::IDLE;
    return completed;
  }

  void rvizTriggerCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
  {
    if (!odom_.valid() || !odom_.isReceived(ros::Time::now().toSec())){
      ROS_ERROR("odom is not initialized while trigger from RViz. ");
      return;
    }

    if (stage_!= -1){
      ROS_ERROR("All stage is not finished!!! Currently in stage %d", stage_);
    }
    else{
      ROS_INFO("Trigger from Rviz!");
      triggerNextStage();
    }

  }

  void odomCallback(const nav_msgs::Odometry::ConstPtr &msg)
  {
    fromRosMsg(msg, odom_);
  }

  void publishMorphing(){
    flygripper_msgs::ActuatorCommand actuator_command_msg;
    actuator_command_msg.header.stamp = ros::Time::now();
    actuator_command_msg.header.frame_id = "robot";
    actuator_command_msg.mode = flygripper_msgs::ActuatorCommand::MODE_POSITION;
    actuator_command_msg.reference_velocity = 0.0;
    actuator_command_msg.reference_effort = 0.0;

    switch (staged_morph_action_[stage_]) {
    case MorphingAction::SHRINKING:
      actuator_command_msg.reference_position = morph_shrink_pos_;
      actuator_pub_.publish(actuator_command_msg);
      break;
    case MorphingAction::EXPANDING:
      actuator_command_msg.reference_position = morph_expand_pos_;
      actuator_pub_.publish(actuator_command_msg);
      break;
    case MorphingAction::IDLE:
      // do nothing is ok
      break;
    }
  }

  void actuatorStateCallback(const flygripper_msgs::ActuatorStateConstPtr &msg)
  {
    fromRosMsg(msg,actuator_state_);
  }

 private:
  ros::Publisher traj_pub_;
  ros::Publisher actuator_pub_;
  ros::Subscriber actuator_sub_;
  ros::Subscriber rviz_trigger_sub_;
  ros::Subscriber odom_sub_;

  Visualizer visualizer_;

  Odometry odom_;
  ActuatorState actuator_state_;
  bool trigger_ = false;

  std::vector<MorphingAction> staged_morph_action_;
  std::vector<MorphingTrigger> staged_morph_trigger_;
  std::vector<Scalar> staged_plan_tolerance_;
  std::vector<Scalar> staged_morph_tolerance_;
  std::vector<Trajectory::HeadingMode> staged_plan_heading_mode_;

  std::vector<std::vector<Eigen::Vector3d>> staged_waypoints_;
  std::vector<Scalar> staged_plan_speed_;
  std::vector<Scalar> staged_plan_acc_;

  Scalar morph_shrink_pos_;
  Scalar morph_expand_pos_;

  PlanStatus plan_status_ = PlanStatus::IDLE;
  MorphingStatus morphing_status_ = MorphingStatus::IDLE;
  int stage_ = -1;

  TrajectoryGeneration traj_gen_;
};


int main(int argc, char **argv)
{
  ros::init(argc, argv, "grasp_planner");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  GraspPlanner app(nh, pnh);
  ros::Rate rate(200);

  while(ros::ok()){
    ros::spinOnce();
    app.run();

    rate.sleep();
  }

  return 0;
}