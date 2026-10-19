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

#include "flygripper_planner/minco.hpp"
#include "flygripper_planner/visualizer.hpp"
#include "flygripper_planner/traj_opt_real.h"
#include "flygripper_planner/rc_utils.hpp"

#include "std_msgs/UInt16.h"

using namespace flygripper_common;
using namespace flygripper_planner;
class TrajectoryGeneration{
 private:
  static constexpr int Dim = 3;

  Visualizer visualizer_;
  Scalar piece_length_;
  TrajOpt::Ptr traj_opt_;

  // Door
  double arc_radius_;
  double arc_angle_;
  int arc_point_num_;

 public:
  TrajectoryGeneration(ros::NodeHandle& nh,
                       ros::NodeHandle& pnh): visualizer_(nh){
    traj_opt_ = std::make_shared<TrajOpt>(nh, pnh);
    if (!getParam("arc_radius",             arc_radius_,             pnh) || // two point mode
        !getParam("arc_angle",              arc_angle_,              pnh) ||
        !getParam("arc_point_num",          arc_point_num_,          pnh)) // multi point mode
    {
      throw ros::Exception("Failed to load parameters");
    }
  }

  void generateQuarterCircle(const Ref<const Vector<3>> ref_pos,
                             const Scalar ref_yaw,
                             const Scalar arc_radius,
                             const Scalar arc_angle,
                             const int arc_point_num,
                             std::vector<Vector<3>>& xyz_path,
                             std::vector<Scalar>& yaw_path) {
    xyz_path.clear();
    yaw_path.clear();
    // 计算每一步的角度增量
    Scalar angle_increment = arc_angle / static_cast<Scalar>(arc_point_num-1);
    Vector<3> origin = ref_pos + Vector<3>(-sin(ref_yaw), cos(ref_yaw), 0) * arc_radius;
    for (int i = 0; i < arc_point_num; ++i) {
      // 计算当前点的角度
      Scalar angle = ref_yaw + angle_increment * i;

      // 计算当前点的x和y坐标
      Scalar x = origin.x() + arc_radius * std::sin(angle);
      Scalar y = origin.y() - arc_radius * std::cos(angle);

      // 添加点到vector中，z坐标保持不变
      xyz_path.push_back(Eigen::Vector3d(x, y, ref_pos.z()));
      yaw_path.push_back(angle);
    }

    return;
  }

  bool getDoorTrajectory(Trajectory& traj,
                         const Vector<Dim>& head_pos, const Scalar& head_yaw_angle,
                         const Scalar& max_vel,
                         const Scalar& max_acc,
                         const Scalar& max_dyaw,
                         const Vector<Dim>& head_vel = Vector<Dim>::Zero(),
                         const Vector<Dim>& tail_vel = Vector<Dim>::Zero())
  {
    // --              --
    // | p_x v_x a_x ... |
    // | p_y v_y a_y ... |
    // | p_z v_z a_z ... |
    // --              --
    // Init head and tail state


    std::vector<Eigen::Vector3d> xyz_path;
    std::vector<Scalar> yaw_path;

    generateQuarterCircle(head_pos, head_yaw_angle,
                          arc_radius_,
                          arc_angle_,
                          arc_point_num_,
                          xyz_path, yaw_path);
    traj_opt_->visualizer_->visualizeWaypoints(xyz_path, "init_path", Color::YELLOW);

    Matrix<Dim, 3> initial_state;
    Matrix<Dim, 3> terminal_state;
    Matrix<1, 2> initial_yaw;
    Matrix<1, 2> terminal_yaw;

    initial_state << xyz_path.front(), head_vel, Vector<3>::Zero();
    initial_yaw << yaw_path.front(), 0.0;

    terminal_state << xyz_path.back(), tail_vel, Vector<3>::Zero();
    terminal_yaw << yaw_path.back(), 0.0;


    ROS_INFO("Begin to generating trajectory...");
    traj_opt_->updateParameter(max_vel, max_acc, max_dyaw);
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
      return true;
    } else {
      ROS_ERROR("Generate trajectory failed.");
      return false;
    }
  }

  bool getGoalTrajectory(Trajectory& traj,
                         const Vector<Dim>& head_pos, const Scalar& head_yaw_angle,
                         const Vector<Dim>& tail_pos, const Scalar& tail_yaw_angle,
                         const std::vector<Vector<Dim>>& iner_xyz_pts,
                         const std::vector<Scalar>& iner_yaw_pts,
                         const Scalar& max_vel,
                         const Scalar& max_acc,
                         const Scalar& max_dyaw,
                         const Vector<Dim>& head_vel = Vector<Dim>::Zero(),
                         const Vector<Dim>& tail_vel = Vector<Dim>::Zero())
  {
    // --              --
    // | p_x v_x a_x ... |
    // | p_y v_y a_y ... |
    // | p_z v_z a_z ... |
    // --              --
    // Init head and tail state
    Matrix<Dim, 3> initial_state;
    Matrix<Dim, 3> terminal_state;
    initial_state << head_pos, head_vel, Vector<Dim>::Zero();
    terminal_state << tail_pos, tail_vel, Vector<Dim>::Zero();
    Matrix<1, 2> initial_yaw;
    Matrix<1, 2> terminal_yaw;
    initial_yaw << head_yaw_angle, 0.0;
    terminal_yaw << tail_yaw_angle, 0.0;
    std::vector<Eigen::Vector3d> xyz_path;
    std::vector<Scalar> yaw_path;
    xyz_path.push_back(head_pos);
    yaw_path.push_back(head_yaw_angle);
    for (int i=0; i < iner_xyz_pts.size(); i++){
      xyz_path.push_back(iner_xyz_pts[i]);
      yaw_path.push_back(iner_yaw_pts[i]);
    }
    xyz_path.push_back(tail_pos);
    yaw_path.push_back(tail_yaw_angle);
    traj_opt_->visualizer_->visualizeWaypoints(xyz_path, "init_path", Color::YELLOW);


    ROS_INFO("Begin to generating trajectory...");
    traj_opt_->updateParameter(max_vel, max_acc, max_dyaw);
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
      return true;
    } else {
      ROS_ERROR("Generate trajectory failed.");
      return false;
    }
  }

};

class TaskScheduler{
 public:
  TaskScheduler(ros::NodeHandle& nh, ros::NodeHandle& pnh):
      traj_gen_(nh, pnh){
    // odom
    odom_sub_ = nh.subscribe<nav_msgs::Odometry>("odom", 10,
                                                 &TaskScheduler::odomCallback, this,
                                                 ros::TransportHints().tcpNoDelay());
    // trigger
    rviz_trigger_sub_ = nh.subscribe<geometry_msgs::PoseStamped>("trigger", 10,
                                                                 &TaskScheduler::rvizTriggerCallback, this,
                                                                 ros::TransportHints().tcpNoDelay());

    // rc in
    rc_in_sub_ = nh.subscribe("/mavros/rc/in", 1, &TaskScheduler::rcInCallback, this,
                              ros::TransportHints().tcpNoDelay());

    // traj
    traj_pub_ = nh.advertise<flygripper_msgs::PolynomialTrajectory>("trajectory", 1);

    // control and feedback from actuator
    actuator_pub_ = nh.advertise<flygripper_msgs::ActuatorCommand>("actuator_command", 10);
    actuator_sub_ = nh.subscribe<flygripper_msgs::ActuatorState>("actuator_state", 10,
                                                                 &TaskScheduler::actuatorStateCallback, this,
                                                                 ros::TransportHints().tcpNoDelay());

    if (!getParam("morph_shrink_pos", morph_shrink_pos_) ||
        !getParam("morph_expand_pos", morph_expand_pos_) ||
        !getParam("use_odom_init_vel", use_odom_init_vel_) ||
        !getParam("morph_duration", morph_duration_) ||
        !getTaskParam(pnh)){
      throw ros::Exception("Failed to load parameters");
    }

    // printing
    for (int i = 0; i < int(stage_data_.size()); i++)
    {
      ROS_INFO("stage %d, vel: %f, morph action: %s, morph trigger: %s", i, stage_data_[i].plan_max_vel,
               morphingActionToString(stage_data_[i].morph_action).c_str(),
               morphingTriggerToString(stage_data_[i].morph_trigger).c_str());
      if (stage_data_[i].plan_type == PlanType::POINT){
        ROS_INFO("Plan Type: POINT");
        for (int j = 0; j < int(stage_data_[i].plan_point_xyz.size()); j++)
        {
          if (stage_data_[i].plan_heading_mode == Trajectory::HeadingMode::HEADING_POLY){
            ROS_INFO("p%d: x: %f, y: %f z: %f, yaw: %f", j,
                     stage_data_[i].plan_point_xyz[j].x(),
                     stage_data_[i].plan_point_xyz[j].y(),
                     stage_data_[i].plan_point_xyz[j].z(),
                     stage_data_[i].plan_point_yaw[j]);
          } else if (stage_data_[i].plan_heading_mode == Trajectory::HeadingMode::HEADING_NONE){
            ROS_INFO("p%d: x: %f y: %f z: %f", j,
                     stage_data_[i].plan_point_xyz[j].x(),
                     stage_data_[i].plan_point_xyz[j].y(),
                     stage_data_[i].plan_point_xyz[j].z());
          }
        }
      } else if (stage_data_[i].plan_type == PlanType::DOOR){
        ROS_INFO("Plan Type: DOOR");
      } else if (stage_data_[i].plan_type == PlanType::GRASP){
        ROS_INFO("Plan Type: GRASP");
        for (int j = 0; j < int(stage_data_[i].plan_point_xyz.size()); j++)
        {
          ROS_INFO("p%d: x: %f, y: %f z: %f, yaw: %f", j,
                   stage_data_[i].plan_point_xyz[j].x(),
                   stage_data_[i].plan_point_xyz[j].y(),
                   stage_data_[i].plan_point_xyz[j].z(),
                   stage_data_[i].plan_point_yaw[j]);
        }
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
    WAITING=1,
    EXECUTING=2,
    DONE=3
  };
  enum class PlanStatus: int{
    IDLE=0,
    EXECUTING=1,
    DONE=2
  };
  enum class PlanType: int{
    POINT = 0,
    DOOR = 1,
    GRASP = 2
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
    case MorphingStatus::WAITING:
      return "WAITING";
    case MorphingStatus::EXECUTING:
      return "EXECUTING";
    case MorphingStatus::DONE:
      return "DONE";
    }
    return "INVALID";
  }

  struct StageData{
    MorphingAction morph_action;
    MorphingTrigger morph_trigger;
    bool morph_auto;
    Scalar morph_tolerance;

    PlanType plan_type;
    Scalar plan_tolerance;
    Scalar plan_max_vel;
    Scalar plan_max_acc;
    Scalar plan_max_dyaw;
    Trajectory plan_traj;
    Vector<3> plan_terminal_vel;

    //! point mode
    std::vector<Vector<3>> plan_point_xyz;
    std::vector<Scalar> plan_point_yaw;
    Trajectory::HeadingMode plan_heading_mode;
  };

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
        rc_input_.valid() && rc_input_.isReceived(now) &&
        actuator_state_.valid() && actuator_state_.isReceived(now) &&
        stage_ >= 0 &&
        stage_ < int(stage_data_.size()))
    {
      //! Trigger to plan
      if (plan_status_ == PlanStatus::IDLE &&
          morphing_status_ == MorphingStatus::IDLE &&
          trigger_){
        ROS_INFO("\033[34m stage %d with velocity %.2f planning\033[34m",
                 stage_,
                 stage_data_[stage_].plan_max_vel);
        plan_status_ = PlanStatus::EXECUTING;
        if (!publishTrajectory()){
          ROS_ERROR("stage %d trajectory generation failed!", stage_);
          ROS_INFO("-------[END] Trajectory Failed -------");
          resetFirstStage();
          ROS_INFO("You can try again by giving trigger from RViz...");
          return;
        };
        //! Morphing start with planning
        if (stage_data_[stage_].morph_trigger == MorphingTrigger::START &&
            morphing_status_ == MorphingStatus::IDLE){

          if(stage_data_[stage_].morph_auto) {
            ROS_INFO("\033[34m stage %d with morphing %s \033[34m",
                     stage_, morphingActionToString(stage_data_[stage_].morph_action).c_str());
            morphing_status_ = MorphingStatus::EXECUTING;
            publishMorphing();
          } else{
            ROS_INFO("\033[34m stage %d waiting manual morphing trigger...\033[34m",
                     stage_);
            morphing_status_ = MorphingStatus::WAITING;
          }
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
          if (stage_data_[stage_].morph_trigger == MorphingTrigger::GOAL &&
              morphing_status_ == MorphingStatus::IDLE){

            if(stage_data_[stage_].morph_auto) {
              ROS_INFO("\033[34m stage %d with morphing %s \033[34m",
                       stage_, morphingActionToString(stage_data_[stage_].morph_action).c_str());
              morphing_status_ = MorphingStatus::EXECUTING;
              publishMorphing();
            } else{
              ROS_INFO("\033[34m stage %d waiting manual morphing trigger...\033[34m",
                       stage_);
              morphing_status_ = MorphingStatus::WAITING;
            }
          }
        }
      }

      //! Morphing waiting
      if (morphing_status_ == MorphingStatus::WAITING) {
        if (rc_input_.isMorphTriggered()){
          rc_input_.resetMorphTrigger();
          ROS_INFO("\033[34m stage %d with morphing %s \033[34m",
                   stage_, morphingActionToString(stage_data_[stage_].morph_action).c_str());
          morphing_status_ = MorphingStatus::EXECUTING;
          publishMorphing();
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
        if (stage_ == int(stage_data_.size())-1){
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

  bool getTaskParam(ros::NodeHandle& pnh){
    //! get task size
    int task_num;
    if(!getParam("task_num", task_num)){
      return false;
    }
    stage_data_.resize(task_num);

    std::string param_name, point_prefix, stage_prefix;

    for (int i = 0; i < task_num; i++){
      stage_prefix = "task_" + std::to_string(i);
      //! add plan type
      param_name = stage_prefix+ "/plan_type";
      int tmp_plan_type;
      if(!getParam(param_name, tmp_plan_type)){
        return false;
      }
      stage_data_[i].plan_type = static_cast<PlanType>(tmp_plan_type);

      //! add plan goal
      if (stage_data_[i].plan_type == PlanType::POINT){
        //! add plan point num
        param_name = stage_prefix+ "/plan_point_num";
        int tmp_plan_point_num;
        if(!getParam(param_name, tmp_plan_point_num)){
          return false;
        }

        stage_data_[i].plan_point_xyz.resize(tmp_plan_point_num);
        //! heading none as default
        stage_data_[i].plan_heading_mode = Trajectory::HeadingMode::HEADING_NONE;
        for (int j = 0; j < tmp_plan_point_num; j++){
          //! Get p0, p1, ..., pn with format of {x, y, z, yaw[optional]}
          point_prefix = "/plan_point_" + std::to_string(j);
          param_name = stage_prefix + point_prefix;
          Vector<3> tmp_xyz;
          Scalar tmp_yaw;
          if (pnh.getParam(param_name + "/x", tmp_xyz.x())
              && pnh.getParam(param_name + "/y", tmp_xyz.y())
              && pnh.getParam(param_name + "/z", tmp_xyz.z())){
            if (pnh.getParam(param_name + "/yaw", tmp_yaw)){
              stage_data_[i].plan_heading_mode = Trajectory::HeadingMode::HEADING_POLY;
              stage_data_[i].plan_point_yaw.push_back(tmp_yaw);
            }
            stage_data_[i].plan_point_xyz[j] = tmp_xyz;
          }
          else{
            ROS_ERROR_STREAM("Please check parameter " << param_name
                                                       << ", as " << stage_prefix+"/plan_point_num" << " is " << tmp_plan_point_num );
            return false;
          }
        }
        if (stage_data_[i].plan_heading_mode == Trajectory::HeadingMode::HEADING_POLY &&
            (stage_data_[i].plan_point_xyz.size() != stage_data_[i].plan_point_yaw.size())){
          ROS_ERROR_STREAM("The "<< stage_prefix+ "/plan_point" << "is invalid, "
                                                                   "as it includes the points with yaw and the points without yaw." );
          return false;
        }
      } else if (stage_data_[i].plan_type == PlanType::GRASP){
        param_name = stage_prefix + "/plan_grasp_align_distance";
        Scalar align_distance;
        if (!getParam(param_name, align_distance)){
          return false;
        }

        param_name = stage_prefix + "/plan_grasp_align_height";
        Scalar align_height;
        if (!getParam(param_name, align_height)){
          return false;
        }

        //! only support grasp point : x, y, z, yaw,
        param_name = stage_prefix + "/plan_grasp_point";
        Vector<3> grasp_point_xyz;
        Scalar grasp_point_yaw;
        if (pnh.getParam(param_name + "/x", grasp_point_xyz.x())
            && pnh.getParam(param_name + "/y", grasp_point_xyz.y())
            && pnh.getParam(param_name + "/z", grasp_point_xyz.z())){
          //! to be noted, yaw is not optional
          if (!pnh.getParam(param_name + "/yaw", grasp_point_yaw)){
            ROS_ERROR("In grasp type, yaw must be defined, "
                "which is missing in %s", param_name.c_str());
            return false;
          }
          stage_data_[i].plan_heading_mode = Trajectory::HeadingMode::HEADING_POLY;
          //! calculate the grasp align_point
          Vector<3> align_point_xyz;
          align_point_xyz.x() = grasp_point_xyz.x() - cos(grasp_point_yaw) * align_distance;
          align_point_xyz.y() = grasp_point_xyz.y() - sin(grasp_point_yaw) * align_distance;
          align_point_xyz.z() = grasp_point_xyz.z() + align_height;
          Scalar align_point_yaw = grasp_point_yaw;
          //! only contain two waypoint: align_point and  grasp_point
          stage_data_[i].plan_point_xyz.push_back(align_point_xyz);
          stage_data_[i].plan_point_xyz.push_back(grasp_point_xyz);
          stage_data_[i].plan_point_yaw.push_back(align_point_yaw);
          stage_data_[i].plan_point_yaw.push_back(grasp_point_yaw);
        }
        else{
          ROS_ERROR_STREAM("Please check parameter " << param_name );
          return false;
        }
      }

      //! add plan terminal vel [optional], zero as default
      param_name = stage_prefix+ "/plan_terminal_vel";
      getParamDefault(param_name, stage_data_[i].plan_terminal_vel, Vector<3>::Zero());

      //! add plan max vel
      double tmp_plan_max_vel;
      param_name = stage_prefix+ "/plan_max_vel";
      if(!getParam(param_name, tmp_plan_max_vel)){
        return false;
      }
      stage_data_[i].plan_max_vel = tmp_plan_max_vel;

      //! add plan max acc
      double tmp_plan_max_acc;
      param_name = stage_prefix+ "/plan_max_acc";
      if(!getParam(param_name, tmp_plan_max_acc)){
        return false;
      }
      stage_data_[i].plan_max_acc = tmp_plan_max_acc;

      //! add plan max dyaw
      double tmp_plan_max_dyaw;
      param_name = stage_prefix+ "/plan_max_dyaw";
      if(!getParam(param_name, tmp_plan_max_dyaw)){
        return false;
      }
      stage_data_[i].plan_max_dyaw = tmp_plan_max_dyaw;

      //! add morph action
      int tmp_morph_action;
      param_name = stage_prefix+ "/morph_action";
      getParamDefault(param_name, tmp_morph_action, 0);
      stage_data_[i].morph_action = static_cast<MorphingAction>(tmp_morph_action);

      //! add morph trigger
      int tmp_morph_trigger;
      param_name = stage_prefix+ "/morph_trigger";
      getParamDefault(param_name, tmp_morph_trigger, 0);
      stage_data_[i].morph_trigger = static_cast<MorphingTrigger>(tmp_morph_trigger);

      //! add morph auto
      bool tmp_morph_auto;
      param_name = stage_prefix+ "/morph_auto";
      getParamDefault(param_name, tmp_morph_auto, true);
      stage_data_[i].morph_auto = tmp_morph_auto;

      //! add morph tolerance
      double tmp_morph_tolerance;
      param_name = stage_prefix+ "/morph_tolerance";
      if(!getParam(param_name, tmp_morph_tolerance)){
        return false;
      }
      stage_data_[i].morph_tolerance = tmp_morph_tolerance;

      //! add plan tolerance
      double tmp_plan_tolerance;
      param_name = stage_prefix+ "/plan_tolerance";
      // NOTE: TRICK!!!! HIGHLY RELY ON THIS
      // if the value is smaller than 0 and morph trigger is goal,
      // it will be calculated based on shrinking time
      if (stage_data_[i].morph_trigger == MorphingTrigger::GOAL){
        getParamDefault(param_name, tmp_plan_tolerance, -1.0);
      } else {
        if(!getParam(param_name, tmp_plan_tolerance)){
          return false;
        }
      }
      stage_data_[i].plan_tolerance = tmp_plan_tolerance;
    }

    return true;
  }

  bool publishTrajectory(){

    flygripper_msgs::PolynomialTrajectory traj_msg;
    auto stage_data = stage_data_[stage_];
    auto& traj = stage_data_[stage_].plan_traj;
    Vector<3> init_vel = use_odom_init_vel_? odom_.vel: Vector<3>::Zero();

    if (stage_data.plan_type == PlanType::POINT || stage_data.plan_type == PlanType::GRASP){
      std::vector<Vector<3>> iner_xyz_pts(stage_data.plan_point_xyz.begin(),
                                          stage_data.plan_point_xyz.end()-1);

      if (stage_data.plan_heading_mode == Trajectory::HeadingMode::HEADING_POLY){
        std::vector<Scalar> iner_yaw_pts(stage_data.plan_point_yaw.begin(),
                                         stage_data.plan_point_yaw.end()-1);
        Scalar cur_yaw = odom_.getYaw(0.0);
        if (traj_gen_.getGoalTrajectory(traj,
                                        odom_.pos, cur_yaw,
                                        stage_data.plan_point_xyz.back(),
                                        stage_data.plan_point_yaw.back(),
                                        iner_xyz_pts,
                                        iner_yaw_pts,
                                        stage_data.plan_max_vel,
                                        stage_data.plan_max_acc,
                                        stage_data.plan_max_dyaw,
                                        init_vel,
                                        stage_data.plan_terminal_vel
        )){
          toRosMsg(traj,traj_msg);
          traj_pub_.publish(traj_msg);
          return true;
        }
      }
      else if (stage_data_[stage_].plan_heading_mode == Trajectory::HeadingMode::HEADING_NONE){

        Scalar cur_yaw = odom_.getYaw(0.0);
        std::vector<Scalar> iner_yaw_pts(iner_xyz_pts.size(), cur_yaw);

        if (traj_gen_.getGoalTrajectory(traj,
                                        odom_.pos, cur_yaw,
                                        stage_data.plan_point_xyz.back(),
                                        cur_yaw,
                                        iner_xyz_pts,
                                        iner_yaw_pts,
                                        stage_data.plan_max_vel,
                                        stage_data.plan_max_acc,
                                        stage_data.plan_max_dyaw,
                                        init_vel,
                                        stage_data.plan_terminal_vel
        )){
          toRosMsg(traj,traj_msg);
          traj_pub_.publish(traj_msg);
          return true;
        }
      }
    }
    else if (stage_data.plan_type == PlanType::DOOR){
      Scalar cur_yaw = odom_.getYaw(0.0);
      if (traj_gen_.getDoorTrajectory(traj,
                                      odom_.pos, cur_yaw,
                                      stage_data.plan_max_vel,
                                      stage_data.plan_max_acc,
                                      stage_data.plan_max_dyaw,
                                      init_vel,
                                      stage_data.plan_terminal_vel
      )){
        toRosMsg(traj,traj_msg);
        traj_pub_.publish(traj_msg);
        return true;
      }
    }
    return false;
  }

  bool isPlanCompleted(){
    if (!odom_.valid() || !odom_.isReceived(ros::Time::now().toSec()))
    {
      ROS_WARN("odom is not initialized while calling isPlanCompleted()");
      return false;
    }
    auto& poly_traj = stage_data_[stage_].plan_traj.xyz_min_jerk_poly;

    if (stage_data_[stage_].plan_tolerance < 0){
      //! plan_tolerance at least 1 cm.
      stage_data_[stage_].plan_tolerance =std::max((poly_traj.getPos(poly_traj.getTotalDuration()) -
          poly_traj.getPos(poly_traj.getTotalDuration() - morph_duration_)).norm(), 0.01);
      ROS_INFO("\033[34m stage %d reform plan tolerance to %f\033[34m",
               stage_,
               stage_data_[stage_].plan_tolerance);
    }

    Scalar dist = (poly_traj.getPos(poly_traj.getTotalDuration()) - odom_.pos).norm();

    return dist <= stage_data_[stage_].plan_tolerance;
  }

  bool isMorphingCompleted(){
    bool completed = false;
    completed |= (stage_data_[stage_].morph_action == MorphingAction::SHRINKING &&
        (actuator_state_.overload ||
            std::abs(actuator_state_.position - morph_shrink_pos_) < stage_data_[stage_].morph_tolerance));
    completed |= (stage_data_[stage_].morph_action == MorphingAction::EXPANDING &&
        std::abs(actuator_state_.position - morph_expand_pos_) < stage_data_[stage_].morph_tolerance);
    completed |= stage_data_[stage_].morph_action == MorphingAction::IDLE;
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

  void rcInCallback(const mavros_msgs::RCInConstPtr &msg)
  {
    fromRosMsg(msg, rc_input_, true);
    rc_input_.update();
  }

  void publishMorphing(){
    flygripper_msgs::ActuatorCommand actuator_command_msg;
    actuator_command_msg.header.stamp = ros::Time::now();
    actuator_command_msg.header.frame_id = "robot";
    actuator_command_msg.mode = flygripper_msgs::ActuatorCommand::MODE_POSITION;
    actuator_command_msg.reference_velocity = 0.0;
    actuator_command_msg.reference_effort = 0.0;

    switch (stage_data_[stage_].morph_action) {
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
  ros::Subscriber rc_in_sub_;

  Visualizer visualizer_;
  RcInput rc_input_;
  Odometry odom_;
  ActuatorState actuator_state_;
  bool trigger_ = false;



  std::vector<StageData> stage_data_;
  Scalar morph_shrink_pos_;
  Scalar morph_expand_pos_;
  Scalar morph_duration_;
  bool use_odom_init_vel_;

  PlanStatus plan_status_ = PlanStatus::IDLE;
  MorphingStatus morphing_status_ = MorphingStatus::IDLE;
  int stage_ = -1;

  TrajectoryGeneration traj_gen_;
};


int main(int argc, char **argv)
{
  ros::init(argc, argv, "task_scheduler");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  TaskScheduler app(nh, pnh);
  ros::Rate rate(333);

  while(ros::ok()){
    ros::spinOnce();
    app.run();
    rate.sleep();
  }

  return 0;
}