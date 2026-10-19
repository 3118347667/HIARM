#pragma once
#include <memory>

#include "flygripper_common/utils/rotation.hpp"
#include "flygripper_common/type/robot_parameter.hpp"

#include "flygripper_planner/minco.hpp"
#include "flygripper_planner/lbfgs.hpp"
#include "flygripper_planner/visualizer.hpp"
#include "flygripper_planner/traj_opt_utils.hpp"
#include "flygripper_planner/traj_opt_param.hpp"

namespace flygripper_planner{
using namespace flygripper_common;

inline double objectiveFuncGoal(void* ptrObj,
                                const double* x,
                                double* grad,
                                const int n);

inline int earlyExitGoal(void* ptrObj,
                         const double* x,
                         const double* grad,
                         const double fx,
                         const double xnorm,
                         const double gnorm,
                         const double step,
                         int n,
                         int k,
                         int ls);

class TrajOpt {
 public:
  using Ptr=std::shared_ptr<TrajOpt>;
  TrajOpt(ros::NodeHandle& nh, ros::NodeHandle& pnh){
    //! visualizer
    ros::NodeHandle visualizer_param_nh(nh, "visualizer");
    visualizer_ = std::make_shared<Visualizer>(nh);

    //! robot parameter
    ros::NodeHandle robot_param_nh(nh, "robot_parameter");
    if (!robot_param_config_.init(robot_param_nh)){
      throw ros::Exception("Failed to initialize robot parameter config");
    }
    robot_param_ = std::make_shared<RobotParameter>();
    robot_param_config_.getParams(*robot_param_);

    if (!traj_opt_param_config_.init(pnh)){
      throw ros::Exception("Failed to initialize trajectory optimization parameter config");
    }
    traj_opt_param_ = std::make_shared<TrajOptParameter>();
    traj_opt_param_config_.getParams(*traj_opt_param_);
  };

  ~TrajOpt() = default;

  void ExtractMidPointsFromPath(const std::vector<Eigen::Vector3d>& path,
                                const double& segment_per_distance,
                                std::vector<Eigen::Vector3d>& mid_points,
                                int& N,
                                int N_min = 2){
    int N_tmp = 0;
    std::vector<Eigen::Vector3d> points;
    // Assign path to points directly (Ray Checking if use gridmap)
    points = path;

    // Interpolate based on segment_per_distance
    for (size_t i = 0; i < points.size()-1; i++){
      Eigen::Vector3d cur_point = points[i];
      Eigen::Vector3d next_point = points[i+1];
      if (i != 0) mid_points.push_back(cur_point); // remove first point
      int n = round((next_point - cur_point).norm() / segment_per_distance);
      if (n > 1){
        Eigen::Vector3d dir = (next_point - cur_point).normalized();
        double dis_thres = (next_point - cur_point).norm() / n;
        for (int k = 1; k < n; k++){
          Eigen::Vector3d pk = cur_point + dir * dis_thres * k;
          mid_points.push_back(pk);
        }
      }
    }

    // N refers to piece num
    N_tmp = mid_points.size() + 1;

    // Interpolate based on N_min
    if (N_tmp < N_min){
      // get interpolated points from segment_per_distance mid points
      std::vector<Eigen::Vector3d> inter_points;
      inter_points.push_back(path.front());
      inter_points.insert(inter_points.end(),
                          mid_points.begin(), mid_points.end());
      inter_points.push_back(path.back());

      mid_points.clear();
      N = N_min;
      int n = ceil(N_min / (inter_points.size() - 1));
      for (size_t i = 0; i < inter_points.size()-1; i++){
        Eigen::Vector3d cur_point = inter_points[i];
        Eigen::Vector3d next_point = inter_points[i+1];
        if (i != 0) mid_points.push_back(cur_point); // remove first point
        if (n > 1){
          Eigen::Vector3d dir = (next_point - cur_point).normalized();
          double dis_thres = (next_point - cur_point).norm() / n;
          for (int k = 1; k < n; k++){
            Eigen::Vector3d pk = cur_point + dir * dis_thres * k;
            if ((pk-next_point).norm() > 0.01) {
              mid_points.push_back(pk);
              if (mid_points.size() + 1 >= N_min) {
                return;
              };
            }
          }
        }
      }
      N = mid_points.size() + 1;
      ROS_ERROR("Accident!");

    }else {
      N = N_tmp;
    }
  }

  void getInitTraj(const int& N,
                   const Eigen::MatrixXd& init_P_vec,
                   Eigen::VectorXd& init_T_vec){
    minco_s3_opt_.reset(init_state_.block<3,3>(0,0),
                        terminal_state_.block<3,3>(0,0), N);
    double desire_vel = robot_param_->velocity_max;
    double max_vel, max_acc;
    int cnt = 0;
    PolynomialTrajectory<3,5> traj;
    do{
      ROS_INFO_STREAM("cnt: " << cnt);
      for (int i = 0; i < N; i++){
        if (i == 0){
          init_T_vec(i) = (init_P_vec.col(i) - init_state_.col(0)).norm() / desire_vel;
        }else if (i == N-1){
          init_T_vec(i) = (terminal_state_.col(0) - init_P_vec.col(i-1)).norm() / desire_vel;
        }else{
          init_T_vec(i) = (init_P_vec.col(i) - init_P_vec.col(i-1)).norm() / desire_vel;
        }
      }
      ROS_INFO_STREAM("desire_vel: " << desire_vel);
      ROS_INFO_STREAM("desire_P_vec: " << init_P_vec.transpose());
      ROS_INFO_STREAM("init_T_vec: " << init_T_vec.transpose());

      minco_s3_opt_.generate(init_P_vec, init_T_vec);
      desire_vel /= 1.5;
      minco_s3_opt_.getTrajectory(traj);
      max_vel = traj.getMaxVelRate();
      max_acc = traj.getMaxAccRate();
      cnt++;
      ROS_INFO_STREAM("max_vel: " << max_vel<<", max_acc: " << max_acc);

    }while(cnt < 3 && (max_vel > robot_param_->velocity_max ||
        max_acc > robot_param_->acceleration_max));
  }

  void printCosts(){
    if (cost_lock_.test_and_set()) return;
    ROS_INFO_STREAM("-------------------[Traj Opt Cost]----------------");
    ROS_INFO_STREAM("dT: " << cost_t_);
    ROS_INFO_STREAM("Vel: " << cost_v_);
    ROS_INFO_STREAM("Acc: " << cost_a_);
    ROS_INFO_STREAM("DYaw: " << cost_dyaw_);

    ROS_INFO_STREAM("Pos Alignment Distance: " << cost_p_alignment_distance_);
    ROS_INFO_STREAM("Pos Alignment Angle: " << cost_p_alignment_angle_);
    ROS_INFO_STREAM("Yaw Alignment Angle: " << cost_yaw_alignment_angle_);

    ROS_INFO_STREAM("Jerk Energy: " << cost_jerk_);
    ROS_INFO_STREAM("DDyaw Energy: " << cost_ddyaw_);
    cost_lock_.clear();
  }

  void resetCosts()
  {
    cost_t_ = 0.0;
    cost_v_ = 0.0;
    cost_a_ = 0.0;
    cost_dyaw_ = 0.0;
    cost_jerk_ = 0.0;
    cost_ddyaw_ = 0.0;

    cost_p_alignment_distance_ = 0.0;
    cost_p_alignment_angle_ = 0.0;
    cost_yaw_alignment_angle_ = 0.0;
  }

  bool generateTraj(const Eigen::MatrixXd& init_state,
                    const Eigen::MatrixXd& terminal_state,
                    const Eigen::MatrixXd& init_yaw,
                    const Eigen::MatrixXd& terminal_yaw,
                    const double& segment_per_distance,
                    const std::vector<Eigen::Vector3d>& init_path,
                    PolynomialTrajectory<3, 5>& xyz_traj,
                    PolynomialTrajectory<1, 3>& yaw_traj){
    //! -1. update params
    traj_opt_param_config_.getParams(*traj_opt_param_);
    robot_param_config_.getParams(*robot_param_);

    //! 0. determine number of segment (here mapping is not implemented, if map, a-star path raycheck-aware)
    if (init_path.size() < 2){
      ROS_ERROR("init_path.size() < 2");
      return false;
    }
    std::vector<Eigen::Vector3d> mid_q_vec;
    int N_min = std::max(2, traj_opt_param_->min_piece_num);
    ExtractMidPointsFromPath(init_path, segment_per_distance, mid_q_vec, N_, N_min);
    ROS_INFO_STREAM("Pieces: " << N_);
    visualizer_->visualizeWaypoints(mid_q_vec, "mid_waypts");

    for (int i = 0; i < N_-1; ++i) {
      ROS_INFO_STREAM("mid_q_vec[" << i << "]: " << mid_q_vec[i].transpose());
    }
    //! 1. set opt varibles
    dim_t_ = N_;
    dim_p_ = N_ - 1;
    x_ = new double[dim_t_ + 4 * dim_p_];
    Eigen::Map<Eigen::VectorXd> t(x_, dim_t_);
    Eigen::Map<Eigen::MatrixXd> P(x_ + dim_t_, 3, dim_p_);
    Eigen::Map<Eigen::VectorXd> yaw(x_ + dim_t_+ 3 * dim_p_, dim_p_);

    //! 2. set boundary & initial value
    init_state_ = init_state;
    terminal_state_ = terminal_state;
    init_yaw_.resize(1, 2);
    terminal_yaw_.resize(1, 2);
    init_yaw_ = init_yaw;
    terminal_yaw_ = terminal_yaw;
    double error_angle = getAngleError(init_yaw(0, 0),
                                       terminal_yaw_(0, 0));
    terminal_yaw_(0, 0) = init_yaw_(0, 0) + error_angle;

    // P initial value
    for (int i = 1; i < N_; ++i){
      P.col(i-1) = mid_q_vec[i-1];
    }
    Eigen::VectorXd T(N_);
    getInitTraj(N_, P, T);
    backwardT(T, t);

    // Yaw initial value
    ROS_INFO_STREAM("init_yaw: " << init_yaw_(0,0)<<", end_yaw: " << terminal_yaw_(0,0));
    for (int i = 0; i < N_-1; ++i){
      yaw(i) = init_yaw(0, 0) + static_cast<double>(i) / N_ * error_angle;
    }

    //! 3. opt begin
    minco_s3_opt_.reset(init_state_.block<3,3>(0,0),
                        terminal_state_.block<3,3>(0,0),
                        N_);
    minco_s2_opt_.reset(init_yaw_, terminal_yaw_, N_);

    minco_s3_opt_.generate(P, T);
    minco_s3_opt_.getTrajectory(xyz_traj);
    minco_s2_opt_.generate(yaw, T);
    minco_s2_opt_.getTrajectory(yaw_traj);

    visualizer_->visualizeTrajectory(xyz_traj, yaw_traj, "debug_trajectory2", 0.1);

    lbfgs::lbfgs_parameter_t lbfgs_params;
    lbfgs::lbfgs_load_default_parameters(&lbfgs_params);
    lbfgs_params.mem_size = 16;
    lbfgs_params.past = 3;
    lbfgs_params.g_epsilon = 0;
    lbfgs_params.min_step = 1e-16;
    lbfgs_params.delta = 1e-4;
    lbfgs_params.line_search_type = 0;
    lbfgs_params.max_iterations = traj_opt_param_->max_iteration;

    double min_objective;

    auto tic = std::chrono::steady_clock::now();
    ROS_INFO_STREAM("begin to opt");
    iter_times_ = 0;
    int opt_ret = lbfgs::lbfgs_optimize(dim_t_ + 4 * dim_p_, x_,
                                        &min_objective,
                                        &objectiveFuncGoal, nullptr,
                                        &earlyExitGoal, this, &lbfgs_params);
    auto toc = std::chrono::steady_clock::now();

    ROS_INFO_STREAM("\033[32m>ret: " << opt_ret << "\033[0m");
    printCosts();
    ROS_INFO_STREAM("iter: " << iter_times_);
    std::cout << "optmization costs: " << (toc - tic).count() * 1e-6 << "ms" << std::endl;

    if (opt_ret < 0) {
      delete[] x_;
      return false;
    }else{
      forwardT(t, T);
      minco_s3_opt_.generate(P, T);
      minco_s3_opt_.getTrajectory(xyz_traj);
      minco_s2_opt_.generate(yaw, T);
      minco_s2_opt_.getTrajectory(yaw_traj);

      std::vector<Eigen::Vector3d> mid_waypts;
      for (int i = 0; i < N_; ++i){
        if (i == 0) continue;
        mid_waypts.push_back(xyz_traj[i].getPos(0.0));
      }
      visualizer_->visualizeWaypoints(mid_waypts, "mid_waypts");

      delete[] x_;
      return true;
    }
  }

  bool gradCostVel(const Eigen::Vector3d& v,
                   Eigen::Vector3d& gradv,
                   double& costv) {
    gradv.setZero();
    costv = 0;
    double vpen = v.squaredNorm() -
        robot_param_->velocity_max * robot_param_->velocity_max;

    if (vpen > 0) {
      // ROS_INFO_STREAM("v: " << v);
      // ROS_INFO_STREAM("vmax: " << robot_param_->velocity_max);
      double grad = 0;
      costv = smoothedL1(vpen, grad);
      gradv = traj_opt_param_->rhoV * grad * 2 * v;
      costv *= traj_opt_param_->rhoV;
      return true;
    }
    return false;


    // double vpen = v.squaredNorm() - vmax_ * vmax_;
    // if (vpen > 0) {
    //   gradv = rhoV_ * 6 * vpen * vpen * v;
    //   costv = rhoV_ * vpen * vpen * vpen;
    //   return true;
    // }
    // return false;

  }

  bool gradCostAcc(const Eigen::Vector3d& a,
                   Eigen::Vector3d& grada,
                   double& costa) {
    grada.setZero();
    costa = 0;
    double apen = a.squaredNorm()
        - robot_param_->acceleration_max * robot_param_->acceleration_max ;

    if (apen > 0) {
      // ROS_INFO_STREAM("a: " << a);
      // ROS_INFO_STREAM("amax: " << robot_param_->acceleration_max);
      double grad = 0;
      costa = smoothedL1(apen, grad);
      grada = traj_opt_param_->rhoA * grad * 2 * a;
      costa *= traj_opt_param_->rhoA;
      return true;
    }
    return false;

    // double apen = a.squaredNorm() - amax_ * amax_;
    // if (apen > 0) {
    //   grada = rhoA_ * 6 * apen * apen * a;
    //   costa = rhoA_ * apen * apen * apen;
    //   return true;
    // }
    // return false;
  }

  bool gradCostDYaw(const double& dyaw,
                    double& grad_dyaw,
                    double& cost_dyaw){
    grad_dyaw = 0;
    cost_dyaw = 0;
    double vpen = dyaw * dyaw
        - robot_param_->bodyrates_max.z() * robot_param_->bodyrates_max.z();

    if (vpen > 0) {
//      ROS_INFO_STREAM("dyaw: " << dyaw);
//      ROS_INFO_STREAM("dyaw_max: " << robot_param_->bodyrates_max.z());
      double grad = 0;
      cost_dyaw = smoothedL1(vpen, grad);
      grad_dyaw = traj_opt_param_->rhoDYaw * grad * 2 * dyaw;
      cost_dyaw *= traj_opt_param_->rhoDYaw;
      return true;
    }
    return false;
  }

  void addTimeIntPenaltyGoal(double& cost){
    // cost of one inner sample points
    double cost_inner;
    // state of innner sample point
    Eigen::Vector3d pos, vel, acc, jer;
    double yaw, dyaw, ddyaw;
    // gradient of the state of innner sample point
    Eigen::Vector3d grad_p, grad_v, grad_a;
    double grad_yaw, grad_dyaw;
    // derivatives of the time base
    Eigen::Matrix<double, 6, 1> beta0, beta1, beta2, beta3;
    Eigen::Matrix<double, 4, 1> beta_yaw0, beta_yaw1, beta_yaw2;
    // sample time step; alpha; trapezoidal integral weights
    double step, alpha, omg;
    // current sample time within the segment
    double st = 0.0;
    // gradient propogate to C & T of each segment
    Eigen::Matrix<double, 6, 3> gradViola_c;
    double gradViola_t;
    Eigen::Matrix<double, 4, 1> gradViola_c_yaw;
    double gradViola_t_yaw;
    // exact term cost and gradient
    double cost_vel, cost_acc, cost_vel_yaw;
    Eigen::Vector3d grad_vel, grad_acc;//, grad_collision_p;
    double grad_vel_yaw;

    double cost_p_alignment_angle, cost_yaw_alignment_angle;
    Eigen::Vector3d grad_p_alignment_angle;
    double grad_yaw_alignment_angle;
    Eigen::Vector3d target_p = terminal_state_.col(0);
    double target_yaw = terminal_yaw_(0);

    int innerLoop = traj_opt_param_->K + 1;

    for (int i = 0; i < N_; ++i) {
      step = minco_s3_opt_.T1(i) / traj_opt_param_->K ;
      const auto& c = minco_s3_opt_.b.block<6, 3>(i * 6, 0);
      const auto& c_yaw = minco_s2_opt_.b.block<4, 1>(i * 4, 0);
      st = 0.0;

      for (int j = 0; j < innerLoop; ++j) {
        omg = (j == 0 || j == innerLoop - 1) ? 0.5 : 1.0;
        alpha = 1.0 / traj_opt_param_->K * j;

        beta0 = cal_timebase_jerk(0, st);
        beta1 = cal_timebase_jerk(1, st);
        beta2 = cal_timebase_jerk(2, st);
        beta3 = cal_timebase_jerk(3, st);

        beta_yaw0 = cal_timebase_acc(0, st);
        beta_yaw1 = cal_timebase_acc(1, st);
        beta_yaw2 = cal_timebase_acc(2, st);
        pos   = c.transpose() * beta0;
        vel   = c.transpose() * beta1;
        acc   = c.transpose() * beta2;
        jer   = c.transpose() * beta3;
        yaw   = c_yaw.transpose() * beta_yaw0;
        dyaw  = c_yaw.transpose() * beta_yaw1;
        ddyaw = c_yaw.transpose() * beta_yaw2;

        gradViola_c.setZero();
        gradViola_t = 0.0;
        grad_p.setZero();
        grad_v.setZero();
        grad_a.setZero();
        gradViola_c_yaw.setZero();
        gradViola_t_yaw = 0.0;
        grad_yaw = 0.0;
        grad_dyaw = 0.0;
        cost_inner = 0.0;

//        if (grad_cost_collision(pos, grad_collision_p, cost_collision_p)) {
//          grad_p += grad_collision_p;
//          cost_inner += cost_collision_p;
//          cost_collision_ += omg * step * cost_collision_p;
//        }

        if (gradCostVel(vel, grad_vel, cost_vel)) {
          grad_v += grad_vel;
          cost_inner += cost_vel;
          cost_v_ += omg * step * cost_vel;
        }
        if (gradCostAcc(acc, grad_acc, cost_acc)) {
          grad_a += grad_acc;
          cost_inner += cost_acc;
          cost_a_ += omg * step * cost_acc;
        }
        if (gradCostDYaw(dyaw, grad_vel_yaw, cost_vel_yaw)) {
          grad_dyaw += grad_vel_yaw;
          cost_inner += cost_vel_yaw;
          cost_dyaw_ += omg * step * cost_vel_yaw;
        }

//        if (i >= 1){
        if (gradCostPosAlignmentAngle(pos, target_p, target_yaw, grad_p_alignment_angle, cost_p_alignment_angle)){
          grad_p += grad_p_alignment_angle;
          cost_inner += cost_p_alignment_angle;
          cost_p_alignment_angle_ += omg * step * cost_p_alignment_angle;
        }
        if (gradCostYawAlignmentAngle(yaw, pos, target_p, target_yaw, grad_yaw_alignment_angle, cost_yaw_alignment_angle)) {
          grad_yaw += grad_yaw_alignment_angle;
          cost_inner += cost_yaw_alignment_angle;
          cost_yaw_alignment_angle_ += omg * step * cost_yaw_alignment_angle;
        }
//        }

        gradViola_c = beta0 * grad_p.transpose();
        gradViola_t = grad_p.transpose() * vel;
        gradViola_c += beta1 * grad_v.transpose();
        gradViola_t += grad_v.transpose() * acc;
        gradViola_c += beta2 * grad_a.transpose();
        gradViola_t += grad_a.transpose() * jer;

        gradViola_c_yaw = beta_yaw0 * grad_yaw;
        gradViola_t_yaw = grad_yaw * dyaw;
        gradViola_c_yaw += beta_yaw1 * grad_dyaw;
        gradViola_t_yaw += grad_dyaw * ddyaw;

        minco_s3_opt_.gdC.block<6, 3>(i * 6, 0) += omg * step * gradViola_c;
        minco_s3_opt_.gdT(i) += omg * (cost_inner / traj_opt_param_->K + alpha * step * gradViola_t + alpha * step * gradViola_t_yaw);
        minco_s2_opt_.gdC.block<4, 1>(i * 4, 0) += omg * step * gradViola_c_yaw;

        cost += omg * step * cost_inner;
        st += step;
      }
    }
  }

  bool gradCostPosAlignmentAngle(const Eigen::Vector3d& p,
                                 const Eigen::Vector3d& target_p,
                                 const double& target_yaw,
                                 Eigen::Vector3d& gradp,
                                 double& costp){
    gradp.setZero();
    costp = 0;

    double track_angle_expect_ = target_yaw - M_PI; //TODO: set from param
    if (track_angle_expect_ > M_PI) {
      track_angle_expect_ -= 2 * M_PI;
    } else if (track_angle_expect_ < -M_PI) {
      track_angle_expect_ += 2 * M_PI;
    }

    Eigen::Vector3d a = p - target_p;
    Eigen::Vector3d b(cos(track_angle_expect_), sin(track_angle_expect_), 0.0);
    // INFO_MSG_BLUE("a: " << a.transpose() << ", b: "<<b.transpose());
    double inner_product = a.dot(b);
    double norm_a = a.norm();
    double norm_b = b.norm();
    if (norm_a < 0.05){
      return false;
    }
    double pen = 1.0 - inner_product / norm_a / norm_b;
//    ROS_INFO_STREAM("grad_cost_tracking_angle: " << inner_product / norm_a / norm_b);
    if (pen > 0) {
      double grad = 0;
      costp = smoothedL1(pen, grad);
      gradp = grad * -(norm_a * b - inner_product / norm_a * a) / norm_a / norm_a / norm_b;
      // INFO_MSG_BLUE("gradp: " << gradp.transpose());

      // gradp = grad * (norm_b * cosTheta / norm_a * a - b);
      gradp *= traj_opt_param_->rhoPAlignmentAngle;
      costp *= traj_opt_param_->rhoPAlignmentAngle;
      return true;
    } else {
      return false;
    }
  }

  bool gradCostYawAlignmentAngle(const double& yaw,
                                 const Eigen::Vector3d& p,
                                 const Eigen::Vector3d& target_p,
                                 const double& target_yaw,
                                 double& grad_yaw,
                                 double& cost_yaw){
    grad_yaw = 0;
    cost_yaw = 0;
    Eigen::Vector3d dp = target_p - p;
//    double exp_yaw = atan2(dp.y(), dp.x());
    double exp_yaw = target_yaw;
    double pen = - getAngleError(yaw, exp_yaw);

    // INFO_MSG_GREEN("p: "<<p.transpose()<<", tar: "<<target_p.transpose());
    std::string sgn;
    cost_yaw = traj_opt_param_->rhoYawAlignmentAngle * pen * pen;
    grad_yaw = traj_opt_param_->rhoYawAlignmentAngle * 2 * pen;
    if (grad_yaw >= 0){
      sgn = "-";
    }else{
      sgn = "+";
    }
    // INFO_MSG("("<<sgn << ")yaw: " << yaw << ", exp_yaw: " << exp_yaw << ", error_angle: " << pen);

    return true;
  }

  int N_;
  int dim_t_;
  int dim_p_;
  double* x_;

  int iter_times_;
  std::atomic_flag cost_lock_ = ATOMIC_FLAG_INIT;

  double cost_t_, cost_v_, cost_a_, cost_jerk_; //cost_thrust_, cost_omega_,
  double cost_dyaw_, cost_ddyaw_; //cost_yaw_,
  double cost_p_alignment_distance_, cost_p_alignment_angle_, cost_yaw_alignment_angle_;

  Eigen::MatrixXd init_state_;
  Eigen::MatrixXd terminal_state_;
  Eigen::MatrixXd init_yaw_;
  Eigen::MatrixXd terminal_yaw_;
  minco::MINCO<3, 3, false> minco_s3_opt_;
  minco::MINCO<1, 2, false> minco_s2_opt_;
  Visualizer::Ptr visualizer_;

  RobotParameterConfig robot_param_config_;
  RobotParameter::Ptr robot_param_;

  TrajOptParameterConfig traj_opt_param_config_;
  TrajOptParameter::Ptr traj_opt_param_;
};

inline double objectiveFuncGoal(void* ptrObj,
                                const double* x,
                                double* grad,
                                const int n){
  TrajOpt& obj = *(TrajOpt*)ptrObj;
  TrajOptParameter::Ptr param = obj.traj_opt_param_;

  obj.iter_times_++;
  obj.resetCosts();

  //! 1. fetch opt varaibles from x_
  Eigen::Map<const Eigen::VectorXd> t(x, obj.dim_t_);
  Eigen::Map<Eigen::VectorXd> grad_t(grad, obj.dim_t_);

  Eigen::Map<const Eigen::MatrixXd> P(x + obj.dim_t_, 3, obj.dim_p_);
  Eigen::Map<Eigen::MatrixXd> grad_P(grad + obj.dim_t_, 3, obj.dim_p_);

  Eigen::Map<const Eigen::MatrixXd> yaw(x + obj.dim_t_ + obj.dim_p_ * 3, 1, obj.dim_p_);
  Eigen::Map<Eigen::MatrixXd> grad_yaw(grad + obj.dim_t_ + obj.dim_p_ * 3, 1, obj.dim_p_);

  //! 2. reform P & T
  Eigen::VectorXd T(obj.N_);
  forwardT(t, T);

  //! 3. generate minco using P & T
  auto tic = std::chrono::steady_clock::now();
  obj.minco_s3_opt_.generate(P, T);
  obj.minco_s2_opt_.generate(yaw, T);

  double cost = 0.0;
  double jerk_energy_cost, ddyaw_energy_cost;
  obj.minco_s3_opt_.getEnergy(jerk_energy_cost);
  cost += jerk_energy_cost;
  obj.minco_s3_opt_.calGrads_CT();
  obj.cost_jerk_ = jerk_energy_cost;

  obj.minco_s2_opt_.getEnergy(ddyaw_energy_cost);
  cost += ddyaw_energy_cost;
  obj.minco_s2_opt_.calGrads_CT();
  obj.cost_ddyaw_ = ddyaw_energy_cost;

  auto toc = std::chrono::steady_clock::now();

  //! 4. calculate penalty and gradient to C & T
  tic = std::chrono::steady_clock::now();
  obj.addTimeIntPenaltyGoal(cost);
  toc = std::chrono::steady_clock::now();

  //! 5. propogate gradient to mid-point P & T
  tic = std::chrono::steady_clock::now();
  obj.minco_s3_opt_.calGrads_PT();

  obj.minco_s2_opt_.calGrads_PT();

  if (param->fix_p){
    obj.minco_s3_opt_.gdP.setZero();
    obj.minco_s2_opt_.gdP.setZero();
  }

  toc = std::chrono::steady_clock::now();

  //! 6. propogate gradient to opt variable P & tau
  obj.minco_s3_opt_.gdT.array() += param->rhoT;
  cost += param->rhoT * T.sum();
  obj.cost_t_ = param->rhoT * T.sum();

  addLayerTGrad(t, obj.minco_s3_opt_.gdT + obj.minco_s2_opt_.gdT, grad_t);
  grad_P = obj.minco_s3_opt_.gdP;
  grad_yaw = obj.minco_s2_opt_.gdP;

//  obj.printCosts();

  return cost;
}

inline int earlyExitGoal(void* ptrObj,
                         const double* x,
                         const double* grad,
                         const double fx,
                         const double xnorm,
                         const double gnorm,
                         const double step,
                         int n,
                         int k,
                         int ls){
  TrajOpt& obj = *(TrajOpt*)ptrObj;
  TrajOptParameter::Ptr param = obj.traj_opt_param_;
  if (param->pause_debug) {
    ROS_INFO_STREAM("earlyExit iter: " << obj.iter_times_);
    Eigen::Map<const Eigen::VectorXd> t(x, obj.dim_t_);
    Eigen::Map<const Eigen::MatrixXd> P(x + obj.dim_t_, 3, obj.dim_p_);
    Eigen::Map<const Eigen::MatrixXd> yaw(x + obj.dim_t_ + obj.dim_p_ * 3, 1, obj.dim_p_);

    Eigen::VectorXd T(obj.N_);
    forwardT(t, T);

    obj.minco_s3_opt_.generate(P, T);
    obj.minco_s2_opt_.generate(yaw, T);

    PolynomialTrajectory<3,5> xyz_traj;
    obj.minco_s3_opt_.getTrajectory(xyz_traj);
    PolynomialTrajectory<1,3> yaw_traj;
    obj.minco_s2_opt_.getTrajectory(yaw_traj);
    obj.visualizer_->visualizeTrajectory(xyz_traj, yaw_traj, "debug_trajectory2", 0.1);

    std::vector<Eigen::Vector3d> int_waypts;
    for (const auto& piece : xyz_traj) {
      const auto& dur = piece.getDuration();
      for (int i = 0; i < param->K; ++i) {
        double t = dur * i / param->K;
        int_waypts.push_back(piece.getPos(t));
      }
    }
    obj.visualizer_->visualizeWaypoints(int_waypts, "int_waypts");

//    for (auto p : int_waypts){
//      Eigen::Vector3d gradp;
//      double costp;
//      obj.gradCostCollision(p, gradp, costp);
//      double          dist = 0;
//      Eigen::Vector3d dist_grad;
//      dist_grad.setZero();
//      // if (obj.esdfmap_.evaluateESDFWithGrad(p, dist, dist_grad)){
//      //   INFO_MSG_GREEN("p:"<<p.transpose()<<", esdf: "<<dist<<", cost: "<<costp<<", grad: "<<gradp.transpose());
//      // }
//
//    }
    ROS_INFO_STREAM("in process");
    obj.printCosts();
//    int a;
//    std::cin >> a;
    // NOTE pause
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
  return 0;
}

}