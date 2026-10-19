#pragma once

#include <ros/ros.h>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <algorithm>
#include <cmath>
#include <queue>
#include <vector>

namespace prediction {

struct TargetState {
  double t;
  Eigen::Vector3d p;
  Eigen::Vector3d v;
  Eigen::Quaterniond q;
};

struct Node {
  Eigen::Vector3d p, v, a;
  double t;
  double score;
  double h;
  Node* parent = nullptr;
};
typedef Node* NodePtr;
class NodeComparator {
 public:
  bool operator()(NodePtr& lhs, NodePtr& rhs) {
    return lhs->score + lhs->h > rhs->score + rhs->h;
  }
};
struct Predict {
 private:
  static constexpr int MAX_MEMORY = 1 << 22;
  // searching

  double dt;
  double pre_dur;
  double rho_a;
  double car_z, vmax;
  NodePtr data[MAX_MEMORY] = {};
  int stack_top;

  inline bool isValid(const Eigen::Vector3d& p, const Eigen::Vector3d& v) const {
    return p.allFinite() && v.allFinite() && (v.norm() < vmax);
  }

 public:
  inline Predict(ros::NodeHandle& nh) {
    nh.getParam("tracking_dur", pre_dur);
    nh.getParam("tracking_dt", dt);
    nh.getParam("prediction/rho_a", rho_a);
    nh.getParam("prediction/vmax", vmax);
  }
  ~Predict() {
    for (int i = 0; i < MAX_MEMORY; ++i) delete data[i];
  }
  inline NodePtr getNode(int index) {
    if (!data[index]) data[index] = new Node;
    return data[index];
  }
  inline bool predict(const Eigen::Vector3d& target_p,
                      const Eigen::Vector3d& target_v,
                      std::vector<Eigen::Vector3d>& target_predcit,
                      const double& max_time = 0.1) {
    auto score = [&](const NodePtr& ptr) -> double {
      return rho_a * ptr->a.norm();
    };
    Eigen::Vector3d end_p = target_p + target_v * pre_dur;
    auto calH = [&](const NodePtr& ptr) -> double {
      return 0.001 * (ptr->p - end_p).norm();
    };
    ros::Time t_start = ros::Time::now();
    std::priority_queue<NodePtr, std::vector<NodePtr>, NodeComparator> open_set;

    Eigen::Vector3d input(0, 0, 0);

    stack_top = 0;
    NodePtr curPtr = getNode(stack_top++);
    curPtr->p = target_p;
    curPtr->v = target_v;
    curPtr->a.setZero();
    curPtr->parent = nullptr;
    curPtr->score = 0;
    curPtr->h = 0;
    curPtr->t = 0;
    double dt2_2 = dt * dt / 2;
    while (curPtr->t < pre_dur) {
      for (input.x() = -3; input.x() <= 3; input.x() += 3)
        for (input.y() = -3; input.y() <= 3; input.y() += 3) {
          Eigen::Vector3d p = curPtr->p + curPtr->v * dt + input * dt2_2;
          Eigen::Vector3d v = curPtr->v + input * dt;
          if (!isValid(p, v)) {
            continue;
          }
          if (stack_top == MAX_MEMORY) {
            std::cout << "[prediction] out of memory!" << std::endl;
            return false;
          }
          double t_cost = (ros::Time::now() - t_start).toSec();
          if (t_cost > max_time) {
            std::cout << "[prediction] too slow!" << std::endl;
            return false;
          }
          NodePtr ptr = getNode(stack_top++);
          ptr->p = p;
          ptr->v = v;
          ptr->a = input;
          ptr->parent = curPtr;
          ptr->t = curPtr->t + dt;
          ptr->score = curPtr->score + score(ptr);
          ptr->h = calH(ptr);
          open_set.push(ptr);
          // std::cout << "open set push: " << state.transpose() << std::endl;
        }
      if (open_set.empty()) {
        std::cout << "[prediction] no way!" << std::endl;
        return false;
      }
      curPtr = open_set.top();
      open_set.pop();
    }
    target_predcit.clear();
    while (curPtr != nullptr) {
      target_predcit.push_back(curPtr->p);
      curPtr = curPtr->parent;
    }
    std::reverse(target_predcit.begin(), target_predcit.end());
    return true;
  }

  // Constant-turn-rate model for a planar vehicle. p/v are at its mocap
  // rigid-body origin; offset is fixed in the rigid-body axes. Each output
  // state describes the same physical grasp point at a known relative time.
  inline bool predictRigidBody(const Eigen::Vector3d& origin_p,
                               const Eigen::Vector3d& origin_v,
                               const Eigen::Quaterniond& orientation,
                               double yaw_rate,
                               const Eigen::Vector3d& offset,
                               double start_delay,
                               std::vector<TargetState>& predicted) const {
    return predictRigidBodyModel(origin_p, origin_v, orientation, yaw_rate,
                                 offset, start_delay, pre_dur, dt, vmax,
                                 predicted);
  }

  static inline bool predictRigidBodyModel(
      const Eigen::Vector3d& origin_p, const Eigen::Vector3d& origin_v,
      const Eigen::Quaterniond& orientation, double yaw_rate,
      const Eigen::Vector3d& offset, double start_delay, double duration,
      double step, double speed_limit, std::vector<TargetState>& predicted) {
    if (step <= 0.0 || duration <= 0.0 || speed_limit <= 0.0 ||
        !std::isfinite(step) || !std::isfinite(duration) ||
        !std::isfinite(speed_limit) || !std::isfinite(start_delay) ||
        !std::isfinite(yaw_rate) || start_delay < 0.0 ||
        !origin_p.allFinite() || !origin_v.allFinite() ||
        !orientation.coeffs().allFinite() || orientation.squaredNorm() < 1e-12 ||
        !offset.allFinite()) {
      return false;
    }

    const Eigen::Quaterniond q0 = orientation.normalized();
    predicted.clear();
    auto append_state = [&](double relative_t) -> bool {
      const double t = start_delay + relative_t;
      const double angle = yaw_rate * t;
      const Eigen::Quaterniond turn(Eigen::AngleAxisd(angle,
                                                       Eigen::Vector3d::UnitZ()));
      const Eigen::Quaterniond q = turn * q0;
      Eigen::Vector3d p = origin_p;
      if (std::abs(yaw_rate) < 1e-6) {
        p += origin_v * t;
      } else {
        const double s = std::sin(angle) / yaw_rate;
        const double c = (1.0 - std::cos(angle)) / yaw_rate;
        p.x() += s * origin_v.x() - c * origin_v.y();
        p.y() += c * origin_v.x() + s * origin_v.y();
        p.z() += origin_v.z() * t;
      }
      const Eigen::Vector3d v_origin = turn * origin_v;
      const Eigen::Vector3d rotated_offset = q * offset;
      TargetState state;
      state.t = relative_t;
      state.p = p + rotated_offset;
      state.v = v_origin +
                Eigen::Vector3d(0.0, 0.0, yaw_rate).cross(rotated_offset);
      state.q = q;
      if (!state.p.allFinite() || !state.v.allFinite() ||
          state.v.norm() >= speed_limit) {
        return false;
      }
      predicted.push_back(state);
      return true;
    };

    for (double t = 0.0; t < duration - 1e-9; t += step) {
      if (!append_state(t)) {
        predicted.clear();
        return false;
      }
    }
    if (!append_state(duration)) {
      predicted.clear();
      return false;
    }
    return true;
  }
};

}  // namespace prediction
