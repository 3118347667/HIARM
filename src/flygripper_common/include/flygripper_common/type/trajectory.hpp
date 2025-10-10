#pragma once
#include <Eigen/Dense>
#include <ros/ros.h>
#include "flygripper_msgs/PolynomialTrajectory.h"

#include "flygripper_common/trajectory/polynomial_trajectory.hpp"
#include "flygripper_common/type/base.hpp"
#include "flygripper_common/utils/math.hpp"

namespace flygripper_common{
class Trajectory
{
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  using Ptr =  std::shared_ptr<Trajectory>;
  Scalar stamp{INF};
  Scalar duration{INF};
  int id{-1};

  //! dim=3, order=7 for x, y ,z
  PolynomialTrajectory<3,5> xyz_min_jerk_poly;
  PolynomialTrajectory<3,7> xyz_min_snap_poly;

  //! if heading_mode is HEADING_POLY, dim=1, order=3 for heading
  PolynomialTrajectory<1,3> heading_poly;

  enum class XYZMode: int{
    XYZ_MIN_JERK = 0,
    XYZ_MIN_SNAP = 1,
  };

  enum class HeadingMode: int{
    HEADING_NONE = 0,
    HEADING_FORWARD = 1,
    HEADING_BACKWARD = 2,
    HEADING_POLY = 3,
  };

  HeadingMode heading_mode{HeadingMode::HEADING_NONE};
  XYZMode xyz_mode{XYZMode::XYZ_MIN_JERK};

 public:
  Trajectory(): heading_mode(HeadingMode::HEADING_NONE), xyz_mode(XYZMode::XYZ_MIN_JERK){}
  ~Trajectory() = default;

  template <int Order>
  inline void update(const Scalar& stamp, const int& id,
                     const PolynomialTrajectory<3,Order>& xyz_poly,
                     const HeadingMode heading_mode,
                     const PolynomialTrajectory<1,3>& heading_poly = PolynomialTrajectory<1,3>()){
    this->stamp = stamp;
    this->id = id;
    if (Order == 5){
      xyz_min_jerk_poly = xyz_poly;
      xyz_mode = XYZMode::XYZ_MIN_JERK;
    } else if (Order == 7){
      xyz_min_snap_poly = xyz_poly;
      xyz_mode = XYZMode::XYZ_MIN_SNAP;
    } else{
      ROS_ERROR("XYZ Trajectory Order must be 5 or 7");
      return;
    }
    this->heading_mode = heading_mode;
    if (heading_mode == HeadingMode::HEADING_POLY){
      this->heading_poly = heading_poly;
      if (!isApprox(xyz_poly.getTotalDuration(),
                   heading_poly.getTotalDuration())){
        ROS_ERROR("heading and xyz polynomial trajectory should have same duration, %f != %f",
                  heading_poly.getTotalDuration(),
                  xyz_poly.getTotalDuration());
        return;
      }
    }
    duration= xyz_poly.getTotalDuration();
  }

  inline void updateFromPoly(const Scalar& stamp, const int& id,
                             const HeadingMode heading_mode = HeadingMode::HEADING_NONE){
    this->stamp = stamp;
    this->id = id;
    if (xyz_min_jerk_poly.getPieceNum()>0){
      xyz_mode = XYZMode::XYZ_MIN_JERK;
      duration= xyz_min_jerk_poly.getTotalDuration();
    } else if (xyz_min_snap_poly.getPieceNum()>0){
      xyz_mode = XYZMode::XYZ_MIN_SNAP;
      duration= xyz_min_snap_poly.getTotalDuration();
    } else{
      ROS_ERROR("XYZ Trajectory Order must be 5 or 7");
      return;
    }
    this->heading_mode = heading_mode;
    if (heading_mode == HeadingMode::HEADING_POLY){
      if (!isApprox(heading_poly.getTotalDuration(),
                   duration)){
        ROS_ERROR("heading and xyz polynomial trajectory should have same duration, %f != %f",
                  heading_poly.getTotalDuration(),
                  duration);
        return;
      }
    }
  }

  inline bool available(const Scalar &now) const{
    if (now >= stamp && now <= stamp + duration){
      return true;
    } else{
//      ROS_ERROR("Trajectory is not available at time %f, trajectory is from %f to %f.",now, stamp, stamp + duration);
      return false;
    }

  }

  inline Scalar getStartTime() const{
    return stamp;
  }

  inline Scalar getEndTime() const{
    return stamp + duration;
  }

  inline bool valid() const{
    return id >= 0 &&
           ((xyz_mode == XYZMode::XYZ_MIN_JERK && xyz_min_jerk_poly.getPieceNum() > 0) ||
            (xyz_mode == XYZMode::XYZ_MIN_SNAP && xyz_min_snap_poly.getPieceNum() > 0)) &&
        (heading_mode != HeadingMode::HEADING_POLY || heading_poly.getPieceNum() > 0) &&
        std::isfinite(duration) &&
        std::isfinite(stamp);
  }

  inline bool isReceived(const Scalar &now) const{
    if (!std::isfinite(stamp)){
//      ROS_ERROR_ONCE("Trajectory has never been received!");
      return false;
    }
    return true;
  }
};

//! Trajectory <-> flygripper_msgs::PolynomialTrajectory
inline void fromRosMsg(flygripper_msgs::PolynomialTrajectoryConstPtr msg, Trajectory& data){
  //! Assign x, y, z
  int xyz_piece_num = msg->xyz_pieces.size();
  if (xyz_piece_num == 0){
    ROS_ERROR("XYZ Trajectory Piece Num must be greater than 0.");
    return;
  }

  int xyz_order = msg->xyz_pieces[0].order;
  if (xyz_order == 5 ){
    data.xyz_mode = Trajectory::XYZMode::XYZ_MIN_JERK;
    auto& xyz_poly = data.xyz_min_jerk_poly;
    xyz_poly.clear();
    xyz_poly.reserve(xyz_piece_num);
    for (int i = 0; i < xyz_piece_num; ++i)
    {
      auto& piece = msg->xyz_pieces[i];
      int dim = piece.dim;
      int order = piece.order;
      double duration = piece.duration;
      Matrix<> coeffMat = Map<const Matrix<>>(piece.coeff.data(), dim, order + 1);
      xyz_poly.emplace_back(duration, coeffMat);
    }
    data.duration = xyz_poly.getTotalDuration();
  } else if (xyz_order == 7){
    data.xyz_mode = Trajectory::XYZMode::XYZ_MIN_SNAP;
    auto& xyz_poly = data.xyz_min_snap_poly;
    xyz_poly.clear();
    xyz_poly.reserve(xyz_piece_num);
    for (int i = 0; i < xyz_piece_num; ++i)
    {
      auto& piece = msg->xyz_pieces[i];
      int dim = piece.dim;
      int order = piece.order;
      double duration = piece.duration;
      Matrix<> coeffMat = Map<const Matrix<>>(piece.coeff.data(), dim, order + 1);
      xyz_poly.emplace_back(duration, coeffMat);
    }
    data.duration = xyz_poly.getTotalDuration();
  } else{
    ROS_ERROR("XYZ Trajectory Order must be 5 or 7");
    return;
  }

  //! Assign heading
  data.heading_mode = Trajectory::HeadingMode(msg->heading_mode);
  // if heading mode is HEADING_POLY
  if (data.heading_mode == Trajectory::HeadingMode::HEADING_POLY){
    int heading_piece_num = msg->heading_pieces.size();
    if (heading_piece_num == 0){
      ROS_ERROR("Heading Trajectory Piece Num must be greater than 0.");
      return;
    }
    auto& heading_poly = data.heading_poly;
    heading_poly.clear();
    heading_poly.reserve(heading_piece_num);
    for (int i = 0; i < heading_piece_num; ++i)
    {
      auto& piece = msg->heading_pieces[i];
      int dim = piece.dim;
      int order = piece.order;
      double duration = piece.duration;
      Matrix<> coeffMat = Map<const Matrix<>>(piece.coeff.data(), dim, order + 1);
      heading_poly.emplace_back(duration, coeffMat);
    }
    if (!isApprox(data.duration, heading_poly.getTotalDuration())){
      ROS_ERROR("heading and xyz polynomial trajectory should have same duration, %f != %f",
                heading_poly.getTotalDuration(), data.duration);
      return;
    }
  }
  //! Assign time and id
  data.stamp = ros::Time::now().toSec();
  data.id = msg->id;
}

inline void toRosMsg(const Trajectory& data, flygripper_msgs::PolynomialTrajectory& msg){
  msg.id = data.id;
  msg.header.stamp = ros::Time(data.stamp);
  //! xyz polynomial
  if (data.xyz_mode == Trajectory::XYZMode::XYZ_MIN_JERK){
    auto& xyz_poly = data.xyz_min_jerk_poly;
    int xyz_piece_num = xyz_poly.getPieceNum();
    for (int i = 0; i < xyz_piece_num; ++i)
    {
      flygripper_msgs::Polynomial xyz_piece;
      xyz_piece.dim = xyz_poly[i].getDim();
      xyz_piece.order = xyz_poly[i].getDegree();
      xyz_piece.duration = xyz_poly[i].getDuration();
      auto cMat = xyz_poly[i].getCoeffMat();
      xyz_piece.coeff.assign(cMat.data(),cMat.data() + cMat.rows()*cMat.cols());
      msg.xyz_pieces.emplace_back(xyz_piece);
    }
  } else if (data.xyz_mode == Trajectory::XYZMode::XYZ_MIN_SNAP){
    auto& xyz_poly = data.xyz_min_snap_poly;
    int xyz_piece_num = xyz_poly.getPieceNum();
    for (int i = 0; i < xyz_piece_num; ++i)
    {
      flygripper_msgs::Polynomial xyz_piece;
      xyz_piece.dim = xyz_poly[i].getDim();
      xyz_piece.order = xyz_poly[i].getDegree();
      xyz_piece.duration = xyz_poly[i].getDuration();
      auto cMat = xyz_poly[i].getCoeffMat();
      xyz_piece.coeff.assign(cMat.data(),cMat.data() + cMat.rows()*cMat.cols());
      msg.xyz_pieces.emplace_back(xyz_piece);
    }
  }
  //! heading
  msg.heading_mode = static_cast<int>(data.heading_mode);
  if (data.heading_mode == Trajectory::HeadingMode::HEADING_POLY){
    auto& heading_poly = data.heading_poly;
    int heading_piece_num = heading_poly.getPieceNum();
    for (int i = 0; i < heading_piece_num; ++i)
    {
      flygripper_msgs::Polynomial heading_piece;
      heading_piece.dim = heading_poly[i].getDim();
      heading_piece.order = heading_poly[i].getDegree();
      heading_piece.duration = heading_poly[i].getDuration();
      auto cMat = heading_poly[i].getCoeffMat();
      heading_piece.coeff.assign(cMat.data(),cMat.data() + cMat.rows()*cMat.cols());
      msg.heading_pieces.emplace_back(heading_piece);
    }
  }
}
} //namespace flygripper_common
