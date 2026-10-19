#pragma once
#include <Eigen/Dense>
#include "flygripper_common/type/base.hpp"

namespace flygripper_common{
inline bool isZero(const Scalar& val){
  return std::abs(val) < EPSILON;
}

inline bool isApprox(const Scalar& val1, const Scalar& val2, const Scalar& eps=EPSILON){
  return std::abs(val1-val2) < eps;
}

inline Vector<> clipVector(const Vector<>& vec, const Vector<>& lo, const Vector<>& hi){
  return vec.cwiseMin(hi).cwiseMax(lo);
}

inline Vector<> clipVector(const Vector<>& vec, const Vector<>& bound){
  return vec.cwiseMin(bound).cwiseMax(-bound);
}

}


