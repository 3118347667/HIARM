#pragma once
#include "flygripper_common/type/base.hpp"
namespace flygripper_common{
static constexpr Scalar kGravity = 9.7936;
const Vector<3> kGravityVector = Vector<3>(0, 0, -kGravity);
const std::string kFixedFrame = "world";

}