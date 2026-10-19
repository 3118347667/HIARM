#pragma once
#include "flygripper_common/type/robot_state.hpp"
#include "flygripper_common/type/reference.hpp"
#include "flygripper_common/type/robot_command.hpp"

using namespace flygripper_common;
namespace flygripper_controller{
class ControllerBase {
 public:
  using Ptr = std::shared_ptr<ControllerBase>;
  ControllerBase() = default;
  virtual ~ControllerBase() = default;
  virtual void prepare(const RobotState& state) = 0;
  virtual void getCommand(const RobotState& state,
                          const ReferenceVector& references,
                          RobotCommand& command,
                          const Scalar& now) = 0;
};
}
