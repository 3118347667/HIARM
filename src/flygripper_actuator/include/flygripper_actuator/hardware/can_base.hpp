#pragma once
#include <linux/can.h>
namespace flygripper_actuator{
class CanBase{
 public:
  using Ptr = std::shared_ptr<CanBase>;
  CanBase() = default;
  virtual ~CanBase() = default;
  virtual bool open() = 0;
  virtual void close() = 0;
  virtual void write(const can_frame& frame) = 0;
};
}
