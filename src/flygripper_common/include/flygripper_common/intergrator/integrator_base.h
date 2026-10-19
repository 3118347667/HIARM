#pragma once
#include "flygripper_common/type/base.hpp"
#include <memory>
namespace flygripper_common
{
class IntegratorBase
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
 public:
  using Ptr = std::shared_ptr<IntegratorBase>;
  IntegratorBase(const Scalar dt = 1e-3): dt_(dt) {}

  virtual ~IntegratorBase() = default;

  bool do_step(const DynamicsFunction& dynamics,
               const Ref<const Vector<>> initial_state,
               Ref<Vector<>> final_state,
               const Scalar duration) const {

    Scalar remain_duration = duration;
    Vector<> state = initial_state;

    do {
      const Scalar cur_dt = std::min(remain_duration, dt_);
      if (!step(dynamics, state, final_state, cur_dt)) return false;
      state = final_state;
      remain_duration -= cur_dt;
    } while (remain_duration > 0.0);

    return true;
  }

  Scalar dt() const { return dt_; }

  virtual bool step(const DynamicsFunction& dynamics,
                    const Ref<const Vector<>> initial_state,
                    Ref<Vector<>> final_state,
                    const Scalar dt) const = 0;


 protected:
  Scalar dt_;
};

} // namespace flygripper_common