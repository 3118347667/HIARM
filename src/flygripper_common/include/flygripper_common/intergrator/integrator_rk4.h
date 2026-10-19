#pragma once
#include "flygripper_common/intergrator/integrator_base.h"
namespace flygripper_common
{
class IntegratorRK4 : public IntegratorBase
{
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  using IntegratorBase::IntegratorBase;

  virtual bool step(const DynamicsFunction& dynamics,
                    const Ref<const Vector<>> initial_state,
                    Ref<Vector<>> final_state,
                    const Scalar dt) const override {
    static const Vector<4> rk4_sum_vec{1.0 / 6.0,
                                       2.0 / 6.0,
                                       2.0 / 6.0,
                                       1.0 / 6.0};
    Matrix<> k = Matrix<>::Zero(initial_state.rows(), 4);

    final_state = initial_state;

    // k_1
    if (!dynamics(final_state, k.col(0))) return false;

    // k_2
    final_state = initial_state + 0.5 * dt * k.col(0);
    if (!dynamics(final_state, k.col(1))) return false;

    // k_3
    final_state = initial_state + 0.5 * dt * k.col(1);
    if (!dynamics(final_state, k.col(2))) return false;

    // k_4
    final_state = initial_state + dt * k.col(2);
    if (!dynamics(final_state, k.col(3))) return false;


    final_state = initial_state + dt * k * rk4_sum_vec;

    return true;
  }
};

} // namespace flygripper_common