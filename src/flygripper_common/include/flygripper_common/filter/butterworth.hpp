#pragma once
#include "flygripper_common/type/base.hpp"
#include <memory>

namespace flygripper_common {

class ButterworthLPF1D{
 public:
  using Ptr = std::shared_ptr<ButterworthLPF1D>;
  ButterworthLPF1D(const Scalar cutoff_frequency,
                const Scalar sampling_frequency,
                const Scalar initial_value):
      cutoff_frequency_(cutoff_frequency),
      sampling_frequency_(sampling_frequency),
      denumerator_(ButterworthLPF1D::initDen(cutoff_frequency, sampling_frequency)),
      numerator_(ButterworthLPF1D::initNum(cutoff_frequency, sampling_frequency)),
      input_(Array<1,2>::Constant(initial_value)),
      output_(Array<1,2>::Constant(initial_value)){}

  ButterworthLPF1D() : ButterworthLPF1D(INF, INF, INF) {}

  ~ButterworthLPF1D() = default;

  void reset(const Scalar initial_value=0.0){
      input_ = Array<1,2>::Constant(initial_value);
      output_ = Array<1,2>::Constant(initial_value);
  }

  Scalar apply(const Scalar& sample) {
    const Scalar x2 = input_[1];
    input_[1] = input_[0];
    input_[0] = sample;
    const Scalar out =
        numerator_[0] * x2 +
        (numerator_ * input_ - denumerator_ * output_).sum();
    output_[1] = output_[0];
    output_[0] = out;

    return out;
  }

  Scalar get() const { return output_[0]; }

  Scalar getDerivative() const {
    return sampling_frequency_ * (output_[0] - output_[1]);
  }

  bool valid() const {
    return std::isfinite(sampling_frequency_) && denumerator_.allFinite() &&
           numerator_.allFinite() && input_.allFinite() && output_.allFinite();
  };

  Scalar getCutoffFrequency() const { return cutoff_frequency_; }

 private:

  static Array<1, 2> initDen(const Scalar& fc,
                                 const Scalar& fs) {
    const Scalar K = tan(M_PI * fc / fs);
    const Scalar poly = K * K + sqrt(2.0) * K + 1.0;

    Array<1, 2> denumerator = Array<1, 2>::Zero();
    denumerator[0] = 2.0 * (K * K - 1.0) / poly;
    denumerator[1] = (K * K - sqrt(2.0) * K + 1.0) / poly;

    return denumerator;
  }

  static Array<1, 2> initNum(const Scalar& fc,
                                 const Scalar& fs) {
    const Scalar K = tan(M_PI * fc / fs);
    const Scalar poly = K * K + sqrt(2.0) * K + 1.0;

    Array<1, 2> numerator = Array<1, 2>::Zero();
    numerator[0] = K * K / poly;
    numerator[1] = 2.0 * numerator[0];

    return numerator;
  }

  const Scalar sampling_frequency_;
  const Scalar cutoff_frequency_;
  const Array<1, 2> denumerator_;
  const Array<1, 2> numerator_;
  Array<1, 2> input_;
  Array<1, 2> output_;
};

template<int Dims>
class ButterworthLPF {
 public:
  using Ptr = std::shared_ptr<ButterworthLPF<Dims>>;
  ButterworthLPF(const Vector<Dims>& cutoff_frequency,
                const Vector<Dims>& sampling_frequency,
                const Vector<Dims>& initial_value=Vector<Dims>::Zero())
      : cutoff_frequency_(cutoff_frequency),
        sampling_frequency_(sampling_frequency),
        denumerator_(ButterworthLPF::initDen(cutoff_frequency, sampling_frequency)),
        numerator_(ButterworthLPF::initNum(cutoff_frequency, sampling_frequency)),
        input_(initial_value.replicate(1, 2)),
        output_(initial_value.replicate(1, 2)) {}

  ButterworthLPF(const Scalar cutoff_frequency,
                const Scalar sampling_frequency,
                const Scalar initial_value=0.0)
      : ButterworthLPF(Vector<Dims>::Constant(cutoff_frequency),
                      Vector<Dims>::Constant(sampling_frequency),
                      Vector<Dims>::Constant(initial_value)) {}

  ButterworthLPF(const Scalar cutoff_frequency,
                const Scalar sampling_frequency,
                const Vector<Dims>& initial_value)
      : ButterworthLPF(Vector<Dims>::Constant(cutoff_frequency),
                      Vector<Dims>::Constant(sampling_frequency),
                      initial_value) {}

  ButterworthLPF() : ButterworthLPF(INF, INF, INF) {}

  ~ButterworthLPF() = default;

  void reset(const Scalar initial_value=0.0) {
    reset(Vector<Dims>::Constant(initial_value));
  }

  void reset(const Vector<Dims>& initial_value) {
    input_ = initial_value.replicate(1, 2);
    output_ = initial_value.replicate(1, 2);
  }

  Vector<Dims> apply(const Vector<Dims>& sample) {
    const ArrayVector<Dims> x2 = input_.col(1);
    input_.col(1) = input_.col(0);
    input_.col(0) = sample;
    const Vector<Dims> out =
        numerator_.col(0) * x2 +
            (numerator_ * input_ - denumerator_ * output_).rowwise().sum();
    output_.col(1) = output_.col(0);
    output_.col(0) = out;
    return out;
  }

  Vector<Dims> get() const { return output_.col(0); }
  Scalar get(const int i) const { return output_(i, 0); }

  Vector<Dims> getDerivative() const {
    return sampling_frequency_.array() *
        (output_.col(0) - output_.col(1)).array();
  }

  Scalar getDerivative(const int i) const {
    return sampling_frequency_(i) * (output_(i, 0) - output_(i, 1));
  }

  bool valid() const {
    return sampling_frequency_.allFinite() && denumerator_.allFinite() &&
        numerator_.allFinite() && input_.allFinite() && output_.allFinite();
  };

  Vector<Dims> getCutoffFrequency() const { return cutoff_frequency_; }
  Scalar getCutoffFrequency(const int i) const { return cutoff_frequency_(i); }

 private:

  static Array<Dims, 2> initDen(const Vector<Dims>& fc,
                                 const Vector<Dims>& fs) {
    const ArrayVector<Dims> K = (M_PI * fc.array() / fs.array()).tan();
    const ArrayVector<Dims> poly = K * K + sqrt(2.0) * K + 1.0;

    Array<Dims, 2> denumerator = Array<Dims, 2>::Zero();
    denumerator.col(0) = 2.0 * (K * K - 1.0) / poly;
    denumerator.col(1) = (K * K - sqrt(2.0) * K + 1.0) / poly;

    return denumerator;
  }

  static Array<Dims, 2> initNum(const Vector<Dims>& fc,
                                 const Vector<Dims>& fs) {
    const ArrayVector<Dims> K = (M_PI * fc.array() / fs.array()).tan();
    const ArrayVector<Dims> poly = K * K + sqrt(2.0) * K + 1.0;

    Array<Dims, 2> numerator = Array<Dims, 2>::Zero();
    numerator.col(0) = K * K / poly;
    numerator.col(1) = 2.0 * numerator.col(0);

    return numerator;
  }
  const Vector<Dims> cutoff_frequency_;
  const Vector<Dims> sampling_frequency_;
  const Array<Dims, 2> denumerator_;
  const Array<Dims, 2> numerator_;
  Array<Dims, 2> input_;
  Array<Dims, 2> output_;
};

}