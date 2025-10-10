#pragma once
#include <iostream>
#include <ros/ros.h>
#include <numeric>

#include "flygripper_common/type/base.hpp"
#include "flygripper_common/filter/butterworth.hpp"

namespace flygripper_flight{
using namespace flygripper_common;
template <int Dims>
class WrenchBuffer{
 public:
  using Ptr = std::shared_ptr<WrenchBuffer<Dims>>;
 private:
  std::array<std::deque<double>,Dims> buffer_;
  std::array<Scalar, Dims> mean_;
  std::array<Scalar, Dims> stddev_;
  int buffer_size_;
  bool init_;
  Scalar cutoff_frequency_;
  Scalar sampling_frequency_;
  typename ButterworthLPF<Dims>::Ptr lpf_;
 public:
  WrenchBuffer(size_t buffer_size,
               const Scalar cutoff_frequency,
               const Scalar sampling_frequency):
      init_(false), buffer_size_(buffer_size),
      cutoff_frequency_(cutoff_frequency), sampling_frequency_(sampling_frequency){
  }

  void reset(){
    init_ = false;
    for (int i = 0; i < Dims; ++i){
      buffer_[i].clear();
    }
  }
  Vector<Dims> getMean(){
    return init_ ? Vector<Dims>(mean_.data()): Vector<Dims>::Zero();
  }

  bool push(const Ref<const Vector<Dims>>& values){
    // outlier check while full
    if (buffer_[0].size() > buffer_size_){
      // if outlier, then return
      for (int i = 0; i < Dims; ++i){
        if (isOutlier(values[i], mean_[i],stddev_[i])){
          return false;
        }
      }
    }

    // if not initialized, initialize the lpf
    Vector<Dims> values_lpf;
    if (!init_){
      lpf_ = std::make_shared<ButterworthLPF<Dims>>(cutoff_frequency_,
                                                    sampling_frequency_,
                                                    values);
      values_lpf = values;
      init_ = true;
    }  else{
      values_lpf = lpf_->apply(values);
    }

    // push the filtered values and pop if full
    for (int i = 0; i < Dims; ++i){
      auto& buffer = buffer_[i];
      buffer.push_back(values_lpf(i));
      mean_[i] = mean(buffer);
      stddev_[i] = stddev(buffer, mean_[i]);
      if (buffer.size() > buffer_size_) {
        buffer.pop_front();
      }
    }
    return true;
  }

  // 计算均值
  double mean(const std::deque<double>& data) {
    return std::accumulate(data.begin(),
                           data.end(),
                           0.0) / data.size();
  }

  // 计算标准差
  double stddev(const std::deque<double>& data, double mean) {
    double sum = 0.0;
    for (double value : data) {
      sum += (value - mean) * (value - mean);
    }
    return std::sqrt(sum / data.size());
  }

  // 判断是否为离群点
  bool isOutlier(double value, double mean, double stddev,
                 double multiplier = 2.0) {
    return std::abs(value - mean) > multiplier * stddev;
  }
};

template <int Dims>
class ExternalWrenchEstimator {
 public:
  using Ptr = std::shared_ptr<ExternalWrenchEstimator<Dims>>;
 private:
  enum class AppliedState{
    RELEASE,
    BEGIN_CONTACT,
    CONTACT,
    BEGIN_RELEASE,
  };

  AppliedState applied_state_;
  Scalar begin_contact_stamp_;
  Scalar begin_release_stamp_;
  Vector<Dims> internal_wrench_;
  Vector<Dims> external_wrench_;
  typename WrenchBuffer<Dims>::Ptr wrench_buffer_;
  typename ButterworthLPF<Dims>::Ptr lpf_;
  bool init_;
  bool torque_contact_, force_contact_;
 public:
  ExternalWrenchEstimator(const Scalar& wrench_buffer_size,
                          const Scalar& cutoff_frequency,
                          const Scalar& sampling_frequency):
      applied_state_(AppliedState::RELEASE),
      begin_contact_stamp_(0.0), begin_release_stamp_(0.0),
      init_(false), torque_contact_(false), force_contact_(false){
    wrench_buffer_ = std::make_shared<WrenchBuffer<Dims>>(
        wrench_buffer_size,
        cutoff_frequency,
        sampling_frequency);
    lpf_ = std::make_shared<ButterworthLPF<Dims>>(cutoff_frequency,
                                                  sampling_frequency,
                                                  0.0);
    reset();
  }

  void reset(){
    if(init_){
      init_ = false;
      torque_contact_ = false;
      force_contact_ = false;
      applied_state_ = AppliedState::RELEASE;
      internal_wrench_ = Vector<Dims>::Zero();
      external_wrench_ = Vector<Dims>::Zero();
      wrench_buffer_->reset();
    }
  }

  bool update(const Ref<const Vector<Dims>>& wrench,
              Ref<Vector<Dims>> external_wrench,
              const Scalar& contact_force_threshold,
              const Scalar& contact_torque_threshold,
              const Scalar& contact_trigger_duration){
    //! if internal wrench is got, then update state
    Vector<Dims> wrench_lpf;
    if (!init_){
      lpf_->reset(wrench);
    }
    else {
      wrench_lpf = lpf_->apply(wrench);
      //! To be noted: use wrench x,y as force norm
      Scalar force_norm = wrench_lpf.segment(0,2).norm();
      //! To be noted: use torque z as torque norm
      Scalar torque_norm = std::abs(wrench_lpf(3));
      force_contact_ = force_norm >= contact_force_threshold;
      torque_contact_ = torque_norm >= contact_torque_threshold;
      //! check if external wrench is applied based on its NORM refer to THRESHOLD
      if (applied_state_ == AppliedState::RELEASE &&
           (torque_contact_ || force_contact_)){
        //! RELEASE => BEGIN_CONTACT
        updateAppliedState(AppliedState::BEGIN_CONTACT, force_norm, torque_norm);
        begin_contact_stamp_ = ros::Time::now().toSec();
      }
      else if (applied_state_ == AppliedState::BEGIN_CONTACT){
        if (torque_contact_ || force_contact_){
          if ((ros::Time::now().toSec() - begin_contact_stamp_) > contact_trigger_duration){
            //! BEGIN_CONTACT => CONTACT
            updateAppliedState(AppliedState::CONTACT, force_norm, torque_norm);
          }
        } else{
          //! BEGIN_CONTACT => BEGIN_RELEASE
          updateAppliedState(AppliedState::BEGIN_RELEASE, force_norm, torque_norm);
          begin_release_stamp_ = ros::Time::now().toSec();
        }
      }
      else if (applied_state_ == AppliedState::CONTACT &&
               (!torque_contact_ && !force_contact_)){
        //! CONTACT => BEGIN_RELEASE
        updateAppliedState(AppliedState::BEGIN_RELEASE, force_norm, torque_norm);
        begin_release_stamp_ = ros::Time::now().toSec();
      } else if (applied_state_ == AppliedState::BEGIN_RELEASE){
        if (!torque_contact_ && !force_contact_){
          if ((ros::Time::now().toSec() - begin_release_stamp_) > contact_trigger_duration){
            //! BEGIN_RELEASE => RELEASE
            updateAppliedState(AppliedState::RELEASE, force_norm, torque_norm);
          }
        } else {
          //! BEGIN_RELEASE => BEGIN_CONTACT
          updateAppliedState(AppliedState::BEGIN_CONTACT, force_norm, torque_norm);
          begin_contact_stamp_ = ros::Time::now().toSec();
        }
      }
    }


    //! if external wrench is not applied from estimator
    if (applied_state_ == AppliedState::RELEASE){
      if (!wrench_buffer_->push(wrench)){
        ROS_WARN("Outlier detected!!!!!!!!!!!");
      } else{
        internal_wrench_ = wrench_buffer_->getMean();
        if(!init_) {
          init_ = true;
        }
      }
    }
    external_wrench_ = wrench_lpf - internal_wrench_;
    if (applied_state_ == AppliedState::CONTACT){
      external_wrench = external_wrench_;
      ROS_ERROR_STREAM_THROTTLE(2.0, "\ninternal_wrench: "<<"["<<internal_wrench_.transpose()<<"]"
                                                        << "\nexternal_wrench: "<<"["<<external_wrench_.transpose()<<"]");
      return true;
    } else{
      ROS_WARN_STREAM_THROTTLE(2.0, "\ninternal_wrench: "<<"["<<internal_wrench_.transpose()<<"]"
                                                       << "\nexternal_wrench: "<<"["<<external_wrench_.transpose()<<"]");
      return false;
    }

  }
  std::string appliedStateToString(AppliedState applied_state){
    switch (applied_state) {
    case AppliedState::RELEASE:
      return "RELEASE";
    case AppliedState::BEGIN_CONTACT:
      return "BEGIN_CONTACT";
    case AppliedState::CONTACT:
      return "CONTACT";
    case AppliedState::BEGIN_RELEASE:
      return "BEGIN_RELEASE";
    default:
      return "UNKNOWN";
    }
  }
  void updateAppliedState(AppliedState applied_state,
                          Scalar force_norm, Scalar torque_norm){
    if (applied_state_ != applied_state){
      ROS_INFO("\033[36mExternal Wrench Applied State: %s => %s\033[0m, force: %.2f, torque: %.2f ",
               appliedStateToString(applied_state_).c_str(),
               appliedStateToString(applied_state).c_str(), force_norm, torque_norm);
      applied_state_ = applied_state;
    }
  }

};
}