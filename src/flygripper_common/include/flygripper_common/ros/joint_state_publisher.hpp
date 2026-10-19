#pragma once

#include <ros/ros.h>
#include <realtime_tools/realtime_publisher.h>
#include <sensor_msgs/JointState.h>

namespace flygripper_common{
struct JointState{
  std::string name;
  double position{0.0};
  double velocity{0.0};
  double effort{0.0};
};

class JointStatePublisher{
 public:
  using Ptr = std::shared_ptr<JointStatePublisher>;
  JointStatePublisher(ros::NodeHandle& nh, ros::NodeHandle& pnh) {
    realtime_pub_.reset(new realtime_tools::RealtimePublisher<sensor_msgs::JointState>(nh, "joint_states", 4));
    pnh.param("joint_state_publisher/publish_frequency", publish_freq_, 50.0);
    last_publish_time_ = ros::Time::now();
  }

  ~JointStatePublisher() = default;

  void publish(const ros::Time& time, const std::vector<JointState>& joint_states){
    if (publish_freq_ > 0.0 &&
        last_publish_time_ + ros::Duration(1.0/publish_freq_) < time){
      if (realtime_pub_->trylock()){
        last_publish_time_ = last_publish_time_ + ros::Duration(1.0/publish_freq_);
        realtime_pub_->msg_.header.stamp = time;

        // iterate using given joint_states
        for (auto joint_state: joint_states){
          auto it = std::find(realtime_pub_->msg_.name.begin(),
                              realtime_pub_->msg_.name.end(), joint_state.name);
          if (it != realtime_pub_->msg_.name.end()){
            auto index = std::distance(realtime_pub_->msg_.name.begin(), it);
            realtime_pub_->msg_.position[index] = joint_state.position;
            realtime_pub_->msg_.velocity[index] = joint_state.velocity;
            realtime_pub_->msg_.effort[index] = joint_state.effort;
          }
          else{
            realtime_pub_->msg_.name.push_back(joint_state.name);
            realtime_pub_->msg_.position.push_back(joint_state.position);
            realtime_pub_->msg_.velocity.push_back(joint_state.velocity);
            realtime_pub_->msg_.effort.push_back(joint_state.effort);
          }
        }

        realtime_pub_->unlockAndPublish();
      }
    }
  }

 private:
  std::shared_ptr<realtime_tools::RealtimePublisher<sensor_msgs::JointState>> realtime_pub_;
  double publish_freq_;
  ros::Time last_publish_time_;
};
}//namespace flygripper_common
