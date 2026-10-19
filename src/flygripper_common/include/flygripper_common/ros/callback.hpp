#pragma once

#include <ros/ros.h>

#include <boost/bind.hpp>
#include <boost/shared_ptr.hpp>
#include <flygripper_common/ros/conversions.hpp>

namespace flygripper_common {

template <typename Msg, typename Data>
void rosCallback(boost::shared_ptr<Msg const> msg, Data& data) {
  fromRosMsg(msg, data);
}

#define SUBSCRIBE_MSG(nh, topic, msg_type, data, queue_size)                 \
  (nh.subscribe<msg_type>(                                                   \
      topic, queue_size,                                                     \
      boost::bind(&flygripper_common::rosCallback<msg_type, decltype(data)>, \
                  _1, std::ref(data)),                                       \
      ros::VoidConstPtr(), ros::TransportHints().tcpNoDelay()))

}  // namespace flygripper_common
