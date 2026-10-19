#pragma once
#include "realtime_tools/realtime_buffer.h"
#include "dynamic_reconfigure/server.h"

namespace flygripper_common {
template<typename P, typename C>
class DynamicParameterConfig {
 public:
  DynamicParameterConfig():
      dynamic_reconfig_initialized_(false) {}

  virtual ~DynamicParameterConfig() = default;

  virtual bool initParams(ros::NodeHandle &pnh, P &params) = 0;

  virtual void dynamicParams2Config(const P &params, C &config) = 0;

  virtual void config2DynamicParams(const C &config, P &params) = 0;

  virtual bool init(ros::NodeHandle &pnh) {
    P params;
    if (!initParams(pnh, params)){
      ROS_ERROR_STREAM("Initializing parameters in namespace " << pnh.getNamespace());
      return false;
    }
    setDynamicParams(params);
    initDynamicReconfig(pnh);
    return true;
  }

  virtual void getParams(P &params) {
    params = *params_buffer_.readFromRT();
  }

 private:

  void setDynamicParams(const P &params) {
    params_buffer_.writeFromNonRT(params);
    updateDynamicReconfig(params);
  }

  void initDynamicReconfig(ros::NodeHandle &nh) {
    ROS_INFO_STREAM("Initializing dynamic reconfigure in namespace " << nh.getNamespace());

    // Start dynamic reconfigure server
    param_reconfig_server_.reset(new DynamicReconfigServer(param_reconfig_mutex_, nh));
    dynamic_reconfig_initialized_ = true;

    // Set config to params
    updateDynamicReconfig();

    // Set callback
    param_reconfig_callback_ = std::bind(&DynamicParameterConfig::dynamicReconfigCallback, this,
                                         std::placeholders::_1, std::placeholders::_2);
    param_reconfig_server_->setCallback(param_reconfig_callback_);
  }

  void dynamicReconfigCallback(C &config, uint32_t level) {
    ROS_DEBUG_STREAM("Reconfigure Request: \n" );
    P params = *params_buffer_.readFromNonRT();
    config2DynamicParams(config, params);
    setDynamicParams(params);
  }

  void updateDynamicReconfig() {
    // Make sure dynamic reconfigure is initialized
    if (!dynamic_reconfig_initialized_)
      return;

    // Get config from current params
    P params = *params_buffer_.readFromNonRT();
    C config;
    dynamicParams2Config(params, config);

    // Update dynamic reconfigure
    updateDynamicReconfig(config);
  }

  void updateDynamicReconfig(const P &params) {
    // Make sure dynamic reconfigure is initialized
    if (!dynamic_reconfig_initialized_)
      return;

    // Get config from given params
    C config;
    dynamicParams2Config(params, config);

    // Update dynamic reconfigure
    updateDynamicReconfig(config);
  }

  void updateDynamicReconfig(const C &config) {
    // Make sure dynamic reconfigure is initialized
    if (!dynamic_reconfig_initialized_)
      return;

    // Update dynamic reconfigure
    param_reconfig_mutex_.lock();
    param_reconfig_server_->updateConfig(config);
    param_reconfig_mutex_.unlock();
  }

  // Store the params in a realtime buffer to allow dynamic reconfigure to update it without
  // blocking the realtime update loop
  realtime_tools::RealtimeBuffer<P> params_buffer_;

  // Dynamic Reconfigure
  bool dynamic_reconfig_initialized_;
  using DynamicReconfigServer = dynamic_reconfigure::Server<C>;
  std::shared_ptr<DynamicReconfigServer> param_reconfig_server_;
  typename DynamicReconfigServer::CallbackType param_reconfig_callback_;
  boost::recursive_mutex param_reconfig_mutex_;
};
}
