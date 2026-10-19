#pragma once
#include <memory>

#include "flygripper_common/type/base.hpp"
#include "flygripper_common/type/rc.hpp"

namespace flygripper_flight{
using namespace flygripper_common;

enum RcInputChannel: unsigned int{
  AXIS_ROLL=0,
  AXIS_PITCH=1,
  AXIS_THROTTLE=2,
  AXIS_YAW=3,
  TOGGLE_GUIDED=4,
  TOGGLE_TRAJ=5,
  AXIS_MORPH=8,
  AXIS_ONE_MORPH=9,
  CHANNEL_SIZE=10
};

struct ToggleSwitch{
  ToggleSwitch(double threshold):
      threshold_(threshold)
  {
    reset();
  }

  ToggleSwitch(): ToggleSwitch(0.75){}

  void update(const double& raw){
    if (!init){
      last_toggle_ = raw > threshold_;
      init = true;
    }
    toggle = raw > threshold_;

    // if tiggered, then stay triggered until it is reset by resetTrigger()
    if (toggle){
      trigger_on = trigger_on || !last_toggle_;
      trigger_off = false;
    } else{
      trigger_on = false;
      trigger_off = trigger_off || last_toggle_;
    }

    last_toggle_ = toggle;
  }

  void reset(){
    init = false;
    resetTrigger();
  }

  void resetTrigger(){
    trigger_on = false;
    trigger_off = false;
  }

  bool toggle;
  bool init;
  bool trigger_on;
  bool trigger_off;
 private:
  double threshold_;
  bool last_toggle_;
};

class RcInput: public Rc<CHANNEL_SIZE>{

 private:
  // user settings
  ToggleSwitch guided_toggle_;
  ToggleSwitch traj_toggle;

  static constexpr double kGuidedModeThreshold = 0.75; // third level
  static constexpr double kCTrajModeThreshold = 0.75; // third level

 public:
  using Ptr = std::shared_ptr<RcInput>;
  RcInput(): Rc(),
             guided_toggle_(ToggleSwitch(kGuidedModeThreshold)),
             traj_toggle(ToggleSwitch(kCTrajModeThreshold))
  {}

  void update(){
    // Check if valid
    if (!valid()){
      return;
    }
    // Update toggle mode
    // guided and traj mode
    guided_toggle_.update(channels[TOGGLE_GUIDED]);
    if (guided_toggle_.toggle){
      traj_toggle.update(channels[TOGGLE_TRAJ]);
    } else {
      traj_toggle.reset();
    }
  }

  bool isGuidedMode() const{
    return guided_toggle_.toggle;
  }

  bool isGuidedModeTriggered() const{
    return guided_toggle_.trigger_on;
  }

  void resetGuidedModeTrigger(){
    return guided_toggle_.resetTrigger();
  }

  bool isTrajMode() const{
    return traj_toggle.toggle;
  }

  bool isTrajModeTriggered() const{
    return traj_toggle.trigger_on;
  }

  void resetTrajModeTrigger(){
    return traj_toggle.resetTrigger();
  }

  friend std::ostream& operator<<(std::ostream& os, const RcInput& p) {
    os.precision(2);
    os << std::scientific;
    os << "Throttle: " << p.channels[AXIS_THROTTLE] << std::endl;
    os << "Roll: " << p.channels[AXIS_ROLL] << std::endl;
    os << "Pitch: " << p.channels[AXIS_PITCH] << std::endl;
    os << "Yaw: " << p.channels[AXIS_YAW] << std::endl;
//    os << "guided_toggle: " << p.channels[TOGGLE_GUIDED] << std::endl;
//    os << "traj_toggle: " << p.channels[TOGGLE_TRAJ] << std::endl;
    os << "Guided Mode: " << p.isGuidedMode() << ", Trigger: " << p.isGuidedModeTriggered()<< std::endl;
    os << "Traj Mode: " << p.isTrajMode() << ", Trigger: " << p.isTrajModeTriggered()<< std::endl;
    os.precision();
    os.unsetf(std::ios::scientific);
    return os;
  }
};

} //namespace flygripper_flight