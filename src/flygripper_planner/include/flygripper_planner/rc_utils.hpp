#pragma once
#include <memory>

#include "flygripper_common/type/base.hpp"
#include "flygripper_common/type/rc.hpp"

namespace flygripper_planner{
using namespace flygripper_common;

enum RcInputChannel: unsigned int{
  AXIS_ROLL=0,
  AXIS_PITCH=1,
  AXIS_THROTTLE=2,
  AXIS_YAW=3,
  TOGGLE_GUIDED=4,
  TOGGLE_TRAJ=5,
  TOGGLE_MORPH=7,
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
  ToggleSwitch morph_toggle_;

  static constexpr double kMorphThreshold = 0.75; // third level

 public:
  using Ptr = std::shared_ptr<RcInput>;
  RcInput(): Rc(),
              morph_toggle_(ToggleSwitch(kMorphThreshold))
  {}

  void update(){
    // Check if valid
    if (!valid()){
      return;
    }
    // Update toggle mode
    // guided and traj mode
    morph_toggle_.update(channels[TOGGLE_MORPH]);
  }

  bool isMorphTriggered() const{
    return morph_toggle_.trigger_on;
  }

  void resetMorphTrigger(){
    return morph_toggle_.resetTrigger();
  }
};

} //namespace flygripper_flight