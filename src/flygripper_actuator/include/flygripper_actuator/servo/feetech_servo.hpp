#pragma once

#include <memory>
#include <iostream>
#include <atomic>
#include <thread>
#include <chrono>
#include <unordered_map>

#include "flygripper_actuator/utils/circular_buffer.hpp"
#include "flygripper_actuator/servo/servo.hpp"
#include "flygripper_common/utils/time.hpp"

#include "SMS_STS.h"

namespace flygripper_actuator{
namespace servo{
template<typename T>
T boundded(const T &value, const T &abs_min) {
  if (value > abs_min) {
    return abs_min;
  } else if (value < -abs_min) {
    return -abs_min;
  } else {
    return value;
  }
}

class FeetechServo {
 public:
  using Ptr = std::shared_ptr<FeetechServo>;
  FeetechServo(const std::string interface = "/dev/FEETECH_SERVO",
               int baud_rate = 1000000,
               int thread_priority = 99) {
    feetech_interface_ = std::make_shared<SMS_STS>();
    if(!feetech_interface_->begin(baud_rate, interface.data())){
      throw ros::Exception("Failed to init Feetech STS Servo!");
      return;
    }
    std::cout << "Feetech STS Servo is init with dev: " << interface << ", baud rate: " << baud_rate << std::endl;
  }

  ~FeetechServo() {
    this->terminate();
  }

  void config(const uint32_t id,
              const std::string &product_type,
              const ActuatorParam &actuator_param) {
    actuator_id_types_[id] = product_type;
    actuator_id_params_[id] = actuator_param;
    actuator_id_states_[id] = ActuatorState{};
    actuator_id_cmds_[id] = ActuatorCommand{};
    feetech_interface_->CalibrationOfs(id);
  }

  void read() {
    for (auto &item : actuator_id_states_) {
      int id = item.first;
      ActuatorState& actuator_state = item.second; // writable
      ActuatorParam actuator_param = actuator_id_params_[id];
      // Update raw frame data
      ros::Time stamp = ros::Time::now();
      if(feetech_interface_->FeedBack(id) == -1){
        continue;
      }
      actuator_state.ecd = feetech_interface_->ReadPos(-1);
      actuator_state.ecd_rpm = feetech_interface_->ReadSpeed(-1);
      actuator_state.temperature = feetech_interface_->ReadTemper(-1);
      actuator_state.current = feetech_interface_->ReadCurrent(-1) * 0.0065;
      actuator_state.voltage = feetech_interface_->ReadVoltage(-1) * 0.1;
      // fake effort from control input
      actuator_state.effort = feetech_interface_->ReadLoad(-1) * actuator_param.act2effort;
      actuator_state.overload = feetech_interface_->ReadState(-1) & 0x20;

      if (actuator_state.seq != 0) {
        // Update round
        if (actuator_state.ecd - actuator_state.last_ecd > 2048)
          actuator_state.round--;
        else if (actuator_state.ecd - actuator_state.last_ecd < -2048)
          actuator_state.round++;
        // Update frequency
        // TODO: some message arrive at the same time, so the frequency is very large, it need to be fixed
        try {
          // Duration will be out of dual 32-bit range while motor failure
          // TODO: chrono vs ros::Time
          actuator_state.frequency = 1e9 / ((stamp - actuator_state.stamp).toNSec());
//              actuator_state.frequency = 1.0/((stamp - actuator_state.stamp).toSec());
//              actuator_state.frequency = 1e9/
//                  (stamp.time_since_epoch().count() - actuator_state.stamp.time_since_epoch().count());
        }
        catch (std::runtime_error &ex) {}
      }

      // Update stamp, seq and last_ecd
      actuator_state.stamp = stamp;
      actuator_state.seq++;
      actuator_state.last_ecd = actuator_state.ecd;

      // Update position, velocity based on params and raw frame data
      actuator_state.position = actuator_param.act2pos *
          static_cast<double>(actuator_state.ecd + 4096 * actuator_state.round)
          + actuator_param.offset_angle;
      actuator_state.velocity = actuator_param.act2vel * static_cast<double>(actuator_state.ecd_rpm);
    }
  }

  void write() {
    for(auto &actuator_id_cmd : actuator_id_cmds_){
      int id = actuator_id_cmd.first;
      ActuatorCommand command = actuator_id_cmd.second;
      ActuatorParam param = actuator_id_params_[id];
      double raw_position = (command.position - param.offset_angle) / param.act2pos;
      double raw_velocity = command.velocity / param.act2vel;
      double raw_acceleration = command.acceleration / param.act2vel * 2;
      feetech_interface_->WritePosEx(id, raw_position,
                                     raw_velocity,
                                     raw_acceleration);
    }
  }

  void enableTorque(bool enable, int id = -1){
    if (id == -1){
      for (auto &actuator_id_cmd : actuator_id_cmds_) {
        feetech_interface_->EnableTorque(actuator_id_cmd.first, enable);
      }
    } else{
      auto id_cmd = actuator_id_cmds_.find(static_cast<unsigned int>(id));
      if (id_cmd != actuator_id_cmds_.end()){
        feetech_interface_->EnableTorque(id_cmd->first, enable);
      } else {
        std::cerr << "Enable torque failed, given id " << id << " is not valid" << std::endl;
      }
    }
  }

  void terminate() {
    for (auto &actuator_id_cmd : actuator_id_cmds_) {
      feetech_interface_->EnableTorque(actuator_id_cmd.first, false);
    }

    std::cerr << "Feetech Servo shutdown" << std::endl;
    feetech_interface_->end();
  }

  ActuatorState *getState(const uint32_t id) {
    if (actuator_id_states_.find(id) != actuator_id_states_.end()) {
      return &actuator_id_states_[id];
    } else {
      std::cerr << "Can not find actuator state for id: " << id << std::endl;
      return nullptr;
    }
  }

  ActuatorCommand *getCommand(const uint32_t id) {
    if (actuator_id_cmds_.find(id) != actuator_id_cmds_.end()) {
      return &actuator_id_cmds_[id];
    } else {
      std::cerr << "Can not find actuator command for can_id: " << id << std::endl;
      return nullptr;
    }
  }

 private:

  std::shared_ptr<SMS_STS> feetech_interface_;

  std::unordered_map<uint32_t, ActuatorParam> actuator_id_params_;
  std::unordered_map<uint32_t, std::string> actuator_id_types_;
  std::unordered_map<uint32_t, ActuatorState> actuator_id_states_;
  std::unordered_map<uint32_t, ActuatorCommand> actuator_id_cmds_;
};
}
}

