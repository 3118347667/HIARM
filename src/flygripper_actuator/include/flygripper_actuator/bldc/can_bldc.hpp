#pragma once

#include <memory>
#include <iostream>
#include <atomic>
#include <thread>
#include <chrono>
#include <unordered_map>

#include "flygripper_actuator/hardware/uart_can.hpp"
#include "flygripper_actuator/hardware/socket_can.hpp"
#include "flygripper_actuator/utils/circular_buffer.hpp"
#include "flygripper_actuator/bldc/bldc.hpp"

namespace flygripper_actuator{
namespace bldc{
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

struct StampedCanFrame {
  can_frame frame;
  TimeStamp stamp;
};

struct RMCommandCanFrame {
  RMCommandCanFrame() {
    for (int i = 0; i < 2; i++) {
      frame[i].can_dlc = 8;
    }
    this->clear();
  }
  void clear() {
    std::fill(std::begin(frame_ready), std::end(frame_ready), false);
    for (int i = 0; i < 2; i++) {
      std::fill(std::begin(frame[i].data), std::end(frame[i].data), 0);
    }
  }
  can_frame frame[2];
  bool frame_ready[2];
};

class CanActuator {
 public:
  using Ptr = std::shared_ptr<CanActuator>;
  CanActuator(const std::string bus = "uart_can",
          const std::string interface = "/dev/CAN_MOTOR",
          int thread_priority = 99) :
      stamped_frame_buffer_(kRecvFrameMaxSize) {

    if (bus == "uart_can") {
      can_ = std::make_shared<UartCan>(interface, 115200, std::bind(&CanActuator::frameCallback,
                                                                    this, std::placeholders::_1), thread_priority);
      while (!can_->open()) {
        std::this_thread::sleep_for(std::chrono::seconds(5));
      };
    } else if (bus == "socket_can") {
      can_ = std::make_shared<SocketCan>(interface, std::bind(&CanActuator::frameCallback,
                                                              this, std::placeholders::_1), thread_priority);
      while (!can_->open()) {
        std::this_thread::sleep_for(std::chrono::seconds(5));
      };
    } else {
      throw ros::Exception("CanActuator: bus type not supported");
    }

    std::cout << "Can Actuator is init" << std::endl;
  }
  ~CanActuator() {
    this->terminate();
  }

  void config(const uint32_t id,
              const std::string &product_type,
              const ActuatorParam &actuator_param) {
    actuator_id_types_[id] = product_type;
    actuator_id_params_[id] = actuator_param;
    actuator_id_states_[id] = ActuatorState{};
    actuator_id_cmds_[id] = ActuatorCommand{};
  }

  void read() {
    // for debug
    bool buffer_timeout = false;
    static constexpr double kTimeoutSec = 0.01;

    StampedCanFrame stamped_frame;
    int can_id;
    while (!stamped_frame_buffer_.isEmpty()) {
      // Pop the stamped can frame from buffer and check if the buffer is empty
      if (!stamped_frame_buffer_.pop(stamped_frame)) {
        std::cerr << "stamped frame buffer is empty" << std::endl;
        break;
      }
      can_frame &frame = stamped_frame.frame;
      TimeStamp &stamp = stamped_frame.stamp;
      can_id = frame.can_id;

      // Check if the can_id of received can_frame is valid
      if (actuator_id_states_.find(can_id) == actuator_id_states_.end() ||
          actuator_id_params_.find(can_id) == actuator_id_params_.end() ||
          actuator_id_types_.find(can_id) == actuator_id_types_.end()) {
        std::cerr << "can id " << can_id << " is not found" << std::endl;
        break;
      } else {
        auto &actuator_state = actuator_id_states_[can_id]; // writable
        auto actuator_param = actuator_id_params_[can_id]; // read_only
        auto actuator_type = actuator_id_types_[can_id]; // read_only
        // Update the actuator state based on the actuator_type and received can_frame
        if (actuator_type.substr(0, 2) == "rm") {
          // Check can_id
          if (0x201 > can_id || can_id > 0x204) {
            std::cerr << "Receive can_id for RoboMaster Motor is invalid, "
                         "it should be between 0x201 to 0x208 instead of "
                      << can_id << std::endl;
          }

          // Check timeout
          //TODO: chrono vs ros::Time
//          auto timeout =
//              (std::chrono::steady_clock::now().time_since_epoch().count() - stamp.time_since_epoch().count()) / 1e9;
          auto timeout = (ros::Time::now() - stamp).toSec();
          if (timeout > kTimeoutSec) {
            // print only once in a loop for debug!
            if (!buffer_timeout) {
              buffer_timeout = true;
              std::cerr << "Read the frame too slow with duration of " << timeout << " secs" << std::endl;
            }
            continue;
          }

          // Update raw frame data
          actuator_state.ecd = (frame.data[0] << 8u) | frame.data[1];
          actuator_state.ecd_rpm = (frame.data[2] << 8u) | frame.data[3];
          actuator_state.current = (frame.data[4] << 8u) | frame.data[5];
          actuator_state.temperature = frame.data[6];

          if (actuator_state.seq != 0) {
            // Update round
            if (actuator_state.ecd - actuator_state.last_ecd > 4096)
              actuator_state.round--;
            else if (actuator_state.ecd - actuator_state.last_ecd < -4096)
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

          // Update position, velocity and effort based on params and raw frame data
          actuator_state.position = actuator_param.act2pos *
              static_cast<double>(actuator_state.ecd + 8191 * actuator_state.round)
              + actuator_param.offset_angle;
          actuator_state.velocity = actuator_param.act2vel * static_cast<double>(actuator_state.ecd_rpm);
          actuator_state.effort = actuator_param.act2effort * static_cast<double>(actuator_state.current);

          // Low pass filter for velocity ( to reduce noise for velocity feedback control )
          if (actuator_state.lpf == nullptr) {
            actuator_state.lpf = std::make_shared<flygripper_common::ButterworthLPF1D>(actuator_param.lpf_cutoff_freq,
                                                                                 actuator_param.lpf_sampling_freq,
                                                                                 actuator_state.velocity);
          } else {
            actuator_state.velocity = actuator_state.lpf->apply(actuator_state.velocity);
          }
        }
      }
    }
  }

  void write() {

    for (auto &item : actuator_id_cmds_) {
      uint32_t can_id = item.first;
      auto actuator_param = actuator_id_params_[can_id];
      auto actuator_type = actuator_id_types_[can_id];
      auto &actuator_cmd = actuator_id_cmds_[can_id];
      if (actuator_type.substr(0, 2) == "rm") {

        double cmd = boundded(actuator_param.effort2act * actuator_cmd.effort,
                              actuator_param.max_out);  // add max_range to act_data

        if (0x201 <= can_id <= 0x204) {
          rm_cmd_can_frame_.frame[0].can_id = 0x200;
          rm_cmd_can_frame_.frame[0].data[2 * static_cast<int>(can_id - 0x201)] =
              static_cast<uint8_t>(static_cast<int16_t>(cmd) >> 8u);
          rm_cmd_can_frame_.frame[0].data[2 * static_cast<int>(can_id - 0x201) + 1] = static_cast<uint8_t>(cmd);
          rm_cmd_can_frame_.frame_ready[0] = true;
        } else if (0x205 <= can_id <= 0x208) {
          rm_cmd_can_frame_.frame[1].can_id = 0x1FF;
          rm_cmd_can_frame_.frame[1].data[2 * static_cast<int>(can_id - 0x205)] =
              static_cast<uint8_t>(static_cast<int16_t>(cmd) >> 8u);
          rm_cmd_can_frame_.frame[1].data[2 * static_cast<int>(can_id - 0x205) + 1] = static_cast<uint8_t>(cmd);
          rm_cmd_can_frame_.frame_ready[1] = true;
        } else {
          std::cerr << "Command can_id for RoboMaster Motor is invalid, "
                       "it should be between 0x201 to 0x208 instead of "
                    << can_id << std::endl;
        }
      }
      // After update command can frame, send it
      // For RoboMaster Motor
      for (int i = 0; i < 2; ++i) {
        if (rm_cmd_can_frame_.frame_ready[i]) {
          can_->write(rm_cmd_can_frame_.frame[i]);
        }
      }
      // Clear the frame
      rm_cmd_can_frame_.clear();
    }
  }

  void terminate() {
    for (auto &actuator_id_cmd : actuator_id_cmds_) {
      actuator_id_cmd.second.effort = 0;
    }
    this->write();
  }

  ActuatorState *getState(const uint32_t can_id) {
    if (actuator_id_states_.find(can_id) != actuator_id_states_.end()) {
      return &actuator_id_states_[can_id];
    } else {
      std::cerr << "Can not find actuator state for can_id: " << can_id << std::endl;
      return nullptr;
    }
  }

  ActuatorCommand *getCommand(const uint32_t can_id) {
    if (actuator_id_cmds_.find(can_id) != actuator_id_cmds_.end()) {
      return &actuator_id_cmds_[can_id];
    } else {
      std::cerr << "Can not find actuator command for can_id: " << can_id << std::endl;
      return nullptr;
    }
  }

 private:

  void frameCallback(const can_frame &frame) {
    stamped_frame_buffer_.push(StampedCanFrame{
        .frame=frame,
        .stamp=ros::Time::now()});
//        .stamp=std::chrono::steady_clock::now()});
  }

  CanBase::Ptr can_;
  static constexpr int kRecvFrameMaxSize = 1000;
  ThreadedCircularBuffer<StampedCanFrame> stamped_frame_buffer_;
  RMCommandCanFrame rm_cmd_can_frame_;

  std::unordered_map<uint32_t, ActuatorParam> actuator_id_params_;
  std::unordered_map<uint32_t, std::string> actuator_id_types_;
  std::unordered_map<uint32_t, ActuatorState> actuator_id_states_;
  std::unordered_map<uint32_t, ActuatorCommand> actuator_id_cmds_;
};
}
}

