#pragma once
#include <memory>
#include <iostream>
#include <iomanip>
#include <stdio.h>
#include <functional>
#include <linux/can.h>
#include <pthread.h>

#include "flygripper_actuator/hardware/can_base.hpp"
#include "flygripper_actuator/hardware/serial.hpp"
#include "flygripper_common/utils/realtime.hpp"

namespace flygripper_actuator {

#pragma pack(push, 1)
/* -------------------------------------   Recv   -------------------------------------------*/

typedef struct {
  uint8_t packet_header;          // 帧头标志位                               0xAA
  uint8_t cmd_id;                 // 命令                                    0x11
  // 0x00 心跳
  // 0x01 接收失败, 0x11 接收成功,
  // 0x02 发送失败, 0x12 发送成功,
  // 0x03 波特率设置失败, 0x13 波特率设置成功
  // 0xEE 通讯错误信息
  uint8_t can_data_len: 6;         // 格式：CAN 数据长度                        0x08
  uint8_t can_ide: 1;              // 格式：CAN ID类型 ide:0 标准帧; 1 扩展帧     0x00
  uint8_t can_rtr: 1;              // 格式：CAN 帧类型 rtr:0 数据帧; 1 远程帧     0x00
  uint32_t can_id;                 // CAN ID                                 {}
  uint8_t can_data[8];            // CAN 数据                                {}
  uint8_t packet_end;             // 帧尾标志位 0x55                          0x55
} RecvPacket;                         // CAN 接收功能相关结构体                     长度 16

static constexpr uint8_t kRecvSOF = 0xAA;
static constexpr uint8_t kRecvEOF = 0x55;
static constexpr size_t kRecvPacketSize = sizeof(RecvPacket);

/* -------------------------------------   Send   -------------------------------------------*/
typedef struct {
  uint16_t packet_header;          // 帧头标志位                              0x55  0xAA
  uint32_t baudrate;               // 波特率                                 {}
  uint8_t data_bits;              // 数据位                                 0x00 （8Bit）
  uint8_t stop_bits;              // 停止位                                 0x00  (1StopBit)
  uint8_t parity_bits;            // 校验                                   0x00  (None)
  uint8_t packet_end;             // 帧尾标志位 0x55                         0xAA 0x55
} UARTConfigPacket;

typedef struct {
  uint16_t packet_header;          // 帧头标志位                          0x55 0xAA 默认小端 0xAA55
  uint8_t data_len;               // 数据长度                            0x1E
  uint8_t cmd_id;                 // 命令                               0x01
  // 0x01  转发CAN数据帧
  // 0x02 PC 与设备握手，设备反馈OK
  // 0x03 非反馈CAN转发，不反馈发送状态
  uint32_t send_index;             // 发送次数                            0x01 0x00 0x00 0x00
  uint32_t send_interval;          // 发送间隔 单位100us                   0x0A 0x00 0x00 0x00
  uint8_t can_ide;                // CAN ID类型 ide:0 标准帧; 1 扩展帧     0x00
  uint32_t can_id;                 // CAN ID                             {}
  uint8_t can_rtr;                // CAN 帧类型 rtr:0 数据帧; 1 远程帧      0x00
  uint8_t can_data_len;           // CAN 数据长度 = 8                     0x08
  uint8_t can_id_acc;             // CAN ID累加操作标志位                  0x00
  uint8_t can_data_acc;           // CAN 数据累加操作标志位                 0x00
  uint8_t can_data[8];            // CAN 数据                                {}
  uint8_t packet_crc;             // 帧crc                               0x88 (随便给)
} SendPacket;                          // CAN 发送功能相关结构体                 长度 30
static constexpr size_t kSendPacketSize = sizeof(SendPacket);

uint8_t kHeartbeatPacket[4] = {0x55, 0x04, 0xAA, 0x55}; // 500ms send heartbeat
uint8_t kConfigCanPacket[5] = {0x55, 0x05, 0x00, 0xAA, 0x55}; // [2] means  index of 1M baudrate
uint8_t kStopSendPacket[4] = {0x55, 0x03, 0xAA, 0x55};

/**
 * @brief class for UART2CAN
 */
class UartCanAdaptor {
 public:
  using Ptr = std::shared_ptr<UartCanAdaptor>;
  using CallbackType = typename std::function<void(const can_frame &)>;
  explicit UartCanAdaptor(CallbackType callback) :
      in_packet_(false), packet_index_(0), callback_(std::move(callback)) {
    recv_packet_ = std::make_shared<RecvPacket>();
    send_packet_ = std::make_shared<SendPacket>();

    send_packet_->packet_header = 0xAA << 8 | 0x55;
    send_packet_->data_len = 30; // will be rewrite while packing
    send_packet_->cmd_id = 0x03; // 0x01 with ack and 0x03 without ack
    send_packet_->send_index = 0x01;
    send_packet_->send_interval = 0x0A;
    send_packet_->can_ide = 0x00;
    send_packet_->can_id = 0x200; // will be rewrite while packing
    send_packet_->can_rtr = 0x00;
    send_packet_->can_data_len = 8; // will be rewrite while packing
    send_packet_->can_id_acc = 0x00;
    send_packet_->can_data_acc = 0x00;
    memset(send_packet_->can_data, 0, 8); // will be rewrite while packing
    send_packet_->packet_crc = 0x88; // random value as receiver never check crc

    memset(recv_packet_bytes_, 0, kRecvPacketSize);
    memset(send_packet_bytes_, 0, kSendPacketSize);

  }
  virtual ~UartCanAdaptor() = default;
  void feed(const uint8_t *bytes, const size_t len) {
    for (size_t i = 0; i < len; i++) {
      if (in_packet_) {
        recv_packet_bytes_[packet_index_] = bytes[i];
        packet_index_++;
        if (packet_index_ == kRecvPacketSize && bytes[i] == kRecvEOF) {
          if (bytes[i] == kRecvEOF) {
            packetHandle();
            in_packet_ = false;
//            return true;
          } else {
            std::cerr << "recv eof error" << std::endl;
            in_packet_ = false;
          }
        }
      } else {
        if (bytes[i] == kRecvSOF) {
          in_packet_ = true;
          recv_packet_bytes_[0] = bytes[i];
          packet_index_ = 1;
        }
//        else{
//          std::cout<< "drop and recv " << std::hex << int(bytes[i]) << " before sof" << std::endl;
//        }
      }
    }
    return;

  }

  int pack(const can_frame &can_frame, uint8_t *send_packet_bytes) {
    send_packet_->data_len = 22 + can_frame.can_dlc; //TODO
    send_packet_->can_id = can_frame.can_id; //TODO
    send_packet_->can_data_len = can_frame.can_dlc; //TODO
    memcpy(send_packet_->can_data, can_frame.data, 8); //TODO:
    memcpy(send_packet_bytes, send_packet_.get(), kSendPacketSize);
    return kSendPacketSize;
  }
 private:

  //! recv packet
  bool in_packet_;
  uint8_t recv_packet_bytes_[kRecvPacketSize];
  uint8_t packet_index_;
  std::shared_ptr<RecvPacket> recv_packet_;
  CallbackType callback_;

  //! send packet
  uint8_t send_packet_bytes_[kSendPacketSize];
  std::shared_ptr<SendPacket> send_packet_;

  inline void packetHandle() {
    memcpy(recv_packet_.get(), recv_packet_bytes_, kRecvPacketSize);
    // TODO: only valid if cmd_id is 0x11 to avoid ack
    if (recv_packet_->cmd_id != 0x11) {
      return;
    }

    can_frame frame;
    frame.can_id = recv_packet_->can_id;
    frame.can_dlc = recv_packet_->can_data_len;
    memcpy(frame.data, recv_packet_->can_data, 8);

    if (callback_) {
      callback_(frame);
    }
  }
};

static void *uartCanReceiverThread(void *uartcan_ptr);

class UartCan: public CanBase{
 public:
  using Ptr = std::shared_ptr<UartCan>;
  UartCan(const std::string &port_name,
          const uint32_t baudrate,
          UartCanAdaptor::CallbackType &&callback,
          int thread_priority) {
    port_name_ = port_name;
    baudrate_ = baudrate;
    thread_priority_ = thread_priority;
    uart_can_adaptor_ = std::make_shared<UartCanAdaptor>(std::move(callback));
  }
  virtual ~UartCan() = default;

  bool open() override {
    device_ = std::make_shared<Serial>(port_name_, baudrate_);
    if (device_->Init()) {
      std::cout << "Serial device init success" << std::endl;
    } else {
      return false;
    }
    return startReceiverThread();
  }

  void close() override {
    terminate_receiver_thread_ = true;
    while (receiver_thread_running_);

    if (device_) {
      device_ = nullptr;
    }
    if (uart_can_adaptor_) {
      uart_can_adaptor_ = nullptr;
    }
  }

  void write(const can_frame &can_frame) override {
    uint8_t send_packet_bytes[kSendPacketSize];
    memset(send_packet_bytes, 0, kSendPacketSize);//TODO
    int len = uart_can_adaptor_->pack(can_frame, send_packet_bytes);
    device_->Write(send_packet_bytes, kSendPacketSize);
  }

  // set to public in order for static receiver thread fucntion
  std::atomic<bool> terminate_receiver_thread_{false};
  std::atomic<bool> receiver_thread_running_{false};
  Serial::Ptr device_{nullptr};
  UartCanAdaptor::Ptr uart_can_adaptor_{nullptr};
  
 private:
  bool startReceiverThread() {
    terminate_receiver_thread_ = false;
    int ret = pthread_create(&receiver_thread_id_, nullptr, &uartCanReceiverThread, this);
    if (ret != 0) {
      std::cerr << "Unable to start receiver thread" << std::endl;
      return false;
    }
    std::cout << "Successfully started receiver thread with ID" << receiver_thread_id_ << std::endl;
    // Set thread priority
    return setThreadPriority(receiver_thread_id_, thread_priority_);
  }
  int thread_priority_;
  std::string port_name_;
  uint32_t baudrate_;
  pthread_t receiver_thread_id_{};

};

static void* uartCanReceiverThread(void *uartcan_ptr) {
  /*
 * The first and only argument to this function
 * is the pointer to the object, which started the thread.
 */
  auto *uart_can = (UartCan *) uartcan_ptr;
  static constexpr size_t kRecvUartBuffMaxSize = 1000;
  uint8_t recv_buff[kRecvUartBuffMaxSize];
  int recv_buff_len;
  uart_can->receiver_thread_running_ = true;
  while (!uart_can->terminate_receiver_thread_) {
    recv_buff_len = uart_can->device_->Read(recv_buff, kRecvUartBuffMaxSize);
    if (recv_buff_len > 0) {
      uart_can->uart_can_adaptor_->feed(recv_buff, recv_buff_len);
    }
  }
  uart_can->receiver_thread_running_ = false;
  return NULL;
}
}