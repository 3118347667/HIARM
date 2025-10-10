#pragma once
#include <functional>
#include <iostream>
#include <linux/can.h>
#include <net/if.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <pthread.h>

#include "flygripper_actuator/hardware/can_base.hpp"
#include "flygripper_common/utils/realtime.hpp"

namespace flygripper_actuator
{
static void* socketCanReceiverThread(void *socketcan_ptr);

class SocketCan: public CanBase
{
 public:
  using Ptr = std::shared_ptr<SocketCan>;
  using CallbackType = typename std::function<void(const can_frame &)>;

  SocketCan(const std::string& interface,
            CallbackType&& callback,
            int thread_priority) {
    callback_ = std::move(callback);
    interface_ = interface;
    thread_priority_ = thread_priority;
  }
  ~SocketCan() {
    if (isOpen()) {
      close();
    }
  }

  virtual bool open() override {
    // Request a socket
    sock_fd_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (sock_fd_ == -1)
    {
      std::cerr << "Error: Unable to create a CAN socket" << std::endl;
      return false;
    }
    char name[16] = {};  // avoid stringop-truncation
    strncpy(name, interface_.c_str(), interface_.size());
    strncpy(interface_request_.ifr_name, name, IFNAMSIZ);
    // Get the index of the network interface
    if (ioctl(sock_fd_, SIOCGIFINDEX, &interface_request_) == -1)
    {
      std::cerr << "Unable to select CAN interface "
                << name <<" : I/O control error" << std::endl;
      // Invalidate unusable socket
      close();
      return false;
    }
    // Bind the socket to the network interface
    address_.can_family = AF_CAN;
    address_.can_ifindex = interface_request_.ifr_ifindex;
    int ret = bind(sock_fd_, reinterpret_cast<struct sockaddr*>(&address_), sizeof(address_));
    if (ret == -1)
    {
      std::cerr << "Failed to bind socket to "
                << name <<" network interface" << std::endl;
      close();
      return false;
    }
    // Start a separate, event-driven thread for frame reception
    return startReceiverThread();
  }

  virtual void close() override {
    terminate_receiver_thread_ = true;
    while (receiver_thread_running_);

    if (!isOpen())
      return;
    // Close the file descriptor for our socket
    ::close(sock_fd_);
    sock_fd_ = -1;
  }

  virtual void write(const can_frame& frame) override {
    if (!isOpen())
    {
      std::cerr << "Unable to write: Socket " << interface_request_.ifr_name
                << " not open" << std::endl;
      return;
    }
    if (::write(sock_fd_, &frame, sizeof(can_frame)) == -1)
      std::cerr << "Unable to write: The " << interface_request_.ifr_name
                << " tx buffer may be full" << std::endl;
  }

// set to public in order for static receiver thread fucntion
  std::atomic<bool> terminate_receiver_thread_{false};
  std::atomic<bool> receiver_thread_running_{false};
  int sock_fd_ = -1;
  CallbackType callback_;

 private:

  bool isOpen() const {
    return sock_fd_ != -1;
  }

  bool startReceiverThread() {
    terminate_receiver_thread_ = false;
    int ret = pthread_create(&receiver_thread_id_, nullptr, &socketCanReceiverThread, this);
    if (ret != 0) {
      std::cerr << "Unable to start receiver thread" << std::endl;
      return false;
    }
    std::cout << "Successfully started receiver thread with ID" << receiver_thread_id_ << std::endl;
    // Set thread priority
    return setThreadPriority(receiver_thread_id_, thread_priority_);
  }


 private:
  std::string interface_;
  int thread_priority_;

  ifreq interface_request_{};
  sockaddr_can address_{};
  pthread_t receiver_thread_id_{};

};

static void* socketCanReceiverThread(void *socketcan_ptr) {
  /*
   * The first and only argument to this function
   * is the pointer to the object, which started the thread.
   */
  auto* socket_can = (SocketCan*)socketcan_ptr;
  // Holds the set of descriptors, that 'select' shall monitor
  fd_set descriptors;
  // Highest file descriptor in set
  int maxfd = socket_can->sock_fd_;
  // How long 'select' shall wait before returning with timeout
  struct timeval timeout{ };
  // Buffer to store incoming frame
  can_frame rx_frame{};
  // Run until termination signal received
  socket_can->receiver_thread_running_ = true;
  while (!socket_can->terminate_receiver_thread_){
    timeout.tv_sec = 1.;  // Should be set each loop
    // Clear descriptor set
    FD_ZERO(&descriptors);
    // Add socket descriptor
    FD_SET(socket_can->sock_fd_, &descriptors);
    // Wait until timeout or activity on any descriptor
    if (select(maxfd + 1, &descriptors, nullptr, nullptr, &timeout))
    {
      size_t len = read(socket_can->sock_fd_, &rx_frame, CAN_MTU);
      if (len < 0)
        continue;
      if (socket_can->callback_)
        socket_can->callback_(rx_frame);
    }
  }
  socket_can->receiver_thread_running_ = false;
  return nullptr;
}

}