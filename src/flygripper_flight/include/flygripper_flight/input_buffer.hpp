#pragma once

#include <algorithm>
#include <array>
#include <atomic>
#include <memory>
#include <mutex>
#include <optional>
#include <ros/callback_queue.h>
#include <ros/ros.h>
#include "flygripper_flight/runtime_io.hpp"

namespace flygripper_flight {

// These are separate queues, not multiple threads competing for one queue.
// Keep each subscription serialized so ROS receipt order (including RC edges)
// is preserved. No callback in the ordinary queue can occupy a critical worker.
class InputCallbackQueues {
 public:
  enum class Channel { Imu, Odom, Safety, Ordinary, Count };
  ros::CallbackQueue* get(Channel channel) {
    return &queues_[static_cast<size_t>(channel)];
  }
  void start() {
    for (size_t i = 0; i < queues_.size(); ++i) {
      spinners_[i] = std::make_unique<ros::AsyncSpinner>(1, &queues_[i]);
      spinners_[i]->start();
    }
  }
  void stop() {
    for (auto& spinner : spinners_) if (spinner) spinner->stop();
  }
  ~InputCallbackQueues() { stop(); }
 private:
  std::array<ros::CallbackQueue, static_cast<size_t>(Channel::Count)> queues_;
  std::array<std::unique_ptr<ros::AsyncSpinner>, static_cast<size_t>(Channel::Count)> spinners_;
};

struct InputCallbackStamp {
  uint64_t ticket{0};
  double receipt_ros{-1.0}, callback_ros{-1.0}, callback_steady{-1.0};
};

template<class Message> struct InputSample {
  typename Message::ConstPtr message;
  InputCallbackStamp timing;
  // received_ros is ROS's transport receipt time; received_steady is callback
  // entry. Neither is the sensor's measurement timestamp (header.stamp).
  double received_ros{-1.0}, received_steady{-1.0};
  double lock_acquired_steady{-1.0}, saved_steady{-1.0};
  double queue_delay{0.0}, lock_wait{0.0}, store_duration{0.0};
  double max_queue_delay{0.0}, max_lock_wait{0.0}, max_store_duration{0.0};
  double max_gap{0.0};
  uint64_t count{0}, callbacks{0}, rejected_updates{0}, sequence_gaps{0};
  uint32_t last_sequence{0};
};

// One short lock per topic. Message parsing and destruction happen outside it.
// EventCapacity > 0 retains bounded RC history, including short release pulses.
template<class Message, size_t EventCapacity = 0, class Mutex = std::mutex>
class InputBuffer {
 public:
  struct Event {
    typename Message::ConstPtr message;
    double received_ros{-1.0};
  };
  struct Batch {
    InputSample<Message> latest;
    std::array<Event, EventCapacity> events;
    size_t size{0};
    bool overflow{false};
  };

  InputCallbackStamp beginCallback(const ros::Time& receipt) {
    InputCallbackStamp stamp;
    // A monotonic ticket protects against an older callback finishing last.
    // Do not order by header.stamp: sensor clock corrections, frozen stamps and
    // publisher restarts must remain visible to the flight freshness checks.
    stamp.ticket = next_ticket_.fetch_add(1, std::memory_order_relaxed) + 1;
    stamp.callback_steady = steadySeconds();
    stamp.callback_ros = ros::Time::now().toSec();
    stamp.receipt_ros = receipt.toSec();
    return stamp;
  }

  void receive(const ros::MessageEvent<const Message>& event) {
    const auto stamp = beginCallback(event.getReceiptTime());
    store(event.getConstMessage(), stamp);
  }

  bool store(typename Message::ConstPtr message, const InputCallbackStamp& stamp) {
    // Keep overwritten references alive until after unlocking, even if their
    // destruction releases a large trajectory/vector or invokes a deleter.
    typename Message::ConstPtr retired, retired_event;
    const double before_lock = steadySeconds();
    std::unique_lock<Mutex> lock(mutex_);
    const double acquired = steadySeconds();
    ++latest_.callbacks;
    latest_.max_lock_wait = std::max(latest_.max_lock_wait, acquired - before_lock);
    if (stamp.ticket <= latest_.timing.ticket) {
      ++latest_.rejected_updates;
      return false;  // Never refresh freshness on an out-of-order completion.
    }
    if (latest_.count) {
      latest_.max_gap = std::max(latest_.max_gap,
                                stamp.callback_steady - latest_.received_steady);
      const uint32_t delta = message->header.seq - latest_.last_sequence;
      if (delta > 1 && delta < 0x80000000u) latest_.sequence_gaps += delta - 1;
    }
    latest_.last_sequence = message->header.seq;
    retired.swap(latest_.message);
    latest_.message = message;
    latest_.timing = stamp;
    latest_.received_ros = stamp.receipt_ros;
    latest_.received_steady = stamp.callback_steady;
    latest_.lock_acquired_steady = acquired;
    latest_.queue_delay = stamp.callback_ros - stamp.receipt_ros;
    latest_.lock_wait = acquired - before_lock;
    latest_.max_queue_delay = std::max(latest_.max_queue_delay, latest_.queue_delay);
    ++latest_.count;
    if constexpr (EventCapacity > 0) {
      if (size_ == EventCapacity) {
        retired_event.swap(events_[head_].message);
        head_ = (head_ + 1) % EventCapacity;
        --size_;
        overflow_ = true;
      }
      auto& event = events_[(head_ + size_) % EventCapacity];
      event.message = message;
      event.received_ros = stamp.receipt_ros;
      ++size_;
    }
    latest_.saved_steady = steadySeconds();
    latest_.store_duration = latest_.saved_steady - acquired;
    latest_.max_store_duration = std::max(latest_.max_store_duration, latest_.store_duration);
    return true;
  }

  InputSample<Message> snapshot() const {
    std::lock_guard<Mutex> lock(mutex_);
    return latest_;  // Only a shared pointer and scalar metadata are copied.
  }

  // Control never waits for a descheduled input worker holding a short lock.
  // On contention it keeps its previous snapshot, including its OLD timestamps;
  // freshness checks still expire it normally. No busy wait and no lock nesting.
  std::optional<InputSample<Message>> trySnapshot() const {
    std::unique_lock<Mutex> lock(mutex_, std::try_to_lock);
    if (!lock.owns_lock()) return std::nullopt;
    return latest_;
  }

  std::optional<Batch> tryDrain() {
    std::unique_lock<Mutex> lock(mutex_, std::try_to_lock);
    if (!lock.owns_lock()) return std::nullopt;
    return drainLocked();
  }

  Batch drain() {
    std::lock_guard<Mutex> lock(mutex_);
    return drainLocked();
  }

 private:
  Batch drainLocked() {
    Batch batch;
    batch.latest = latest_;
    batch.size = size_;
    batch.overflow = overflow_;
    if constexpr (EventCapacity > 0) {
      for (size_t i = 0; i < size_; ++i)
        std::swap(batch.events[i], events_[(head_ + i) % EventCapacity]);
    }
    head_ = size_ = 0;
    overflow_ = false;
    return batch;
  }

  mutable Mutex mutex_;
  std::atomic<uint64_t> next_ticket_{0};
  InputSample<Message> latest_;
  std::array<Event, EventCapacity> events_;
  size_t head_{0}, size_{0};
  bool overflow_{false};
};

}  // namespace flygripper_flight
