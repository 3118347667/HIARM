#pragma once

#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <functional>
#include <mutex>
#include <string>
#include <thread>

namespace flygripper_flight {
inline double steadySeconds() {
  return std::chrono::duration<double>(
      std::chrono::steady_clock::now().time_since_epoch()).count();
}

// One worker, at most one pending request. Results never modify flight state.
// Cancellation invalidates queued work/results; an RPC already on the wire
// cannot be recalled. FCU state feedback remains authoritative.
class ModeWorker {
 public:
  struct Status {
    bool busy{false}, pending{false}, success{false};
    std::string active_mode, result_mode;
    double elapsed{0.0}, last_duration{0.0};
    uint64_t completed{0}, ignored{0};
  };
  using Current = std::function<bool()>;
  using Call = std::function<bool(const std::string&, bool, const Current&)>;
  explicit ModeWorker(Call call) : call_(std::move(call)), thread_([this] { run(); }) {}
  ~ModeWorker() { stop(); }
  bool request(const std::string& mode, bool verify_thrust) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (stopping_ || (status_.busy && status_.active_mode == mode) ||
        (status_.pending && pending_mode_ == mode)) return false;
    ++generation_;
    pending_mode_ = mode;
    verify_thrust_ = verify_thrust;
    status_.pending = true;
    cv_.notify_one();
    return true;
  }
  void cancel() {
    std::lock_guard<std::mutex> lock(mutex_);
    ++generation_;
    status_.pending = false;
  }
  Status status() const {
    std::lock_guard<std::mutex> lock(mutex_);
    auto result = status_;
    result.elapsed = result.busy ? steadySeconds() - started_ : 0.0;
    return result;
  }
  void stop() {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      stopping_ = true;
      ++generation_;
      status_.pending = false;
      cv_.notify_one();
    }
    if (thread_.joinable()) thread_.join();
  }
 private:
  void run() {
    std::unique_lock<std::mutex> lock(mutex_);
    while (true) {
      cv_.wait(lock, [this] { return stopping_ || status_.pending; });
      if (stopping_) return;
      const auto mode = pending_mode_;
      const bool verify = verify_thrust_;
      const auto generation = generation_;
      status_.pending = false;
      status_.busy = true;
      status_.active_mode = mode;
      started_ = steadySeconds();
      lock.unlock();
      const Current current = [this, generation] {
        std::lock_guard<std::mutex> guard(mutex_);
        return !stopping_ && generation == generation_;
      };
      bool success = false;
      try { if (current()) success = call_(mode, verify, current); }
      catch (...) { success = false; }
      lock.lock();
      status_.last_duration = steadySeconds() - started_;
      status_.busy = false;
      if (generation == generation_) {
        status_.result_mode = mode;
        status_.success = success;
        ++status_.completed;
      } else {
        ++status_.ignored;
      }
    }
  }
  Call call_;
  mutable std::mutex mutex_;
  std::condition_variable cv_;
  Status status_;
  bool stopping_{false}, verify_thrust_{false};
  std::string pending_mode_;
  uint64_t generation_{0};
  double started_{0.0};
  std::thread thread_;
};

// scope exit also measures control paths that return on a fault.
class ScopeExit {
 public:
  explicit ScopeExit(std::function<void()> fn) : fn_(std::move(fn)) {}
  ~ScopeExit() { fn_(); }
 private:
  std::function<void()> fn_;
};
}  // namespace flygripper_flight
