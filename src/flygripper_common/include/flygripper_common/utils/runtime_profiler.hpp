#pragma once

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <mutex>
#include <numeric>
#include <string>
#include <utility>
#include <vector>

namespace flygripper_common {

class RuntimeProfiler {
public:
  explicit RuntimeProfiler(std::string module_name,
                           std::size_t report_every = 100)
      : module_name_(std::move(module_name)), report_every_(report_every),
        enabled_(profilingEnabled()) {}

  bool enabled() const { return enabled_; }

  void record(double elapsed_ms) {
    if (!enabled_) {
      return;
    }

    std::lock_guard<std::mutex> lock(mutex_);
    if (samples_ms_.empty()) {
      first_sample_time_ = Clock::now();
    }
    samples_ms_.push_back(elapsed_ms);
    if (report_every_ == 0 || samples_ms_.size() % report_every_ != 0) {
      return;
    }

    std::vector<double> sorted_samples(samples_ms_);
    std::sort(sorted_samples.begin(), sorted_samples.end());
    const double mean_ms =
        std::accumulate(sorted_samples.begin(), sorted_samples.end(), 0.0) /
        static_cast<double>(sorted_samples.size());
    const std::size_t p95_index = static_cast<std::size_t>(
        std::ceil(0.95 * static_cast<double>(sorted_samples.size())) - 1.0);
    const double elapsed_seconds =
        std::chrono::duration<double>(Clock::now() - first_sample_time_).count();
    const double rate_hz = elapsed_seconds > 0.0
                               ? static_cast<double>(samples_ms_.size()) /
                                     elapsed_seconds
                               : 0.0;

    std::cout << "[PROFILE] module=" << module_name_
              << " samples=" << samples_ms_.size() << " mean_ms=" << mean_ms
              << " p95_ms=" << sorted_samples[p95_index]
              << " max_ms=" << sorted_samples.back() << " rate_hz=" << rate_hz
              << std::endl;
  }

private:
  using Clock = std::chrono::steady_clock;

  static bool profilingEnabled() {
    const char *value = std::getenv("DYNAMIC_GRASP_PROFILE");
    return value != nullptr && std::string(value) != "0";
  }

  std::string module_name_;
  std::size_t report_every_;
  bool enabled_;
  std::mutex mutex_;
  std::vector<double> samples_ms_;
  Clock::time_point first_sample_time_;
};

class ScopedRuntimeSample {
public:
  explicit ScopedRuntimeSample(RuntimeProfiler &profiler)
      : profiler_(profiler), enabled_(profiler.enabled()) {
    if (enabled_) {
      start_time_ = Clock::now();
    }
  }

  ~ScopedRuntimeSample() {
    if (!enabled_) {
      return;
    }
    const double elapsed_ms =
        std::chrono::duration<double, std::milli>(Clock::now() - start_time_)
            .count();
    profiler_.record(elapsed_ms);
  }

private:
  using Clock = std::chrono::steady_clock;

  RuntimeProfiler &profiler_;
  bool enabled_;
  Clock::time_point start_time_;
};

} // namespace flygripper_common
