#pragma once
// Scheduler
#include "pthread.h"
#include "sched.h"

// Realtime buffer
#include <realtime_tools/realtime_buffer.h>
#include <realtime_tools/realtime_publisher.h>


inline void checkCycleTime(const double &desired_update_time,
                           const double &cycle_time_error_threshold,
                           ros::Duration &elapsed_time) {
  static std::chrono::steady_clock::time_point last_time = std::chrono::steady_clock::now();
  static std::chrono::steady_clock::time_point current_time = std::chrono::steady_clock::now();

  current_time = std::chrono::steady_clock::now();
  auto time_span = std::chrono::duration_cast<std::chrono::duration<double>>(current_time - last_time);
  elapsed_time = ros::Duration(time_span.count());
  last_time = current_time;

  // Check cycle time for excess delay
  const double cycle_time_error = elapsed_time.toSec() - desired_update_time;
  double exceed_threshold = cycle_time_error - cycle_time_error_threshold;
  if (exceed_threshold > 0) {
    // print hh::mm::ss as prefix
    auto now = std::chrono::system_clock::now();
    auto localtime = std::chrono::system_clock::to_time_t(now);
    auto tm = *std::localtime(&localtime);
    char buffer[9];
    std::strftime(buffer, sizeof(buffer), "%T", &tm);
    ROS_WARN_STREAM(buffer
                    <<" Cycle time exceeded error threshold by: "
                    << exceed_threshold << "s, "
                    << "cycle time: " << elapsed_time << "s, "
                    << "threshold: " << cycle_time_error_threshold << "s");
  }
}


inline bool setThreadPriority(pthread_t thread_id, int priority) {
  int min_priority = sched_get_priority_min(2);
  int max_priority = sched_get_priority_max(2);
  // If priority is out of priority range, use Non-RT mode
  bool use_rt = true;
  if (priority < min_priority || priority > max_priority) {
    ROS_WARN("Invalid pthread priority %d, which should be in the range of (%d, %d), set to Non-RT mode.",
             priority, min_priority, max_priority);
    use_rt = false;
    priority = 0;
  } else{
    ROS_INFO( "Set RT mode and pthread priority to %d", priority);
  }

  // RT: SCHED_RR, SCHED_FIFO(*)
  // Non-RT: SCHED_OTHER
  int policy = (use_rt ? SCHED_FIFO : SCHED_OTHER);
  sched_param sched{.sched_priority = priority};
  int ret = pthread_setschedparam(thread_id, policy, &sched);
  if (ret != 0) {
    ROS_ERROR("Setting pthread priority produced error: \"%s\"", strerror(errno));
    return false;
  }
  return true;
}


inline bool setSchedulePriority(int priority) {
  int min_priority = sched_get_priority_min(2);
  int max_priority = sched_get_priority_max(2);
  // If priority is out of priority range, use Non-RT mode
  bool use_rt = true;
  if (priority < min_priority || priority > max_priority) {
    ROS_WARN("Invalid schedule priority %d, which should be in the range of (%d, %d), set to Non-RT mode.",
             priority, min_priority, max_priority);
    use_rt = false;
    priority = 0;
  } else{
    ROS_INFO( "Set RT mode and schedule priority to %d", priority);
  }
  // RT: SCHED_RR, SCHED_FIFO(*)
  // Non-RT: SCHED_OTHER
  int policy = (use_rt ? SCHED_FIFO : SCHED_OTHER);
  sched_param sched{.sched_priority = priority};
  int ret = sched_setscheduler(0, policy, &sched);
  if (ret != 0) {
    ROS_ERROR("Setting schedule priority produced error: \"%s\"", strerror(errno));
    return false;
  }
  return true;
}
