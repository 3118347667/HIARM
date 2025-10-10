#pragma once
#include <chrono>
#include <map>
namespace flygripper_common{
class Timer {
 public:
  // Start the timer
  void tic()
  {
    start_time_ = std::chrono::high_resolution_clock::now();
  }

  // Stop the timer and calculate the elapsed time
  double toc()
  {
    auto end_time = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsed_time = end_time - start_time_;
    return elapsed_time.count();
  }

  double toc(std::string name)
  {
    if (named_buffer_.find(name) == named_buffer_.end()){
      named_buffer_[name] = toc();
    } else{
      named_buffer_[name] = 0.9*named_buffer_[name] + 0.1*toc();
    }
    return named_buffer_[name];
  }

  double get(std::string name){
    if(named_buffer_.find(name) == named_buffer_.end()){
      return -1;
    }
    return named_buffer_[name];
  }

  std::string analyze(std::initializer_list<std::string> names) {
    std::string result;
    auto last = names.end()-1;
    double total_time = get(*last) * 1000.0;
    if (total_time < 0) {
      return "last time name" + *last +" is not valid";
    }
    double interval, percentage, prev_interval_raw;
    // Process subsequent times
    for (auto it = names.begin(); it != names.end(); ++it) {
      // Process the first time
      if (it == names.begin()){
        if (get(*it) > 0){
          interval = get(*it)*1000.0;
          percentage = interval/total_time*100.0;
          prev_interval_raw = get(*it);
          result += "\n" + *it + ": ";
          result += std::to_string(interval) + " ms " +
                    "(" + std::to_string(percentage) + "%)";
        } else{
          return "first time name" + *it +" is not valid";
        }
      } else{
        if (get(*it) > 0){
          interval = (get(*it)-prev_interval_raw)*1000.0;
          percentage = interval/total_time*100.0;
          prev_interval_raw = get(*it);
          result += "\n" + *it + ": ";
          result += std::to_string(interval) + " ms " +
                    "(" + std::to_string(percentage) + "%)";
        }
      }
    }
    // Calculate and append total time
    result += "\nTotal time: " + std::to_string(total_time) + " ms";


    return result;
  }

 private:
  std::chrono::high_resolution_clock::time_point start_time_;
  std::unordered_map<std::string, double> named_buffer_;
};
}
