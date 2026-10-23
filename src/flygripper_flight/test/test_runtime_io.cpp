#include "flygripper_flight/runtime_io.hpp"
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdlib>
#include <iostream>
#include <mutex>
#include <thread>

using namespace flygripper_flight;
void require(bool condition, const char* message) {
  if (!condition) {
    std::cerr << message << '\n';
    // Exit directly: unwinding a blocked worker would wait for its test gate.
    std::exit(EXIT_FAILURE);
  }
}
template<class Predicate> void waitFor(Predicate predicate, const char* message) {
  const auto end = steadySeconds() + 3.0;
  while (!predicate() && steadySeconds() < end)
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  require(predicate(), message);
}
int main() {
  std::mutex gate;
  std::condition_variable cv;
  bool release = false;
  std::atomic<int> calls{0};
  ModeWorker worker([&](const std::string&, bool, const ModeWorker::Current& current) {
    ++calls;
    std::unique_lock<std::mutex> lock(gate);
    cv.wait(lock, [&] { return release; });
    return current();
  });
  const auto start = steadySeconds();
  bool accepted = worker.request("GUIDED_NOGPS", true);
  require(accepted, "first GUIDED request accepted");
  require(steadySeconds() - start < 0.1, "request must not block on the service");
  waitFor([&] { return calls == 1; }, "GUIDED request reached worker");
  require(worker.status().busy, "blocked worker is busy");
  accepted = worker.request("GUIDED_NOGPS", true);
  require(!accepted, "duplicate busy request rejected");
  accepted = worker.request("ALT_HOLD", false);
  require(accepted, "handover request queued");
  worker.cancel();  // Pilot override cancels pending work and the late result.
  {
    std::lock_guard<std::mutex> lock(gate);
    release = true;
  }
  cv.notify_all();
  waitFor([&] { return !worker.status().busy; }, "cancelled RPC returned");
  require(calls == 1, "cancelled queued request was not executed");
  require(!worker.status().pending, "cancel clears pending request");
  require(worker.status().ignored == 1, "cancelled RPC result ignored");
  require(worker.status().completed == 0, "cancelled RPC cannot authorize a session");
  accepted = worker.request("ALT_HOLD", false);
  require(accepted, "new handover request accepted after cancellation");
  waitFor([&] { return worker.status().completed == 1; }, "new handover request completed");
  require(calls == 2, "exactly two service calls executed");
  require(worker.status().success, "new handover request succeeded");
  worker.stop();
  accepted = worker.request("GUIDED_NOGPS", true);
  require(!accepted, "stopped worker rejects requests");
  std::cout << "Runtime worker request, cancellation and completion checks passed.\n";
}
