#include "flygripper_flight/input_buffer.hpp"
#include <gtest/gtest.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <mavros_msgs/RCIn.h>
#include <boost/make_shared.hpp>
#include <iostream>
#include <thread>
#include <vector>

using namespace flygripper_flight;

namespace {
template<class Predicate> bool waitFor(Predicate predicate, double timeout = 3.0) {
  const double end = steadySeconds() + timeout;
  while (!predicate() && steadySeconds() < end)
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  return predicate();
}

sensor_msgs::Imu::ConstPtr message(uint32_t sequence) {
  auto msg = boost::make_shared<sensor_msgs::Imu>();
  msg->header.seq = sequence;
  msg->header.stamp = ros::Time::now();
  return msg;
}

TEST(InputBuffer, OlderCompletionCannotOverwriteOrRefreshNewerInput) {
  InputBuffer<sensor_msgs::Imu> buffer;
  const auto old = buffer.beginCallback(ros::Time::now());
  const auto fresh = buffer.beginCallback(ros::Time::now());
  ASSERT_TRUE(buffer.store(message(2), fresh));
  const auto before = buffer.snapshot();
  std::thread delayed([&] { EXPECT_FALSE(buffer.store(message(1), old)); });
  delayed.join();
  const auto after = buffer.snapshot();
  EXPECT_EQ(after.message->header.seq, 2u);
  EXPECT_EQ(after.count, 1u);
  EXPECT_EQ(after.callbacks, 2u);
  EXPECT_EQ(after.rejected_updates, 1u);
  EXPECT_EQ(after.received_steady, before.received_steady);
  EXPECT_EQ(after.saved_steady, before.saved_steady);

  // Source clocks are not an ordering key; retain stale samples so the flight
  // node can diagnose them and refuse autonomy, including publisher restarts.
  auto frozen = boost::make_shared<sensor_msgs::Imu>();
  frozen->header.stamp = ros::Time(1);
  buffer.receive(ros::MessageEvent<const sensor_msgs::Imu>(frozen, ros::Time::now()));
  EXPECT_EQ(buffer.snapshot().message->header.stamp, ros::Time(1));
}

TEST(InputBuffer, ConcurrentSnapshotsStayConsistent) {
  InputBuffer<sensor_msgs::Imu> buffer;
  std::atomic<int> running{4};
  std::vector<std::thread> writers;
  for (int i = 0; i < 4; ++i) writers.emplace_back([&] {
    for (int j = 0; j < 2000; ++j) {
      const auto stamp = buffer.beginCallback(ros::Time::now());
      buffer.store(message(stamp.ticket), stamp);
    }
    --running;
  });
  uint64_t previous = 0;
  do {
    const auto sample = buffer.snapshot();
    EXPECT_GE(sample.timing.ticket, previous);
    if (sample.message) EXPECT_EQ(sample.message->header.seq, sample.timing.ticket);
    EXPECT_GE(sample.saved_steady, sample.lock_acquired_steady);
    previous = sample.timing.ticket;
  } while (running);
  for (auto& writer : writers) writer.join();
  const auto final = buffer.snapshot();
  EXPECT_EQ(final.callbacks, 8000u);
  EXPECT_EQ(final.count + final.rejected_updates, 8000u);
  EXPECT_EQ(final.timing.ticket, 8000u);
}

TEST(InputBuffer, RcPulseAndOverflowSurviveLatestValueUpdates) {
  InputBuffer<mavros_msgs::RCIn, 64> buffer;
  const auto send = [&buffer](uint16_t ch5) {
    auto msg = boost::make_shared<mavros_msgs::RCIn>();
    msg->channels = {1500, 1500, 1000, 1500, ch5};
    buffer.receive(ros::MessageEvent<const mavros_msgs::RCIn>(msg, ros::Time::now()));
  };
  send(2000); send(1000); send(2000);
  const auto pulse = buffer.drain();
  ASSERT_EQ(pulse.size, 3u);
  EXPECT_EQ(pulse.latest.message->channels[4], 2000u);
  EXPECT_EQ(pulse.events[1].message->channels[4], 1000u);
  EXPECT_FALSE(pulse.overflow);
  EXPECT_EQ(buffer.drain().size, 0u);
  for (int i = 0; i < 65; ++i) send(2000);
  const auto overflow = buffer.drain();
  EXPECT_EQ(overflow.size, 64u);
  EXPECT_TRUE(overflow.overflow);
  EXPECT_FALSE(buffer.drain().overflow);
}

// Expose only the test mutex, to hold the very lock used by store(). There is no
// fault-injection sleep or test parameter in the production flight node.
struct ObservableMutex {
  inline static ObservableMutex* instance{nullptr};
  std::mutex native;
  ObservableMutex() { instance = this; }
  void lock() { native.lock(); }
  bool try_lock() { return native.try_lock(); }
  void unlock() { native.unlock(); }
};

TEST(InputBuffer, QueueDelayAndLockWaitAreMeasuredSeparately) {
  InputBuffer<sensor_msgs::Imu, 0, ObservableMutex> buffer;
  const auto msg = message(1);
  buffer.receive(ros::MessageEvent<const sensor_msgs::Imu>(
      msg, ros::Time::now() - ros::Duration(0.08)));
  const auto queued = buffer.snapshot();
  EXPECT_GE(queued.queue_delay, 0.08);
  EXPECT_LT(queued.lock_wait, 0.03);

  auto* mutex = ObservableMutex::instance;
  mutex->lock();
  std::atomic<bool> entered{false};
  std::thread writer([&] {
    const auto stamp = buffer.beginCallback(ros::Time::now());
    entered = true;
    buffer.store(message(2), stamp);
  });
  const bool ready = waitFor([&] { return entered.load(); });
  std::this_thread::sleep_for(std::chrono::milliseconds(60));
  mutex->unlock();
  writer.join();
  ASSERT_TRUE(ready);
  const auto blocked = buffer.snapshot();
  EXPECT_LT(blocked.queue_delay, 0.03);
  EXPECT_GE(blocked.lock_wait, 0.04);
  EXPECT_GE(blocked.max_queue_delay, queued.queue_delay);
  EXPECT_GE(blocked.max_lock_wait, blocked.lock_wait);
  EXPECT_LT(blocked.store_duration, 0.03);
  EXPECT_GE(blocked.lock_acquired_steady, blocked.timing.callback_steady);
  EXPECT_GE(blocked.saved_steady, blocked.lock_acquired_steady);
  std::cout << "Injected lock wait: " << blocked.lock_wait * 1000.0
            << " ms, queue delay: " << blocked.queue_delay * 1000.0
            << " ms, store: " << blocked.store_duration * 1000.0 << " ms\n";
}

TEST(InputBuffer, BusyInputLockCannotBlockControlSnapshotsOrDiscardRcEvents) {
  InputBuffer<sensor_msgs::Imu, 64, ObservableMutex> buffer;
  buffer.receive(ros::MessageEvent<const sensor_msgs::Imu>(message(1), ros::Time::now()));
  auto* mutex = ObservableMutex::instance;
  mutex->lock();
  std::atomic<bool> finished{false};
  std::thread control([&] {
    EXPECT_FALSE(buffer.trySnapshot());
    EXPECT_FALSE(buffer.tryDrain());
    finished = true;
  });
  const bool did_not_wait = waitFor([&] { return finished.load(); }, 1.0);
  mutex->unlock();
  control.join();
  EXPECT_TRUE(did_not_wait);
  ASSERT_TRUE(buffer.trySnapshot());
  const auto batch = buffer.tryDrain();
  ASSERT_TRUE(batch);
  EXPECT_EQ(batch->size, 1u);
  EXPECT_EQ(batch->latest.count, 1u);
  EXPECT_EQ(batch->events[0].message->header.seq, 1u);
}

TEST(InputBuffer, ReplacedMessagesAreDestroyedOutsideLock) {
  InputBuffer<sensor_msgs::Imu> buffer;
  std::atomic<bool> deleted{false};
  sensor_msgs::Imu::ConstPtr first(new sensor_msgs::Imu, [&](const sensor_msgs::Imu* msg) {
    // Would deadlock if destruction still held this buffer's mutex.
    EXPECT_EQ(buffer.snapshot().message->header.seq, 2u);
    deleted = true;
    delete msg;
  });
  buffer.store(first, buffer.beginCallback(ros::Time::now()));
  first.reset();
  buffer.store(message(2), buffer.beginCallback(ros::Time::now()));
  EXPECT_TRUE(deleted);
}

class FunctionCallback : public ros::CallbackInterface {
 public:
  explicit FunctionCallback(std::function<void()> call) : call_(std::move(call)) {}
  CallResult call() override { call_(); return Success; }
 private:
  std::function<void()> call_;
};

template<class Message, size_t Capacity>
ros::Subscriber subscribe(ros::NodeHandle& nh, const std::string& topic,
                          InputBuffer<Message, Capacity>& buffer, ros::CallbackQueue* queue) {
  ros::SubscribeOptions options;
  options.initByFullCallbackType<const ros::MessageEvent<const Message>&>(
      topic, 64, [&buffer](const ros::MessageEvent<const Message>& event) { buffer.receive(event); });
  options.callback_queue = queue;
  options.allow_concurrent_callbacks = false;
  options.transport_hints = ros::TransportHints().tcpNoDelay();
  return nh.subscribe(options);
}

TEST(InputQueues, BlockedOrdinaryCallbackDoesNotBlockSensorsOrRc) {
  ros::NodeHandle nh;
  InputCallbackQueues queues;
  using Channel = InputCallbackQueues::Channel;
  InputBuffer<sensor_msgs::Imu> imu, ordinary;
  InputBuffer<nav_msgs::Odometry> odom;
  InputBuffer<mavros_msgs::RCIn, 64> rc;
  auto imu_sub = subscribe(nh, "test_imu", imu, queues.get(Channel::Imu));
  auto odom_sub = subscribe(nh, "test_odom", odom, queues.get(Channel::Odom));
  auto rc_sub = subscribe(nh, "test_rc", rc, queues.get(Channel::Safety));
  auto ordinary_sub = subscribe(nh, "test_ordinary", ordinary, queues.get(Channel::Ordinary));
  auto imu_pub = nh.advertise<sensor_msgs::Imu>("test_imu", 64);
  auto odom_pub = nh.advertise<nav_msgs::Odometry>("test_odom", 64);
  auto rc_pub = nh.advertise<mavros_msgs::RCIn>("test_rc", 64);
  auto ordinary_pub = nh.advertise<sensor_msgs::Imu>("test_ordinary", 64);
  queues.start();
  ASSERT_TRUE(waitFor([&] { return imu_pub.getNumSubscribers() && odom_pub.getNumSubscribers()
                              && rc_pub.getNumSubscribers() && ordinary_pub.getNumSubscribers(); }));
  std::atomic<bool> entered{false}, release{false};
  ScopeExit stop([&] { release = true; queues.stop(); });
  queues.get(Channel::Ordinary)->addCallback(boost::make_shared<FunctionCallback>([&] {
    entered = true;
    waitFor([&] { return release.load(); });
  }));
  ASSERT_TRUE(waitFor([&] { return entered.load(); }));
  ordinary_pub.publish(message(1));
  const double started = steadySeconds();
  for (int i = 0; i < 30; ++i) {
    imu_pub.publish(message(i));
    nav_msgs::Odometry sample;
    sample.header.stamp = ros::Time::now();
    odom_pub.publish(sample);
    mavros_msgs::RCIn event;
    event.header.stamp = ros::Time::now();
    event.channels = {1500, 1500, 1000, 1500, static_cast<uint16_t>(i == 10 ? 1000 : 2000)};
    rc_pub.publish(event);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  const bool critical_updated = waitFor([&] {
    return imu.snapshot().count == 30 && odom.snapshot().count == 30 && rc.snapshot().count == 30;
  }, 1.0);
  const auto before_release = ordinary.snapshot();
  const auto rc_events = rc.drain();
  const double held = steadySeconds() - started;
  release = true;
  EXPECT_TRUE(critical_updated);
  EXPECT_EQ(before_release.count, 0u);
  ASSERT_EQ(rc_events.size, 30u);
  EXPECT_EQ(rc_events.events[10].message->channels[4], 1000u);
  ASSERT_TRUE(waitFor([&] { return ordinary.snapshot().count == 1; }));
  EXPECT_GE(ordinary.snapshot().queue_delay, 0.25);
  EXPECT_LT(imu.snapshot().max_queue_delay, ordinary.snapshot().queue_delay);
  EXPECT_LT(odom.snapshot().max_queue_delay, ordinary.snapshot().queue_delay);
  EXPECT_LT(rc.snapshot().max_queue_delay, ordinary.snapshot().queue_delay);
  std::cout << "Ordinary callback blocked " << held * 1000.0 << " ms; IMU/EKF/RC each received 30. "
            << "queue max ms: " << imu.snapshot().max_queue_delay * 1000.0 << ", "
            << odom.snapshot().max_queue_delay * 1000.0 << ", "
            << rc.snapshot().max_queue_delay * 1000.0 << "; ordinary "
            << ordinary.snapshot().max_queue_delay * 1000.0 << '\n';
  queues.stop();
}

TEST(InputQueues, ImuStallDoesNotOccupyOdomOrSafetyWorkers) {
  InputCallbackQueues queues;
  using Channel = InputCallbackQueues::Channel;
  queues.start();
  std::atomic<bool> entered{false}, release{false}, odom{false}, rc{false};
  ScopeExit stop([&] { release = true; queues.stop(); });
  queues.get(Channel::Imu)->addCallback(boost::make_shared<FunctionCallback>([&] {
    entered = true;
    waitFor([&] { return release.load(); });
  }));
  const bool blocked = waitFor([&] { return entered.load(); });
  queues.get(Channel::Odom)->addCallback(boost::make_shared<FunctionCallback>([&] { odom = true; }));
  queues.get(Channel::Safety)->addCallback(boost::make_shared<FunctionCallback>([&] { rc = true; }));
  const bool independent = waitFor([&] { return odom && rc; }, 1.0);
  release = true;
  EXPECT_TRUE(blocked);
  EXPECT_TRUE(independent);
  queues.stop();
}
}  // namespace

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "input_buffer_contract");
  ros::NodeHandle nh;
  return RUN_ALL_TESTS();
}
