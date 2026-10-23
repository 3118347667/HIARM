#!/usr/bin/env python3
"""Isolated ROS integration tests: no MAVROS or robot hardware required."""
import json
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import time
import unittest

import rosbag
import rosgraph
import rospy
from dynamic_reconfigure.msg import Config, DoubleParameter

ROOT = Path(__file__).resolve().parent


class RecordingTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        os.environ['ROS_MASTER_URI'] = 'http://127.0.0.1:11329'
        cls.master = subprocess.Popen(['roscore', '-p', '11329'], stdout=subprocess.DEVNULL,
                                      stderr=subprocess.STDOUT, start_new_session=True)
        for _ in range(100):
            try:
                rosgraph.Master('/recording_test').getPid()
                break
            except Exception:
                time.sleep(.1)
        rospy.init_node('recording_test', anonymous=True, disable_signals=True)

    @classmethod
    def tearDownClass(cls):
        rospy.signal_shutdown('test complete')
        os.killpg(cls.master.pid, signal.SIGINT)
        cls.master.wait(timeout=10)

    def setUp(self):
        if rospy.has_param('/flygripper'):
            rospy.delete_param('/flygripper')
        self.directory = tempfile.TemporaryDirectory(prefix='hiarm-recording-')
        self.log = open(Path(self.directory.name) / 'output.log', 'w+')
        self.process = None
        self.publishers = []

    def start(self, timeout=2):
        env = dict(os.environ, SNAPSHOT_TIMEOUT_SECONDS=str(timeout))
        self.process = subprocess.Popen(['bash', str(ROOT / 'bag.sh'), 'test'],
                                       cwd=self.directory.name, env=env,
                                       stdout=self.log, stderr=subprocess.STDOUT,
                                       start_new_session=True)

    def output(self):
        self.log.flush()
        return Path(self.log.name).read_text()

    def wait_for(self, predicate, timeout=10):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            if predicate():
                return
            time.sleep(.05)
        self.fail(self.output())

    def parameters(self, updates=True):
        rospy.set_param('/flygripper', {'geometric_controller': {'k_pos': {'x': 3.0}},
                                       'robot_parameter': {'mass': .62}})
        if updates:
            for name in ('geometric_controller', 'robot_parameter', 'flygripper_flight_node'):
                pub = rospy.Publisher('/flygripper/' + name + '/parameter_updates',
                                      Config, queue_size=1, latch=True)
                pub.publish(Config(doubles=[DoubleParameter(name='gain', value=3.0)]))
                self.publishers.append(pub)

    def stop_recording(self):
        self.wait_for(lambda: bool(list(Path(self.directory.name).glob('*.bag.active'))))
        time.sleep(1.0)  # Subscription handshake and latched deliveries.
        rospy.logwarn('recording test event after rosbag subscription')
        for pub in self.publishers:
            pub.publish(Config(doubles=[DoubleParameter(name='gain', value=4.0)]))
        time.sleep(.5)
        os.killpg(self.process.pid, signal.SIGINT)
        self.process.wait(timeout=15)

    def test_delayed_parameters_and_latched_updates_recorded(self):
        self.start(timeout=5)
        time.sleep(.5)
        self.assertIsNone(self.process.poll())
        self.parameters()
        self.stop_recording()
        self.assertEqual(self.process.returncode, 0, self.output())
        filename = next(Path(self.directory.name).glob('*.bag'))
        with rosbag.Bag(str(filename)) as bag:
            topics = bag.get_type_and_topic_info().topics
            self.assertIn('/rosout', topics)
            for name in ('geometric_controller', 'robot_parameter', 'flygripper_flight_node'):
                self.assertGreaterEqual(topics['/flygripper/' + name + '/parameter_updates'].message_count, 2)
            _, msg, _ = next(bag.read_messages(topics=['/flygripper/rosparam_snapshot']))
            self.assertEqual(json.loads(msg.data)['parameters']['robot_parameter']['mass'], .62)
        self.assertIn('Parameter recording verification passed.', self.output())

    def test_missing_parameters_fail_before_recording(self):
        self.start(timeout=1)
        self.process.wait(timeout=10)
        self.assertNotEqual(self.process.returncode, 0)
        self.assertFalse(list(Path(self.directory.name).glob('*.bag*')))
        self.assertIn('Recording was NOT started', self.output())

    def test_missing_updates_are_detected_in_closed_bag(self):
        self.parameters(updates=False)
        self.start()
        self.stop_recording()
        self.assertNotEqual(self.process.returncode, 0)
        self.assertTrue(list(Path(self.directory.name).glob('*.bag')))
        self.assertIn('Bag verification FAILED', self.output())
        self.assertIn('parameter_updates', self.output())

    def tearDown(self):
        if self.process and self.process.poll() is None:
            os.killpg(self.process.pid, signal.SIGINT)
            try:
                self.process.wait(timeout=10)
            except subprocess.TimeoutExpired:
                os.killpg(self.process.pid, signal.SIGKILL)
                self.process.wait()
        for publisher in self.publishers:
            publisher.unregister()
        self.log.close()
        self.directory.cleanup()


if __name__ == '__main__':
    unittest.main(verbosity=2)
