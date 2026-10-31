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

# Configure the private loopback network before importing ROS modules, which
# may cache the advertised host address during import.
os.environ['ROS_MASTER_URI'] = 'http://127.0.0.1:11329'
os.environ['ROS_HOSTNAME'] = '127.0.0.1'
os.environ.pop('ROS_IP', None)

import rosbag
import rosgraph
import rospy
from dynamic_reconfigure.msg import Config, DoubleParameter
from flygripper_msgs.msg import FlightTrajectoryStatus
from geometry_msgs.msg import PoseStamped
from quadrotor_msgs.msg import GraspState, ReplanState
from std_msgs.msg import String

ROOT = Path(__file__).resolve().parent


class RecordingTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.master = subprocess.Popen(['roscore', '-p', '11329'], stdout=subprocess.DEVNULL,
                                      stderr=subprocess.STDOUT, start_new_session=True)
        for _ in range(100):
            try:
                rosgraph.Master('/recording_test').getPid()
                break
            except Exception:
                if cls.master.poll() is not None:
                    raise RuntimeError('Private ROS master exited before becoming ready')
                time.sleep(.1)
        else:
            cls.master.terminate()
            cls.master.wait(timeout=10)
            raise RuntimeError('Private ROS master did not become ready')
        rospy.init_node('recording_test', anonymous=True, disable_signals=True)

    @classmethod
    def tearDownClass(cls):
        rospy.signal_shutdown('test complete')
        os.killpg(cls.master.pid, signal.SIGINT)
        cls.master.wait(timeout=10)

    def setUp(self):
        if rospy.has_param('/flygripper'):
            rospy.delete_param('/flygripper')
        if rospy.has_param('/drone0/planning'):
            rospy.delete_param('/drone0/planning')
        for name in ('/ekf', '/vrpn_client_node', '/late_node'):
            if rospy.has_param(name):
                rospy.delete_param(name)
        self.directory = tempfile.TemporaryDirectory(prefix='hiarm-recording-')
        self.log = open(Path(self.directory.name) / 'output.log', 'w+')
        self.process = None
        self.publishers = []
        self.other_publishers = []
        self.subscribers = []

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

    def test_planning_started_after_recording_and_parameter_change(self):
        self.parameters()
        observed = []
        self.subscribers.append(rospy.Subscriber(
            '/drone0/planning/rosparam_snapshot', String,
            lambda message: observed.append(json.loads(message.data))))
        self.start()
        self.wait_for(lambda: bool(list(Path(self.directory.name).glob('*.bag.active'))))
        rospy.set_param('/drone0/planning', {'vmax': 0.4, 'grasp_start_dis': 0.05})
        self.wait_for(lambda: any(item['parameters'].get('vmax') == 0.4 for item in observed))
        time.sleep(1.0)
        rospy.set_param('/drone0/planning/vmax', 0.3)
        self.wait_for(lambda: any(item['parameters'].get('vmax') == 0.3 for item in observed))
        self.stop_recording()
        self.assertEqual(self.process.returncode, 0, self.output())
        filename = next(Path(self.directory.name).glob('*.bag'))
        with rosbag.Bag(str(filename)) as bag:
            snapshots = [json.loads(msg.data) for _, msg, _ in
                         bag.read_messages(topics=['/drone0/planning/rosparam_snapshot'])]
        self.assertEqual([item['parameters']['vmax'] for item in snapshots], [0.4, 0.3])
        self.assertEqual(snapshots[0]['parameters']['grasp_start_dis'], 0.05)

    def test_full_tree_captures_initial_late_changed_and_deleted_parameters(self):
        self.parameters()
        rospy.set_param('/ekf', {'imu_trans_x': 0.04, 'imu_trans_y': -0.08})
        rospy.set_param('/vrpn_client_node', {'use_server_time': True})
        observed = []
        self.subscribers.append(rospy.Subscriber(
            '/rosparam_snapshot', String,
            lambda message: observed.append(json.loads(message.data))))
        self.start()
        self.wait_for(lambda: bool(list(Path(self.directory.name).glob('*.bag.active'))))
        self.wait_for(lambda: bool(observed))
        self.assertEqual(observed[-1]['parameters'].get('ekf', {}).get('imu_trans_y'), -0.08,
                         repr(observed[-1]))
        time.sleep(1.0)
        rospy.set_param('/late_node', {'enabled': True, 'gain': 2.0})
        self.wait_for(lambda: any(x['parameters'].get('late_node', {}).get('gain') == 2.0
                                  for x in observed))
        rospy.set_param('/late_node/gain', 3.0)
        self.wait_for(lambda: any(x['parameters'].get('late_node', {}).get('gain') == 3.0
                                  for x in observed))
        rospy.delete_param('/late_node')
        self.wait_for(lambda: bool(observed) and 'late_node' not in observed[-1]['parameters'])
        self.stop_recording()
        self.assertEqual(self.process.returncode, 0, self.output())
        filename = next(Path(self.directory.name).glob('*.bag'))
        with rosbag.Bag(str(filename)) as bag:
            full = [json.loads(msg.data) for _, msg, _ in
                    bag.read_messages(topics=['/rosparam_snapshot'])]
        self.assertTrue(all(item['namespace'] == '/' for item in full))
        self.assertEqual(full[0]['parameters']['ekf']['imu_trans_x'], 0.04)
        self.assertTrue(full[0]['parameters']['vrpn_client_node']['use_server_time'])
        self.assertTrue(any(x['parameters'].get('late_node', {}).get('gain') == 2.0 for x in full))
        self.assertTrue(any(x['parameters'].get('late_node', {}).get('gain') == 3.0 for x in full))
        self.assertNotIn('late_node', full[-1]['parameters'])

    def test_moving_grasp_status_topics_recorded(self):
        self.parameters()
        messages = {
            '/flygripper/flight_trajectory_status': FlightTrajectoryStatus(
                trajectory_active=True, trajectory_id=42),
            '/drone0/planning/grasp_phase': String(data='closing_assumed'),
            '/graspState': GraspState(graspState=1),
            '/drone0/replanState': ReplanState(state=3),
            '/triger': PoseStamped(),
            '/land_triger': PoseStamped(),
        }
        for topic, message in messages.items():
            self.other_publishers.append(rospy.Publisher(topic, type(message), queue_size=1))
        self.start()
        self.wait_for(lambda: all(pub.get_num_connections() for pub in self.other_publishers))
        # A reported connection can precede completion of the TCP handshake.
        for _ in range(3):
            for pub, message in zip(self.other_publishers, messages.values()):
                pub.publish(message)
            time.sleep(.2)
        self.stop_recording()
        self.assertEqual(self.process.returncode, 0, self.output())
        filename = next(Path(self.directory.name).glob('*.bag'))
        with rosbag.Bag(str(filename)) as bag:
            recorded = {topic: msg for topic, msg, _ in bag.read_messages(topics=list(messages))}
        self.assertEqual(set(recorded), set(messages))
        self.assertEqual(recorded['/flygripper/flight_trajectory_status'].trajectory_id, 42)
        self.assertEqual(recorded['/drone0/planning/grasp_phase'].data, 'closing_assumed')
        self.assertEqual(recorded['/graspState'].graspState, 1)
        self.assertEqual(recorded['/drone0/replanState'].state, 3)

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
        for publisher in self.other_publishers:
            publisher.unregister()
        for subscriber in self.subscribers:
            subscriber.unregister()
        self.log.close()
        self.directory.cleanup()


if __name__ == '__main__':
    unittest.main(verbosity=2)
