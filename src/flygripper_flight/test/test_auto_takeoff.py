#!/usr/bin/env python3
"""Exercise the real flight node against isolated ROS mocks, without hardware.

The odometry follower copies controller references; this tests the state machine
and MAVROS contract, not vehicle dynamics or ArduPilot's motor/land detector.
"""
import math
import os
import signal
import subprocess
import tempfile
import threading
import time
import unittest

import rosparam
import rospkg
import rospy
import rostest
from flygripper_msgs.msg import GeometricControllerData
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import AttitudeTarget, ExtendedState, ParamValue, RCIn, State
from mavros_msgs.srv import ParamGet, ParamGetResponse, SetMode, SetModeResponse
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu


class AutoTakeoffTest(unittest.TestCase):
    def setUp(self):
        self.running = True
        self.armed = True
        self.mode = "STABILIZE"
        self.landed = ExtendedState.LANDED_STATE_ON_GROUND
        self.guid_options = 9
        self.confirm_modes = True
        self.follow = True
        self.publish_odom = True
        self.frozen_odom_stamp = None
        self.publish_extended = True
        self.ch5 = self.ch6 = 1000
        self.origin = [2.0, -3.0, 0.6]
        self.position = self.origin[:]
        self.velocity = [0.0] * 3
        self.commands = []
        self.references = []
        self.mode_requests = []
        self.triggers = []
        self.process = None
        self.latest_reference = None
        self.directory = tempfile.TemporaryDirectory(prefix="hiarm-takeoff-test-")
        self.log = open(os.path.join(self.directory.name, "flight.log"), "w+")
        self.pubs = {
            "state": rospy.Publisher("/mavros/state", State, queue_size=10),
            "extended": rospy.Publisher("/mavros/extended_state", ExtendedState, queue_size=10),
            "imu": rospy.Publisher("/mavros/imu/data", Imu, queue_size=10),
            "odom": rospy.Publisher("/ekf/ekf_odom", Odometry, queue_size=10),
            "rc": rospy.Publisher("/mavros/rc/in", RCIn, queue_size=10),
        }
        self.subs = [
            rospy.Subscriber("/mavros/setpoint_raw/attitude", AttitudeTarget,
                             lambda msg: self.commands.append(msg)),
            rospy.Subscriber("/flygripper/geometric_controller/data", GeometricControllerData,
                             self.reference_callback),
            rospy.Subscriber("/move_base_simple/goal", PoseStamped,
                             lambda msg: self.triggers.append(msg)),
        ]
        self.services = [
            rospy.Service("/mavros/set_mode", SetMode, self.set_mode),
            rospy.Service("/mavros/param/get", ParamGet, self.get_param),
        ]
        self.thread = threading.Thread(target=self.publish, daemon=True)
        self.thread.start()

    def start_node(self, overrides=None):
        root = rospkg.RosPack().get_path("flygripper_flight")
        controller = rospkg.RosPack().get_path("flygripper_controller")
        for filename in [os.path.join(root, "params/flygripper.yaml"),
                         os.path.join(controller, "params/flygripper.yaml")]:
            for values, _ in rosparam.load_file(filename):
                rosparam.upload_params("/flygripper", values)
        for values, _ in rosparam.load_file(os.path.join(root, "params/fsm.yaml")):
            values.update(takeoff_height=0.2, takeoff_velocity=0.4,
                          takeoff_acceleration=1.0, takeoff_jerk=4.0,
                          takeoff_settle_time=0.15, takeoff_liftoff_timeout=2.5,
                          takeoff_timeout=5.0, mode_switch_timeout=0.7,
                          odom_timeout=0.15, imu_timeout=0.15)
            values.update(overrides or {})
            rosparam.upload_params("/flygripper/flight_test", values)
        self.process = subprocess.Popen([
            "rosrun", "flygripper_flight", "flygripper_flight_node",
            "__ns:=/flygripper", "__name:=flight_test", "odom:=/ekf/ekf_odom",
            "imu:=/mavros/imu/data", "trigger:=/move_base_simple/goal"],
            stdout=self.log, stderr=subprocess.STDOUT)
        self.wait_for(lambda: self.pubs["rc"].get_num_connections() > 0,
                      "flight subscriber startup")
        time.sleep(0.2)  # Establish CH5 low before the first rising edge.

    def set_mode(self, request):
        self.mode_requests.append(request.custom_mode)
        if self.confirm_modes:
            self.mode = request.custom_mode
        return SetModeResponse(mode_sent=True)

    def get_param(self, request):
        return ParamGetResponse(success=request.param_id == "GUID_OPTIONS",
                                value=ParamValue(integer=self.guid_options))

    def reference_callback(self, msg):
        self.references.append(msg)
        self.latest_reference = msg

    def publish(self):
        while self.running and not rospy.is_shutdown():
            stamp = rospy.Time.now()
            if self.follow and self.latest_reference is not None and self.mode == "GUIDED_NOGPS":
                r = self.latest_reference
                self.position = [r.position_reference.x, r.position_reference.y,
                                 r.position_reference.z]
                self.velocity = [r.velocity_reference.x, r.velocity_reference.y,
                                 r.velocity_reference.z]
                if self.position[2] - self.origin[2] > 0.03:
                    self.landed = ExtendedState.LANDED_STATE_IN_AIR
            state = State(connected=True, armed=self.armed, mode=self.mode,
                          system_status=4 if self.armed else 3)
            state.header.stamp = stamp
            self.pubs["state"].publish(state)
            if self.publish_extended:
                ext = ExtendedState(landed_state=self.landed)
                ext.header.stamp = stamp
                self.pubs["extended"].publish(ext)
            imu = Imu()
            imu.header.stamp = stamp
            imu.orientation.w = 1.0
            imu.linear_acceleration.z = 9.81
            self.pubs["imu"].publish(imu)
            if self.publish_odom:
                odom = Odometry()
                odom.header.stamp = self.frozen_odom_stamp or stamp
                odom.pose.pose.orientation.w = 1.0
                odom.pose.pose.position.x, odom.pose.pose.position.y, odom.pose.pose.position.z = self.position
                odom.twist.twist.linear.x, odom.twist.twist.linear.y, odom.twist.twist.linear.z = self.velocity
                self.pubs["odom"].publish(odom)
            rc = RCIn(channels=[1500, 1500, 1000, 1500, self.ch5, self.ch6,
                                1000, 1000, 1500, 1000])
            rc.header.stamp = stamp
            self.pubs["rc"].publish(rc)
            time.sleep(0.01)

    def wait_for(self, predicate, description, timeout=5):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if predicate():
                return
            if self.process is not None and self.process.poll() is not None:
                break
            time.sleep(0.02)
        self.log.flush()
        self.log.seek(0)
        self.fail(description + "\n" + self.log.read()[-6000:])

    def trigger(self):
        self.ch5 = 2000

    def wait_ascent(self):
        self.wait_for(lambda: self.references and
                      self.references[-1].position_reference.z > self.origin[2] + 0.01,
                      "ascent started")

    def assert_stopped(self):
        time.sleep(0.2)  # Drain commands already queued before FCU mode feedback.
        count = len(self.commands)
        time.sleep(0.2)
        self.assertEqual(count, len(self.commands))

    def test_success_relative_height_center_gate_ch6_and_second_sortie(self):
        self.start_node()
        self.trigger()
        self.wait_ascent()
        self.ch6 = 2000
        self.wait_for(lambda: self.position[2] >= 0.799, "relative target reached")
        time.sleep(0.7)
        self.assertFalse(self.triggers, "CH6 during ascent must be consumed")
        self.assertAlmostEqual(self.references[-1].position_reference.z, 0.8, places=5)
        self.assertEqual("GUIDED_NOGPS", self.mode)
        self.assertTrue(all(math.isfinite(c.thrust) and 0 < c.thrust <= 1 for c in self.commands))
        self.ch5 = 1000
        self.wait_for(lambda: self.mode == "ALT_HOLD", "CH5 takeover")
        self.assert_stopped()
        self.follow = False
        self.latest_reference = None
        self.origin = [5.0, -2.0, 1.7]
        self.position = self.origin[:]
        self.velocity = [0.0] * 3
        self.landed = ExtendedState.LANDED_STATE_ON_GROUND
        self.mode = "STABILIZE"
        self.ch6 = 1000
        time.sleep(0.2)
        self.follow = True
        self.trigger()
        self.wait_for(lambda: self.position[2] >= 1.899, "new sortie uses new origin")
        self.assertAlmostEqual(self.references[-1].position_reference.x, 5.0)

    def test_disarmed_rejected(self):
        self.armed = False
        self.start_node()
        self.trigger()
        time.sleep(0.5)
        self.assertFalse(self.mode_requests)
        self.assertFalse(self.commands)

    def test_wrong_thrust_semantics_rejected(self):
        self.guid_options = 0
        self.start_node()
        self.trigger()
        time.sleep(0.5)
        self.assertFalse(self.mode_requests)
        self.assertFalse(self.commands)

    def test_missing_landed_state_rejected(self):
        self.publish_extended = False
        self.start_node()
        self.trigger()
        time.sleep(0.5)
        self.assertFalse(self.mode_requests)
        self.assertFalse(self.commands)

    def test_mode_sent_without_confirmation_cannot_start(self):
        self.confirm_modes = False
        self.start_node()
        self.trigger()
        self.wait_for(lambda: "ALT_HOLD" in self.mode_requests, "mode confirmation timeout")
        self.assertFalse(self.commands)

    def test_no_liftoff_requests_takeover(self):
        self.follow = False
        self.start_node({"takeoff_liftoff_timeout": 1.0})
        self.trigger()
        self.wait_for(lambda: "ALT_HOLD" in self.mode_requests, "no liftoff takeover")
        self.assert_stopped()

    def test_stale_odometry_requests_takeover(self):
        self.start_node()
        self.trigger()
        self.wait_ascent()
        self.publish_odom = False
        self.wait_for(lambda: self.mode == "ALT_HOLD", "stale odometry takeover")
        self.assert_stopped()

    def test_stale_source_stamp_requests_takeover(self):
        self.start_node()
        self.trigger()
        self.wait_ascent()
        self.frozen_odom_stamp = rospy.Time.now() - rospy.Duration(1.0)
        self.wait_for(lambda: self.mode == "ALT_HOLD", "old odometry source timestamp")
        self.assert_stopped()

    def test_input_recovery_requires_new_ch5_edge(self):
        self.publish_odom = False
        self.start_node()
        self.trigger()
        time.sleep(0.3)
        self.publish_odom = True
        time.sleep(0.3)
        self.assertFalse(self.mode_requests, "do not queue takeoff while inputs are absent")
        self.ch5 = 1000
        time.sleep(0.15)
        self.trigger()
        self.wait_ascent()

    def test_ch5_cancel_during_ascent(self):
        self.start_node()
        self.trigger()
        self.wait_ascent()
        self.ch5 = 1000
        self.wait_for(lambda: self.mode == "ALT_HOLD", "cancel ascent with CH5")
        self.assert_stopped()

    def test_external_mode_has_priority(self):
        self.start_node()
        self.trigger()
        self.wait_ascent()
        count = len(self.mode_requests)
        self.mode = "LAND"
        self.assert_stopped()
        self.assertEqual(count, len(self.mode_requests), "do not override FCU/pilot mode")

    def test_armed_shutdown_does_not_send_motor_stop(self):
        self.start_node()
        self.trigger()
        self.wait_ascent()
        self.process.send_signal(signal.SIGINT)
        self.process.wait(timeout=5)
        time.sleep(0.1)
        self.assertTrue(self.commands)
        self.assertTrue(all(c.thrust > 0 for c in self.commands))

    def tearDown(self):
        if self.process is not None and self.process.poll() is None:
            self.process.send_signal(signal.SIGINT)
            try:
                self.process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                self.process.kill()
                self.process.wait()
        self.running = False
        self.thread.join(timeout=2)
        for service in self.services:
            service.shutdown()
        for subscriber in self.subs:
            subscriber.unregister()
        for publisher in self.pubs.values():
            publisher.unregister()
        self.log.close()
        self.directory.cleanup()


if __name__ == "__main__":
    rospy.init_node("auto_takeoff_contract_test")
    rostest.rosrun("flygripper_flight", "auto_takeoff_contract", AutoTakeoffTest)
