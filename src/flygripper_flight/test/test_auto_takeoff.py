#!/usr/bin/env python3
"""Exercise the real flight node against isolated ROS mocks, without hardware.

The odometry follower copies controller references; this tests the state machine
and MAVROS contract, not vehicle dynamics or ArduPilot's motor/land detector.
"""
import json
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
from dynamic_reconfigure.client import Client
from diagnostic_msgs.msg import DiagnosticArray
from flygripper_msgs.msg import (FlightTrajectoryStatus, GeometricControllerData,
                                Polynomial, PolynomialTrajectory)
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
        self.param_success = True
        self.param_delay = 0.0
        self.param_requests = []
        self.confirm_modes = True
        self.mode_delay = 0.0
        self.runtime = []
        self.publish_times = []
        self.publish_imu = True
        self.publish_rc = True
        self.follow = True
        self.publish_odom = True
        self.frozen_odom_stamp = None
        self.imu_time_offset = 0.0
        self.imu_roll = 0.0
        self.publish_extended = True
        self.extended_period = 0.0
        self.last_extended_publish = 0.0
        self.ch5 = self.ch6 = 1000
        self.pitch = 1500
        self.throttle = 1000
        self.origin = [2.0, -3.0, 0.6]
        self.position = self.origin[:]
        self.velocity = [0.0] * 3
        self.commands = []
        self.references = []
        self.mode_requests = []
        self.triggers = []
        self.flight_status = []
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
            "trajectory": rospy.Publisher("/flygripper/trajectory", PolynomialTrajectory, queue_size=10),
        }
        self.subs = [
            rospy.Subscriber("/flygripper/runtime_diagnostics", DiagnosticArray,
                             lambda msg: self.runtime.append(msg)),
            rospy.Subscriber("/flygripper/flight_trajectory_status", FlightTrajectoryStatus,
                             lambda msg: self.flight_status.append(msg)),
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

    def start_node(self, overrides=None, controller_overrides=None):
        root = rospkg.RosPack().get_path("flygripper_flight")
        controller = rospkg.RosPack().get_path("flygripper_controller")
        for filename in [os.path.join(root, "params/flygripper.yaml"),
                         os.path.join(controller, "params/flygripper.yaml")]:
            for values, _ in rosparam.load_file(filename):
                rosparam.upload_params("/flygripper", values)
        for key, value in (controller_overrides or {}).items():
            rospy.set_param("/flygripper/geometric_controller/" + key, value)
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
        time.sleep(self.mode_delay)
        if self.confirm_modes:
            self.mode = request.custom_mode
        return SetModeResponse(mode_sent=True)

    def get_param(self, request):
        self.param_requests.append(request.param_id)
        time.sleep(self.param_delay)
        return ParamGetResponse(success=self.param_success and request.param_id == "GUID_OPTIONS",
                                value=ParamValue(integer=self.guid_options))

    def reference_callback(self, msg):
        self.references.append(msg)
        self.latest_reference = msg

    def publish(self):
        while self.running and not rospy.is_shutdown():
            stamp = rospy.Time.now()
            self.publish_times.append(stamp.to_sec())
            if self.follow and self.latest_reference is not None and self.mode == "GUIDED_NOGPS":
                r = self.latest_reference
                self.position = [r.position_reference.x, r.position_reference.y,
                                 r.position_reference.z]
                self.velocity = [r.velocity_reference.x, r.velocity_reference.y,
                                 r.velocity_reference.z]
                if self.position[2] - self.origin[2] > 0.03:
                    self.landed = ExtendedState.LANDED_STATE_IN_AIR
            state = State(connected=True, armed=self.armed, mode=self.mode,
                          guided=self.mode == "GUIDED_NOGPS",
                          system_status=4 if self.armed else 3)
            state.header.stamp = stamp
            self.pubs["state"].publish(state)
            if (self.publish_extended and
                    time.monotonic() - self.last_extended_publish >= self.extended_period):
                ext = ExtendedState(landed_state=self.landed)
                ext.header.stamp = stamp
                self.pubs["extended"].publish(ext)
                self.last_extended_publish = time.monotonic()
            imu = Imu()
            imu.header.stamp = stamp + rospy.Duration(self.imu_time_offset)
            imu.orientation.x = math.sin(self.imu_roll / 2)
            imu.orientation.w = math.cos(self.imu_roll / 2)
            imu.linear_acceleration.z = 9.81
            if self.publish_imu:
                self.pubs["imu"].publish(imu)
            if self.publish_odom:
                odom = Odometry()
                odom.header.stamp = self.frozen_odom_stamp or stamp
                odom.pose.pose.orientation.w = 1.0
                odom.pose.pose.position.x, odom.pose.pose.position.y, odom.pose.pose.position.z = self.position
                odom.twist.twist.linear.x, odom.twist.twist.linear.y, odom.twist.twist.linear.z = self.velocity
                self.pubs["odom"].publish(odom)
            rc = RCIn(channels=[1500, self.pitch, self.throttle, 1500, self.ch5, self.ch6,
                                1000, 1000, 1500, 1000])
            rc.header.stamp = stamp
            if self.publish_rc:
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
        self.wait_for(lambda: float(self.runtime_values()["l1_initialization_count"]) == 2.0,
                      "initialize once for the second control session")

    def test_flight_trajectory_status_tracks_actual_control(self):
        self.start_node()
        self.wait_for(lambda: self.flight_status, "initial status")
        self.assertFalse(self.flight_status[-1].trajectory_active)

        self.trigger()
        self.wait_for(lambda: self.position[2] >= 0.799, "takeoff target")
        self.wait_for(lambda: self.flight_status and
                      self.flight_status[-1].fsm_state == FlightTrajectoryStatus.FSM_GUIDED and
                      self.flight_status[-1].reference_type == FlightTrajectoryStatus.REF_HOVER,
                      "takeoff completed")
        self.ch6 = 2000
        self.wait_for(lambda: self.flight_status and
                      self.flight_status[-1].fsm_state == FlightTrajectoryStatus.FSM_TRAJ,
                      "trajectory mode")
        self.assertFalse(self.flight_status[-1].trajectory_active)

        trajectory = PolynomialTrajectory(id=42, heading_mode=PolynomialTrajectory.HAEDING_NONE)
        trajectory.header.stamp = rospy.Time.now()
        trajectory.xyz_pieces = [Polynomial(order=5, dim=3, duration=3.0,
                                            coeff=[0.0] * 15 + self.position[:])]
        self.pubs["trajectory"].publish(trajectory)
        self.wait_for(lambda: self.flight_status and self.flight_status[-1].trajectory_active and
                      self.flight_status[-1].trajectory_id == 42, "sampled trajectory status")
        active = self.flight_status[-1]
        self.assertEqual(trajectory.header.stamp, active.trajectory_start)
        self.assertTrue(active.fcu_state_fresh and active.fcu_armed and active.fcu_guided_nogps)
        self.assertEqual(active.reference_type, FlightTrajectoryStatus.REF_TRAJ)

        # A newly received malformed message must invalidate the old trajectory,
        # including when the sender reuses its ID and timestamp.
        malformed = PolynomialTrajectory(id=42)
        malformed.header.stamp = trajectory.header.stamp
        self.pubs["trajectory"].publish(malformed)
        self.wait_for(lambda: self.flight_status and not self.flight_status[-1].trajectory_active,
                      "invalid trajectory clears active status")

        trajectory.id = 43
        trajectory.header.stamp = rospy.Time.now()
        self.pubs["trajectory"].publish(trajectory)
        self.wait_for(lambda: self.flight_status and self.flight_status[-1].trajectory_active and
                      self.flight_status[-1].trajectory_id == 43, "new trajectory accepted")

        future = PolynomialTrajectory(id=44, heading_mode=PolynomialTrajectory.HAEDING_NONE)
        future.xyz_pieces = trajectory.xyz_pieces
        future.header.stamp = rospy.Time.now() + rospy.Duration(0.2)
        checkpoint = len(self.flight_status)
        self.pubs["trajectory"].publish(future)
        time.sleep(0.12)
        self.assertTrue(self.flight_status[checkpoint:])
        self.assertTrue(all(s.trajectory_active and s.trajectory_id == 43
                            for s in self.flight_status[checkpoint:]),
                        "future successor must not interrupt the active trajectory")
        self.wait_for(lambda: self.flight_status and self.flight_status[-1].trajectory_active and
                      self.flight_status[-1].trajectory_id == 44, "future trajectory activated")

        too_far = PolynomialTrajectory(id=45, heading_mode=PolynomialTrajectory.HAEDING_NONE)
        too_far.header.stamp = rospy.Time.now()
        too_far.xyz_pieces = [Polynomial(order=5, dim=3, duration=3.0,
                                         coeff=[0.0] * 15 + [100.0, 100.0, 100.0])]
        self.pubs["trajectory"].publish(too_far)
        self.wait_for(lambda: self.flight_status and not self.flight_status[-1].trajectory_active,
                      "sampler rejection clears active status")
        trajectory.id = 46
        trajectory.header.stamp = rospy.Time.now()
        self.pubs["trajectory"].publish(trajectory)
        self.wait_for(lambda: self.flight_status and self.flight_status[-1].trajectory_active and
                      self.flight_status[-1].trajectory_id == 46, "accepted after rejection")
        self.ch6 = 1000
        self.wait_for(lambda: self.flight_status and not self.flight_status[-1].trajectory_active,
                      "CH6 exit clears active status")
        self.ch6 = 2000
        self.wait_for(lambda: self.flight_status and
                      self.flight_status[-1].fsm_state == FlightTrajectoryStatus.FSM_TRAJ,
                      "reenter trajectory mode")
        trajectory.id = 47
        trajectory.header.stamp = rospy.Time.now()
        self.pubs["trajectory"].publish(trajectory)
        self.wait_for(lambda: self.flight_status and self.flight_status[-1].trajectory_active and
                      self.flight_status[-1].trajectory_id == 47, "trajectory before disarming")
        self.armed = False
        self.wait_for(lambda: self.flight_status and not self.flight_status[-1].trajectory_active and
                      not self.flight_status[-1].fcu_armed, "disarming clears active status")


    def test_reentry_rejects_old_and_distant_trajectory(self):
        self.start_node()
        self.trigger()
        self.wait_for(lambda: self.position[2] >= 0.799, "takeoff target")
        self.wait_for(lambda: self.flight_status and
                      self.flight_status[-1].fsm_state == FlightTrajectoryStatus.FSM_GUIDED and
                      self.flight_status[-1].reference_type == FlightTrajectoryStatus.REF_HOVER,
                      "takeoff completed")
        self.ch6 = 2000
        self.wait_for(lambda: self.flight_status and
                      self.flight_status[-1].fsm_state == FlightTrajectoryStatus.FSM_TRAJ,
                      "first trajectory mode")

        def hold(ident, position):
            trajectory = PolynomialTrajectory(id=ident, heading_mode=PolynomialTrajectory.HAEDING_NONE)
            trajectory.header.stamp = rospy.Time.now()
            trajectory.xyz_pieces = [Polynomial(order=5, dim=3, duration=10.0,
                                                coeff=[0.0] * 15 + position[:])]
            return trajectory

        old_position = self.position[:]
        self.pubs["trajectory"].publish(hold(100, old_position))
        self.wait_for(lambda: self.flight_status and self.flight_status[-1].trajectory_active and
                      self.flight_status[-1].trajectory_id == 100, "first trajectory active")
        self.ch6 = 1000
        self.wait_for(lambda: self.flight_status and
                      self.flight_status[-1].fsm_state == FlightTrajectoryStatus.FSM_GUIDED,
                      "return to guided")
        # Move the reference and the simulated vehicle in GUIDED mode.
        self.throttle = 1500
        time.sleep(0.3)
        self.pitch = 1000
        self.wait_for(lambda: self.position[0] > old_position[0] + 0.8,
                      "manual motion away from old hold", timeout=7)
        self.pitch = 1500
        self.wait_for(lambda: self.flight_status and
                      self.flight_status[-1].reference_type == FlightTrajectoryStatus.REF_HOVER,
                      "manual movement settled")
        self.ch6 = 2000
        self.wait_for(lambda: self.flight_status and
                      self.flight_status[-1].fsm_state == FlightTrajectoryStatus.FSM_TRAJ,
                      "second trajectory mode")
        # A cached trajectory and a freshly stamped old hold must both be rejected.
        self.pubs["trajectory"].publish(hold(101, old_position))
        time.sleep(0.3)
        self.assertFalse(self.flight_status[-1].trajectory_active)
        self.assertEqual("GUIDED_NOGPS", self.mode)
        self.assertGreater(self.references[-1].position_reference.x, old_position[0] + 0.8)
        wrong_heading = hold(103, self.position)
        wrong_heading.heading_mode = PolynomialTrajectory.HEADING_POLY
        wrong_heading.heading_pieces = [Polynomial(order=3, dim=1, duration=10.0,
                                                    coeff=[0.0, 0.0, 0.0, math.pi / 2])]
        self.pubs["trajectory"].publish(wrong_heading)
        time.sleep(0.2)
        self.assertFalse(self.flight_status[-1].trajectory_active,
                         "a stale heading must not be accepted at the current position")
        wrong_velocity = hold(104, self.position)
        wrong_velocity.xyz_pieces[0].coeff[12] = 0.5
        self.pubs["trajectory"].publish(wrong_velocity)
        time.sleep(0.2)
        self.assertFalse(self.flight_status[-1].trajectory_active,
                         "a trajectory with a large velocity jump must not be accepted")
        self.pubs["trajectory"].publish(hold(102, self.position))
        self.wait_for(lambda: self.flight_status and self.flight_status[-1].trajectory_active and
                      self.flight_status[-1].trajectory_id == 102, "current trajectory active")

    def test_disarmed_rejected(self):
        self.armed = False
        self.start_node()
        self.trigger()
        time.sleep(0.5)
        self.assertFalse(self.mode_requests)
        self.assertFalse(self.commands)

    def test_l1_continues_from_takeoff_through_hover_and_landed_state_jitter(self):
        # Reproduce the bag's 1 Hz state stream with an interval that crosses
        # the 1 s freshness threshold. It must not reset the active observer.
        self.start_node(controller_overrides={"acc_compensation": 2, "alpha_compensation": 2})
        self.extended_period = 1.05
        self.trigger()
        self.wait_ascent()
        self.wait_for(lambda: self.runtime and
                      float(self.runtime_values()["l1_initialization_count"]) == 1.0,
                      "L1 starts with the first takeoff control output")
        self.assertEqual(float(self.runtime_values()["l1_enabled"]), 1.0)
        self.wait_for(lambda: self.position[2] >= self.origin[2] + 0.199,
                      "finish ascent with continuous L1")
        ascent = [r for r in self.references if r.velocity_reference.z > 0.01]
        self.assertTrue(ascent, "observe takeoff reference samples")
        self.assertTrue(any(abs(r.external_force_l1.z) > 1e-6 for r in ascent),
                        "L1 compensation is active during ascent")
        time.sleep(2.3)  # Cross multiple slow landed-state publication intervals.
        self.assertEqual(self.mode, "GUIDED_NOGPS")
        self.assertAlmostEqual(self.position[2], self.origin[2] + 0.2, places=3)
        output_count = len(self.commands)
        self.publish_extended = False
        time.sleep(1.2)
        self.assertEqual(self.mode, "GUIDED_NOGPS")
        self.assertGreater(len(self.commands) - output_count, 50,
                           "landed-state loss must not interrupt control output")
        # Even a changed land-detector report cannot reset an ongoing session.
        self.follow = False
        self.landed = ExtendedState.LANDED_STATE_ON_GROUND
        self.extended_period = 0.0
        self.publish_extended = True
        time.sleep(0.3)
        samples = [dict((v.key, v.value) for v in msg.status[0].values) for msg in self.runtime]
        active = [v for v in samples if float(v.get("l1_initialization_count", 0)) > 0]
        self.assertTrue(active)
        self.assertTrue(all(float(v["l1_enabled"]) == 1.0 for v in active))
        self.assertTrue(all(float(v["l1_initialization_count"]) == 1.0 for v in active),
                        "takeoff, hover, stale state and ground reports must preserve observer state")

    def assert_upright_fallback_during_takeover(self):
        self.wait_for(lambda: self.runtime and
                      float(self.runtime_values().get("l1_initialization_count", 0)) > 0,
                      "observer initialization diagnostic")
        initializations = float(self.runtime_values()["l1_initialization_count"])
        l1_selected = float(self.runtime_values()["l1_enabled"])
        self.wait_for(lambda: "ALT_HOLD" in self.mode_requests, "unsafe thrust takeover")
        self.wait_for(lambda: self.runtime and
                      float(self.runtime_values()["attitude_safety_handover"]) == 1.0,
                      "latched attitude safety diagnostic")
        start = len(self.commands)
        time.sleep(0.25)
        fallback = self.commands[start:]
        self.assertGreater(len(fallback), 10, "continue bounded output until confirmation")
        self.assertTrue(all(1 - 2 * (c.orientation.x ** 2 + c.orientation.y ** 2) > 0.999
                            for c in fallback), "fallback stays upright")
        self.assertTrue(all(abs(c.thrust - 0.28) < 0.002 for c in fallback),
                        "fallback uses hover thrust, not the 0.089 crash command")
        self.assertEqual(float(self.runtime_values()["l1_enabled"]), l1_selected)
        self.assertEqual(float(self.runtime_values()["l1_initialization_count"]), initializations,
                         "safe takeover output must not reset the in-flight L1 observer")
        self.assertTrue(all(1 - 2 * (c.orientation.x ** 2 + c.orientation.y ** 2) > 0
                            for c in self.commands), "never publish an inverted target")
        tilt_limit = math.radians(rospy.get_param("/flygripper/robot_parameter/tilt_angle_max_deg"))
        self.assertTrue(all(1 - 2 * (c.orientation.x ** 2 + c.orientation.y ** 2) >=
                            math.cos(tilt_limit) - 1e-9 for c in self.commands),
                        "every transmitted FCU target respects the configured tilt limit")
        self.wait_for(lambda: self.mode == "ALT_HOLD", "delayed takeover confirmed")
        self.assert_stopped()

    def test_l1_negative_vertical_thrust_triggers_upright_takeover(self):
        self.follow = False
        self.landed = ExtendedState.LANDED_STATE_IN_AIR
        # Keep the injected fault independent of the flight tuning in YAML.
        self.start_node(controller_overrides={"acc_compensation": 2, "k_vel/z": 8.367})
        self.trigger()
        self.wait_for(lambda: len(self.commands) > 30, "airborne hover control")
        self.mode_delay = 0.8
        # At fixed position this upward velocity step makes L1 subtract more
        # than the remaining nominal upward acceleration, as in the crash bag.
        self.velocity[2] = 0.8
        self.assert_upright_fallback_during_takeover()
        self.assertIn("Unsafe thrust", self.runtime_values()["last_handover_reason"])

    def test_inverted_frame_alignment_triggers_upright_takeover(self):
        self.follow = False
        self.start_node()
        self.trigger()
        self.wait_for(lambda: len(self.commands) > 30, "takeoff control")
        self.mode_delay = 0.8
        self.imu_roll = math.pi
        self.assert_upright_fallback_during_takeover()
        self.assertIn("Unsafe FCU attitude", self.runtime_values()["last_handover_reason"])

    def test_excessive_non_inverted_frame_tilt_triggers_upright_takeover(self):
        self.follow = False
        self.start_node()
        self.trigger()
        self.wait_for(lambda: len(self.commands) > 30, "takeoff control")
        self.mode_delay = 0.8
        self.imu_roll = math.radians(60)  # Upright, but above the configured 30 degrees.
        self.assert_upright_fallback_during_takeover()
        self.assertIn("tilt limit exceeded", self.runtime_values()["last_handover_reason"])

    def test_tilt_limit_degree_dynamic_reconfigure(self):
        self.follow = False
        self.start_node()
        client = Client("/flygripper/robot_parameter", timeout=5)
        self.assertAlmostEqual(client.get_configuration()["tilt_angle_max_deg"], 30.0)
        client.update_configuration({"tilt_angle_max_deg": 45.0})
        self.trigger()
        self.wait_for(lambda: len(self.commands) > 30, "takeoff control")
        self.imu_roll = math.radians(40)
        self.wait_for(lambda: any(1 - 2 * (c.orientation.x ** 2 + c.orientation.y ** 2)
                                  < math.cos(math.radians(35)) for c in self.commands),
                      "40-degree target allowed by 45-degree limit")
        self.assertEqual("GUIDED_NOGPS", self.mode)
        client.update_configuration({"tilt_angle_max_deg": 30.0})
        self.wait_for(lambda: "ALT_HOLD" in self.mode_requests,
                      "30-degree limit triggers takeover for 40-degree target")

    def test_wrong_thrust_semantics_rejected(self):
        self.guid_options = 0
        self.start_node()
        self.trigger()
        time.sleep(0.5)
        self.assertFalse(self.mode_requests)
        self.assertFalse(self.commands)

    def assert_guided_entry_rejected(self):
        self.trigger()
        self.wait_for(lambda: self.mode == "ALT_HOLD", "unverified GUIDED entry handed back")
        self.assertGreater(len(self.param_requests), 0, "entry must check GUID_OPTIONS")
        self.assertEqual(len(self.commands), 0, "unverified entry must never output thrust")
        self.assertEqual(len(self.references), 0, "unverified entry must not start takeoff")
        self.assert_stopped()

    def test_existing_guided_wrong_thrust_semantics_rejected(self):
        self.mode = "GUIDED_NOGPS"
        self.guid_options = 0
        # Keep the takeover RPC in flight to also cover unverified HANDOVER output.
        self.mode_delay = 0.3
        self.start_node()
        self.assert_guided_entry_rejected()

    def test_existing_guided_param_failure_rejected(self):
        self.mode = "GUIDED_NOGPS"
        self.param_success = False
        self.start_node()
        self.assert_guided_entry_rejected()

    def test_existing_guided_waits_for_slow_parameter_verification(self):
        self.mode = "GUIDED_NOGPS"
        self.param_delay = 0.8
        self.start_node({"mode_switch_timeout": 2.0})
        self.trigger()
        self.wait_for(lambda: bool(self.param_requests), "parameter verification started")
        time.sleep(0.3)
        self.assertEqual(len(self.commands), 0, "no thrust before verification completes")
        self.assertEqual(len(self.references), 0, "no takeoff before verification completes")
        self.wait_ascent()

    def test_existing_guided_rechecks_parameters_for_each_session(self):
        self.mode = "GUIDED_NOGPS"
        self.start_node()
        self.trigger()
        self.wait_ascent()
        self.ch5 = 1000
        self.wait_for(lambda: self.mode == "ALT_HOLD", "first session handed back")
        self.assert_stopped()
        self.commands.clear()
        self.references.clear()
        self.param_requests.clear()
        self.guid_options = 0
        self.mode = "GUIDED_NOGPS"
        time.sleep(0.15)
        self.assert_guided_entry_rejected()

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

    def test_small_timesync_future_offset_is_allowed(self):
        self.imu_time_offset = 0.005
        self.start_node()
        self.trigger()
        self.wait_ascent()
        time.sleep(0.2)
        self.assertEqual(self.mode, "GUIDED_NOGPS")

    def test_large_future_offset_requests_takeover(self):
        self.start_node()
        self.trigger()
        self.wait_ascent()
        self.imu_time_offset = 0.5
        self.wait_for(lambda: self.mode == "ALT_HOLD", "large future IMU clock offset")
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

    def runtime_values(self):
        return dict((v.key, v.value) for v in self.runtime[-1].status[0].values)

    def test_input_timing_reports_each_stage(self):
        self.start_node()
        self.wait_for(lambda: self.runtime and
                      float(self.runtime_values().get("rc_callbacks", 0)) > 5,
                      "segmented input timing published")
        values = self.runtime_values()
        for name in ("imu", "odom", "rc", "fc"):
            with self.subTest(input=name):
                v = lambda suffix: float(values[name + "_" + suffix])
                self.assertGreater(v("accepted_updates"), 0)
                self.assertEqual(v("rejected_updates"), 0)
                self.assertGreater(v("receipt_ros_s"), 0)
                self.assertAlmostEqual((v("callback_ros_s") - v("receipt_ros_s")) * 1000,
                                       v("queue_delay_ms"), delta=0.003)
                self.assertGreaterEqual(v("lock_acquired_steady_s"), v("callback_steady_s"))
                self.assertGreaterEqual(v("saved_steady_s"), v("lock_acquired_steady_s"))
                self.assertAlmostEqual((v("saved_steady_s") - v("lock_acquired_steady_s")) * 1000,
                                       v("store_ms"), delta=0.003)
                for stage in ("queue_delay", "lock_wait", "store"):
                    self.assertGreaterEqual(v(stage + "_max_ms"), v(stage + "_ms"))
        self.assertAlmostEqual(float(values["input_snapshot_ms"]),
                               float(values["input_lock_copy_ms"]) + float(values["input_decode_ms"]),
                               delta=0.003)

    def test_short_ch5_release_is_not_overwritten_by_high(self):
        self.start_node()
        self.trigger()
        self.wait_ascent()
        self.publish_rc = False
        time.sleep(0.03)  # Drain any publisher iteration already in progress.
        for ch5 in (1000, 2000):
            rc = RCIn(channels=[1500, 1500, 1000, 1500, ch5, 1000,
                                1000, 1000, 1500, 1000])
            rc.header.stamp = rospy.Time.now()
            self.pubs["rc"].publish(rc)
        self.publish_rc = True  # Continue with CH5 high, after the brief release.
        self.wait_for(lambda: self.mode == "ALT_HOLD", "brief CH5 release triggers handover")
        self.wait_for(lambda: self.runtime and
                      "CH5 released" in self.runtime_values()["last_handover_reason"],
                      "handover caused by retained RC edge")
        self.assert_stopped()

    def test_slow_mode_service_does_not_block_inputs_or_control(self):
        self.start_node()
        self.trigger()
        self.wait_ascent()
        self.mode_delay = 2.0
        self.ch5 = 1000
        self.wait_for(lambda: "ALT_HOLD" in self.mode_requests, "handover request started")
        self.wait_for(lambda: self.runtime and
                      float(self.runtime_values()["mode_request_busy"]) == 1,
                      "worker busy diagnostic")
        before = float(self.runtime_values()["imu_callbacks"])
        diag_count = len(self.runtime)
        time.sleep(0.7)
        values = self.runtime_values()
        self.assertGreater(len(self.runtime) - diag_count, 3,
                           "control diagnostics must continue during slow RPC")
        self.assertGreater(float(values["imu_callbacks"]) - before, 25)
        self.assertLess(float(values["imu_callback_gap_ms"]), 150)
        self.assertLess(float(values["control_work_max_ms"]), 500)
        self.assertGreater(float(values["mode_request_elapsed_ms"]), 500)
        self.wait_for(lambda: self.mode == "ALT_HOLD", "delayed mode confirmation")
        self.assert_stopped()

    def test_late_guided_after_cancel_does_not_start_takeoff(self):
        self.mode_delay = 1.2
        self.start_node({"mode_switch_timeout": 0.4})
        self.trigger()
        self.wait_for(lambda: "GUIDED_NOGPS" in self.mode_requests, "guided RPC started")
        self.ch5 = 1000
        self.wait_for(lambda: self.mode == "ALT_HOLD", "late guided handed back", timeout=5)
        self.assertTrue(all(r.position_reference.z <= self.origin[2] + 0.001
                            for r in self.references),
                        "late GUIDED must not start a cancelled takeoff")
        self.wait_for(lambda: self.runtime and
                      float(self.runtime_values()["mode_request_ignored"]) >= 1,
                      "cancelled result ignored")

    def test_imu_callback_stall_is_distinct_from_old_source_stamp(self):
        self.start_node()
        self.trigger()
        self.wait_ascent()
        self.publish_imu = False
        self.wait_for(lambda: self.mode == "ALT_HOLD", "missing IMU takeover")
        self.wait_for(lambda: self.runtime and
                      float(self.runtime_values()["imu_callback_gap_ms"]) > 150,
                      "monotonic IMU callback age")
        self.assert_stopped()

    def test_recent_callbacks_with_old_imu_stamp_still_fail(self):
        self.start_node()
        self.trigger()
        self.wait_ascent()
        self.imu_time_offset = -1.0
        self.wait_for(lambda: self.mode == "ALT_HOLD", "old IMU timestamp takeover")
        self.wait_for(lambda: self.runtime and
                      float(self.runtime_values()["imu_header_age_ms"]) > 900,
                      "old header age diagnostic")
        self.assertLess(float(self.runtime_values()["imu_callback_gap_ms"]), 150)
        self.assertGreater(float(self.runtime_values()["imu_age_at_callback_ms"]), 900)
        self.assert_stopped()

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
        artifact_root = os.environ.get("HIARM_TEST_ARTIFACTS")
        if artifact_root:
            os.makedirs(artifact_root, exist_ok=True)
            prefix = os.path.join(artifact_root, self._testMethodName)
            self.log.flush()
            self.log.seek(0)
            with open(prefix + ".log", "w") as output:
                output.write(self.log.read())
            with open(prefix + ".json", "w") as output:
                json.dump({
                    "mock_publish_ros_s": self.publish_times,
                    "runtime": [{"stamp": msg.header.stamp.to_sec(),
                                 "values": {v.key: v.value for v in msg.status[0].values}}
                                for msg in self.runtime if msg.status],
                }, output)
        self.log.close()
        self.directory.cleanup()


if __name__ == "__main__":
    rospy.init_node("auto_takeoff_contract_test")
    rostest.rosrun("flygripper_flight", "auto_takeoff_contract", AutoTakeoffTest)
