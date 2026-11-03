#!/usr/bin/env python3
"""Real planner/flight ROS contract tests; mocks do not model motors or support forces."""
import math
import json
import os
import signal
import subprocess
import time
import unittest

import rosparam
import rospkg
import rospy
import rostest
from flygripper_msgs.msg import (ActuatorCommand, ActuatorState, PerchingCommand,
                                PerchingStatus, FlightTrajectoryStatus, Polynomial,
                                PolynomialTrajectory)
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import ExtendedState, RCIn, State
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from test_auto_takeoff import AutoTakeoffTest as Harness


class PerchingTest(unittest.TestCase):
    # Reuse the isolated flight-node lifecycle, not its takeoff test cases.
    start_node = Harness.start_node
    set_mode = Harness.set_mode
    get_param = Harness.get_param
    reference_callback = Harness.reference_callback
    wait_for = Harness.wait_for
    trigger = Harness.trigger

    def setUp(self):
        self.yaw = 0.9
        self.imu_acc_z = 9.81
        self.odom_age = 0.0
        self.actuator_position = 0.0
        self.actuator_fresh = True
        self.actuator_follow = True
        self.statuses = []
        self.gripper_commands = []
        self.planner = None
        Harness.setUp(self)
        self.pubs["actuator"] = rospy.Publisher("/flygripper/actuator_state", ActuatorState, queue_size=10)
        self.pubs["perching"] = rospy.Publisher("/flygripper/perching_command", PerchingCommand, queue_size=10)
        self.subs.extend([
            rospy.Subscriber("/flygripper/perching_status", PerchingStatus,
                             lambda msg: self.statuses.append((time.monotonic(), msg))),
            rospy.Subscriber("/flygripper/actuator_command", ActuatorCommand, self.gripper),
        ])

    def gripper(self, msg):
        self.gripper_commands.append((time.monotonic(), msg))
        if self.actuator_follow:
            self.actuator_position = msg.reference_position

    def publish(self):
        while self.running and not rospy.is_shutdown():
            stamp = rospy.Time.now()
            if self.follow and self.latest_reference is not None and self.mode == "GUIDED_NOGPS":
                r = self.latest_reference
                self.position = [r.position_reference.x, r.position_reference.y, r.position_reference.z]
                self.velocity = [r.velocity_reference.x, r.velocity_reference.y, r.velocity_reference.z]
            state = State(connected=True, armed=self.armed, guided=self.mode == "GUIDED_NOGPS",
                          mode=self.mode, system_status=4)
            state.header.stamp = stamp
            self.pubs["state"].publish(state)
            imu = Imu()
            imu.header.stamp = stamp
            imu.orientation.z, imu.orientation.w = math.sin(self.yaw/2), math.cos(self.yaw/2)
            imu.linear_acceleration.z = self.imu_acc_z
            if self.publish_imu:
                self.pubs["imu"].publish(imu)
            odom = Odometry()
            odom.header.stamp = stamp - rospy.Duration(self.odom_age)
            odom.pose.pose.orientation = imu.orientation
            odom.pose.pose.position.x, odom.pose.pose.position.y, odom.pose.pose.position.z = self.position
            odom.twist.twist.linear.x, odom.twist.twist.linear.y, odom.twist.twist.linear.z = self.velocity
            if self.publish_odom:
                self.pubs["odom"].publish(odom)
            rc = RCIn(channels=[1500, 1500, 1500, 1500, self.ch5, self.ch6, 1000, 1000, 1500, 1000])
            rc.header.stamp = stamp
            if self.publish_rc:
                self.pubs["rc"].publish(rc)
            if "actuator" in self.pubs and self.actuator_fresh:
                actuator = ActuatorState(position=self.actuator_position)
                actuator.header.stamp = stamp
                self.pubs["actuator"].publish(actuator)
            time.sleep(0.01)

    def enter_trajectory_mode(self):
        self.start_node({"auto_takeoff": False})
        self.trigger()
        self.wait_for(lambda: self.mode == "GUIDED_NOGPS" and self.references, "guided hover")
        self.ch6 = 2000
        self.wait_for(lambda: self.flight_status and self.flight_status[-1].fsm_state ==
                      FlightTrajectoryStatus.FSM_TRAJ, "trajectory mode")
        time.sleep(0.1)

    def request(self, mission_id=1, off=0.3, timeout=4.0):
        return PerchingCommand(mission_id=mission_id, command=PerchingCommand.PREPARE,
                               closed_position=-11.0, open_position=0.0,
                               closed_tolerance=1.0, open_tolerance=0.1,
                               motors_off_duration=off, spinup_duration=0.5,
                               recovery_duration=0.5, retreat_distance=0.3,
                               retreat_duration=0.7, settle_duration=0.1,
                               max_settle_speed=0.1, position_tolerance=0.03, timeout=timeout,
                               use_actuator_feedback=True, gripper_open_duration=0.2,
                               unload_duration=0.3, support_verify_duration=0.2,
                               final_unload_duration=0.2, test_throttle_ratio=0.15,
                               max_height_drop=0.02, max_downward_speed=0.08,
                               min_vertical_acceleration=-1.5, sensor_max_age=0.08)

    def send(self, command):
        command.header.stamp = rospy.Time.now()
        self.pubs["perching"].publish(command)

    def phase(self, phase, mission_id=1):
        return any(s.phase == phase and s.mission_id == mission_id for _, s in self.statuses)

    def prepare(self, command):
        self.send(command)
        self.wait_for(lambda: self.phase(PerchingStatus.PREPARED, command.mission_id), "prepare accepted")

    def start_perching(self, command):
        self.actuator_position = -11.0
        command.command = PerchingCommand.START
        self.send(command)
        self.wait_for(lambda: self.phase(PerchingStatus.MOTORS_OFF, command.mission_id), "zero-thrust phase")

    def test_real_planner_five_second_stop_restore_open_and_body_backward(self):
        self.start_node({"auto_takeoff": False})
        self.trigger()
        self.wait_for(lambda: self.mode == "GUIDED_NOGPS" and self.references, "guided hover")
        planner_root = rospkg.RosPack().get_path("flygripper_planner")
        for values, _ in rosparam.load_file(os.path.join(planner_root, "params/real_perching.yaml")):
            # Keep the production 5 s motor-off and 1 s spinup durations. Move
            # only the staging/approach points into this isolated mock world.
            staging = [self.origin[0] + 0.12*math.cos(self.yaw),
                       self.origin[1] + 0.12*math.sin(self.yaw), self.origin[2]]
            values["task_0"]["plan_point_0"] = dict(zip(["x", "y", "z"], staging), yaw=self.yaw)
            target = [self.origin[0] + 0.4*math.cos(self.yaw),
                      self.origin[1] + 0.4*math.sin(self.yaw), self.origin[2]]
            values["task_1"]["plan_point_0"] = dict(zip(["x", "y", "z"], target))
            values["task_1"]["plan_tolerance"] = 0.03
            rosparam.upload_params("/flygripper/perching_planner", values)
        self.planner = subprocess.Popen([
            "rosrun", "flygripper_planner", "real", "__ns:=/flygripper", "__name:=perching_planner",
            "odom:=/ekf/ekf_odom", "trigger:=/move_base_simple/goal"],
            stdout=self.log, stderr=subprocess.STDOUT)
        self.wait_for(lambda: self.pubs["rc"].get_num_connections() >= 2, "planner startup")
        time.sleep(0.2)
        self.ch6 = 2000
        self.wait_for(lambda: self.phase(PerchingStatus.MOTORS_OFF,
                      self.statuses[-1][1].mission_id), "planner starts perching", timeout=20)
        mission = self.statuses[-1][1].mission_id
        off_time = next(t for t, s in self.statuses if s.phase == PerchingStatus.MOTORS_OFF)
        before = self.position[:]
        time.sleep(0.4)
        self.send(PerchingCommand(mission_id=mission, command=PerchingCommand.START))
        self.wait_for(lambda: self.phase(PerchingStatus.COMPLETE, mission), "complete reflight", timeout=15)
        spinup = next((t, s) for t, s in self.statuses if s.phase == PerchingStatus.SPINUP)
        self.assertGreaterEqual(spinup[0] - off_time, 4.9)
        self.assertLess(spinup[0] - off_time, 5.25)
        opening = next(t for t, s in self.statuses if s.phase == PerchingStatus.OPENING)
        self.assertGreaterEqual(opening - spinup[0], 0.95)
        zero_commands = [c for c in self.commands if c.thrust == 0.0]
        self.assertGreater(len(zero_commands), 100)
        zero_start, zero_end = zero_commands[0].header.stamp.to_sec(), zero_commands[-1].header.stamp.to_sec()
        self.assertGreaterEqual(zero_end - zero_start, 4.95)
        restored = next(c for c in self.commands if c.header.stamp.to_sec() > zero_end and c.thrust > 0)
        self.assertAlmostEqual(restored.thrust, spinup[1].saved_throttle, places=5)
        first_retreat = next(s for _, s in self.statuses if s.phase == PerchingStatus.RETREAT)
        self.assertGreaterEqual(first_retreat.output_throttle, spinup[1].saved_throttle - 0.002)
        for axis, expected in enumerate([-0.5*math.cos(self.yaw), -0.5*math.sin(self.yaw), 0.0]):
            self.assertAlmostEqual(self.position[axis] - before[axis], expected, places=3)
        self.assertAlmostEqual(self.references[-1].attitude_desired.z,
                               math.sin(self.yaw/2), places=3)
        # Replaying START for a completed mission must never stop the motors again.
        self.send(PerchingCommand(mission_id=mission, command=PerchingCommand.START))
        count = len(zero_commands)
        time.sleep(0.3)
        self.assertEqual(count, len([c for c in self.commands if c.thrust == 0.0]))

    def test_stale_closed_feedback_cannot_stop_motors(self):
        self.enter_trajectory_mode()
        request = self.request()
        self.prepare(request)
        self.actuator_position = -11.0
        self.actuator_fresh = False
        time.sleep(0.2)
        request.command = PerchingCommand.START
        self.send(request)
        time.sleep(0.4)
        self.assertFalse(any(c.thrust == 0 for c in self.commands))
        self.assertFalse(self.phase(PerchingStatus.MOTORS_OFF))

    def test_ch5_cancel_and_repeated_start_cannot_restart(self):
        self.enter_trajectory_mode()
        request = self.request(off=1.0)
        self.prepare(request)
        self.start_perching(request)
        self.ch5 = 1000
        self.wait_for(lambda: self.mode == "ALT_HOLD", "pilot takeover")
        self.wait_for(lambda: self.phase(PerchingStatus.ABORTED), "abort status delivered")
        self.send(request)
        time.sleep(1.5)
        self.assertFalse(self.gripper_commands)
        self.assertFalse(self.phase(PerchingStatus.SPINUP))

    def test_missing_open_feedback_never_retreats_and_times_out(self):
        self.enter_trajectory_mode()
        request = self.request(timeout=3.5)
        self.prepare(request)
        self.actuator_follow = False
        self.start_perching(request)
        self.wait_for(lambda: self.phase(PerchingStatus.OPENING), "opening phase")
        self.wait_for(lambda: self.phase(PerchingStatus.ABORTED), "open feedback timeout", timeout=4)
        self.assertFalse(self.phase(PerchingStatus.RETREAT))
        self.assertTrue(self.gripper_commands)
        self.assertTrue(all(c.reference_position == 0 for _, c in self.gripper_commands))

    def test_actuator_loss_during_stop_keeps_gripper_closed_and_aborts(self):
        self.enter_trajectory_mode()
        request = self.request(off=1.0)
        self.prepare(request)
        self.start_perching(request)
        self.actuator_fresh = False
        self.wait_for(lambda: self.phase(PerchingStatus.ABORTED), "stale actuator abort")
        self.assertFalse(self.gripper_commands)
        self.assertFalse(self.phase(PerchingStatus.OPENING))

    def test_fcu_disarm_aborts_without_rearming_or_opening(self):
        self.enter_trajectory_mode()
        request = self.request(off=1.0)
        self.prepare(request)
        self.start_perching(request)
        self.armed = False
        self.wait_for(lambda: self.phase(PerchingStatus.ABORTED), "FCU disarm abort")
        count = len(self.commands)
        self.send(request)
        time.sleep(0.3)
        self.assertLessEqual(len(self.commands) - count, 1)
        self.assertFalse(self.gripper_commands)
        self.assertFalse(self.phase(PerchingStatus.SPINUP))

    def test_failed_grasp_height_drop_restores_hover_without_motor_stop(self):
        self.enter_trajectory_mode()
        request = self.request()
        request.use_actuator_feedback = False
        self.prepare(request)
        request.command = PerchingCommand.START
        self.send(request)
        self.wait_for(lambda: self.phase(PerchingStatus.UNLOADING), "unloading starts without trusting servo")
        saved = self.statuses[-1][1].saved_throttle
        self.follow = False
        self.position[2] -= 0.025  # unsupported vehicle begins losing altitude
        self.wait_for(lambda: self.phase(PerchingStatus.ABORTED), "failed support test")
        self.assertFalse(self.phase(PerchingStatus.MOTORS_OFF))
        self.assertFalse(any(c.thrust == 0 for c in self.commands))
        self.assertGreaterEqual(self.commands[-1].thrust, saved - 0.001)
        self.assertEqual(self.mode, "GUIDED_NOGPS", "support rejection holds powered hover")
        self.follow = True
        self.wait_for(lambda: abs(self.position[2] - self.origin[2]) < 0.001, "original hover altitude restored")
        self.assertFalse(self.gripper_commands)

    def test_downward_velocity_rejects_support_before_height_drop(self):
        self.enter_trajectory_mode()
        request = self.request()
        request.use_actuator_feedback = False
        self.prepare(request)
        request.command = PerchingCommand.START
        self.send(request)
        self.wait_for(lambda: self.phase(PerchingStatus.UNLOADING), "unloading starts")
        self.follow = False
        self.velocity[2] = -0.1
        self.wait_for(lambda: self.phase(PerchingStatus.ABORTED), "fall velocity rejection")
        self.assertFalse(any(c.thrust == 0 for c in self.commands))

    def test_downward_acceleration_rejects_support_before_motion(self):
        self.enter_trajectory_mode()
        request = self.request()
        request.use_actuator_feedback = False
        self.prepare(request)
        request.command = PerchingCommand.START
        self.send(request)
        self.wait_for(lambda: self.phase(PerchingStatus.UNLOADING), "unloading starts")
        self.imu_acc_z = 8.0  # unsupported proper acceleration: world az < -1.5
        self.wait_for(lambda: self.phase(PerchingStatus.ABORTED), "fall acceleration rejection")
        self.assertFalse(any(c.thrust == 0 for c in self.commands))

    def test_old_odometry_with_recent_callbacks_rejects_unloading(self):
        self.enter_trajectory_mode()
        request = self.request()
        request.use_actuator_feedback = False
        self.prepare(request)
        request.command = PerchingCommand.START
        self.send(request)
        self.wait_for(lambda: self.phase(PerchingStatus.UNLOADING), "unloading starts")
        self.odom_age = 0.09  # below normal flight timeout, above support-test limit
        self.wait_for(lambda: self.phase(PerchingStatus.ABORTED), "support-test measurement-age rejection")
        self.assertFalse(any(c.thrust == 0 for c in self.commands))

    def test_planner_cancel_during_zero_output_restores_before_takeover(self):
        self.enter_trajectory_mode()
        request = self.request(off=1.0)
        self.prepare(request)
        self.start_perching(request)
        request.command = PerchingCommand.CANCEL
        sent_at = rospy.Time.now().to_sec()
        self.send(request)
        self.wait_for(lambda: self.phase(PerchingStatus.ABORTED), "planner cancellation")
        restored = [c for c in self.commands if c.header.stamp.to_sec() >= sent_at and c.thrust > 0]
        self.assertTrue(restored)
        self.assertGreaterEqual(restored[0].thrust, self.statuses[-1][1].saved_throttle - 0.001)
        self.assertFalse(self.gripper_commands)

    def start_approach_planner(self):
        self.start_node({"auto_takeoff": False})
        self.trigger()
        self.wait_for(lambda: self.mode == "GUIDED_NOGPS" and self.references, "guided hover")
        planner_root = rospkg.RosPack().get_path("flygripper_planner")
        for values, _ in rosparam.load_file(os.path.join(planner_root, "params/real_perching.yaml")):
            staging = [self.origin[0] + 0.12*math.cos(self.yaw),
                       self.origin[1] + 0.12*math.sin(self.yaw), self.origin[2]]
            target = [self.origin[0] + 1.0*math.cos(self.yaw),
                      self.origin[1] + 1.0*math.sin(self.yaw), self.origin[2]]
            values["task_0"]["plan_point_0"] = dict(zip(["x", "y", "z"], staging), yaw=self.yaw)
            values["task_1"]["plan_point_0"] = dict(zip(["x", "y", "z"], target))
            rosparam.upload_params("/flygripper/perching_planner", values)
        self.planner = subprocess.Popen([
            "rosrun", "flygripper_planner", "real", "__ns:=/flygripper", "__name:=perching_planner",
            "odom:=/ekf/ekf_odom", "trigger:=/move_base_simple/goal"],
            stdout=self.log, stderr=subprocess.STDOUT)
        self.wait_for(lambda: self.pubs["rc"].get_num_connections() >= 2, "planner startup")
        time.sleep(0.2)
        self.ch6 = 2000
        self.wait_for(lambda: any(s.phase == PerchingStatus.PREPARED for _, s in self.statuses),
                      "approach prepared", timeout=15)
        mission = next(s.mission_id for _, s in reversed(self.statuses) if s.phase == PerchingStatus.PREPARED)
        self.wait_for(lambda: self.flight_status and self.flight_status[-1].trajectory_active and
                      self.position[0] > staging[0] + 0.02, "final approach executing", timeout=5)
        return mission, target

    def test_planner_ch6_exit_during_approach_never_closes(self):
        mission, target = self.start_approach_planner()
        self.follow = False
        self.ch6 = 1000
        self.wait_for(lambda: self.phase(PerchingStatus.ABORTED, mission), "prepared mission aborted")
        self.position = target
        time.sleep(0.7)
        self.assertFalse(any(c.reference_position < -1 for _, c in self.gripper_commands))

    def test_planner_cancel_during_approach_never_closes(self):
        mission, target = self.start_approach_planner()
        self.follow = False
        self.send(PerchingCommand(mission_id=mission, command=PerchingCommand.CANCEL))
        self.wait_for(lambda: self.phase(PerchingStatus.ABORTED, mission), "prepared mission cancelled")
        self.wait_for(lambda: self.flight_status[-1].fsm_state == FlightTrajectoryStatus.FSM_GUIDED and
                      not self.flight_status[-1].trajectory_active, "cancelled approach stops in hover")
        self.wait_for(lambda: abs(self.references[-1].velocity_reference.x) < 1e-6 and
                      abs(self.references[-1].velocity_reference.y) < 1e-6,
                      "approach velocity reference revoked")
        hold = self.references[-1].position_reference
        hold_position = [hold.x, hold.y, hold.z]
        self.position = target
        time.sleep(0.7)
        current = self.references[-1].position_reference
        for actual, expected in zip([current.x, current.y, current.z], hold_position):
            self.assertAlmostEqual(actual, expected, places=6, msg="cancelled reference must stop advancing")
        self.assertFalse(any(c.reference_position < -1 for _, c in self.gripper_commands))
        # Keep the planner from starting another mission on the deliberate new
        # CH6 edge below; exercise the flight node's trajectory ownership gate.
        self.planner.send_signal(signal.SIGINT)
        self.planner.wait(timeout=5)
        self.position, self.velocity = hold_position, [0.0] * 3
        self.assert_perching_requires_new_trigger()
        self.send(PerchingCommand(mission_id=mission, command=PerchingCommand.CANCEL))
        time.sleep(0.1)
        self.assertTrue(self.flight_status[-1].trajectory_active,
                        "replayed cancellation must not stop a later trajectory session")

    def test_prepared_timeout_revokes_approach_trajectory(self):
        self.enter_trajectory_mode()
        self.wait_for(lambda: self.pubs["trajectory"].get_num_connections() > 0,
                      "trajectory publisher connected before timeout fixture")
        request = self.request(timeout=2.8)
        self.prepare(request)
        trajectory = PolynomialTrajectory(id=212, heading_mode=PolynomialTrajectory.HAEDING_NONE)
        trajectory.header.stamp = rospy.Time.now()
        trajectory.xyz_pieces = [Polynomial(order=5, dim=3, duration=10.0,
                                            coeff=[0.0]*15 + self.position[:])]
        self.pubs["trajectory"].publish(trajectory)
        self.wait_for(lambda: self.flight_status[-1].trajectory_active, "prepared trajectory sampled")
        self.wait_for(lambda: self.phase(PerchingStatus.ABORTED), "prepared mission expired")
        self.wait_for(lambda: self.flight_status[-1].fsm_state == FlightTrajectoryStatus.FSM_GUIDED,
                      "expired approach leaves trajectory mode")
        self.assertFalse(self.flight_status[-1].trajectory_active)
        self.assertFalse(self.gripper_commands)

    def test_planner_stale_status_during_approach_never_closes(self):
        _, target = self.start_approach_planner()
        self.follow = False
        self.process.send_signal(signal.SIGSTOP)
        try:
            time.sleep(0.8)  # Both status streams are now older than the 0.5 s limit.
            self.position = target
            time.sleep(0.5)
            self.assertFalse(any(c.reference_position < -1 for _, c in self.gripper_commands))
        finally:
            self.process.send_signal(signal.SIGCONT)

    def test_planner_other_trajectory_during_approach_never_closes(self):
        _, target = self.start_approach_planner()
        self.follow = False
        trajectory = PolynomialTrajectory(id=211, heading_mode=PolynomialTrajectory.HAEDING_NONE)
        trajectory.header.stamp = rospy.Time.now()
        trajectory.xyz_pieces = [Polynomial(order=5, dim=3, duration=10.0,
                                            coeff=[0.0]*15 + self.position[:])]
        self.pubs["trajectory"].publish(trajectory)
        self.wait_for(lambda: any(s.trajectory_active and s.trajectory_id == 211 and
                                 s.trajectory_start == trajectory.header.stamp for s in self.flight_status),
                      "other trajectory sampled before cancellation")
        self.wait_for(lambda: self.flight_status[-1].fsm_state == FlightTrajectoryStatus.FSM_GUIDED and
                      not self.flight_status[-1].trajectory_active, "replacement cancels the approach")
        time.sleep(0.2)
        self.position = target
        time.sleep(0.5)
        self.assertFalse(any(c.reference_position < -1 for _, c in self.gripper_commands))

    def assert_perching_requires_new_trigger(self):
        self.wait_for(lambda: self.flight_status and
                      self.flight_status[-1].fsm_state == FlightTrajectoryStatus.FSM_GUIDED,
                      "perching exit holds in guided")

        def hold(ident, pos):
            traj = PolynomialTrajectory(id=ident, heading_mode=PolynomialTrajectory.HAEDING_NONE)
            traj.header.stamp = rospy.Time.now()
            traj.xyz_pieces = [Polynomial(order=5, dim=3, duration=10.0, coeff=[0.0]*15 + pos[:])]
            return traj

        far = self.position[:]
        far[0] += 0.5
        self.pubs["trajectory"].publish(hold(201, far))
        time.sleep(0.2)
        self.assertEqual(self.flight_status[-1].fsm_state, FlightTrajectoryStatus.FSM_GUIDED)
        self.assertFalse(self.flight_status[-1].trajectory_active)
        self.ch6 = 1000
        time.sleep(0.15)
        self.ch6 = 2000
        self.wait_for(lambda: self.flight_status[-1].fsm_state == FlightTrajectoryStatus.FSM_TRAJ,
                      "new CH6 edge enters trajectory mode")
        self.pubs["trajectory"].publish(hold(202, far))
        time.sleep(0.2)
        self.assertFalse(self.flight_status[-1].trajectory_active, "distant first trajectory rejected")
        self.pubs["trajectory"].publish(hold(203, self.position))
        self.wait_for(lambda: self.flight_status[-1].trajectory_active and
                      self.flight_status[-1].trajectory_id == 203, "continuous new trajectory accepted")

    def test_failed_support_requires_new_trajectory_trigger(self):
        self.enter_trajectory_mode()
        request = self.request()
        request.use_actuator_feedback = False
        self.prepare(request)
        request.command = PerchingCommand.START
        self.send(request)
        self.wait_for(lambda: self.phase(PerchingStatus.UNLOADING), "unloading")
        self.imu_acc_z = 8.0
        self.wait_for(lambda: self.phase(PerchingStatus.ABORTED), "support rejected")
        self.imu_acc_z = 9.81
        self.assert_perching_requires_new_trigger()

    def test_completed_perching_requires_new_trajectory_trigger(self):
        self.enter_trajectory_mode()
        request = self.request()
        request.use_actuator_feedback = False
        self.prepare(request)
        request.command = PerchingCommand.START
        self.send(request)
        self.wait_for(lambda: self.phase(PerchingStatus.COMPLETE), "perching completed", timeout=6)
        self.assert_perching_requires_new_trigger()

    def tearDown(self):
        if self.planner is not None and self.planner.poll() is None:
            self.planner.send_signal(signal.SIGINT)
            self.planner.wait(timeout=5)
        root = os.environ.get("HIARM_TEST_ARTIFACTS")
        if root:
            os.makedirs(root, exist_ok=True)
            with open(os.path.join(root, self._testMethodName + "-perching.json"), "w") as output:
                json.dump({"statuses": [{"time": t, "phase": s.phase, "mission_id": s.mission_id,
                                         "saved_throttle": s.saved_throttle,
                                         "output_throttle": s.output_throttle, "detail": s.detail}
                                        for t, s in self.statuses],
                           "commands": [{"time": c.header.stamp.to_sec(), "thrust": c.thrust}
                                        for c in self.commands],
                           "gripper": [{"time": t, "position": c.reference_position}
                                       for t, c in self.gripper_commands]}, output)
        Harness.tearDown(self)


if __name__ == "__main__":
    rospy.init_node("perching_contract_test")
    rostest.rosrun("flygripper_flight", "perching_contract", PerchingTest)
