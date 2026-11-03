#!/usr/bin/env python3
"""Audit the current CH6 -> restored thrust -> ALT_HOLD implementation.

These tests assert recovery, takeover, bounded stale-feedback handling and pilot
priority using the real node with isolated ROS mocks; no physical hardware.
"""
import json
import os
import time
import unittest

import rospy
import rostest
from mavros_msgs.msg import RCIn
from mavros_msgs.srv import SetModeResponse
from flygripper_msgs.msg import PerchingCommand, PerchingStatus as S, FlightTrajectoryStatus as F
from test_perching import PerchingTest as P


class GatePublisher:
    def __init__(self, publisher):
        self.publisher, self.enabled = publisher, True

    def publish(self, msg):
        if self.enabled:
            self.publisher.publish(msg)

    def unregister(self):
        self.publisher.unregister()


class Ch6ExitAudit(unittest.TestCase):
    publish = P.publish
    gripper = P.gripper
    start_node = P.start_node
    reference_callback = P.reference_callback
    get_param = P.get_param
    wait_for = P.wait_for
    trigger = P.trigger
    request = P.request
    send = P.send
    phase = P.phase
    prepare = P.prepare
    enter_trajectory_mode = P.enter_trajectory_mode

    def setUp(self):
        self.audit_events = []
        self.audit_results = []
        self.mode_reply = True
        P.setUp(self)
        self.state_gate = GatePublisher(self.pubs["state"])
        self.pubs["state"] = self.state_gate

    def set_mode(self, request):
        self.mode_requests.append(request.custom_mode)
        self.audit_events.append(dict(event="mode_request", mode=request.custom_mode,
                                      ros=rospy.Time.now().to_sec()))
        time.sleep(self.mode_delay)
        if self.mode_reply and self.confirm_modes:
            self.mode = request.custom_mode
            self.audit_events.append(dict(event="mock_mode_confirm", mode=self.mode,
                                          ros=rospy.Time.now().to_sec()))
        return SetModeResponse(mode_sent=self.mode_reply)

    def reenter(self):
        self.ch5 = self.ch6 = 1000
        self.mode_delay = 0.0
        self.mode = "STABILIZE"
        self.velocity = [0.0] * 3
        self.latest_reference = None
        self.wait_for(lambda: self.flight_status[-1].fsm_state == F.FSM_MANUAL, "manual between cases")
        time.sleep(0.15)
        self.ch5 = 2000
        self.wait_for(lambda: self.flight_status[-1].fsm_state == F.FSM_GUIDED, "new guided session")
        self.ch6 = 2000
        self.wait_for(lambda: self.flight_status[-1].fsm_state == F.FSM_TRAJ, "new trajectory session")
        time.sleep(0.05)

    def reach(self, phase, mission=1):
        request = self.request(mission_id=mission, off=5.0, timeout=12.0)
        request.use_actuator_feedback = False  # matches current real_perching.yaml
        request.gripper_open_duration = 0.4
        request.retreat_duration = 1.0
        self.prepare(request)
        request.command = PerchingCommand.START
        self.send(request)
        self.wait_for(lambda: self.phase(phase, mission), "requested perching phase", timeout=10)
        return request

    def exit_and_measure(self, phase, delay=0.25, confirm=True, reply=True):
        self.mode_delay, self.confirm_modes, self.mode_reply = delay, confirm, reply
        before_gripper = len(self.gripper_commands)
        sent = rospy.Time.now().to_sec()
        self.audit_events.append(dict(event="ch6_release", ros=sent, phase=phase))
        self.ch6 = 1000
        self.wait_for(lambda: self.statuses[-1][1].phase == S.ABORTED, "CH6 cancels the active mission")
        self.wait_for(lambda: any(e["event"] == "mode_request" and e["mode"] == "ALT_HOLD" and
                                 e["ros"] >= sent for e in self.audit_events), "ALT_HOLD requested")
        saved = self.statuses[-1][1].saved_throttle
        threshold = 0.0 if phase == S.RETREAT else saved - 0.001
        self.wait_for(lambda: any(c.header.stamp.to_sec() >= sent and c.thrust > threshold for c in self.commands),
                      "positive thrust restored before confirmation")
        positive = next(c for c in self.commands if c.header.stamp.to_sec() >= sent and c.thrust > threshold)
        rpc = next(e for e in self.audit_events if e["event"] == "mode_request" and
                   e["mode"] == "ALT_HOLD" and e["ros"] >= sent)
        result = dict(phase=phase, exit_ros=sent, first_positive_ros=positive.header.stamp.to_sec(),
                      recovery_ms=1000*(positive.header.stamp.to_sec()-sent),
                      first_positive_throttle=positive.thrust,
                      saved_throttle=self.statuses[-1][1].saved_throttle,
                      mode_request_ros=rpc["ros"], delay=delay, confirm=confirm, reply=reply)
        self.audit_results.append(result)
        self.assertLess(result["recovery_ms"], 150.0)
        if phase != S.RETREAT:
            self.assertGreaterEqual(positive.thrust, result["saved_throttle"] - 0.001)
            self.assertLessEqual(positive.header.stamp.to_sec(), rpc["ros"])
        if confirm and reply:
            self.wait_for(lambda: self.mode == "ALT_HOLD", "mode confirmed", timeout=4)
        else:
            time.sleep(1.25)
            self.assertEqual(self.mode, "GUIDED_NOGPS")
            self.assertEqual(self.flight_status[-1].fsm_state, F.FSM_HANDOVER)
        after = [c for c in self.commands if c.header.stamp.to_sec() >= positive.header.stamp.to_sec()]
        self.assertTrue(all(c.thrust > 0 for c in after), "motor-stop commands must not resume")
        stamps = [c.header.stamp.to_sec() for c in after]
        result["max_output_gap_ms"] = max([1000*(b-a) for a, b in zip(stamps, stamps[1:])] or [0])
        result["outputs_after_restore"] = len(after)
        self.assertLess(result["max_output_gap_ms"], 150.0)
        self.assertFalse(any(c.header.stamp.to_sec() > sent + 0.05 for _, c in
                             self.gripper_commands[before_gripper:]), "no new opening after cancellation")

    def test_all_active_phases_exit_before_alt_hold_confirmation(self):
        self.enter_trajectory_mode()
        phases = [S.UNLOADING, S.SUPPORT_VERIFY, S.FINAL_UNLOAD, S.MOTORS_OFF,
                  S.SPINUP, S.OPENING, S.RETREAT]
        for mission, phase in enumerate(phases, 1):
            with self.subTest(phase=phase):
                if mission > 1:
                    self.reenter()
                self.reach(phase, mission)
                self.exit_and_measure(phase)

    def test_zero_output_exit_with_one_second_mode_service_delay(self):
        self.enter_trajectory_mode()
        self.reach(S.MOTORS_OFF)
        self.exit_and_measure(S.MOTORS_OFF, delay=1.0)

    def test_zero_output_exit_mode_service_rejected(self):
        self.enter_trajectory_mode()
        self.reach(S.MOTORS_OFF)
        self.exit_and_measure(S.MOTORS_OFF, confirm=False, reply=False)

    def test_zero_output_exit_mode_sent_without_confirmation(self):
        self.enter_trajectory_mode()
        self.reach(S.MOTORS_OFF)
        self.exit_and_measure(S.MOTORS_OFF, confirm=False)

    def test_short_ch6_release_retained_in_rc_history(self):
        self.enter_trajectory_mode()
        self.reach(S.MOTORS_OFF)
        self.publish_rc = False
        time.sleep(0.03)
        sent = rospy.Time.now().to_sec()
        for ch6 in (1000, 2000):
            rc = RCIn(channels=[1500, 1500, 1500, 1500, 2000, ch6, 1000, 1000, 1500, 1000])
            rc.header.stamp = rospy.Time.now()
            self.pubs["rc"].publish(rc)
        self.publish_rc = True
        time.sleep(0.25)
        cancelled = self.statuses[-1][1].phase == S.ABORTED
        after = [c for c in self.commands if c.header.stamp.to_sec() > sent + 0.05]
        self.audit_results.append(dict(case="short_ch6_release", cancelled=cancelled,
                                       zero_commands_after_pulse=sum(c.thrust == 0 for c in after)))
        self.assertTrue(cancelled, "CH6 low/high retained in one RC batch must cancel motor-stop")

    def test_short_rc_stream_during_zero_output_restores_thrust(self):
        self.enter_trajectory_mode()
        self.reach(S.MOTORS_OFF)
        self.mode_delay = 0.25
        self.publish_rc = False
        time.sleep(0.03)
        sent = rospy.Time.now().to_sec()
        deadline = time.monotonic() + 0.4
        while time.monotonic() < deadline:
            rc = RCIn(channels=[1500, 1500, 1500, 1500, 1000, 1000])
            rc.header.stamp = rospy.Time.now()
            self.pubs["rc"].publish(rc)
            time.sleep(0.01)
        self.wait_for(lambda: self.statuses[-1][1].phase == S.ABORTED, "short RC cancels zero output")
        self.assertIn("Malformed RC frame", self.statuses[-1][1].detail)
        self.wait_for(lambda: self.mode == "ALT_HOLD", "invalid RC requests FCU takeover")
        saved = self.statuses[-1][1].saved_throttle
        restored = next(c for c in self.commands if c.header.stamp.to_sec() >= sent and c.thrust > 0)
        rpc = next(e for e in self.audit_events if e["event"] == "mode_request" and
                   e["mode"] == "ALT_HOLD" and e["ros"] >= sent)
        self.assertLess(restored.header.stamp.to_sec() - sent, 0.15)
        self.assertGreaterEqual(restored.thrust, saved - 0.001)
        self.assertLessEqual(restored.header.stamp.to_sec(), rpc["ros"])
        self.assertTrue(all(c.thrust > 0 for c in self.commands
                            if c.header.stamp.to_sec() > sent + 0.05))
        self.assertFalse(self.gripper_commands)

    def test_invalid_rc_followed_by_valid_high_cannot_hide_fault(self):
        self.enter_trajectory_mode()
        valid = [1500, 1500, 1500, 1500, 2000, 2000, 1000, 1000, 1500, 1000]
        invalid = [[], valid[:6], [3000] + valid[1:]]
        for mission, channels in enumerate(invalid, 1):
            with self.subTest(channels=channels):
                if mission > 1:
                    self.reenter()
                self.reach(S.MOTORS_OFF, mission)
                self.publish_rc = False
                time.sleep(0.03)
                # A normal high frame immediately after the bad frame must not
                # erase the fault retained while consuming the RC event batch.
                for values in (channels, valid):
                    rc = RCIn(channels=values)
                    rc.header.stamp = rospy.Time.now()
                    self.pubs["rc"].publish(rc)
                self.publish_rc = True
                self.wait_for(lambda: self.statuses[-1][1].phase == S.ABORTED,
                              "invalid frame retained despite following valid high")
                self.assertIn("Malformed RC frame", self.statuses[-1][1].detail)
                self.wait_for(lambda: self.flight_status[-1].fsm_state == F.FSM_MANUAL,
                              "FCU takeover confirmed")
                time.sleep(0.15)
                self.assertEqual(self.flight_status[-1].fsm_state, F.FSM_MANUAL,
                                 "high switches must not automatically resume autonomy")
                self.assertFalse(self.gripper_commands)

    def test_fc_heartbeat_loss_then_ch6_exit_restores_thrust(self):
        self.enter_trajectory_mode()
        self.reach(S.MOTORS_OFF)
        sent = rospy.Time.now().to_sec()
        self.state_gate.enabled = False
        self.mode_reply = self.confirm_modes = False
        time.sleep(3.3)  # Production fc_state_timeout is 3 s; other inputs remain fresh.
        self.ch6 = 1000
        time.sleep(0.25)
        after = [c for c in self.commands if c.header.stamp.to_sec() >= sent]
        recovered = any(c.thrust > 0 for c in after)
        self.audit_results.append(dict(case="fc_heartbeat_loss", recovered=recovered,
                                       output_count=len(after), positive_count=sum(c.thrust > 0 for c in after),
                                       phase=self.statuses[-1][1].phase))
        self.assertTrue(recovered, "stale FCU heartbeat must not leave the last command at zero thrust")

    def test_stale_feedback_recovery_is_bounded(self):
        self.enter_trajectory_mode()
        self.reach(S.MOTORS_OFF)
        self.state_gate.enabled = False
        self.mode_reply = self.confirm_modes = False
        self.wait_for(lambda: self.statuses[-1][1].phase == S.ABORTED, "heartbeat expiry abort", timeout=5)
        time.sleep(0.8)  # this fixture's recovery_duration is 0.5 s
        count = len(self.commands)
        time.sleep(0.25)
        self.assertEqual(count, len(self.commands), "never stream blind thrust indefinitely")
        self.assertGreater(self.commands[-1].thrust, 0)

    def test_new_external_mode_wins_during_stale_feedback_recovery(self):
        self.enter_trajectory_mode()
        self.reach(S.MOTORS_OFF)
        self.state_gate.enabled = False
        self.mode_reply = self.confirm_modes = False
        self.wait_for(lambda: self.statuses[-1][1].phase == S.ABORTED, "heartbeat expiry abort", timeout=5)
        self.mode = "LAND"
        self.state_gate.enabled = True
        self.wait_for(lambda: self.flight_status[-1].fsm_state == F.FSM_MANUAL, "pilot LAND takes priority")
        time.sleep(0.1)
        count = len(self.commands)
        time.sleep(0.2)
        self.assertEqual(count, len(self.commands))
        self.assertEqual(self.mode, "LAND")

    def test_generic_ch6_pulse_returns_to_hover_without_reentry(self):
        self.enter_trajectory_mode()
        self.publish_rc = False
        time.sleep(0.03)
        for ch6 in (1000, 2000):
            rc = RCIn(channels=[1500, 1500, 1500, 1500, 2000, ch6, 1000, 1000, 1500, 1000])
            rc.header.stamp = rospy.Time.now()
            self.pubs["rc"].publish(rc)
        self.publish_rc = True
        self.wait_for(lambda: self.flight_status[-1].fsm_state == F.FSM_GUIDED, "retained CH6 exit")
        time.sleep(0.25)
        self.assertEqual(self.flight_status[-1].fsm_state, F.FSM_GUIDED)
        self.assertEqual(self.mode, "GUIDED_NOGPS")

    def tearDown(self):
        root = os.environ.get("HIARM_TEST_ARTIFACTS")
        P.tearDown(self)
        if root:
            with open(os.path.join(root, self._testMethodName + "-audit.json"), "w") as output:
                json.dump(dict(events=self.audit_events, results=self.audit_results), output, indent=2)


if __name__ == "__main__":
    rospy.init_node("perching_ch6_exit_audit")
    rostest.rosrun("flygripper_flight", "perching_ch6_exit_audit", Ch6ExitAudit)
