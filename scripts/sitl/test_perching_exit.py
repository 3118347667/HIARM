#!/usr/bin/env python3
"""Measured ArduCopter/MAVROS validation of perching cancellation.

Ground is an explicit support fixture for the zero-thrust case, not a simulated
gripper. Airborne tests use unmodified quad dynamics and measured IMU/odometry.
Only the heartbeat-forwarding case withholds /mavros/state from the flight node.
"""
import argparse
import json
import os
from pathlib import Path
import socket
import struct
import threading
import time

import rospy
import rosparam
from mavros_msgs.msg import AttitudeTarget, RCOut, State
from mavros_msgs.srv import CommandLong, CommandTOL
from flygripper_msgs.msg import PerchingCommand, PerchingStatus, FlightTrajectoryStatus
from test_auto_takeoff import SitlTest
from pymavlink import mavutil


class PerchingExitTest(SitlTest):
    def __init__(self, output, workspace, case):
        super().__init__(output, workspace)
        self.case = case
        self.forward_state = True
        self.perching = None
        self.flight_status = None
        self.perching_history, self.fc_history, self.rotor_outputs, self.targets = [], [], [], []
        self.pending_flight_args = None
        self.report.update(case=case, support_fixture="physical SITL ground" if case in ("ground", "heartbeat") else "none",
                           sensor_override=False, reference_follower=False)

    def spawn(self, name, args, cwd=None):
        if name == "flight":
            rospy.set_param('/flygripper/flight_sitl/auto_takeoff', False)
            if self.case in ("ground", "heartbeat"):
                # Stock quad hover calibration is ~0.39. A lower command keeps
                # the grounded fixture from lifting off during preparation.
                rospy.set_param('/flygripper/robot_parameter/hover_percentage', 0.30)
                self.report['simulation_overrides']['robot_parameter/hover_percentage'] = 0.30
            self.state_pub = rospy.Publisher('/perching_validation/state', State, queue_size=10)
            self.subs.append(rospy.Subscriber('/mavros/state', State, self.forward_fc_state))
            args = args + ['/mavros/state:=/perching_validation/state']
            if self.case == 'airborne':
                # Establish the flight fixture with the native FCU controller.
                # This test concerns cancellation, not ground/spool-up takeoff.
                self.pending_flight_args = args
                return None
        return super().spawn(name, args, cwd)

    def forward_fc_state(self, msg):
        self.fc_history.append(dict(t=time.time(), mode=msg.mode, armed=msg.armed,
                                    forwarded=self.forward_state))
        if self.forward_state:
            self.state_pub.publish(msg)

    def rotor_callback(self, msg):
        self.rotor_outputs.append(dict(t=time.time(), channels=list(msg.channels[:4]), mode=self.state.mode))

    def perching_callback(self, msg):
        self.perching = msg
        self.perching_history.append(dict(t=time.time(), phase=msg.phase,
                                         saved=msg.saved_throttle, output=msg.output_throttle,
                                         detail=msg.detail))

    def rc_loop(self):
        heartbeat = 0
        while self.running:
            try:
                if time.monotonic() - heartbeat > 1:
                    self.link.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS,
                                                mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)
                    heartbeat = time.monotonic()
                self.rc_socket.sendto(struct.pack('<16H', *(self.channels + [1500]*6)), ('127.0.0.1', 5501))
                while True:
                    msg = self.link.recv_match(blocking=False)
                    if msg is None:
                        break
                    if msg.get_type() == 'AUTOPILOT_VERSION':
                        version = msg.flight_sw_version
                        self.report['running_firmware_version'] = '.'.join(str(v) for v in
                            [(version >> 24) & 255, (version >> 16) & 255, (version >> 8) & 255])
                    elif msg.get_type() == 'ATTITUDE_TARGET':
                        self.targets.append(dict(t=time.time(), thrust=msg.thrust,
                                                 time_boot_ms=msg.time_boot_ms))
            except Exception as error:
                self.events.append(dict(time=time.time(), rc_error=str(error)))
            time.sleep(0.02)

    def send(self, command):
        command.header.stamp = rospy.Time.now()
        self.perching_pub.publish(command)

    def establish_airborne_hover(self):
        self.channels[2] = 1000
        self.channels[4] = 1000
        self.mode(custom_mode='GUIDED')
        self.wait(lambda: self.state.mode == 'GUIDED', 'native GUIDED mode')
        deadline = time.monotonic() + 30
        while time.monotonic() < deadline and not self.state.armed:
            self.arm(value=True)
            time.sleep(1)
        self.wait(lambda: self.state.armed, 'normal native arming')
        origin = self.position[:]
        rospy.wait_for_service('/mavros/cmd/takeoff', 10)
        response = rospy.ServiceProxy('/mavros/cmd/takeoff', CommandTOL)(altitude=2.0)
        if not response.success:
            raise RuntimeError('Native SITL takeoff rejected')
        self.wait(lambda: self.position[2] > origin[2] + 1.8 and
                  abs(self.velocity.twist.linear.z) < 0.1, 'measured native 2 m hover', 35)
        time.sleep(2)
        calibrated = [t['thrust'] for t in self.targets if t['t'] > time.time()-1.5 and 0.1 < t['thrust'] < 0.9]
        if len(calibrated) < 10:
            raise RuntimeError('Insufficient FCU hover-thrust telemetry for the stock quad fixture')
        hover = sum(calibrated)/len(calibrated)
        self.report['simulation_overrides']['robot_parameter/hover_percentage'] = hover
        self.report['preparation'] = 'Native FCU 2 m takeoff; hover throttle measured from FCU ATTITUDE_TARGET'
        rospy.set_param('/flygripper/robot_parameter/hover_percentage', hover)
        # The stock SITL quad is not the 0.62 kg HIARM propulsion/inner loop.
        # Use explicit low-bandwidth outer-loop gains for this plant. Retain L1,
        # all freshness/tilt/arming checks, and the production perching guards.
        gains = {'k_pos': {'x': 1.0, 'y': 1.0, 'z': 2.0},
                 'k_vel': {'x': 2.0, 'y': 2.0, 'z': 3.0}}
        for key, value in gains.items():
            rospy.set_param('/flygripper/geometric_controller/' + key, value)
            self.report['simulation_overrides']['geometric_controller/' + key] = value
        self.channels[2] = 1500
        self.mode(custom_mode='ALT_HOLD')
        self.wait(lambda: self.state.mode == 'ALT_HOLD', 'native altitude hold')
        super().spawn('flight', self.pending_flight_args)
        self.wait(lambda: self.flight_status is not None and self.flight_status.fsm_state ==
                  FlightTrajectoryStatus.FSM_MANUAL, 'project flight node ready')
        # A rising edge before the first healthy FCU-state callback is correctly
        # consumed by the flight safety gate; establish health while CH5 is low.
        self.wait(lambda: self.flight_status.fcu_state_fresh and self.flight_status.fcu_armed,
                  'project FCU input health established')
        time.sleep(0.2)
        self.channels[4] = 2000
        self.wait(lambda: self.state.mode == 'GUIDED_NOGPS', 'project verified guided entry')
        self.wait(lambda: self.flight_status.fsm_state == FlightTrajectoryStatus.FSM_GUIDED,
                  'project hover reference')
        time.sleep(1)
        self.log('Measured native hover throttle %.4f; project controller now owns airborne hover.' % hover)
        return origin

    def run(self):
        self.setup()
        self.perching_pub = rospy.Publisher('/flygripper/perching_command', PerchingCommand, queue_size=10)
        self.subs.extend([
            rospy.Subscriber('/flygripper/perching_status', PerchingStatus, self.perching_callback),
            rospy.Subscriber('/flygripper/flight_trajectory_status', FlightTrajectoryStatus,
                             lambda m: setattr(self, 'flight_status', m)),
            rospy.Subscriber('/mavros/rc/out', RCOut, self.rotor_callback),
            rospy.Subscriber('/mavros/setpoint_raw/target_attitude', AttitudeTarget,
                             lambda m: self.targets.append(dict(t=time.time(), thrust=m.thrust,
                                                                 source="FCU ATTITUDE_TARGET via MAVROS"))),
        ])
        interval = rospy.ServiceProxy('/mavros/cmd/command', CommandLong)
        interval(command=511, param1=83, param2=50000)
        interval(command=511, param1=36, param2=20000)
        start = time.time()
        if self.case == "airborne":
            origin = self.establish_airborne_hover()
        else:
            origin = self.begin_takeoff()
        time.sleep(1)
        self.channels[2] = 1500
        self.channels[5] = 2000
        self.wait(lambda: self.flight_status is not None and self.flight_status.fsm_state ==
                  FlightTrajectoryStatus.FSM_TRAJ, 'CH6 trajectory mode')
        cfg = rosparam.load_file(str(self.workspace / 'src/flygripper_planner/params/real_perching.yaml'))[0][0]
        command = PerchingCommand(mission_id=time.time_ns(), command=PerchingCommand.PREPARE,
                                  closed_position=cfg['morph_shrink_pos'], open_position=cfg['morph_expand_pos'],
                                  closed_tolerance=1.0, open_tolerance=0.1, use_actuator_feedback=False)
        for key, value in cfg['perching'].items():
            if hasattr(command, key):
                setattr(command, key, value)
        self.wait(lambda: self.perching_pub.get_num_connections() > 0, 'perching subscriber')
        self.send(command)
        self.wait(lambda: self.perching is not None and self.perching.phase == PerchingStatus.PREPARED,
                  'saved powered command', 8)
        self.report['saved_throttle'] = self.perching.saved_throttle
        command.command = PerchingCommand.START
        self.send(command)
        phase = PerchingStatus.UNLOADING if self.case == 'airborne' else PerchingStatus.MOTORS_OFF
        self.wait(lambda: self.perching.phase == phase, 'perching test phase', 12)
        if self.case == 'ground':
            time.sleep(3.0)  # exercise FCU landed/spool logic after several seconds of zero input
        before_z = self.position[2]
        exit_at = time.time()
        if self.case == 'heartbeat':
            self.forward_state = False
            self.log('Withholding only FCU-state feedback from the project node; real MAVROS link stays live.')
        else:
            self.channels[5] = 1000
            self.log('CH6 low on the native SITL RC receiver.')
        self.wait(lambda: self.perching.phase == PerchingStatus.ABORTED, 'perching cancellation', 6)
        self.wait(lambda: self.state.mode == 'ALT_HOLD', 'actual ArduPilot ALT_HOLD confirmation', 6)
        time.sleep(1.0)
        self.forward_state = True
        after = [c for c in self.commands if c['t'] >= exit_at and c['thrust'] > 0]
        if not after:
            raise RuntimeError('No positive recovery command')
        confirmed = next(e['t'] for e in self.fc_history if e['t'] >= exit_at and e['mode'] == 'ALT_HOLD')
        targets = [t for t in self.targets if exit_at <= t['t'] <= confirmed + 1]
        outputs = [o for o in self.rotor_outputs if exit_at <= o['t'] <= confirmed + 1]
        heights = [s['position'][2] for s in self.samples if exit_at <= s['t'] <= confirmed + 1]
        result = dict(name=self.case, passed=False, exit_at=exit_at,
                      first_positive_command_delay_s=after[0]['t']-exit_at,
                      actual_mode_confirmation_delay_s=confirmed-exit_at,
                      maximum_height_loss_m=before_z-min(heights) if heights else None,
                      fcu_target_count=len(targets), fcu_positive_targets=sum(t['thrust'] > 0 for t in targets),
                      motor_output_count=len(outputs),
                      motor_pwm_min=min(min(o['channels']) for o in outputs) if outputs else None,
                      motor_pwm_max=max(max(o['channels']) for o in outputs) if outputs else None,
                      armed_after_exit=self.state.armed,
                      landed_after_exit=self.extended.landed_state)
        self.report['cases'].append(result)
        if not self.state.armed or not targets or not outputs:
            raise RuntimeError('FCU did not retain armed state or return target/motor telemetry')
        if self.case == 'airborne' and not any(t['thrust'] > 0 for t in targets):
            raise RuntimeError('Airborne FCU did not report positive thrust target')
        if self.case != 'airborne':
            result['ground_limit'] = 'FCU may remain LANDED with idle motor output; not proof of airborne lift recovery'
        if self.case == 'airborne' and result['maximum_height_loss_m'] > 0.20:
            raise RuntimeError('Airborne recovery height loss exceeded 0.20 m')
        result['passed'] = True
        self.log('PASS ' + json.dumps(result))
        self.land()
        self.report['success'] = True

    def close(self):
        super().close()
        for name, data in [('perching', self.perching_history), ('fc_state', self.fc_history),
                           ('motor_outputs', self.rotor_outputs), ('fcu_targets', self.targets)]:
            (self.output / (name + '.json')).write_text(json.dumps(data))


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--case', choices=['ground', 'airborne', 'heartbeat'], required=True)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    test = PerchingExitTest(args.output.resolve(), Path(__file__).resolve().parents[2], args.case)
    try:
        test.run()
    except Exception as error:
        test.report['error'] = str(error)
        test.log('FAIL: ' + str(error))
        raise
    finally:
        test.close()
