#!/usr/bin/env python3
"""Closed-loop test: project controller -> real MAVROS -> ArduCopter SITL.

Only SITL's local TCP ports are used. The odometry adapter forwards measured
SITL pose and world-frame velocity; it never copies the controller reference.
"""
import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import signal
import socket
import struct
import subprocess
import threading
import time
import xmlrpc.client

import rosparam
import rospy
from geometry_msgs.msg import PoseStamped, TwistStamped
from mavros_msgs.msg import AttitudeTarget, ExtendedState, RCIn, State
from mavros_msgs.srv import CommandBool, CommandLong, SetMode
from nav_msgs.msg import Odometry, Path as RosPath
from rosgraph_msgs.msg import Log
from sensor_msgs.msg import Imu
# Keep the auxiliary GCS heartbeat link on MAVLink 2.
os.environ['MAVLINK20'] = '1'
from pymavlink import mavutil


class SitlTest:
    def __init__(self, output, workspace):
        self.output = output
        self.workspace = workspace
        self.processes = []
        self.files = []
        self.running = True
        self.bridge_enabled = True
        self.position = None
        self.velocity = None
        self.pose = None
        self.imu = None
        self.state = State()
        self.extended = ExtendedState()
        self.rc_channels = []
        self.channels = [1500, 1500, 1000, 1500, 1000, 1000, 1500, 1000, 1500, 1500]
        self.commands = []
        self.samples = []
        self.events = []
        self.last_reference = None
        self.last_odom_received = 0
        delta = subprocess.check_output(['git', 'diff', '--binary'], cwd=workspace)
        (output / 'project.patch').write_bytes(delta)
        self.report = {"firmware_commit": Path('/opt/ardupilot-build-commit').read_text().strip(),
                       "working_tree_patch_sha256": hashlib.sha256(delta).hexdigest(),
                       "firmware_ref": os.environ['HIARM_COPTER_REF'],
                       "project_commit": subprocess.check_output(
                           ['git', 'rev-parse', 'HEAD'], cwd=workspace, text=True).strip(),
                       "plant": "ArduCopter built-in quad dynamics",
                       "simulation_overrides": {"robot_parameter/hover_percentage": 0.39},
                       "cases": [], "success": False}

    def log(self, text):
        print(text, flush=True)
        self.events.append({"time": time.time(), "message": text})

    def spawn(self, name, args, cwd=None):
        stream = open(self.output / (name + '.log'), 'w')
        self.files.append(stream)
        process = subprocess.Popen(args, cwd=cwd, stdout=stream,
                                   stderr=subprocess.STDOUT, start_new_session=True)
        self.processes.append(process)
        return process

    def wait(self, predicate, description, timeout=30):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if predicate():
                return
            if any(p.poll() is not None for p in self.processes):
                raise RuntimeError('A subprocess exited while waiting for ' + description)
            time.sleep(0.05)
        raise RuntimeError('Timed out: ' + description +
                           f'; mode={self.state.mode}, armed={self.state.armed}, pos={self.position}' +
                           f'; RC={self.rc_channels}; latest sensor={self.samples[-1] if self.samples else None}')

    def pose_callback(self, msg):
        self.pose = msg
        self.position = [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]
        self.last_odom_received = time.monotonic()
        if not self.bridge_enabled or self.velocity is None:
            return
        odom = Odometry()
        odom.header = msg.header
        odom.child_frame_id = 'base_link'
        odom.pose.pose = msg.pose
        # flygripper's Odometry adapter expects velocity in the world frame.
        odom.twist.twist.linear = self.velocity.twist.linear
        self.odom_pub.publish(odom)
        self.samples.append({"t": time.time(), "position": self.position[:],
                             "velocity": [self.velocity.twist.linear.x,
                                          self.velocity.twist.linear.y,
                                          self.velocity.twist.linear.z],
                             "mode": self.state.mode, "armed": self.state.armed,
                             "landed_state": self.extended.landed_state,
                             "pose_age_s": rospy.Time.now().to_sec() - msg.header.stamp.to_sec(),
                             "imu_age_s": (rospy.Time.now().to_sec() - self.imu.header.stamp.to_sec())
                                          if self.imu is not None else None,
                             "imu_quat_norm": math.sqrt(sum(v * v for v in [self.imu.orientation.x,
                                 self.imu.orientation.y, self.imu.orientation.z, self.imu.orientation.w]))
                                 if self.imu is not None else None,
                             "reference": self.last_reference})

    def reference_callback(self, msg):
        if msg.poses:
            p = msg.poses[0].pose.position
            self.last_reference = [p.x, p.y, p.z]

    def ros_log(self, msg):
        if 'flight' in msg.name and (msg.level >= Log.WARN or 'Takeoff' in msg.msg):
            self.events.append({"time": time.time(), "message": msg.msg})

    def rc_loop(self):
        heartbeat_time = 0
        while self.running:
            try:
                if time.monotonic() - heartbeat_time > 1:
                    self.link.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS,
                                                mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)
                    heartbeat_time = time.monotonic()
                # A MAVLink override does not increase the physical receiver's
                # channel count. Feed SITL's native 16-channel receiver instead.
                self.rc_socket.sendto(struct.pack('<16H', *(self.channels + [1500] * 6)),
                                      ('127.0.0.1', 5501))
                while True:
                    msg = self.link.recv_match(blocking=False)
                    if msg is None:
                        break
                    if msg.get_type() == 'AUTOPILOT_VERSION':
                        version = msg.flight_sw_version
                        self.report['running_firmware_version'] = '.'.join(str(v) for v in
                            [(version >> 24) & 255, (version >> 16) & 255, (version >> 8) & 255])
            except Exception as error:
                self.events.append({"time": time.time(), "rc_error": str(error)})
            time.sleep(0.04)

    def setup(self):
        defaults = self.output / 'sitl.parm'
        defaults.write_text('GUID_OPTIONS 9\nGUID_TIMEOUT 3\nFLTMODE_CH 8\n'
                            'FLTMODE1 0\nSERIAL0_PROTOCOL 2\nSERIAL1_PROTOCOL 2\nSIM_RC_CHANCOUNT 16\nSYSID_MYGCS 255\n'
                            'RC_OVERRIDE_TIME 3\nBRD_SAFETY_DEFLT 0\n')
        self.spawn('roscore', ['roscore'])
        master = xmlrpc.client.ServerProxy(os.environ['ROS_MASTER_URI'])
        def master_up():
            try:
                return master.getPid('/sitl_check')[0] == 1
            except OSError:
                return False
        self.wait(master_up, 'ROS master')
        rospy.init_node('hiarm_sitl_test', disable_signals=True)
        self.odom_pub = rospy.Publisher('/ekf/ekf_odom', Odometry, queue_size=5)
        self.subs = [
            rospy.Subscriber('/mavros/state', State, lambda m: setattr(self, 'state', m)),
            rospy.Subscriber('/mavros/extended_state', ExtendedState,
                             lambda m: setattr(self, 'extended', m)),
            rospy.Subscriber('/mavros/rc/in', RCIn,
                             lambda m: setattr(self, 'rc_channels', list(m.channels))),
            rospy.Subscriber('/mavros/local_position/velocity_local', TwistStamped,
                             lambda m: setattr(self, 'velocity', m)),
            rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self.pose_callback),
            rospy.Subscriber('/mavros/imu/data', Imu, lambda m: setattr(self, 'imu', m)),
            rospy.Subscriber('/flygripper/reference', RosPath, self.reference_callback),
            rospy.Subscriber('/mavros/setpoint_raw/attitude', AttitudeTarget,
                             lambda m: self.commands.append({"t": time.time(), "thrust": m.thrust})),
            rospy.Subscriber('/rosout_agg', Log, self.ros_log),
        ]
        default_file = '/opt/ardupilot/Tools/autotest/default_params/copter.parm'
        self.spawn('arducopter', ['/opt/ardupilot/build/sitl/bin/arducopter', '-S',
                   '--model', '+', '--speedup', '1', '--home', '-35.363261,149.165230,584,0',
                   '--defaults', default_file + ',' + str(defaults),
                   '--serial1', 'tcp:5762'], cwd=self.output)
        self.spawn('mavros', ['roslaunch', 'mavros', 'apm.launch',
                              'fcu_url:=tcp://127.0.0.1:5760', 'gcs_url:='])
        self.wait(lambda: self.state.connected, 'real MAVROS/SITL heartbeat', 40)
        self.link = mavutil.mavlink_connection('tcp:127.0.0.1:5762',
                                              source_system=255, source_component=191)
        self.rc_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.link.mav.command_long_send(1, 1, 520, 0, 1, 0, 0, 0, 0, 0, 0)
        self.rc_thread = threading.Thread(target=self.rc_loop, daemon=True)
        self.rc_thread.start()
        rospy.wait_for_service('/mavros/cmd/command', 20)
        interval = rospy.ServiceProxy('/mavros/cmd/command', CommandLong)
        for message, hz in [(27, 100), (30, 100), (31, 100), (32, 50), (65, 30), (245, 5)]:
            response = interval(command=511, param1=message, param2=1e6 / hz)
            if not response.success:
                raise RuntimeError(f'Message interval request failed for {message}: {response}')
        self.wait(lambda: self.position is not None and self.velocity is not None and
                  self.extended.landed_state == ExtendedState.LANDED_STATE_ON_GROUND,
                  'SITL odometry and landed state', 60)
        self.wait(lambda: len(self.rc_channels) >= 10,
                  'at least ten actual FCU RC channels from SITL receiver', 10)
        self.wait(lambda: 'running_firmware_version' in self.report, 'firmware version reply', 10)
        expected = self.report['firmware_ref'].removeprefix('Copter-')
        if self.report['running_firmware_version'] != expected:
            raise RuntimeError('Running SITL firmware version does not match requested release')
        self.arm = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
        self.mode = rospy.ServiceProxy('/mavros/set_mode', SetMode)
        for folder in ['src/flygripper_flight', 'src/flygripper_controller']:
            for values, _ in rosparam.load_file(str(self.workspace / folder / 'params/flygripper.yaml')):
                rosparam.upload_params('/flygripper', values)
        # The built-in stock quad has a different propulsion model from HIARM.
        rospy.set_param('/flygripper/robot_parameter/hover_percentage', 0.39)
        for values, _ in rosparam.load_file(str(self.workspace / 'src/flygripper_flight/params/fsm.yaml')):
            rosparam.upload_params('/flygripper/flight_sitl', values)
        self.takeoff_height = rospy.get_param('/flygripper/flight_sitl/takeoff_height')
        self.spawn('flight', ['rosrun', 'flygripper_flight', 'flygripper_flight_node',
                              '__ns:=/flygripper', '__name:=flight_sitl',
                              'odom:=/ekf/ekf_odom', 'imu:=/mavros/imu/data'])
        self.log('SITL and MAVROS connected; waiting for estimator/pre-arm readiness.')
        time.sleep(2)

    def begin_takeoff(self):
        self.channels[2] = 1000
        self.channels[4] = 1000
        self.bridge_enabled = True
        self.mode(custom_mode='STABILIZE')
        self.wait(lambda: self.state.mode == 'STABILIZE', 'STABILIZE confirmation')
        time.sleep(1)
        deadline = time.monotonic() + 30
        while time.monotonic() < deadline:
            if self.arm(value=True).success:
                break
            time.sleep(2)
        self.wait(lambda: self.state.armed, 'normal arming (no force)', 5)
        origin = self.position[:]
        self.channels[4] = 2000
        self.wait(lambda: self.state.mode == 'GUIDED_NOGPS', 'CH5 GUIDED_NOGPS', 8)
        self.log(f'Automatic takeoff started from measured position {origin}.')
        return origin

    def land(self):
        self.channels[4] = 1000
        self.channels[2] = 1500
        self.mode(custom_mode='LAND')
        self.wait(lambda: self.state.mode == 'LAND', 'LAND confirmation')
        self.wait(lambda: self.extended.landed_state == ExtendedState.LANDED_STATE_ON_GROUND,
                  'actual SITL landing', 30)
        if self.state.armed:
            if not self.arm(value=False).success:
                raise RuntimeError('Normal disarm rejected after confirmed landing')
        self.wait(lambda: not self.state.armed, 'disarm confirmation')
        self.channels[2] = 1000
        time.sleep(1)

    def run(self):
        self.setup()
        start = time.time()
        origin = self.begin_takeoff()
        self.wait(lambda: any(e['time'] >= start and 'Takeoff complete' in e.get('message', '')
                              for e in self.events), 'closed-loop takeoff completion', 25)
        time.sleep(3)  # Low throttle must not become an immediate descent command.
        positions = [s['position'] for s in self.samples if s['t'] > time.time() - 2]
        errors = [abs(p[2] - origin[2] - self.takeoff_height) for p in positions]
        xy = [math.hypot(p[0] - origin[0], p[1] - origin[1]) for p in positions]
        if not errors or max(errors) > 0.15 or max(xy) > 0.35:
            raise RuntimeError(f'Hover error exceeds limits: z={errors}, xy={xy}')
        if not self.commands or not all(math.isfinite(c['thrust']) and 0 < c['thrust'] <= 1
                                         for c in self.commands):
            raise RuntimeError('Invalid normalized thrust output')
        result = {"name": "takeoff_and_low_throttle_hover", "passed": True,
                  "origin": origin, "max_last_2s_height_error_m": max(errors),
                  "max_last_2s_xy_drift_m": max(xy), "measured_height_m": self.position[2] - origin[2]}
        self.report['cases'].append(result)
        self.log(f'PASS: actual {self.takeoff_height:g} m takeoff and stable hover with low RC throttle.')
        self.channels[2] = 1500
        self.channels[4] = 1000
        self.wait(lambda: self.state.mode == 'ALT_HOLD', 'CH5 ALT_HOLD takeover', 6)
        self.report['cases'].append({"name": "ch5_takeover_after_hover", "passed": True})
        self.land()
        origin = self.begin_takeoff()
        self.wait(lambda: self.position[2] - origin[2] > 0.25, 'second measured liftoff', 12)
        self.channels[2] = 1500
        self.channels[4] = 1000
        self.wait(lambda: self.state.mode == 'ALT_HOLD', 'mid-ascent CH5 takeover', 6)
        self.report['cases'].append({"name": "cancel_during_ascent", "passed": True})
        self.log('PASS: CH5 cancels the second takeoff and APM confirms ALT_HOLD.')
        self.land()
        origin = self.begin_takeoff()
        self.wait(lambda: self.position[2] - origin[2] > 0.25, 'third measured liftoff', 12)
        self.channels[2] = 1500
        self.bridge_enabled = False
        self.wait(lambda: self.state.mode == 'ALT_HOLD', 'odometry-loss takeover', 6)
        self.report['cases'].append({"name": "odometry_loss_takeover", "passed": True})
        self.log('PASS: odometry loss requests and confirms APM ALT_HOLD.')
        self.bridge_enabled = True
        self.land()
        self.report['success'] = True

    def close(self):
        self.running = False
        for process in reversed(self.processes):
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGINT)
        for process in reversed(self.processes):
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait()
        for stream in self.files:
            stream.close()
        (self.output / 'result.json').write_text(json.dumps(self.report, indent=2))
        (self.output / 'events.json').write_text(json.dumps(self.events, indent=2))
        (self.output / 'telemetry.json').write_text(json.dumps(self.samples))
        (self.output / 'commands.json').write_text(json.dumps(self.commands))


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    workspace = Path(__file__).resolve().parents[2]
    test = SitlTest(args.output.resolve(), workspace)
    try:
        test.run()
    except Exception as error:
        test.report['error'] = str(error)
        test.log('FAIL: ' + str(error))
        raise
    finally:
        test.close()
