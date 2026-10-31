#!/usr/bin/env python3
"""Isolated kinematic ROS checks, never starts MAVROS, flight or servo nodes.
Run against the built workspace with its setup.bash sourced. This starts a
private localhost ROS master and the actual grasping.launch. It is not SITL
or physical grasp validation.
"""
import math
from pathlib import Path
import xml.etree.ElementTree as ET
import os
import subprocess
import tempfile
import threading
import time

os.environ['ROS_MASTER_URI'] = 'http://127.0.0.1:11371'
os.environ['ROS_HOSTNAME'] = '127.0.0.1'
os.environ.pop('ROS_IP', None)

import rospy
from geometry_msgs.msg import PoseStamped, TwistStamped
from nav_msgs.msg import Odometry, Path as RosPath
from std_msgs.msg import String, Bool
from rosgraph_msgs.msg import Log
from flygripper_msgs.msg import PolynomialTrajectory, ActuatorCommand, FlightTrajectoryStatus
from quadrotor_msgs.msg import GraspState, ReplanState


def check(value, message):
    if not value:
        raise AssertionError(message)


def polynomial(piece, t, derivative=0):
    n = piece.order
    result = [0.0] * piece.dim
    # Eigen coefficients are column-major: xyz for each descending power.
    for j in range(n + 1):
        power = n-j
        if power < derivative:
            continue
        factor = math.prod(range(power-derivative+1, power+1))
        for axis in range(piece.dim):
            result[axis] += piece.coeff[j*piece.dim+axis] * factor * t**(power-derivative)
    return result


def evaluate(pieces, t, derivative=0):
    for p in pieces:
        if t <= p.duration:
            return polynomial(p, max(0.0,t), derivative)
        t -= p.duration
    return polynomial(pieces[-1], pieces[-1].duration, derivative)


class Rig:
    def __init__(self):
        self.lock = threading.RLock()
        self.traj = None
        self.trajectory_events = []
        self.previous = None
        self.phases = []
        self.commands = []
        self.snapshots = []
        self.continuity = []
        self.yaw_continuity = []
        self.yaw_limits = []
        self.target_on = True
        self.twist_on = True
        self.freeze_twist_stamp = False
        self.twist_stamp = None
        self.twist_override = None
        self.replans = []
        self.pregrasp_targets = []
        self.pregrasp_events = []
        self.grasp_point_modes = []
        self.prediction_paths = []
        self.planner_logs = []
        self.odom_on = True
        self.track_reference = True
        self.force_yaw = None
        self.turn_rate = 0.0
        self.body_yaw_rate_offset = 0.0
        self.running = True
        self.origin = time.monotonic()
        self.car_yaw = math.pi/2
        self.speed = 0.15
        self.target_x_offset = 0.0
        self.velocity_error = 0.0
        self.initial_error = 0.0
        self.position_error = 0.0
        self.lift_height_override = None
        self.flight_status_on = True
        self.flight_status_stamp_offset = 0.0
        self.flight_status_active = True
        self.flight_status_fcu_fresh = True
        self.flight_status_armed = True
        self.flight_status_guided = True
        self.flight_status_fsm = 2
        self.flight_status_reference = 4
        self.flight_status_override = None
        self.target_pub = rospy.Publisher('/vrpn_client_node/HIARM_car/pose',Odometry,queue_size=10)
        self.twist_pub = rospy.Publisher('/vrpn_client_node/HIARM_car/twist',TwistStamped,queue_size=10)
        self.odom_pub = rospy.Publisher('/ekf/ekf_odom',Odometry,queue_size=10)
        self.trigger_pub = rospy.Publisher('/triger',PoseStamped,queue_size=1)
        self.land_trigger_pub = rospy.Publisher('/land_triger',PoseStamped,queue_size=1)
        self.status_pub = rospy.Publisher('/graspState',GraspState,queue_size=1)
        self.flight_status_pub = rospy.Publisher('/flygripper/flight_trajectory_status',FlightTrajectoryStatus,queue_size=10)
        self.subscribers = [rospy.Subscriber('/flygripper/trajectory',PolynomialTrajectory,self.trajectory),
                            rospy.Subscriber('/flygripper/actuator_command',ActuatorCommand,self.command),
                            rospy.Subscriber('/drone0/planning/grasp_phase',String,self.phase),
                            rospy.Subscriber('/drone0/planning/pregrasp_target',Odometry,self.pregrasp),
                            rospy.Subscriber('/drone0/planning/grasp_point_mode',Bool,self.grasp_point_mode),
                            rospy.Subscriber('/drone0/planning/car_predict',RosPath,self.prediction_path),
                            rospy.Subscriber('/rosout',Log,self.planner_log),
                            rospy.Subscriber('/drone0/replanState',ReplanState,self.replan)]
        self.thread = threading.Thread(target=self.loop,daemon=True)
        self.thread.start()

    def trajectory(self,msg):
        with self.lock:
            self.trajectory_events.append((time.monotonic(), msg))
            if self.traj and msg.header.stamp > self.traj.header.stamp:
                t=(msg.header.stamp-self.traj.header.stamp).to_sec()
                if t < sum(p.duration for p in self.traj.xyz_pieces):
                    errors=[]
                    for d in range(3):
                        old=evaluate(self.traj.xyz_pieces,t,d)
                        new=evaluate(msg.xyz_pieces,0,d)
                        errors.append(math.dist(old,new))
                    self.continuity.append(errors)
                    self.yaw_continuity.append([
                        abs(evaluate(self.traj.heading_pieces,t,d)[0]-evaluate(msg.heading_pieces,0,d)[0])
                        for d in (0,1)])
            for piece in msg.heading_pieces:
                ts=[0,piece.duration]
                if abs(piece.coeff[0])>1e-12:
                    vertex=-piece.coeff[1]/(3*piece.coeff[0])
                    if 0<vertex<piece.duration: ts.append(vertex)
                self.yaw_limits.append((max(abs(polynomial(piece,t,1)[0]) for t in ts),
                                        max(abs(polynomial(piece,t,2)[0]) for t in ts)))
            self.previous=self.traj
            self.traj=msg

    def command(self,msg):
        with self.lock:
            self.commands.append((time.monotonic(),msg))

    def replan(self,msg):
        with self.lock:
            v=msg.target.twist.twist.linear
            self.replans.append((time.monotonic(),v.x,v.y,v.z))

    def pregrasp(self,msg):
        with self.lock:
            self.pregrasp_targets.append(msg)
            self.pregrasp_events.append((time.monotonic(),msg))

    def grasp_point_mode(self,msg):
        with self.lock:
            self.grasp_point_modes.append((time.monotonic(),msg.data))

    def prediction_path(self,msg):
        with self.lock:
            self.prediction_paths.append((time.monotonic(),msg))

    def planner_log(self,msg):
        if 'No valid grasp-point or approach trajectory' in msg.msg:
            with self.lock:
                self.planner_logs.append((time.monotonic(),msg.msg))

    def phase(self,msg):
        with self.lock:
            if not self.phases or self.phases[-1][1] != msg.data:
                self.phases.append((time.monotonic(),msg.data))

    def wait_phase(self,name,timeout=12):
        end=time.monotonic()+timeout
        while time.monotonic()<end:
            with self.lock:
                for t,p in self.phases:
                    if p==name: return t
                if any(p=='aborted' for _,p in self.phases) and name!='aborted':
                    raise AssertionError(f'Unexpected abort waiting for {name}: {self.phases}')
            time.sleep(0.02)
        raise AssertionError(f'Timeout waiting for {name}: {self.phases}')

    def trigger(self):
        end=time.monotonic()+10
        while self.trigger_pub.get_num_connections()==0 and time.monotonic()<end: time.sleep(0.05)
        check(self.trigger_pub.get_num_connections()>0,'Planner trigger subscriber missing')
        self.trigger_pub.publish(PoseStamped())

    def land_trigger(self, x=0.0):
        end=time.monotonic()+10
        while self.land_trigger_pub.get_num_connections()==0 and time.monotonic()<end: time.sleep(0.05)
        check(self.land_trigger_pub.get_num_connections()>0,'Planner landing trigger subscriber missing')
        msg=PoseStamped()
        msg.pose.position.x=x
        self.land_trigger_pub.publish(msg)

    def wait_new_phase(self,name,after,timeout=12):
        end=time.monotonic()+timeout
        while time.monotonic()<end:
            with self.lock:
                for t,p in self.phases:
                    if t>=after and p==name: return t
            time.sleep(0.02)
        raise AssertionError(f'Timeout waiting for new {name}: {self.phases}')

    def loop(self):
        rate=rospy.Rate(100)
        while self.running and not rospy.is_shutdown():
            stamp=rospy.Time.now()
            elapsed=time.monotonic()-self.origin
            w=self.turn_rate
            yaw=self.car_yaw+w*elapsed
            if abs(w)>1e-8:
                x=self.speed*(math.sin(yaw)-math.sin(self.car_yaw))/w
                y=self.speed*(-math.cos(yaw)+math.cos(self.car_yaw))/w
            else:
                x=self.speed*math.cos(yaw)*elapsed; y=self.speed*math.sin(yaw)*elapsed
            x += self.target_x_offset
            pos=[x,y,0.71]
            vel=[self.speed*math.cos(yaw),self.speed*math.sin(yaw),0]
            target=Odometry();target.header.stamp=stamp;target.header.frame_id='world'
            target.pose.pose.position.x=x;target.pose.pose.position.y=y;target.pose.pose.position.z=0.71
            # Body nose is deliberately different from velocity on the straight case.
            body_yaw=(w+self.body_yaw_rate_offset)*elapsed
            target.pose.pose.orientation.z=math.sin(body_yaw/2)
            target.pose.pose.orientation.w=math.cos(body_yaw/2)
            with self.lock:
                twist=TwistStamped()
                # Real driver publishes a server-clock twist header that can
                # differ greatly from corrected pose/local timestamps.
                if not self.freeze_twist_stamp or self.twist_stamp is None:
                    self.twist_stamp=stamp-rospy.Duration(144.0)
                twist.header.stamp=self.twist_stamp
                twist.header.frame_id='world'
                measured=self.twist_override(elapsed) if self.twist_override else vel
                twist.twist.linear.x,twist.twist.linear.y,twist.twist.linear.z=measured
                selected=self.traj
                if selected and stamp < selected.header.stamp: selected=self.previous
                status=FlightTrajectoryStatus()
                status.header.stamp=stamp-rospy.Duration(self.flight_status_stamp_offset)
                status.fcu_state_fresh=self.flight_status_fcu_fresh
                status.fcu_armed=self.flight_status_armed
                status.fcu_guided_nogps=self.flight_status_guided
                status.fsm_state=self.flight_status_fsm
                status.reference_type=self.flight_status_reference
                if self.flight_status_override is not None:
                    status.trajectory_id,status.trajectory_start=self.flight_status_override
                    status.trajectory_active=self.flight_status_active
                elif selected:
                    status.trajectory_id=selected.id
                    status.trajectory_start=selected.header.stamp
                    status.trajectory_active=(self.flight_status_active and stamp>=selected.header.stamp and
                        stamp<selected.header.stamp+rospy.Duration(sum(x.duration for x in selected.xyz_pieces)))
                else:
                    status.trajectory_active=False
                if selected and self.track_reference:
                    dt=(stamp-selected.header.stamp).to_sec()
                    p=evaluate(selected.xyz_pieces,dt)
                    v=evaluate(selected.xyz_pieces,dt,1)
                    heading=evaluate(selected.heading_pieces,dt)[0]
                else:
                    p=[x+self.initial_error,y,0.8];v=vel.copy();heading=yaw
                if self.force_yaw is not None: heading=self.force_yaw
                v[0]+=self.velocity_error
                odom=Odometry();odom.header.stamp=stamp;odom.header.frame_id='world'
                odom.pose.pose.position.x,odom.pose.pose.position.y,odom.pose.pose.position.z=p
                odom.pose.pose.position.x += self.position_error
                if self.lift_height_override is not None:
                    odom.pose.pose.position.z = self.lift_height_override
                odom.twist.twist.linear.x,odom.twist.twist.linear.y,odom.twist.twist.linear.z=v
                odom.pose.pose.orientation.z=math.sin(heading/2);odom.pose.pose.orientation.w=math.cos(heading/2)
                phase=self.phases[-1][1] if self.phases else ''
                self.snapshots.append((time.monotonic(),phase,p[:],pos,heading,yaw))
                if self.target_on: self.target_pub.publish(target)
                if self.twist_on: self.twist_pub.publish(twist)
                if self.odom_on: self.odom_pub.publish(odom)
                if self.flight_status_on: self.flight_status_pub.publish(status)
            rate.sleep()

    def close(self):
        self.running=False
        self.thread.join(2)
        for sub in self.subscribers: sub.unregister()
        for pub in (self.target_pub,self.twist_pub,self.odom_pub,self.trigger_pub,self.land_trigger_pub,self.status_pub,self.flight_status_pub):pub.unregister()


def run_case(name,configure,exercise,launch_command=None):
    selected=os.environ.get('MOVING_GRASP_CASES','')
    if selected and not any(part in name for part in selected.split('|')):
        return
    with tempfile.TemporaryFile(mode='w+') as output:
        launch=subprocess.Popen(launch_command or ['roslaunch','planning','grasping.launch'],stdout=output,stderr=subprocess.STDOUT)
        rig=Rig()
        try:
            configure(rig)
            time.sleep(1.4)
            rig.trigger()
            exercise(rig)
            print('PASS',name,flush=True)
        except Exception:
            output.seek(0)
            print(output.read()[-16000:])
            raise
        finally:
            rig.close()
            launch.terminate()
            try:launch.wait(8)
            except subprocess.TimeoutExpired:launch.kill();launch.wait()
            time.sleep(0.4)


def configure_landing_hold(rig):
    rig.speed=0.0
    rig.force_yaw=0.0
    # Block mission bootstrap until the initial trigger and landing trigger
    # have both been processed, so the test cannot close in between them.
    rig.flight_status_fsm=1


def stationary_pregrasp(rig):
    deadline=time.monotonic()+10
    while time.monotonic()<deadline:
        with rig.lock:
            if rig.pregrasp_targets and any(m.id > 0 for _,m in rig.trajectory_events):
                msg=rig.pregrasp_targets[-1]
                yaw=2*math.atan2(msg.pose.pose.orientation.z,msg.pose.pose.orientation.w)
                check(abs(msg.pose.pose.position.x-0.25)<0.01, 'Stationary pregrasp must be on aircraft side')
                check(abs(msg.pose.pose.position.y)<0.01 and abs(msg.pose.pose.position.z-0.8)<0.01,
                      'Stationary pregrasp height and lateral offset')
                check(abs(abs(yaw)-math.pi)<0.02, 'Stationary initial yaw must point toward the target')
                candidates=[m for _,m in rig.trajectory_events if m.id > 0]
                endpoint=evaluate(candidates[-1].xyz_pieces, sum(p.duration for p in candidates[-1].xyz_pieces))
                check(math.dist(endpoint,[0.25,0.0,0.8])<0.02,
                      'Optimized trajectory must end at the stationary pregrasp point')
                return
        time.sleep(0.02)
    raise AssertionError('No stationary pregrasp target published')


def moving_pregrasp(rig):
    deadline=time.monotonic()+10
    while time.monotonic()<deadline:
        with rig.lock:
            if rig.pregrasp_targets:
                msg=rig.pregrasp_targets[-1]
                yaw=2*math.atan2(msg.pose.pose.orientation.z,msg.pose.pose.orientation.w)
                if abs(msg.twist.twist.linear.x-0.15)<0.01 and abs(yaw)<0.05:
                    # Position is evaluated at the future stamp, not the
                    # current sample; compare against the constant velocity model.
                    elapsed=time.monotonic()-rig.origin
                    expected=rig.speed*(elapsed+(msg.header.stamp-rospy.Time.now()).to_sec())-0.25
                    check(abs(msg.pose.pose.position.x-expected)<0.06, 'Moving pregrasp is behind predicted target')
                    check(abs(msg.pose.pose.position.y)<0.02 and abs(msg.pose.pose.position.z-0.8)<0.01,
                          'Moving pregrasp is horizontal and keeps grasp height')
                    return
        time.sleep(0.02)
    raise AssertionError('No velocity-aligned moving pregrasp target')


def turning_pregrasp(rig):
    deadline=time.monotonic()+10
    while time.monotonic()<deadline:
        with rig.lock:
            if rig.pregrasp_targets:
                msg=rig.pregrasp_targets[-1]
                elapsed=time.monotonic()-rig.origin
                future=elapsed+(msg.header.stamp-rospy.Time.now()).to_sec()
                theta=rig.car_yaw+rig.turn_rate*future
                yaw=2*math.atan2(msg.pose.pose.orientation.z,msg.pose.pose.orientation.w)
                if elapsed>0.7 and abs(math.remainder(yaw-theta,2*math.pi))<0.20:
                    x=rig.speed*(math.sin(theta)-math.sin(rig.car_yaw))/rig.turn_rate
                    y=rig.speed*(-math.cos(theta)+math.cos(rig.car_yaw))/rig.turn_rate
                    expected=[x-0.25*math.cos(yaw),y-0.25*math.sin(yaw),0.8]
                    p=msg.pose.pose.position
                    check(math.dist([p.x,p.y,p.z],expected)<0.06,
                          'Turning pregrasp must remain behind the predicted grasp point')
                    return
        time.sleep(0.02)
    raise AssertionError('No turning pregrasp with confirmed velocity direction')


def retained_pregrasp_direction(rig):
    deadline=time.monotonic()+8
    while time.monotonic()<deadline:
        with rig.lock:
            messages=rig.pregrasp_targets[:]
        if messages:
            yaw=2*math.atan2(messages[-1].pose.pose.orientation.z,messages[-1].pose.pose.orientation.w)
            if abs(yaw)<0.05: break
        time.sleep(0.02)
    else: raise AssertionError('Moving direction was not confirmed')
    with rig.lock: rig.twist_override=lambda _: (0.0,0.0,0.0)
    time.sleep(0.8)
    triggered=time.monotonic()
    rig.trigger()
    rig.wait_new_phase('opening_assumed',triggered)
    with rig.lock: before=len(rig.pregrasp_targets)
    deadline=time.monotonic()+12
    while time.monotonic()<deadline:
        with rig.lock:
            messages=rig.pregrasp_targets[before:]
        if messages:
            yaw=2*math.atan2(messages[-1].pose.pose.orientation.z,messages[-1].pose.pose.orientation.w)
            check(abs(yaw)<0.05, 'Retrigger while stopped must retain last confirmed motion direction')
            check(abs(messages[-1].pose.pose.position.x+0.25)<0.04,
                  'Stopped pregrasp stays behind last confirmed motion direction')
            return
        time.sleep(0.02)
    raise AssertionError('No pregrasp target after stationary retrigger')


def configure_retained_direction(rig):
    rig.speed=0.0
    rig.initial_error=0.7
    rig.track_reference=False
    rig.twist_override=lambda _: (0.15,0.0,0.0)


def configure_grasp_point_latch(rig):
    rig.speed=0.0
    rig.initial_error=0.35
    rig.track_reference=False
    rig.force_yaw=0.0  # The stationary target is to the left, so never close.


def grasp_point_latch_and_retrigger(rig):
    deadline=time.monotonic()+10
    while time.monotonic()<deadline:
        with rig.lock:
            if rig.pregrasp_events and rig.grasp_point_modes and not rig.grasp_point_modes[-1][1]:
                rig.position_error=-0.08  # 0.27 m from the grasp point.
                break
        time.sleep(0.02)
    else: raise AssertionError('No far pregrasp planning before threshold')
    deadline=time.monotonic()+5
    while time.monotonic()<deadline:
        with rig.lock:
            if rig.grasp_point_modes and rig.grasp_point_modes[-1][1]:
                rig.position_error=0.0  # Move back outside to 0.35 m.
                break
        time.sleep(0.02)
    else: raise AssertionError('Grasp-point mode did not latch below 0.30 m')
    time.sleep(0.25)  # Allow pending messages from the pregrasp phase to drain.
    with rig.lock:
        checkpoint=time.monotonic()
        pre_count=len(rig.pregrasp_events)
    time.sleep(1.0)
    with rig.lock:
        check(rig.grasp_point_modes[-1][1],'Leaving 0.30 m reverted the latch')
        check(len(rig.pregrasp_events)==pre_count,'Pregrasp planning resumed after the latch')
        later=[msg for when,msg in rig.trajectory_events if when>checkpoint]
        check(later,'No grasp-point trajectories after leaving 0.30 m')
        endpoints=[evaluate(msg.xyz_pieces,sum(p.duration for p in msg.xyz_pieces)) for msg in later]
        check(all(abs(p[0])<0.03 for p in endpoints),
              f'Post-latch trajectory did not target the grasp point: {endpoints[-4:]}')
    triggered=time.monotonic()
    rig.trigger()
    deadline=time.monotonic()+15
    while time.monotonic()<deadline:
        with rig.lock:
            reset=any(t>=triggered and not mode for t,mode in rig.grasp_point_modes)
            new_pregrasp=any(t>=triggered for t,_ in rig.pregrasp_events)
            if reset and new_pregrasp: return
        time.sleep(0.02)
    raise AssertionError('New trigger did not reset the latch and resume pregrasp planning')


def grasp_point_fallback(rig):
    # Force the quintic follow target speed above vmax after the latch. The
    # fallback must predict the real grasp point, not its pregrasp offset.
    deadline=time.monotonic()+8
    while time.monotonic()<deadline:
        with rig.lock:
            if rig.grasp_point_modes and rig.grasp_point_modes[-1][1] and len(rig.trajectory_events)>2:
                rig.twist_override=lambda _: (1.0,0.0,0.0)
                checkpoint=time.monotonic()
                break
        time.sleep(0.02)
    else: raise AssertionError('Grasp-point mode or initial trajectory missing')
    deadline=time.monotonic()+8
    while time.monotonic()<deadline:
        with rig.lock:
            paths=[msg for t,msg in rig.prediction_paths if t>=checkpoint and len(msg.poses)>1]
            for path in paths:
                p0=path.poses[0].pose.position
                plast=path.poses[-1].pose.position
                # A long positive-X prediction demonstrates that target speed
                # is above vmax and hence the constrained follow cannot pass.
                if plast.x-p0.x>1.1:
                    check(abs(p0.x)<0.06,'Fallback prediction used a pregrasp-offset target')
                    check(rig.grasp_point_modes[-1][1],'Fallback cleared grasp-point mode')
                    return
        time.sleep(0.02)
    raise AssertionError('No high-speed grasp-point fallback prediction observed')


def configure_grasp_point_fallback(rig):
    configure_grasp_point_latch(rig)
    rig.initial_error=0.27


def configure_first_grasp_plan_failure(rig):
    configure_grasp_point_fallback(rig)
    rig.twist_override=lambda _: (1.0,0.0,0.0)


def first_grasp_plan_failure_brakes(rig):
    rig.wait_phase('aborted',12)
    with rig.lock:
        check(any(mode for _,mode in rig.grasp_point_modes),'Grasp-point mode was never latched')
        check(rig.planner_logs,'Failure was not caused by an unavailable grasp-point trajectory')
        check(not any(msg.reference_position < -1.0 for _,msg in rig.commands),
              'A failed first grasp-point trajectory sent a close command')


def reverse_during_retrigger(rig):
    deadline=time.monotonic()+8
    while time.monotonic()<deadline:
        with rig.lock:
            messages=rig.pregrasp_targets[:]
        if messages:
            yaw=2*math.atan2(messages[-1].pose.pose.orientation.z,messages[-1].pose.pose.orientation.w)
            if abs(yaw)<0.05: break
        time.sleep(0.02)
    else: raise AssertionError('Initial motion direction was not confirmed')
    triggered=time.monotonic()
    rig.trigger()
    rig.wait_new_phase('opening_assumed',triggered)
    with rig.lock:
        rig.twist_override=lambda _: (-0.15,0.0,0.0)
        before=len(rig.pregrasp_events)
    deadline=time.monotonic()+12
    while time.monotonic()<deadline:
        with rig.lock: events=rig.pregrasp_events[before:]
        if events:
            first_yaw=2*math.atan2(events[0][1].pose.pose.orientation.z,events[0][1].pose.pose.orientation.w)
            check(abs(first_yaw)<0.05,'Retrigger must hold the prior yaw on the first reversed sample')
            for when,msg in events:
                yaw=2*math.atan2(msg.pose.pose.orientation.z,msg.pose.pose.orientation.w)
                if abs(abs(yaw)-math.pi)<0.05:
                    check(when-events[0][0]>=0.20,'Reverse direction bypassed confirmation dwell')
                    return
        time.sleep(0.02)
    raise AssertionError('Reversed motion was not reconfirmed after retrigger')


def start_landing(rig, x=0.0):
    time.sleep(0.3)  # Allow the initial mission trigger callback to complete.
    rig.land_trigger(x)
    time.sleep(0.3)  # Deliver the landing trigger before enabling trajectory mode.
    rig.flight_status_fsm=2


def landing_hold(rig):
    start_landing(rig)
    end=time.monotonic()+10
    while time.monotonic()<end:
        with rig.lock:
            holds=[event for event in rig.trajectory_events
                   if abs(sum(p.duration for p in event[1].xyz_pieces)-1.0)<1e-6]
            if len(holds)>=3: break
        time.sleep(0.02)
    check(len(holds)>=3,'Landing hold was not refreshed')
    time.sleep(2.2)  # More than the one-second lifetime of each hold trajectory.
    with rig.lock:
        holds=[event for event in rig.trajectory_events
               if abs(sum(p.duration for p in event[1].xyz_pieces)-1.0)<1e-6]
        check(len(holds)>=10,'Landing hold expired instead of being refreshed')
        check(not any(p in ('braking','aborted') for _,p in rig.phases),
              f'Landing hold caused an unexpected flight-loss abort: {rig.phases}')
        positions=[evaluate(event[1].xyz_pieces,0) for event in holds]
        check(max(math.dist(positions[0],p) for p in positions)<1e-6,
              'Landing hold point changed during the hold')
        hold_point=positions[0]
        checkpoint=time.monotonic()
        rig.target_x_offset=0.15
    end=time.monotonic()+8
    resumed=None
    while time.monotonic()<end:
        with rig.lock:
            for when, msg in rig.trajectory_events:
                duration=sum(p.duration for p in msg.xyz_pieces)
                if when>=checkpoint and math.dist(evaluate(msg.xyz_pieces,0),
                                                   evaluate(msg.xyz_pieces,duration))>0.01:
                    resumed=msg
                    break
            if resumed: break
            check(not any(p in ('braking','aborted') for _,p in rig.phases),
                  f'Landing replan unexpectedly aborted: {rig.phases}')
        time.sleep(0.02)
    check(resumed is not None,'Landing did not replan after the target moved')
    check(math.dist(evaluate(resumed.xyz_pieces,0),hold_point)<1e-5,
          'Landing replan did not start from the retained hold reference')
    with rig.lock:
        rig.flight_status_active=False
    rig.wait_phase('aborted')


def landing_hold_status_loss(rig):
    start_landing(rig)
    end=time.monotonic()+10
    while time.monotonic()<end:
        with rig.lock:
            holds=[event for event in rig.trajectory_events
                   if abs(sum(p.duration for p in event[1].xyz_pieces)-1.0)<1e-6]
            if len(holds)>=3:
                rig.flight_status_active=False
                break
        time.sleep(0.02)
    check(len(holds)>=3,'Landing hold was not reached before status-loss test')
    rig.wait_phase('aborted')


def landing_then_retrigger(rig):
    # A nonzero landing offset makes stale landing state observable in the
    # trajectory, as well as through the close gate on the next mission.
    start_landing(rig, 0.15)
    deadline=time.monotonic()+10
    while time.monotonic()<deadline:
        with rig.lock:
            reached=rig.snapshots and rig.snapshots[-1][2][0]>0.10
        if reached: break
        time.sleep(0.02)
    check(reached, 'Landing offset was not reached before retrigger')
    # The new stationary-target heading points from the offset landing
    # position toward the grasp point; let the simulated aircraft follow it.
    with rig.lock:
        rig.force_yaw=None
    start=time.monotonic()
    rig.trigger()
    opened,_=wait_command(rig,is_open,start)
    closed,_=wait_command(rig,is_close,opened,15)
    with rig.lock:
        samples=[s for s in rig.snapshots if opened <= s[0] <= closed]
    check(closed-opened>=0.82, 'New mission closed before opening completed')
    check(samples and abs(samples[-1][2][0]-samples[-1][3][0])<0.03,
          'New mission did not return to the unoffset grasp point')


def configure_landing_yaw_change(rig):
    # Start at 90 degrees, but move along world X. Keep the measured yaw
    # misaligned while the planned yaw rotates, preventing an early grasp.
    rig.speed=0.09
    rig.car_yaw=0.0
    rig.force_yaw=math.pi/2


def landing_hold_yaw_continuity(rig, rotating):
    end=time.monotonic()+12
    ready=False
    while time.monotonic()<end:
        with rig.lock:
            if rig.traj and rig.snapshots:
                t=(rospy.Time.now()-rig.traj.header.stamp).to_sec()
                yaw=evaluate(rig.traj.heading_pieces,t)[0]
                rate=evaluate(rig.traj.heading_pieces,t,1)[0]
                ready=(0.5<yaw<1.4 and rate<-0.25) if rotating else abs(yaw)<0.05
                if ready:
                    count=len(rig.trajectory_events)
                    continuity_start=len(rig.yaw_continuity)
                    limit_start=len(rig.yaw_limits)
                    break
        time.sleep(0.01)
    check(ready,'Approach did not reach the requested heading/rate before landing')
    rig.land_trigger()
    with rig.lock:
        # Stop at the current world position to exercise sustained hold, not
        # the separate target-moved replan branch covered by landing_hold().
        rig.target_x_offset=rig.snapshots[-1][3][0]
        rig.speed=0.0
        rig.force_yaw=None
    # Cover entry, more than one trajectory lifetime of refreshes, and settling.
    time.sleep(3.0)
    with rig.lock:
        holds=[msg for _,msg in rig.trajectory_events[count:]
               if abs(sum(p.duration for p in msg.xyz_pieces)-1.0)<1e-6]
        continuity=rig.yaw_continuity[continuity_start:]
        limits=rig.yaw_limits[limit_start:]
        check(len(holds)>=10,'Landing heading check did not reach sustained hold')
        check(continuity,'No landing heading boundaries were checked')
        check(max(x[0] for x in continuity)<1e-4,'Landing yaw reference jumped')
        check(max(x[1] for x in continuity)<1e-4,'Landing yaw-rate reference jumped')
        check(max(x[0] for x in limits)<=0.6+1e-6,'Landing yaw rate exceeds its limit')
        check(max(x[1] for x in limits)<=1.2+1e-6,'Landing yaw acceleration exceeds its limit')
        first_yaw=evaluate(holds[0].heading_pieces,0)[0]
        first_rate=evaluate(holds[0].heading_pieces,0,1)[0]
        check(abs(first_yaw-math.pi/2)>0.1,'Landing returned to the trigger heading')
        if rotating:
            check(abs(first_rate)>0.2,'Nonzero entry yaw rate was not exercised')
        final_yaw=evaluate(holds[-1].heading_pieces,0)[0]
        final_rate=evaluate(holds[-1].heading_pieces,0,1)[0]
        check(abs(final_yaw-first_yaw)<0.03,'Landing heading did not settle near the entry heading')
        check(abs(final_rate)<0.03,'Landing yaw rate did not settle toward zero')
        check(not any(p in ('braking','aborted') for _,p in rig.phases),
              f'Landing heading transition caused an unexpected abort: {rig.phases}')
        print('Landing yaw continuity:', {'rotating':rotating,
              'max_yaw_jump_rad':max(x[0] for x in continuity),
              'max_yaw_rate_jump':max(x[1] for x in continuity),
              'entry_rate':first_rate,'settled_rate':final_rate},flush=True)


def success(rig):
    closed=rig.wait_phase('closing_assumed')
    # A bogus external status must not alter the internal mission state.
    rig.status_pub.publish(GraspState(graspState=0))
    lifted=rig.wait_phase('lifting_assumed')
    rig.wait_phase('hold_assumed',16)
    first=rig.commands[0][0]
    check(rig.commands[0][1].mode==ActuatorCommand.MODE_POSITION and
          rig.commands[0][1].reference_position < -1.0,
          'The first mission trigger must not send a re-open command')
    check(0.80 <= lifted-first <= 1.10,f'Close timer was restarted/early: {lifted-first}')
    samples=[s for s in rig.snapshots if closed+0.1<s[0]<lifted-0.05]
    check(len(samples)>10,'Not enough closing samples')
    check(math.dist(samples[0][2][:2],samples[-1][2][:2])>0.06,'Aircraft froze while closing')
    check(max(math.dist(s[2][:2],s[3][:2]) for s in samples)<0.04,'Failed to follow car while closing')
    check(max(abs(s[2][2]-0.8) for s in samples)<0.01,'Lifted before closing completed')
    check(max(abs(math.remainder(s[4]-s[5],2*math.pi)) for s in samples)<0.18,'Yaw not aligned with car velocity')
    final=rig.snapshots[-1][2]
    check(abs(final[2]-1.0)<0.025,f'Wrong lift height: {final}')
    check(all(max(errors)<2e-3 for errors in rig.continuity),f'PVA discontinuity: {rig.continuity[-8:]}')
    check(all(max(errors)<2e-3 for errors in rig.yaw_continuity),'Yaw or yaw-rate discontinuity')
    check(all(v<=0.60001 and a<=1.20001 for v,a in rig.yaw_limits),'Yaw limits exceeded')
    print(f'  close_to_lift={lifted-first:.3f}s; closing_xy_motion='
          f'{math.dist(samples[0][2][:2],samples[-1][2][:2]):.3f}m; final_z={final[2]:.3f}m',flush=True)


def lost_target(rig):
    rig.wait_phase('closing_assumed')
    time.sleep(0.15)
    rig.target_on=False
    rig.wait_phase('aborted',8)
    check(not any(p=='lifting_assumed' for _,p in rig.phases),'Lost target still lifted')
    rig.target_on=True
    time.sleep(0.5)
    check(rig.phases[-1][1]=='aborted','Fresh target silently cancelled abort')
    # Explicit trigger is the only restart; no actuator feedback is published.
    with rig.lock:
        rig.phases.clear();rig.traj=None;rig.previous=None
    rig.trigger()
    rig.wait_phase('closing_assumed')


def bad_yaw(rig):
    time.sleep(1.0)
    check(not rig.commands,'Closed with a 90 degree yaw error')
    rig.force_yaw=None
    rig.wait_phase('closing_assumed',10)


def slow_speed_bad_yaw(rig):
    # Regression: 0.075 m/s toward +Y with yaw 0 was previously treated as
    # stationary while the velocity filter was still settling, allowing close.
    start=time.monotonic()
    time.sleep(1.5)
    assert_no_close(rig,start,'Closed while a slow car moved 90 degrees from aircraft yaw')


def bad_speed(rig):
    time.sleep(0.8)
    check(not rig.commands,'Closed based on distance despite relative-speed error')
    rig.velocity_error=0
    rig.wait_phase('closing_assumed',10)


def lost_odom(rig):
    rig.wait_phase('closing_assumed')
    rig.odom_on=False
    rig.wait_phase('aborted',8)
    check(not any(p=='lifting_assumed' for _,p in rig.phases),'Stale odometry still lifted')


def excessive_error(rig):
    rig.wait_phase('closing_assumed')
    rig.position_error=0.3
    rig.wait_phase('aborted',8)
    check(not any(p=='lifting_assumed' for _,p in rig.phases),'Excessive tracking error still lifted')


def body_rotation_straight_velocity(rig):
    # Motion is straight while the body orientation rotates independently.
    # The yaw trajectory should not use the body's angular velocity.
    deadline=time.monotonic()+8
    while time.monotonic()<deadline:
        with rig.lock:
            msg=rig.traj
            if msg and time.monotonic()-rig.origin>2.3:
                end=sum(piece.duration for piece in msg.heading_pieces)
                end_yaw=evaluate(msg.heading_pieces,end)[0]
                error=abs(math.remainder(end_yaw-rig.car_yaw,2*math.pi))
                check(error<0.05,f'Straight velocity produced terminal yaw error {error:.3f} rad')
                return
        time.sleep(0.02)
    raise AssertionError('No trajectory for straight-velocity/body-turn test')


def stalled_lift(rig):
    rig.wait_phase('lifting_assumed')
    with rig.lock:
        rig.lift_height_override=0.81
    rig.wait_phase('aborted',8)
    check(not any(p=='hold_assumed' for _,p in rig.phases),
          'Nearly stalled lift incorrectly reached successful hold')


def configured_lift_error(rig):
    rig.wait_phase('lifting_assumed')
    with rig.lock: rig.position_error=0.12
    time.sleep(0.35)
    with rig.lock:
        check(rig.phases[-1][1]=='lifting_assumed',
              f'Position error 0.12 m should be allowed by configured 0.15 m: {rig.phases}')
        rig.position_error=0.18
    rig.wait_phase('aborted',8)


def missing_twist(rig):
    time.sleep(0.8)
    check(not rig.commands and rig.traj is None,'Pose alone must not authorize planning or closing')
    rig.twist_on=True
    rig.wait_phase('closing_assumed',10)


def lost_twist(rig):
    rig.wait_phase('closing_assumed')
    rig.twist_on=False
    rig.wait_phase('aborted',8)
    check(not any(p=='lifting_assumed' for _,p in rig.phases),'Lost velocity still lifted')


def frozen_twist(rig):
    rig.wait_phase('closing_assumed')
    rig.freeze_twist_stamp=True
    rig.wait_phase('aborted',8)
    check(not any(p=='lifting_assumed' for _,p in rig.phases),'Repeated old twist stamp still lifted')


def is_close(message):
    return message.mode==ActuatorCommand.MODE_POSITION and message.reference_position < -1.0


def is_open(message):
    return message.mode==ActuatorCommand.MODE_POSITION and abs(message.reference_position)<1e-9


def assert_no_close(rig,after,reason):
    with rig.lock:
        closes=[t for t,msg in rig.commands if t>=after and is_close(msg)]
    check(not closes,f'{reason}: {closes}')


def wait_command(rig,predicate,after,timeout=8):
    deadline=time.monotonic()+timeout
    while time.monotonic()<deadline:
        with rig.lock:
            for t,msg in rig.commands:
                if t>=after and predicate(msg): return t,msg
        time.sleep(0.01)
    raise AssertionError(f'Missing actuator command; commands={[(t,m.mode,m.reference_position) for t,m in rig.commands[-8:]]}')


def absent_flight_status(rig):
    start=time.monotonic()
    time.sleep(0.8)
    assert_no_close(rig,start,'Closed without any flight status')
    rig.flight_status_on=True
    rig.wait_phase('closing_assumed',10)


def generic_without_flight_status(rig):
    check(rospy.get_param('/drone0/planning/require_flight_status') is False,
          'Generic planning unexpectedly requires flight status')
    check(rospy.get_param('/drone0/planning/target_velocity_source') == 'pose_difference',
          'Generic planning must derive target velocity from pose')
    deadline=time.monotonic()+8
    while time.monotonic()<deadline:
        with rig.lock:
            trajectories=list(rig.trajectory_events)
            phases=list(rig.phases)
        if trajectories:
            check(not any(p=='aborted' for _,p in phases),
                  f'Generic planning aborted without flight status: {phases}')
            time.sleep(0.4)
            with rig.lock:
                check(not rig.commands, 'Generic planning closed without flight confirmation')
            return
        time.sleep(0.02)
    raise AssertionError(f'Generic planning published no trajectory without flight status: {phases}')


def invalid_flight_states(rig):
    # Each state persists longer than the 0.15 s grasp gate. The synthetic
    # active bit remains true to make sure each field itself is checked.
    for field,value in [('flight_status_armed',False),('flight_status_guided',False),
                        ('flight_status_fcu_fresh',False),('flight_status_fsm',1),
                        ('flight_status_reference',1),('flight_status_active',False),
                        ('flight_status_stamp_offset',1.0),('flight_status_stamp_offset',-1.0)]:
        with rig.lock: setattr(rig,field,value)
        start=time.monotonic()
        time.sleep(0.4)
        assert_no_close(rig,start,f'Closed with invalid {field}')
        # Restore this field while holding the next one invalid, without a
        # transient healthy period that could accidentally satisfy the gate.
        with rig.lock:
            if field=='flight_status_armed':
                rig.flight_status_guided=False
            elif field=='flight_status_guided':
                rig.flight_status_fcu_fresh=False
            elif field=='flight_status_fcu_fresh':
                rig.flight_status_fsm=1
            elif field=='flight_status_fsm':
                rig.flight_status_reference=1
            elif field=='flight_status_reference':
                rig.flight_status_active=False
            elif field=='flight_status_active':
                rig.flight_status_stamp_offset=1.0
            if field=='flight_status_stamp_offset' and value==1.0:
                rig.flight_status_stamp_offset=-1.0
                continue
            setattr(rig,field, 2 if field=='flight_status_fsm' else
                    4 if field=='flight_status_reference' else
                    0.0 if field=='flight_status_stamp_offset' else True)
    rig.wait_phase('closing_assumed',10)


def lost_flight_status(rig,phase):
    rig.wait_phase(phase,12)
    with rig.lock: rig.flight_status_on=False
    rig.wait_phase('aborted',8)
    with rig.lock:
        phases_before=list(rig.phases)
        rig.flight_status_on=True
    time.sleep(0.6)
    with rig.lock: phases_after=list(rig.phases)
    check(phases_after[-1][1]=='aborted',f'Status recovery resumed mission automatically: {phases_after}')
    if phase=='closing_assumed':
        check(not any(p=='lifting_assumed' for _,p in phases_before),'Lost status during closing still lifted')


def transient_bad_ack(rig):
    rig.wait_phase('closing_assumed')
    with rig.lock:
        check(rig.traj is not None,'Missing trajectory to invalidate')
        rig.flight_status_override=(rig.traj.id+100000,rig.traj.header.stamp)
    # The fake flight loop publishes at 100 Hz. Recover before the normal
    # planning cycle where possible: a bad ACK must remain latched.
    time.sleep(0.04)
    with rig.lock: rig.flight_status_override=None
    rig.wait_phase('aborted',8)
    time.sleep(0.5)
    with rig.lock: phases=list(rig.phases)
    check(phases[-1][1]=='aborted',f'Transient bad ACK did not latch abort: {phases}')
    check(not any(p=='lifting_assumed' for _,p in phases),'Transient bad ACK still allowed lift')


def opening_after_trigger(rig,phase):
    rig.wait_phase(phase,16)
    if phase=='approach':
        # The planner publishes a latched initial 'approach' before receiving
        # the first trigger. Wait for an actual mission trajectory so the
        # retrigger cannot replace the first trigger in its subscriber queue.
        deadline=time.monotonic()+10
        while time.monotonic()<deadline:
            with rig.lock:
                if rig.traj is not None:
                    break
            time.sleep(0.02)
        else:
            raise AssertionError(f'First trigger produced no approach trajectory; phases={rig.phases}, '
                                 f'commands={[(t,m.mode,m.reference_position) for t,m in rig.commands]}')
    trigger_time=time.monotonic()
    rig.trigger()
    opened,msg=wait_command(rig,is_open,trigger_time)
    check(opened-trigger_time<0.30,f'Retrigger did not send an immediate open command: {opened-trigger_time:.3f}s')
    check(msg.mode==ActuatorCommand.MODE_POSITION,'Retrigger must command position, not disable torque')
    rig.wait_new_phase('opening_assumed',trigger_time)
    time.sleep(0.65)
    assert_no_close(rig,opened,'An old or new close command overwrote opening before 0.85 s')
    if phase=='approach':
        rig.force_yaw=None
    new_close,_=wait_command(rig,is_close,opened,12)
    check(new_close-opened>=0.82,f'Closed before open timer completed: {new_close-opened:.3f}s')
    print(f'  {phase}: open received after {opened-trigger_time:.3f}s; next close after {new_close-opened:.3f}s',flush=True)


def repeated_opening(rig):
    rig.wait_phase('closing_assumed')
    t1=time.monotonic();rig.trigger()
    open1,_=wait_command(rig,is_open,t1)
    time.sleep(0.35)
    t2=time.monotonic();rig.trigger()
    open2,_=wait_command(rig,is_open,t2)
    time.sleep(0.65)
    assert_no_close(rig,open2,'Repeated trigger failed to restart opening duration')
    closed,_=wait_command(rig,is_close,open2,12)
    check(closed-t2>=0.82,f'Repeated open was too short: {closed-t2:.3f}s')


def old_ack_after_retrigger(rig):
    rig.wait_phase('closing_assumed')
    with rig.lock:
        check(rig.traj is not None,'Missing prior trajectory')
        old=(rig.traj.id,rig.traj.header.stamp)
        rig.flight_status_override=old
    t=time.monotonic();rig.trigger()
    opened,_=wait_command(rig,is_open,t)
    rig.wait_new_phase('opening_assumed',t)
    time.sleep(1.35)
    assert_no_close(rig,opened,'Previous task trajectory ACK authorized a new close')
    with rig.lock: rig.flight_status_override=None
    wait_command(rig,is_close,opened,12)


def reopens_without_status(rig):
    rig.wait_phase('closing_assumed')
    with rig.lock: rig.flight_status_on=False
    t=time.monotonic();rig.trigger()
    opened,_=wait_command(rig,is_open,t)
    check(opened-t<0.30,'Missing flight status delayed open command')
    time.sleep(1.1)
    assert_no_close(rig,opened,'Reclosed without restored flight status')


def retrigger_aborted(rig):
    rig.wait_phase('closing_assumed')
    with rig.lock: rig.flight_status_on=False
    rig.wait_phase('aborted',8)
    with rig.lock: rig.flight_status_on=True
    t=time.monotonic();rig.trigger()
    opened,_=wait_command(rig,is_open,t)
    check(opened-t<0.30,'Retrigger after abort did not immediately command open')
    closed,_=wait_command(rig,is_close,opened,12)
    check(closed-opened>=0.82,'Retrigger after abort did not wait for opening')


def wait_for_flight_ack(rig):
    start=time.monotonic()
    time.sleep(1.3)
    with rig.lock:
        trajectories=[m for t,m in rig.trajectory_events if t>=start]
        phases=list(rig.phases)
    check(trajectories, 'No bootstrap trajectory was published')
    for msg in trajectories:
        check(len(msg.xyz_pieces)==1 and
              max(abs(x) for x in msg.xyz_pieces[0].coeff[:12]) < 1e-9,
              'Approach trajectory was published before flight acknowledged bootstrap')
    check(not any(p in ('closing_assumed','lifting_assumed') for _,p in phases),
          'Mission advanced before flight acknowledged bootstrap')
    rig.flight_status_active=True
    rig.wait_phase('closing_assumed',10)


def lost_flight_during_approach(rig):
    deadline=time.monotonic()+5
    while time.monotonic()<deadline:
        with rig.lock:
            if len(rig.trajectory_events)>2:
                break
        time.sleep(0.02)
    else:
        raise AssertionError('Approach did not start before simulated flight loss')
    rig.flight_status_on=False
    rig.wait_phase('aborted',8)
    assert_no_close(rig,0,'Closed after flight status was lost during approach')


def retrigger_after_manual_move(rig):
    rig.wait_phase('hold_assumed',16)
    with rig.lock:
        rig.flight_status_fsm=1
        rig.flight_status_reference=1
        rig.track_reference=False
        rig.initial_error=1.0
    time.sleep(0.35)
    switched=time.monotonic()
    with rig.lock:
        rig.flight_status_fsm=2
        rig.flight_status_active=False
    time.sleep(0.15)
    with rig.lock:
        check(not any(t>switched for t,_ in rig.trajectory_events),
              'Old stop was resumed on mode reentry before the new trigger')
    start=time.monotonic()
    rig.trigger()
    opened,_=wait_command(rig,is_open,start)
    time.sleep(1.3)
    with rig.lock:
        opening_trajectories=[m for t,m in rig.trajectory_events if opened+0.05<t<opened+0.80]
        bootstrap=[m for t,m in rig.trajectory_events if t>opened+0.85]
        samples=list(rig.snapshots)
    check(not opening_trajectories, 'Old hold was republished during opening after manual movement')
    check(bootstrap, 'Missing bootstrap after opening')
    initial=evaluate(bootstrap[0].xyz_pieces,0)
    current=samples[-1][2]
    check(math.dist(initial,current)<0.15, f'Bootstrap used stale hold: {initial} vs {current}')
    assert_no_close(rig,opened,'Closed before flight accepted the new trajectory')
    with rig.lock:
        rig.flight_status_reference=4
        rig.flight_status_active=True
    deadline=time.monotonic()+5
    while time.monotonic()<deadline:
        with rig.lock:
            if any(len(m.xyz_pieces)>1 for t,m in rig.trajectory_events if t>opened+1.3):
                return
        time.sleep(0.02)
    raise AssertionError('No approach trajectory after bootstrap was acknowledged')


def explicit_velocity(rig):
    time.sleep(0.7)
    check(not rig.commands,'Closed despite actual twist relative-speed error')
    check(any(abs(x-0.20)<0.01 for _,x,_,_ in rig.replans),
          f'Published target velocity did not use twist: {rig.replans[-4:]}')


def stationary_noise(rig):
    rig.wait_phase('closing_assumed', 8)
    with rig.lock:
        samples=rig.snapshots[:]
        commands=rig.commands[:]
    check(any(is_close(m) for _,m in commands),'Stationary noisy target did not receive a close command')
    check(len(samples)>10,'Stationary noise scenario did not run')
    max_yaw=max(abs(math.remainder(s[4]-math.pi/2,2*math.pi)) for s in samples)
    check(max_yaw<0.05,f'Stationary velocity noise changed yaw: {max_yaw}')


def static_noise(t):
    sign=1 if int(t*100)%2 else -1
    spike=0.13*sign if int(t*100)%31==0 else 0.04*math.sin(17*t)
    return (spike,0.04*math.cos(23*t),0)


def configure_static_velocity(rig):
    rig.speed=0.0
    rig.track_reference=False
    rig.force_yaw=0.0
    rig.twist_override=lambda t:(0.20,0.0,0.0)


def configure_stationary_noise(rig):
    rig.speed=0.0
    rig.twist_override=static_noise


def configure_velocity_filter(rig):
    rig.speed=0.0
    rig.track_reference=False
    rig.initial_error=0.08  # Stay in near-target follow without entering closure.
    rig.twist_override=lambda t:(0.04*math.sin(2*math.pi*8*t),0.0,0.0)


def velocity_filter_prediction(rig,enabled):
    start=time.monotonic()
    time.sleep(1.5)
    with rig.lock:
        replans=[x for x in rig.replans if x[0]>start+0.3]
        trajectories=[msg for t,msg in rig.trajectory_events if t>start+0.3]
    check(len(replans)>=10 and len(trajectories)>=10,'Insufficient filtered trajectory samples')
    check(not rig.commands,'Closed despite 8 cm position error')
    diagnostic_peak=max(abs(x) for _,x,_,_ in replans)
    endpoint_peak=max(abs(evaluate(m.xyz_pieces,sum(p.duration for p in m.xyz_pieces),1)[0])
                      for m in trajectories)
    if enabled:
        check(diagnostic_peak<0.025 and endpoint_peak<0.025,
              f'3 Hz cutoff did not attenuate 8 Hz noise: {diagnostic_peak}, {endpoint_peak}')
    else:
        check(diagnostic_peak>0.03 and endpoint_peak>0.03,
              f'0 Hz did not bypass filtering: {diagnostic_peak}, {endpoint_peak}')


def configure_filtered_velocity_gate(rig):
    rig.speed=0.0
    rig.track_reference=False
    rig.force_yaw=0.0
    rig.twist_override=lambda t:(0.2*math.sin(2*math.pi*8*t),0.0,0.0)


def filtered_velocity_gate(rig):
    rig.wait_phase('closing_assumed', 8)
    with rig.lock:
        commands=rig.commands[:]
    check(any(is_close(m) for _,m in commands),'Filtered speed gate did not allow closure')


def configure_filtered_abort(rig):
    rig.speed=0.0
    rig.track_reference=False
    rig.force_yaw=0.0


def filtered_abort_noise(rig):
    rig.wait_phase('closing_assumed', 8)
    start=time.monotonic()
    rig.twist_override=lambda t:(0.8*math.sin(2*math.pi*30*t),0.0,0.0)
    time.sleep(0.30)
    rig.twist_override=lambda t:(0.0,0.0,0.0)
    with rig.lock:
        check(not any(phase in ('braking','aborted') for when,phase in rig.phases if when>=start),
              f'Raw noise caused abort during closing: {rig.phases}')


def filtered_abort_sustained_motion(rig):
    rig.wait_phase('closing_assumed', 8)
    rig.twist_override=lambda t:(1.2,0.0,0.0)
    rig.wait_phase('aborted', 8)


def direction_cutoff_configuration(rig):
    cutoff=rospy.get_param('/drone0/planning/grasp_yaw_velocity_filter_hz')
    check(abs(cutoff-2.0)<1e-9,f'Incorrect direction cutoff parameter: {cutoff}')
    rig.wait_phase('closing_assumed', 10)


def configure_slow_bad_yaw(rig):
    rig.speed=0.075
    rig.force_yaw=0.0


def run_configured_case(name, configure, exercise, overrides):
    # Test alternate fixed settings in a temporary launch, without editing the
    # user's planning.launch or exposing production launch arguments.
    source = Path(__file__).resolve().parents[1] / 'launch' / 'planning.launch'
    tree = ET.parse(source)
    for key, value in overrides.items():
        arg = tree.getroot().find("arg[@name='%s']" % key)
        check(arg is not None and 'value' in arg.attrib, 'Unknown fixed argument: ' + key)
        arg.set('value', str(value))
    with tempfile.TemporaryDirectory() as directory:
        planner = Path(directory) / 'planning.launch'
        wrapper = Path(directory) / 'test.launch'
        tree.write(planner, encoding='unicode')
        wrapper.write_text('<launch><group ns="drone0"><include file="%s">'
                           '<arg name="car_grasp" value="true"/>'
                           '</include></group></launch>' % planner)
        run_case(name, configure, exercise, ['roslaunch', str(wrapper)])


def run_planner_param_case(name, configure, exercise, overrides):
    source = Path(__file__).resolve().parents[1] / 'launch' / 'planning.launch'
    tree = ET.parse(source)
    node = tree.getroot().find("node[@name='planning']")
    for key, value in overrides.items():
        param = node.find("param[@name='%s']" % key)
        check(param is not None, 'Unknown planner parameter: ' + key)
        param.set('value', str(value))
    with tempfile.TemporaryDirectory() as directory:
        planner = Path(directory) / 'planning.launch'
        wrapper = Path(directory) / 'test.launch'
        tree.write(planner, encoding='unicode')
        wrapper.write_text('<launch><group ns="drone0"><include file="%s">'
                           '<arg name="car_grasp" value="true"/>'
                           '</include></group></launch>' % planner)
        run_case(name, configure, exercise, ['roslaunch', str(wrapper)])


def main():
    master=subprocess.Popen(['roscore','-p','11371'],stdout=subprocess.DEVNULL,stderr=subprocess.STDOUT)
    try:
        time.sleep(1.5)
        check(master.poll() is None,'Private ROS master did not start')
        rospy.init_node('moving_grasp_kinematic_checks',anonymous=True)
        run_case('stationary pregrasp bearing',lambda r:(setattr(r,'speed',0.0),setattr(r,'initial_error',0.7)),stationary_pregrasp)
        run_case('moving pregrasp velocity direction',lambda r:(setattr(r,'car_yaw',0.0),setattr(r,'initial_error',-0.7)),moving_pregrasp)
        run_case('turning pregrasp prediction',lambda r:(setattr(r,'turn_rate',0.15),setattr(r,'initial_error',0.7)),turning_pregrasp)
        run_case('pregrasp direction retained on stationary retrigger',configure_retained_direction,retained_pregrasp_direction)
        run_case('pregrasp reverse reconfirmed after retrigger',configure_retained_direction,reverse_during_retrigger)
        run_case('grasp point latch survives distance oscillation and resets on retrigger',
                 configure_grasp_point_latch,grasp_point_latch_and_retrigger)
        run_case('grasp point fallback after infeasible follow',
                 configure_grasp_point_fallback,grasp_point_fallback)
        run_planner_param_case('grasp point first plan failure brakes',
                 configure_first_grasp_plan_failure,first_grasp_plan_failure_brakes,
                 {'tracking_dur':'0.0'})
        run_case('landing hold refresh and flight loss',configure_landing_hold,landing_hold)
        run_case('landing hold status loss',configure_landing_hold,landing_hold_status_loss)
        run_case('landing offset then retrigger starts a fresh grasp',configure_landing_hold,landing_then_retrigger)
        run_case('landing hold after yaw change',configure_landing_yaw_change,
                 lambda r:landing_hold_yaw_continuity(r,False))
        run_case('landing hold while rotating',configure_landing_yaw_change,
                 lambda r:landing_hold_yaw_continuity(r,True))
        run_case('straight follow, timed close, lift and stop',lambda r:None,success)
        run_case('turning car velocity heading',lambda r:setattr(r,'turn_rate',0.10),success)
        run_case('straight velocity with rotating body',lambda r:setattr(r,'body_yaw_rate_offset',0.5),body_rotation_straight_velocity)
        run_case('optimized approach from 0.7 m lateral offset',lambda r:setattr(r,'initial_error',0.7),success)
        run_case('yaw gate',lambda r:setattr(r,'force_yaw',0.0),bad_yaw)
        run_case('slow motion with wrong yaw cannot close',configure_slow_bad_yaw,slow_speed_bad_yaw)
        run_case('relative speed gate',lambda r:setattr(r,'velocity_error',0.2),bad_speed)
        run_case('twist topic supplies velocity when pose is stationary',configure_static_velocity,explicit_velocity)
        run_case('stationary speed noise does not rotate yaw',configure_stationary_noise,stationary_noise)
        run_configured_case('direction filter cutoff in Hz',lambda r:None,direction_cutoff_configuration,
                 {'grasp_yaw_velocity_filter_hz':'2.0'})
        run_configured_case('velocity prediction filter 3 Hz',configure_velocity_filter,
                 lambda r:velocity_filter_prediction(r,True), {'target_velocity_cutoff_hz':'3.0'})
        run_configured_case('velocity prediction filter bypass 0 Hz',configure_velocity_filter,
                 lambda r:velocity_filter_prediction(r,False), {'target_velocity_cutoff_hz':'0'})
        run_configured_case('filtered speed gate allows stationary noise',configure_filtered_velocity_gate,filtered_velocity_gate,
                 {'target_velocity_cutoff_hz':'3.0', 'velocity_heading':'false'})
        run_configured_case('filtered abort ignores high frequency noise',configure_filtered_abort,filtered_abort_noise,
                 {'target_velocity_cutoff_hz':'3.0', 'velocity_heading':'false'})
        run_configured_case('filtered abort responds to sustained motion',configure_filtered_abort,filtered_abort_sustained_motion,
                 {'target_velocity_cutoff_hz':'3.0', 'velocity_heading':'false'})
        run_case('missing flight status blocks close',lambda r:setattr(r,'flight_status_on',False),absent_flight_status)
        run_case('bootstrap waits for active flight acknowledgement',lambda r:setattr(r,'flight_status_active',False),wait_for_flight_ack)
        run_case('flight-status loss during approach',lambda r:setattr(r,'force_yaw',0.0),lost_flight_during_approach)
        run_case('invalid flight states block close',lambda r:setattr(r,'flight_status_armed',False),invalid_flight_states)
        run_case('flight-status loss during closing',lambda r:None,lambda r:lost_flight_status(r,'closing_assumed'))
        run_case('flight-status loss during lift',lambda r:None,lambda r:lost_flight_status(r,'lifting_assumed'))
        run_case('transient wrong trajectory ACK latches abort',lambda r:None,transient_bad_ack)
        run_case('retrigger during approach opens first',lambda r:setattr(r,'force_yaw',0.0),lambda r:opening_after_trigger(r,'approach'))
        run_case('retrigger during closing opens first',lambda r:None,lambda r:opening_after_trigger(r,'closing_assumed'))
        run_case('retrigger during lifting opens first',lambda r:None,lambda r:opening_after_trigger(r,'lifting_assumed'))
        run_case('retrigger during braking opens first',lambda r:None,lambda r:opening_after_trigger(r,'braking'))
        run_case('retrigger during hold opens first',lambda r:None,lambda r:opening_after_trigger(r,'hold_assumed'))
        run_case('retrigger after manual movement uses current position',lambda r:None,retrigger_after_manual_move)
        run_case('repeated trigger restarts open timer',lambda r:None,repeated_opening)
        run_case('old trajectory ACK cannot authorize new mission',lambda r:None,old_ack_after_retrigger)
        run_case('retrigger opens immediately even without flight status',lambda r:None,reopens_without_status)
        run_case('retrigger after abort opens before restarting',lambda r:None,retrigger_aborted)
        run_case('missing twist prevents closing',lambda r:setattr(r,'twist_on',False),missing_twist)
        run_case('twist dropout during closing',lambda r:None,lost_twist)
        run_case('repeated twist timestamp during closing',lambda r:None,frozen_twist)
        run_case('target loss and explicit retrigger',lambda r:None,lost_target)
        run_case('odometry loss',lambda r:None,lost_odom)
        run_case('excessive closing error',lambda r:None,excessive_error)
        run_case('lift stall must abort',lambda r:None,stalled_lift)
        run_configured_case('configured lift error threshold',lambda r:None,configured_lift_error,
                 {'grasp_lift_abort_distance':'0.15'})
        # Existing generic simulation includes planning.launch directly and
        # receives target odometry without a separate twist topic.
        with tempfile.NamedTemporaryFile(mode='w',suffix='.launch') as launch:
            launch.write('''<launch><group ns="drone0">
              <include file="$(find planning)/launch/planning.launch">
                <arg name="rhosVisibility_" value="0"/>
                <arg name="rhoTracking_" value="1000"/>
                <arg name="target_name_" value="/vrpn_client_node/HIARM_car/pose"/>
              </include></group></launch>''')
            launch.flush()
            run_case('generic planning preserves pose-difference compatibility',
                     lambda r:setattr(r,'twist_on',False),success,['roslaunch',launch.name])
            run_case('generic planning publishes without flight status or twist',
                     lambda r:(setattr(r,'twist_on',False),
                               setattr(r,'flight_status_on',False),
                               setattr(r,'initial_error',0.7)),
                     generic_without_flight_status,['roslaunch',launch.name])
    finally:
        master.terminate()
        try:master.wait(8)
        except subprocess.TimeoutExpired:master.kill();master.wait()

if __name__=='__main__': main()
