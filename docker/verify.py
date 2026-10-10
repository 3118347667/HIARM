#!/usr/bin/env python3
"""Check live Gazebo feedback, takeoff/hover, and the original trajectory planner."""
# 这是短 demo 的运行验收，不执行四物体十阶段抓取任务。
import json
import math
import time
from collections import Counter, deque
from pathlib import Path

import rosnode
import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from flygripper_msgs.msg import ActuatorState, PolynomialTrajectory
from mavros_msgs.msg import ActuatorControl

counts = Counter()
# 回调只保留最近有限数量的里程计点，避免长时间运行时缓存无限增长。
samples = deque(maxlen=5000)
last = {}

def odom(msg):
    # 汇总位置、速度和姿态，并更新里程计消息计数。
    p = msg.pose.pose.position
    v = msg.twist.twist.linear
    q = msg.pose.pose.orientation
    item = (msg.header.stamp.to_sec(), p.x, p.y, p.z, v.x, v.y, v.z, q.x, q.y)
    samples.append(item)
    last['odom'] = item
    counts['odom'] += 1

def count(name):
    # 为其他 ROS topic 生成通用计数回调，记录消息数及最近接收墙钟时间。
    def cb(msg):
        counts[name] += 1
        last[name] = time.monotonic()
    return cb


def wait_until(predicate, seconds, description):
    # 在限时内等待条件成立；超时错误带上最后一次里程计，方便排查位置异常。
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline and not rospy.is_shutdown():
        if predicate():
            return
        time.sleep(0.1)
    raise RuntimeError(description + '; last odometry=' + repr(last.get('odom')))

def stable_at(target):
    # 仅检查最近三秒数据；既要求位置误差小，也要求线速度低。
    if not samples:
        return False
    rows = [s for s in list(samples) if s[0] >= samples[-1][0] - 3.0]
    if len(rows) < 50 or rows[-1][0] - rows[0][0] < 2.8:
        return False
    return all(math.dist(s[1:4], target) < 0.15 and math.sqrt(sum(x*x for x in s[4:7])) < 0.25 for s in rows)

report = {'test': 'Gazebo + flight controller + original real planner', 'passed': False}
try:
    # rospy 初始化时会决定时钟来源，因此先等 /use_sim_time 再 init_node。
    # rospy chooses its clock during init_node. Wait BEFORE initialization so
    # startup cannot permanently select wall time while roslaunch is still booting.
    print('Waiting for ROS master and /use_sim_time=true (up to 180 seconds)...', flush=True)
    deadline = time.monotonic() + 180
    readiness_error = 'ROS master has not enabled /use_sim_time'
    while time.monotonic() < deadline:
        try:
            if rospy.get_param('/use_sim_time', False):
                break
            readiness_error = '/use_sim_time is missing or false'
        except Exception as exc:
            readiness_error = str(exc)
        time.sleep(0.2)
    else:
        raise RuntimeError('Simulation startup timed out: ' + readiness_error +
                           '; inspect docker/logs/simulation.log and container logs')
    rospy.init_node('hiarm_verify', anonymous=True)
    # 订阅飞行状态、传感器、执行器、旋翼输出和规划器轨迹作为验收证据。
    subscriptions = [rospy.Subscriber('/flygripper/odom', Odometry, odom, queue_size=100)]
    for topic, cls, name in [('/flygripper/imu', Imu, 'imu'),
                             ('/flygripper/actuator_state', ActuatorState, 'actuator_state'),
                             ('/flygripper/rotor_command', ActuatorControl, 'rotor_command'),
                             ('/flygripper/trajectory', PolynomialTrajectory, 'trajectory')]:
        subscriptions.append(rospy.Subscriber(topic, cls, count(name), queue_size=100))
    required = {'/gazebo', '/flygripper/mavros_node', '/flygripper/flygripper_flight_node', '/flygripper/real', '/hiarm_virtual_rc'}
    # 先确认节点和模拟传感器在线，再开始切换遥控模式。
    wait_until(lambda: required.issubset(set(rosnode.get_node_names())), 120, 'Required ROS nodes did not start')
    wait_until(lambda: counts['odom'] > 50 and counts['imu'] > 50 and counts['actuator_state'] > 10, 120, 'Missing simulated sensor feedback')
    # 每轮从 manual 状态开始，先让中立 RC 输入持续一秒，再触发模式边沿。
    assert rospy.get_param('/hiarm/demo_mode', 'manual') == 'manual', 'Restart the simulation before repeating verification'
    t0 = samples[-1][0]
    wait_until(lambda: samples[-1][0] - t0 > 1.0, 30, 'Simulation clock did not advance')
    rospy.set_param('/hiarm/demo_mode', 'hover')
    # 先检查自动起飞并悬停在 1 米，再触发原始轨迹规划器飞向单个目标点。
    print('Taking off to 1 m...', flush=True)
    wait_until(lambda: stable_at((0, 0, 1)), 240, 'Failed to take off and hold 1 m')
    report['hover_position_m'] = list(samples[-1][1:4])
    print('Hover verified; triggering the original planner...', flush=True)
    before_traj = counts['trajectory']
    rospy.set_param('/hiarm/demo_mode', 'trajectory')
    wait_until(lambda: counts['trajectory'] > before_traj, 60, 'Planner did not publish a trajectory')
    wait_until(lambda: stable_at((0.6, 0, 1)), 240, 'Failed to reach and hold the planned target')
    report['target_position_m'] = [0.6, 0.0, 1.0]
    report['final_position_m'] = list(samples[-1][1:4])
    report['position_error_m'] = math.dist(report['final_position_m'], report['target_position_m'])
    sim_start, wall_start = samples[-1][0], time.monotonic()
    # 用一段墙钟等待估计仿真实时因子，便于判断仿真运行速度。
    time.sleep(5)
    report['real_time_factor_5s'] = (samples[-1][0] - sim_start) / (time.monotonic() - wall_start)
    assert counts['rotor_command'] > 100, 'Missing controller output'
    report['nodes'] = rosnode.get_node_names()
    report['passed'] = True
except Exception as exc:
    # 记录错误后继续抛出，调用方能通过退出码识别验收失败。
    report['error'] = str(exc)
    raise
finally:
    # 成功或失败都保存计数、UTC 时间和 JSON 结果。
    report['message_counts'] = dict(counts)
    report['timestamp_utc'] = time.strftime('%Y-%m-%dT%H:%M:%SZ', time.gmtime())
    Path('/ws/docker/logs').mkdir(parents=True, exist_ok=True)
    Path('/ws/docker/logs/verification.json').write_text(json.dumps(report, indent=2) + '\n')
    print(json.dumps(report, indent=2), flush=True)
