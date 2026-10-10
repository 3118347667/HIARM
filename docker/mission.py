#!/usr/bin/env python3
"""Run the original ten-stage planner with simulation approach overrides and audit each carried object."""
# 本脚本负责启动顺序、过程监控和结果验收；轨迹生成与飞行控制仍由 ROS 规划器/飞控节点完成。
import fcntl
import hashlib
import json
import math
import re
import time
from collections import deque
from grasp_report import evaluate
from shake_report import evaluate as evaluate_shake
from pathlib import Path

import rospy
from nav_msgs.msg import Odometry
from gazebo_msgs.msg import LinkStates
from rosgraph_msgs.msg import Log
from std_msgs.msg import Bool, String


def main():
    # 用文件锁避免同一仿真中误启动两份 mission.py。
    lock = open('/tmp/hiarm-mission.lock', 'w')
    fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    # 必须在 init_node 前等仿真时钟，否则 rospy 可能在启动竞态中选用真实墙钟。
    deadline = time.monotonic() + 180
    while time.monotonic() < deadline:
        try:
            if rospy.get_param('/use_sim_time', False):
                break
        except Exception:
            pass
        time.sleep(.2)
    else:
        raise RuntimeError('Simulation clock parameter not ready')
    rospy.init_node('hiarm_mission')
    # mission_status 使用 latch，晚加入的网页节点也能立即拿到最近一次任务状态。
    publisher = rospy.Publisher('/hiarm/mission_status', String, queue_size=1, latch=True)
    output = Path('/ws/docker/logs/mission.json')
    report = dict(state='starting', stage=None, completed=[], events=[], grasp_events=[],
                  passed=False, physical_grasp_verified=False, automatic_manual_triggers=True,
                  scope='Original ten-stage endpoints with simulation grasp approach overrides',
                  object_samples=[], grasp_plugin_events=[], support_contacts=[], can_contacts=[])
    motion_file = open('/ws/docker/logs/motion.jsonl', 'w', buffering=1)
    motion_last = [0.0]
    odometry = deque(maxlen=2000)
    control = dict(waiting=None, done=False, failure=None, grasp=None)

    def save():
        # 同时写完整 JSON 文件和精简 ROS 状态；临时文件替换可避免留下半截报告。
        report['sim_time'] = rospy.Time.now().to_sec()
        report['timestamp_utc'] = time.strftime('%Y-%m-%dT%H:%M:%SZ', time.gmtime())
        if odometry:
            report['position'] = list(odometry[-1][1:4])
        payload = json.dumps(report, ensure_ascii=False)
        summary = {k:v for k,v in report.items() if k not in ('object_samples','task_parameters','scene_matching')}
        summary['events'] = summary['events'][-20:]
        publisher.publish(String(data=json.dumps(summary, ensure_ascii=False)))
        temp = output.with_suffix('.tmp')
        temp.write_text(payload + '\n')
        temp.replace(output)

    def odom(msg):
        # 保留有限长度的里程计窗口，并定期把姿态/速度写入运动审计文件。
        p, v = msg.pose.pose.position, msg.twist.twist.linear
        odometry.append((msg.header.stamp.to_sec(), p.x, p.y, p.z, v.x, v.y, v.z))
        now = msg.header.stamp.to_sec()
        if now-motion_last[0] >= .02:
            motion_last[0] = now
            q=msg.pose.pose.orientation; w=msg.twist.twist.angular
            motion_file.write(json.dumps(dict(t=now,stage=report['stage'],p=[p.x,p.y,p.z],q=[q.x,q.y,q.z,q.w],v=[v.x,v.y,v.z],omega=[w.x,w.y,w.z]))+'\n')

    def objects(msg):
        # 以约 10 Hz 保存 Gazebo 中演示物体和无人机的位置，供搬运验证使用。
        now = rospy.Time.now().to_sec()
        if report['object_samples'] and now-report['object_samples'][-1]['sim_time'] < .1:
            return
        poses = {n: [p.position.x,p.position.y,p.position.z] for n,p in zip(msg.name,msg.pose) if '_demo::' in n}
        report['object_samples'].append(dict(sim_time=now,stage=report['stage'],objects=poses,robot_position=list(odometry[-1][1:4]) if odometry else None))

    def grasp(msg):
        # 记录夹爪插件的附着/脱离布尔状态；任意一次附着只代表曾抓到过物体。
        control['grasp'] = msg.data
        report['grasp_events'].append(dict(sim_time=rospy.Time.now().to_sec(), attached=msg.data, stage=report['stage']))
        report['physical_grasp_verified'] |= msg.data

    def log(msg):
        # 收集接触插件诊断；规划器阶段状态只解析 real 节点发出的日志。
        if 'CanContact ' in msg.msg or 'CanRotorContact ' in msg.msg:
            report['can_contacts'].append(dict(sim_time=msg.header.stamp.to_sec(),message=msg.msg))
        if 'GripperSupportContact' in msg.msg:
            report['support_contacts'].append(dict(sim_time=msg.header.stamp.to_sec(),message=msg.msg))
        if 'GazeboGraspPlugin plugin: Object ' in msg.msg:
            report['grasp_plugin_events'].append(dict(sim_time=msg.header.stamp.to_sec(), message=msg.msg, stage=report['stage']))
        if msg.name != '/flygripper/real':
            return
        text = re.sub(r'\x1b\[[0-9;]*m', '', msg.msg)
        if 'stage' not in text and 'trajectory generation failed' not in text:
            return
        report['events'].append(dict(sim_time=msg.header.stamp.to_sec(), message=text))
        match = re.search(r'stage (\d+)', text)
        if match:
            stage = int(match.group(1))
            report['stage'] = stage
            if 'waiting manual morphing' in text:
                control['waiting'] = stage
            if re.search(r'stage \d+ Completed', text) and stage not in report['completed']:
                report['completed'].append(stage)
        if 'All stage reached' in text:
            control['done'] = True
        if 'trajectory generation failed' in text or 'Trajectory Failed' in text:
            control['failure'] = text
        print(text, flush=True)

    subscriptions = [rospy.Subscriber('/gazebo/link_states', LinkStates, objects, queue_size=1),
                     rospy.Subscriber('/flygripper/odom', Odometry, odom, queue_size=20),
                     rospy.Subscriber('/rosout_agg', Log, log, queue_size=100),
                     rospy.Subscriber('/flygripper/grasp_status', Bool, grasp, queue_size=20)]

    def wait(predicate, timeout, error):
        # 用墙钟限制等待时长，同时持续保存进度；仿真暂停时也不会无限等待。
        deadline = time.monotonic() + timeout
        while not rospy.is_shutdown() and time.monotonic() < deadline:
            if predicate():
                return
            save()
            time.sleep(.2)
        raise RuntimeError(error)

    def hover_stable():
        # 要求最近约三秒都接近 (0,0,1) 米且速度足够小，才认为起飞悬停完成。
        if not odometry:
            return False
        rows = [r for r in list(odometry) if r[0] > odometry[-1][0] - 3]
        return (len(rows) > 50 and rows[-1][0] - rows[0][0] > 2.8 and
                all(math.dist(r[1:4], (0, 0, 1)) < .15 and
                    math.sqrt(sum(v*v for v in r[4:7])) < .25 for r in rows))

    try:
        # 确认传感器、仿真时钟和十阶段原始任务已经就绪，并保存本轮输入配置指纹。
        wait(lambda: rospy.has_param('/flygripper/real/task_num') and len(odometry) > 50,
             180, 'Simulation did not become ready')
        if not rospy.get_param('/use_sim_time', False):
            raise RuntimeError('Requires simulation time')
        if (rospy.get_param('/hiarm/mission', '') != 'original' or
                rospy.get_param('/flygripper/real/task_num') != 10):
            raise RuntimeError('Start the original mission profile first')
        if rospy.get_param('/hiarm/demo_mode', 'manual') != 'manual':
            raise RuntimeError('Restart simulation before running a new mission')
        report['world_file'] = rospy.get_param('/hiarm/world_file')
        files = ['docker/matched.world', 'docker/grasp-plan.yaml',
                 'src/flygripper_description/urdf/flygripper_base.xacro',
                 'devel/lib/libgazebo_grasp_plugin.so', 'devel/lib/libgazebo_reconfigurable_plugin.so']
        report['input_sha256'] = {name: hashlib.sha256((Path('/ws')/name).read_bytes()).hexdigest() for name in files}
        report['scene_matching'] = json.loads(Path('/ws/docker/scene-matching.json').read_text())
        report['task_parameters'] = {f'task_{i}': rospy.get_param(f'/flygripper/real/task_{i}') for i in range(10)}
        rospy.set_param('/hiarm/morph_high', False)
        time.sleep(1)
        # 先通过虚拟遥控切到 hover；稳定后再切 trajectory，让 FSM 启动任务规划。
        report['state'] = 'takeoff'
        rospy.set_param('/hiarm/demo_mode', 'hover')
        wait(hover_stable, 240, 'Takeoff/hover did not stabilize')
        report['state'] = 'running'
        rospy.set_param('/hiarm/demo_mode', 'trajectory')
        deadline = time.monotonic() + 1200
        pulse_end = None
        triggered = set()
        while not rospy.is_shutdown() and not control['done']:
            # 安全中止条件：任务开始后高度过低、规划器报错或整轮任务超时。
            if odometry and odometry[-1][3] < .15 and report['stage'] is not None:
                raise RuntimeError('Vehicle fell below safe mission altitude; inspect grasp and controller logs')
            if control['failure']:
                raise RuntimeError(control['failure'])
            if time.monotonic() > deadline:
                raise RuntimeError('Original mission timed out')
            now = rospy.Time.now().to_sec()
            stage = control['waiting']
            # 仅当规划器明确等待手动 Morph 时才发脉冲；自动形变配置不会进入此分支。
            if stage is not None and stage not in triggered and pulse_end is None:
                rospy.set_param('/hiarm/morph_high', True)
                triggered.add(stage)
                pulse_end = now + .6
                report['events'].append(dict(sim_time=now, message=f'Virtual RC morph trigger for stage {stage}'))
            if pulse_end is not None and now >= pulse_end:
                rospy.set_param('/hiarm/morph_high', False)
                pulse_end = None
            save()
            time.sleep(.2)
        # 所有十阶段完成后再单独验收抓取转移与易拉罐稳定性；passed 本身表示路线完成。
        if rospy.is_shutdown() or set(report['completed']) != set(range(10)):
            raise RuntimeError('Mission interrupted or missing completed stages')
        report.update(state='completed', passed=True)
        report['grasp_validation'] = evaluate(report)
        report['all_transfers_verified'] = report['grasp_validation']['all_transfers_verified']
        motion_file.flush()
        motion_rows = [json.loads(line) for line in Path(motion_file.name).read_text().splitlines()]
        report['can_motion_validation'] = evaluate_shake(report, motion_rows)
        report['grasp_and_stability_verified'] = (report['all_transfers_verified'] and report['can_motion_validation']['stable'])
    except Exception as exc:
        # 异常写入报告后继续抛出，让容器命令也以失败状态结束。
        report.update(state='failed', error=str(exc))
        raise
    finally:
        # 无论成功或失败都把 Morph 输入拉低并保存最后状态。
        try:
            rospy.set_param('/hiarm/morph_high', False)
            save()
        finally:
            print(json.dumps(report, ensure_ascii=False, indent=2), flush=True)


if __name__ == '__main__':
    main()
