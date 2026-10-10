#!/usr/bin/env python3
"""Audit attachment, carried-object motion and release timing independently of planner completion."""
# 从任务日志、Gazebo 抓取插件和物体位置采样交叉检查四个物体的搬运过程。
import json
import math
import sys
from pathlib import Path


def evaluate(report):
    # scene-matching.json 给出每个物体对应的抓取阶段和 Gazebo 模型名。
    rows=[]
    for obj in report['scene_matching']['objects']:
        stage=obj['stage']; name=obj['model']
        events=report['grasp_events']
        # 抓取动作通常发生在 stage+1 的展开之前；先定位该次计划展开时间。
        opening=next((e['sim_time'] for e in report['events'] if f'stage {stage+1} with morphing EXPANDING' in e['message']),None)
        # The physical pickup can begin before the contact plugin's debounce finishes.
        # Match the actual object identity and require attachment before planned opening.
        plugin=report.get('grasp_plugin_events',[])
        # 插件日志用于确认抓到的是目标模型；Bool 状态再与插件日志按时间配对。
        attach_log=next((e for e in plugin if name+'::' in e['message'] and 'has been grasped' in e['message']
                         and e['stage'] in (stage,stage+1) and opening and e['sim_time']<opening),None)
        attach=next((e for e in events if e['attached'] and attach_log and abs(e['sim_time']-attach_log['sim_time'])<.3),None)
        release_log=next((e for e in plugin if name+'::' in e['message'] and 'has been released' in e['message']
                          and attach and e['sim_time']>attach['sim_time']),None)
        release=next((e for e in events if not e['attached'] and release_log and abs(e['sim_time']-release_log['sim_time'])<.3),None)
        opened=next((e['sim_time'] for e in report['events'] if f'stage {stage+1} Morphing reached' in e['message']),None)
        samples=[]
        # 只取附着到释放之间的目标物体位置，用来验证物体是否随无人机移动。
        if attach and release:
            for r in report['object_samples']:
                if attach['sim_time'] <= r['sim_time'] <= release['sim_time']:
                    pose=next((p for k,p in r['objects'].items() if k.startswith(name+'::')),None)
                    if pose is not None: samples.append((r,pose))
        travel=math.dist(samples[0][1],samples[-1][1]) if len(samples)>1 else 0
        distances=[math.dist(p,r['robot_position']) for r,p in samples if r.get('robot_position')]
        # 成功条件包含：正确物体身份、一直拿到计划展开释放、且物体确实随飞机搬运。
        identity=bool(attach and any(name+'::' in e['message'] and 'has been grasped' in e['message']
                      and abs(e['sim_time']-attach['sim_time'])<.3 for e in report.get('grasp_plugin_events',[])))
        held=bool(attach and release and opening and opened is not None and opening <= release['sim_time'] <= opened+.2)
        carried=bool(len(distances)>2 and travel>.15 and max(distances)<.4)
        rows.append(dict(stage=stage,object=name,attachment_sim_time=attach['sim_time'] if attach else None,
                         release_sim_time=release['sim_time'] if release else None,planned_opening_sim_time=opening,
                         held_until_opening=held,carried_distance_m=travel,
                         max_object_to_robot_distance_m=max(distances) if distances else None,
                         carried_motion_verified=carried,object_identity_verified=identity,passed=held and carried and identity))
    # 此报告不检查最终落点/接收区域精度，只验证附着、搬运和释放时序。
    return dict(route_completed=report['passed'],completed_stage_count=len(report['completed']),objects=rows,
                all_transfers_verified=report['passed'] and len(rows)==4 and all(r['passed'] for r in rows),
                scope='Four dynamic objects attached through contact, carried near the robot, released after planned opening; landing/recipient accuracy is not checked.')

if __name__=='__main__':
    # 独立命令行模式：读取 mission.json，输出并保存同目录的转移检查结果。
    path=Path(sys.argv[1])
    result=evaluate(json.loads(path.read_text()))
    output=path.with_name(path.stem+'-result.json')
    output.write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(result,indent=2))
