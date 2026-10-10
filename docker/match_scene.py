#!/usr/bin/env python3
"""Generate an auditable world matched to the unchanged real_sim grasp waypoints."""
# 根据原任务航点和机体夹爪偏移摆放四个物体，生成独立的仿真场景与匹配记录。
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parent.parent
# 夹爪内部抓取腔相对于里程计 link 的机体坐标偏移，单位为米。
OFFSET = (.09, -.09, -.01)
# 每项依次为：抓取阶段、模型名、最终里程计目标、模型几何中心偏移和底部高度。
OBJECTS = [
    (1, 'coke_can', (7.80,.18,.94,.68), (-.000568,-.000706,.06445), .0025),
    (3, 'soccer_ball', (6.19,.14,.51,1.57), (0,0,0), -.05),
    (5, 'mustard_bottle', (6.13,-.74,.93,-1.57), (-.0230085,-.03524775,.13874625), -.0047295),
    (7, 'unit_box', (5.10,2.22,.45,.44), (0,0,0), -.05),
]

def generate():
    # 从基础世界开始，移除原有场景物体，只保留太阳和地面，再放入匹配后的抓取物。
    tree = ET.parse(ROOT/'src/flygripper_sim/flygripper_gazebo/worlds/basic.world')
    world = tree.getroot().find('world')
    for inc in list(world.findall('include')):
        if inc.findtext('uri') not in ('model://sun','model://ground_plane'):
            world.remove(inc)
    rows = []
    for stage, model, goal, center, bottom in OBJECTS:
        # 根据目标 yaw 将夹爪偏移旋转到世界坐标，再反算模型实体的世界位姿。
        x,y,z,yaw = goal
        c,s = math.cos(yaw), math.sin(yaw)
        # The tall can must sit above the upper rotors' swept collision volume.
        offset = {1: (.065,-.065,.06), 7: (.075,-.075,-.01)}.get(stage, OFFSET)
        target = [x+c*offset[0]-s*offset[1], y+s*offset[0]+c*offset[1], z+offset[2]]
        pose = [target[0]-c*center[0]+s*center[1],target[1]-s*center[0]-c*center[1],target[2]-center[2],0,0,yaw]
        if model == 'coke_can':
            # 易拉罐使用单独配置的圆柱碰撞体和接触参数，其他模型沿用模型库资源。
            entity = ET.parse(ROOT/'src/flygripper_description/models/coke_can/model.sdf').getroot().find('model')
            entity.set('name', model+'_demo')
            ET.SubElement(entity,'pose').text = ' '.join(map(str,pose))
            collision = entity.find('link/collision')
            collision.find('pose').text = ' '.join(map(str,center))+' 0 0 0'
            geometry = collision.find('geometry'); geometry.clear()
            cylinder=ET.SubElement(geometry,'cylinder')
            ET.SubElement(cylinder,'radius').text='.0335'
            ET.SubElement(cylinder,'length').text='.1239'
            contact=collision.find('surface/contact/ode')
            # Contact parameters of the validated cylinder collision model.
            contact.find('kp').text='100000'
            contact.find('kd').text='100'
            contact.find('max_vel').text='0.1'
            contact.find('min_depth').text='0.001'
            world.append(entity)
        else:
            # 其余物体通过 Gazebo include 引入，在新位姿下命名为 *_demo。
            inc = ET.SubElement(world,'include')
            ET.SubElement(inc,'uri').text = 'model://'+model
            ET.SubElement(inc,'name').text = model+'_demo'
            ET.SubElement(inc,'pose').text = ' '.join(map(str,pose))
        height = pose[2]+bottom
        # 添加静态支撑台，使物体底部有支撑；易拉罐使用窄支柱避开折叠后的旋翼。
        pedestal = ET.SubElement(world,'model',name='support_stage_'+str(stage))
        ET.SubElement(pedestal,'static').text = 'true'
        ET.SubElement(pedestal,'pose').text = f'{target[0]} {target[1]} {height/2} 0 0 {yaw}'
        # A narrow central post leaves clearance for the folded rotor bodies.
        width = .01 if stage == 1 else .18
        link = ET.SubElement(pedestal,'link',name='link')
        for kind in ('collision','visual'):
            item=ET.SubElement(link,kind,name=kind)
            ET.SubElement(ET.SubElement(ET.SubElement(item,'geometry'),'box'),'size').text=f'{width} {width} {height}'
            if kind=='visual':
                ET.SubElement(ET.SubElement(item,'material'),'diffuse').text='.22 .38 .46 1'
        rows.append(dict(stage=stage,model=model+'_demo',odom_goal=goal,body_grip_offset=offset,grip_center=target,model_pose=pose,support_height=height,support_width=width))
    # 同时输出可直接加载的世界文件和每个物体的几何/目标匹配审计数据。
    ET.indent(tree,space='  ')
    tree.write(ROOT/'docker/matched.world',encoding='unicode',xml_declaration=True)
    (ROOT/'docker/scene-matching.json').write_text(json.dumps(dict(body_grip_offset=OFFSET,objects=rows),indent=2)+'\n')

if __name__=='__main__': generate()
