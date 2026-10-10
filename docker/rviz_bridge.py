#!/usr/bin/env python3
"""Publish visualization-only TF from Gazebo ground-truth link poses."""
# 将 Gazebo 真值位姿转换成 RViz 可显示的 TF；这些变换只服务可视化，不参与飞控反馈。
import xml.etree.ElementTree as ET
import rospy
import tf2_ros
from gazebo_msgs.msg import LinkStates
from geometry_msgs.msg import TransformStamped
from tf.transformations import quaternion_from_euler

rospy.init_node('gazebo_rviz_frames')
broadcaster = tf2_ros.TransformBroadcaster()
static = tf2_ros.StaticTransformBroadcaster()
last = 0.0
known = None
robot = ET.fromstring(rospy.get_param('/flygripper/robot_description'))


def transform(parent, child, xyz, quat, stamp):
    # 将父/子坐标系、平移、四元数和时间戳组装成一个 TF 消息。
    t = TransformStamped()
    t.header.frame_id, t.child_frame_id, t.header.stamp = parent, child, stamp
    t.transform.translation.x, t.transform.translation.y, t.transform.translation.z = xyz
    t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w = quat
    return t


def update(msg):
    # Gazebo 给出机器人每个 link 在 world 下的真值位姿；限速至约 30 Hz 后广播。
    global last, known
    stamp = rospy.Time.now()
    now = stamp.to_sec()
    if 0 <= now-last < 1/30:
        return
    last = now
    poses = {n.split('::', 1)[1]: p for n, p in zip(msg.name, msg.pose) if n.startswith('flygripper::')}
    if not poses:
        return
    if known != set(poses):
        # Gazebo 可能把固定关节合并进父 link；为未单独出现的固定子 link 补发静态 TF。
        known = set(poses)
        fixed = []
        for joint in robot.findall('joint'):
            child = joint.find('child').get('link')
            if joint.get('type') != 'fixed' or child in known:
                continue
            origin = joint.find('origin')
            xyz = list(map(float, origin.get('xyz', '0 0 0').split())) if origin is not None else [0]*3
            rpy = list(map(float, origin.get('rpy', '0 0 0').split())) if origin is not None else [0]*3
            fixed.append(transform('flygripper/'+joint.find('parent').get('link'),
                                   'flygripper/'+child, xyz, quaternion_from_euler(*rpy), stamp))
        if fixed:
            static.sendTransform(fixed)
    # 动态发布 Gazebo 中实际存在的 link 位姿，坐标父系统一为 world。
    broadcaster.sendTransform([transform('world', 'flygripper/'+name,
        [p.position.x, p.position.y, p.position.z],
        [p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w], stamp)
        for name, p in poses.items()])


rospy.Subscriber('/gazebo/link_states', LinkStates, update, queue_size=1)
rospy.spin()
