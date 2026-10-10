#!/usr/bin/env python3
"""Simulation-only RC: persistent neutral input, mode selected by ROS parameter."""
import rospy
from mavros_msgs.msg import RCIn

rospy.init_node('hiarm_virtual_rc')
if not rospy.get_param('/use_sim_time', False):
    raise SystemExit('Refusing to publish RC without /use_sim_time=true')
pub = rospy.Publisher('/mavros/rc/in', RCIn, queue_size=1)
rate = rospy.Rate(30)
while not rospy.is_shutdown():
    mode = rospy.get_param('/hiarm/demo_mode', 'manual')
    msg = RCIn()
    msg.header.stamp = rospy.Time.now()
    msg.rssi = 255
    msg.channels = [1500, 1500, 1500, 1500, 1060, 1060, 1060, 1060, 1060, 1060, 1060, 1060]
    if mode in ('hover', 'trajectory'):
        msg.channels[4] = 1940
    if mode == 'trajectory':
        msg.channels[5] = 1940
    # Original planner's TOGGLE_MORPH is channel 7 (zero based).
    if rospy.get_param('/hiarm/morph_high', False):
        msg.channels[7] = 1940
    pub.publish(msg)
    rate.sleep()
