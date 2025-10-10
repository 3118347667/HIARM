#!/usr/bin/env python3

import rospy
import rosbag
from flygripper_msgs.msg import PolynomialTrajectory
import time


if __name__ == '__main__':
    # Replace 'your_bag_file.bag' with the actual bag file path
    # traj_v13_a17.bag
    # traj_v21_a35.bag
    # traj_v3_a5.bag
    # traj_v5_a8.bag

    bag_file = '/home/flygripperyoung/workspace/flygripper_robotics/traj_v3_a5.bag'
    topic_name = '/flygripper_simple/trajectory'
    pub_topic_name = '/flygripper/trajectory'

    traj_repub = rospy.Publisher(pub_topic_name, PolynomialTrajectory, queue_size=10)
    rospy.init_node('trajectory_republisher')
    # Create a publisher for the republished trajectory
    time.sleep(1)
    # Open the bag file
    with rosbag.Bag(bag_file, 'r') as bag:
        # types = bag.get_type_and_topic_info()[0].keys()
        # topics = bag.get_type_and_topic_info()[1].keys()
        for topic, msg, t in bag.read_messages(topics=[topic_name]):
            ros_msg = msg
            # Break after republishing the first message
            break

    current_time = rospy.Time.now()
    ros_msg.header.stamp = current_time
    ros_msg.header.seq = 6
    traj_repub.publish(ros_msg)
    rospy.loginfo(
        "Republished trajectory message with name {} at time {}.".format(topic_name, ros_msg.header.stamp))


