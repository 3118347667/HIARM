#!/usr/bin/env python3
"""Publish a latched ROS parameter snapshot for rosbag record."""

import datetime
import json
import sys

import rosnode
import rospy
from std_msgs.msg import String


def main():
    mav_name, ready_file, skipped_file = sys.argv[1:]
    namespace = "/" + mav_name
    rospy.init_node("rosparam_snapshot", anonymous=True)

    parameters = rospy.get_param(namespace, None)
    if not isinstance(parameters, dict) or not isinstance(
        parameters.get("geometric_controller"), dict
    ):
        rospy.loginfo("Controller parameters are not loaded in %s; skipping snapshot", namespace)
        with open(skipped_file, "w", encoding="utf-8") as marker:
            marker.write(namespace + "\n")
        return

    flight_node = namespace + "/flygripper_flight_node"
    try:
        flight_node_running = rosnode.rosnode_ping(flight_node, max_count=1, verbose=False)
    except (rosnode.ROSNodeIOException, OSError) as error:
        rospy.logwarn("Cannot check whether %s is running: %s", flight_node, error)
        flight_node_running = False
    if not flight_node_running:
        rospy.loginfo("Flight node %s is not running; skipping snapshot", flight_node)
        with open(skipped_file, "w", encoding="utf-8") as marker:
            marker.write(namespace + "\n")
        return

    snapshot = {
        "namespace": namespace,
        "captured_at_utc": datetime.datetime.now(datetime.timezone.utc).isoformat(),
        "ros_time": rospy.Time.now().to_sec(),
        "parameters": parameters,
    }
    topic = namespace + "/rosparam_snapshot"
    publisher = rospy.Publisher(topic, String, queue_size=1, latch=True)
    publisher.publish(String(data=json.dumps(snapshot, sort_keys=True)))
    with open(ready_file, "w", encoding="utf-8") as marker:
        marker.write(topic + "\n")
    rospy.loginfo("Published ROS parameter snapshot on %s", topic)
    rospy.spin()


if __name__ == "__main__":
    main()
