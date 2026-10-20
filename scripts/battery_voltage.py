#!/usr/bin/env python3
"""Print the battery voltage reported by MAVROS."""

import math

import rospy
from sensor_msgs.msg import BatteryState


def battery_callback(msg):
    voltage = msg.voltage
    if not math.isfinite(voltage):
        rospy.logwarn_throttle(5.0, "MAVROS battery voltage is unavailable (NaN/Inf)")
        return

    rospy.loginfo_throttle(1.0, "Battery voltage: %.2f V", voltage)


def main():
    rospy.init_node("battery_voltage")
    topic = rospy.get_param("~topic", "/mavros/battery")
    rospy.loginfo("Listening for battery voltage on %s", topic)
    rospy.Subscriber(topic, BatteryState, battery_callback, queue_size=10)
    rospy.spin()


if __name__ == "__main__":
    main()
