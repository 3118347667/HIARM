#!/usr/bin/env python3
"""Print the battery voltage reported by MAVROS."""

import argparse
import math
import sys
import time

import rospy
from sensor_msgs.msg import BatteryState


def battery_callback(msg):
    voltage = msg.voltage
    if not math.isfinite(voltage) or voltage <= 0:
        rospy.logwarn_throttle(5.0, "MAVROS battery voltage is unavailable")
        return

    rospy.loginfo_throttle(1.0, "Battery voltage: %.2f V", voltage)


def main():
    parser = argparse.ArgumentParser(description="Read battery voltage from MAVROS")
    parser.add_argument("--once", action="store_true", help="print one valid reading and exit")
    parser.add_argument("--timeout", type=float, default=10.0,
                        help="seconds to wait for a valid reading with --once (default: 10)")
    parser.add_argument("--topic", default=None, help="battery topic (default: /mavros/battery)")
    args = parser.parse_args(rospy.myargv()[1:])
    if args.once and (not math.isfinite(args.timeout) or args.timeout <= 0):
        parser.error("--timeout must be a positive finite number")

    rospy.init_node("battery_voltage", anonymous=True)
    topic = args.topic or rospy.get_param("~topic", "/mavros/battery")
    if args.once:
        deadline = time.monotonic() + args.timeout
        while not rospy.is_shutdown():
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                break
            try:
                msg = rospy.wait_for_message(topic, BatteryState, timeout=remaining)
            except rospy.ROSException:
                break
            if math.isfinite(msg.voltage) and msg.voltage > 0:
                print("Battery voltage: {:.2f} V".format(msg.voltage))
                return 0
        print("No valid battery voltage received on {} within {:.1f} s".format(
            topic, args.timeout), file=sys.stderr)
        return 1

    rospy.loginfo("Listening for battery voltage on %s", topic)
    rospy.Subscriber(topic, BatteryState, battery_callback, queue_size=10)
    rospy.spin()
    return 0


if __name__ == "__main__":
    sys.exit(main())
