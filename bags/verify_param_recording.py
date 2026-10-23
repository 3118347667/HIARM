#!/usr/bin/env python3
"""Verify a closed ROS1 bag, not just the recorder's subscription list."""
import json
import sys
import rosbag


def verify(filename, mav_name):
    prefix = "/" + mav_name.strip("/")
    snapshot_topic = prefix + "/rosparam_snapshot"
    required = [snapshot_topic] + [prefix + suffix for suffix in (
        "/geometric_controller/parameter_updates",
        "/robot_parameter/parameter_updates",
        "/flygripper_flight_node/parameter_updates",
    )]
    with rosbag.Bag(filename, "r") as bag:
        topics = bag.get_type_and_topic_info().topics
        missing = [topic for topic in required
                   if topic not in topics or topics[topic].message_count < 1]
        if missing:
            raise RuntimeError("Missing recorded messages: " + ", ".join(missing))
        _, message, _ = next(bag.read_messages(topics=[snapshot_topic]))
        snapshot = json.loads(message.data)
        if snapshot.get("namespace") != prefix or not all(
            isinstance(snapshot.get("parameters", {}).get(key), dict)
            and snapshot["parameters"][key]
            for key in ("geometric_controller", "robot_parameter")
        ):
            raise RuntimeError("Parameter snapshot content is incomplete")
        for topic in required:
            print("Recorded: {} ({} messages)".format(topic, topics[topic].message_count))
        for topic in ("/rosout", "/rosout_agg", prefix + "/runtime_diagnostics",
                      "/diagnostics", "/mavros/timesync_status",
                      "/mavros/imu/data_raw"):
            if topic in topics and topics[topic].message_count:
                print("Recorded: {} ({} messages)".format(topic, topics[topic].message_count))
            else:
                print("WARNING: no recorded messages on " + topic, file=sys.stderr)


if __name__ == "__main__":
    try:
        verify(*sys.argv[1:])
    except Exception as error:
        print("\033[1;31mBag verification FAILED: {}\033[0m".format(error), file=sys.stderr)
        sys.exit(1)
    print("Parameter recording verification passed.")
