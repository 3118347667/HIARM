#!/usr/bin/env python3
"""Verify a closed ROS1 bag, not just the recorder's subscription list."""
import json
import sys
import rosbag


def verify(filename, mav_name):
    prefix = "/" + mav_name.strip("/")
    snapshot_topic = prefix + "/rosparam_snapshot"
    required = ["/rosparam_snapshot", snapshot_topic] + [prefix + suffix for suffix in (
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
        for _, message, _ in bag.read_messages(topics=["/rosparam_snapshot"]):
            full = json.loads(message.data)
            if full.get("namespace") != "/" or not isinstance(full.get("parameters"), dict) or not full["parameters"]:
                raise RuntimeError("Complete ROS parameter snapshot content is invalid")
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
        planning_topic = "/drone0/planning/rosparam_snapshot"
        if planning_topic in topics and topics[planning_topic].message_count:
            for _, message, _ in bag.read_messages(topics=[planning_topic]):
                planning = json.loads(message.data)
                if (planning.get("namespace") != "/drone0/planning" or
                        not isinstance(planning.get("parameters"), dict) or
                        not planning["parameters"]):
                    raise RuntimeError("Planning parameter snapshot content is incomplete")
            print("Recorded: {} ({} messages)".format(
                planning_topic, topics[planning_topic].message_count))
        else:
            print("WARNING: no planning parameter snapshot recorded; "
                  "planning may not have started during recording", file=sys.stderr)
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
