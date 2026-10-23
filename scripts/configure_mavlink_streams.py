#!/usr/bin/env python3
"""Apply bounded telemetry rates on the ground, checking MAVLink command ACKs.

This configures telemetry only. Requested rates must still be measured. Optional
streams use -1 to disable; MAVLink interval 0 would restore the default instead.
"""
import argparse
import math
import os
import sys


def stream_profile():
    streams = [
        (27, "RAW_IMU", 150),
        (30, "ATTITUDE", 150),
        (65, "RC_CHANNELS", 50),
        (147, "BATTERY_STATUS", 3),
        # (11030, "ESC_TELEMETRY_1_TO_4", 10),  # unused in attitude output mode
        (11030, "ESC_TELEMETRY_1_TO_4", -1),  # explicitly stop any earlier request
        (245, "EXTENDED_SYS_STATE", 5),
        (100, "OPTICAL_FLOW", -1),
        (250, "DEBUG_VECTOR", -1),
        (385, "TUNNEL", -1),
    ]
    result = []
    for msg_id, name, default in streams:
        rate = float(os.environ.get("MAVLINK_" + name + "_RATE", default))
        if not math.isfinite(rate) or not (0 < rate <= 1000 or rate == -1):
            raise ValueError("MAVLINK_{}_RATE must be -1 or in (0, 1000]".format(name))
        if msg_id in (27, 30, 65, 147, 245) and rate <= 0:
            raise ValueError(name + " is required by the flight profile")
        result.append((msg_id, name, rate))
    return result


def apply_profile(streams, send, ensure_grounded, report):
    for msg_id, name, rate in streams:
        ensure_grounded()
        interval = -1.0 if rate < 0 else 1e6 / rate
        response = send(msg_id, interval)
        if not response.success or response.result != 0:
            error = "{} request rejected: result={}".format(name, response.result)
            if rate < 0:
                # A firmware may not implement an unused message. Keep the
                # failure visible without preventing core telemetry startup.
                report("WARNING: " + error + "; disable was NOT confirmed")
                continue
            raise RuntimeError(error)
        report("{} ({}): {}".format(name, msg_id,
               "disabled" if rate < 0 else "requested {:g} Hz".format(rate)))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--dry-run", action="store_true")
    args = parser.parse_args()
    streams = stream_profile()
    if args.dry_run:
        for msg_id, name, rate in streams:
            print("{} {} {:g}".format(msg_id, name, rate))
        return
    import rospy
    from mavros_msgs.msg import State
    from mavros_msgs.srv import CommandLong
    rospy.init_node("hiarm_configure_streams", anonymous=True)
    rospy.wait_for_service("/mavros/cmd/command", timeout=20)
    command = rospy.ServiceProxy("/mavros/cmd/command", CommandLong)

    def ensure_grounded():
        state = rospy.wait_for_message("/mavros/state", State, timeout=3)
        age = (rospy.Time.now() - state.header.stamp).to_sec()
        if not state.connected or state.armed or not (0 <= age <= 3):
            raise RuntimeError("Telemetry setup requires a fresh connected, DISARMED FCU")

    def send(msg_id, interval):
        return command(broadcast=False, command=511, confirmation=0,
                       param1=msg_id, param2=interval, param3=0, param4=0,
                       param5=0, param6=0, param7=0)
    apply_profile(streams, send, ensure_grounded, rospy.loginfo)
    rospy.loginfo("Telemetry profile applied; verify actual rates and header ages before flight.")


if __name__ == "__main__":
    try:
        main()
    except Exception as error:
        print("\033[1;31mTelemetry setup failed: {}\033[0m".format(error), file=sys.stderr)
        sys.exit(1)
