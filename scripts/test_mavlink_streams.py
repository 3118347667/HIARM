#!/usr/bin/env python3
"""Offline telemetry contract checks; --ros also exercises the real ROS CLI."""
import os
from pathlib import Path
import subprocess
import sys
from types import SimpleNamespace
import unittest
from unittest.mock import patch

from configure_mavlink_streams import apply_profile, stream_profile


class StreamContract(unittest.TestCase):
    def test_interval_units_and_explicit_disable(self):
        requests, reports = [], []
        def send(msg_id, interval):
            requests.append((msg_id, interval))
            return SimpleNamespace(success=True, result=0)
        apply_profile([(27, "RAW_IMU", 150), (100, "OPTICAL_FLOW", -1)],
                      send, lambda: None, reports.append)
        self.assertAlmostEqual(requests[0][1], 1e6 / 150)
        self.assertEqual(requests[1], (100, -1.0))

    def test_core_rejection_stops_remaining_requests(self):
        requests = []
        def send(msg_id, interval):
            requests.append(msg_id)
            return SimpleNamespace(success=False, result=4)
        with self.assertRaises(RuntimeError):
            apply_profile([(27, "RAW_IMU", 333), (30, "ATTITUDE", 333)],
                          send, lambda: None, lambda _: None)
        self.assertEqual(requests, [27])

    def test_ground_check_prevents_command(self):
        def reject():
            raise RuntimeError("armed")
        def send(*_):
            self.fail("A rejected ground check must not send a command")
        with self.assertRaises(RuntimeError):
            apply_profile([(27, "RAW_IMU", 333)], send, reject, lambda _: None)

    def test_optional_disable_rejection_is_visible(self):
        reports = []
        apply_profile([(100, "OPTICAL_FLOW", -1)],
                      lambda *_: SimpleNamespace(success=False, result=3),
                      lambda: None, reports.append)
        self.assertIn("disable was NOT confirmed", reports[0])

    def test_invalid_rate_cannot_silently_restore_default_or_remove_imu(self):
        for rate in ("0", "-1", "nan", "inf", "1001"):
            with self.subTest(rate=rate), patch.dict(os.environ, {"MAVLINK_RAW_IMU_RATE": rate}):
                with self.assertRaises(ValueError):
                    stream_profile()


def ros_contract():
    import threading
    import rospy
    from mavros_msgs.msg import State
    from mavros_msgs.srv import CommandLong, CommandLongResponse
    rospy.init_node("stream_configuration_contract", anonymous=True)
    requests = []
    armed = [False]
    response = [0]
    def command(req):
        requests.append(req)
        return CommandLongResponse(success=response[0] == 0, result=response[0])
    service = rospy.Service("/mavros/cmd/command", CommandLong, command)
    pub = rospy.Publisher("/mavros/state", State, queue_size=1)
    stop = threading.Event()
    def publish():
        while not stop.wait(0.03):
            state = State()
            state.header.stamp = rospy.Time.now()
            state.connected, state.armed = True, armed[0]
            pub.publish(state)
    thread = threading.Thread(target=publish, daemon=True)
    thread.start()
    script = str(Path(__file__).with_name("configure_mavlink_streams.py"))
    try:
        run = subprocess.run([sys.executable, script], capture_output=True, text=True, timeout=20)
        assert run.returncode == 0, run.stdout + run.stderr
        assert len(requests) == 9, len(requests)
        assert all(req.command == 511 for req in requests)
        assert [req.param1 for req in requests[:2]] == [27, 30]
        assert all(abs(req.param2 - 1e6 / 150) < 0.01 for req in requests[:2])
        assert [req.param2 for req in requests if req.param1 in (100, 250, 385, 11030)] == [-1.0] * 4
        requests.clear()
        armed[0] = True
        run = subprocess.run([sys.executable, script], capture_output=True, text=True, timeout=10)
        assert run.returncode != 0 and not requests, run.stdout + run.stderr
        armed[0], response[0] = False, 4
        run = subprocess.run([sys.executable, script], capture_output=True, text=True, timeout=10)
        assert run.returncode != 0 and len(requests) == 1, run.stdout + run.stderr
        print("ROS CLI contracts passed: requests, disable encoding, armed rejection, failed ACK")
    finally:
        stop.set()
        thread.join()
        service.shutdown()
        pub.unregister()


if __name__ == "__main__":
    if "--ros" in sys.argv:
        ros_contract()
    else:
        unittest.main()
