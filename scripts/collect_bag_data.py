#!/usr/bin/python3

import datetime
import os
import sys
import time

from mavros_msgs.msg import ActuatorControl, ESCTelemetry, State
from sensor_msgs.msg import BatteryState
# from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest

import numpy as np
import rospy


class AcquireThrustMapping:
    def __init__(self, data_dir):
        rospy.init_node('acquire_thrust_map')
        self.data_dir = data_dir

        # raw rotor speed data
        self._raw_esc = []
        # raw command data
        self._raw_cmds = []
        # raw voltage data
        self._raw_voltage = []

        # ros message data
        self.esc_msg = None
        self.cmd_msg = None
        self.voltage_msg = None
        self.state_msg = None

        # msg time
        self.timeout = 1.0
        self.update_time = None

        # rate
        self._rate = rospy.Rate(50)

        # Data acquisition
        self._cmd_sub = rospy.Subscriber("/mavros/actuator_control", ActuatorControl, self.cmd_callback)
        self._rpm_sub = rospy.Subscriber("/mavros/esc_telemetry", ESCTelemetry, self.esc_callback)
        self._voltage_sub = rospy.Subscriber("/mavros/battery", BatteryState, self.voltage_callback)
        self._state_sub = rospy.Subscriber("/mavros/state", State, self.state_callback)


    def run(self):

        wait_rate = rospy.Rate(5)
        while self.state_msg is None or self.state_msg.armed is False:
            if rospy.is_shutdown():
                rospy.logerr('ros is shutdown before arming, exit')
                return
            rospy.loginfo_throttle(2.0, 'Wait after arming...')
            wait_rate.sleep()

        while self.state_msg.mode != 'GUIDED_NOGPS':
            if rospy.is_shutdown():
                rospy.logerr('ros is shutdown before switching to GUIDED_NOGPS mode, exit')
                return
            rospy.loginfo_throttle(2.0, 'Wait after switching to GUIDED_NOGPS mode...')
            wait_rate.sleep()
        rospy.logout("Wait ended...")

        rospy.logout("Start thrust mapping acquisition from ros bag...")
        # Empty raw buffers before start
        self.reset_buffer()
        wait_rate = rospy.Rate(1)
        # Do steps
        while not rospy.is_shutdown():
            if self.esc_msg is None:
                rospy.logerr("Not saving data since esc_msg has not been received")
                wait_rate.sleep()
            elif self.cmd_msg is None:
                rospy.logerr("Not saving data since cmd_msg has not been received")
                wait_rate.sleep()
            elif self.voltage_msg is None:
                rospy.logerr("Not saving data since voltage_msg has been received")
                wait_rate.sleep()
            elif self.state_msg.mode != 'GUIDED_NOGPS':
                rospy.logerr("Rosbag data is out of GUIDED mode.")
                break
            elif self.update_time is not None and \
                    (rospy.Time.now() - self.update_time).to_sec() > self.timeout:
                rospy.logerr("Rosbag data is finished or timeout for {} secs".format(self.timeout))
                break
            else:
                self._raw_esc.append(self.esc_msg)
                self._raw_cmds.append(self.cmd_msg)
                self._raw_voltage.append(self.voltage_msg)
                self._rate.sleep()

        if not len(self._raw_esc) > 0:
            rospy.logerr("No esc data captured; check connection!")
            return

        # Finished run, save data
        self.write_data_to_disk()

        rospy.logout("Finished run")

    def cmd_callback(self, msg):
        self.update_time = rospy.Time.now()
        self.cmd_msg = msg

    def esc_callback(self, msg):
        self.update_time = rospy.Time.now()
        self.esc_msg = msg

    def voltage_callback(self, msg):
        self.update_time = rospy.Time.now()
        self.voltage_msg = msg

    def state_callback(self, msg):
        self.state_msg = msg

    def reset_buffer(self):
        self._raw_esc = []
        self._raw_cmds = []
        self._raw_voltage = []

    def write_data_to_disk(self):
        length = len(self._raw_esc)
        assert len(self._raw_esc) == len(self._raw_cmds) == len(self._raw_voltage)

        cmd_data = np.zeros((length, 1))
        cmd_data[:, 0] = np.array([msg.controls[0] for msg in self._raw_cmds])

        voltage_data = np.zeros((length, 1))
        voltage_data[:, 0] = np.array([msg.voltage for msg in self._raw_voltage])

        esc_datas = [np.zeros((length, 1)) for _ in range(4)]
        for i in range(4):
            esc_datas[i][:, 0] = np.array([msg.esc_telemetry[i].rpm for msg in self._raw_esc])

        # Save to file
        current_time = datetime.datetime.now().strftime("%Y%m%d-%H%M%S")
        curr_dir = os.path.join(self.data_dir, current_time)
        os.makedirs(curr_dir)
        np.savetxt(os.path.join(curr_dir, 'cmd_data.csv'), cmd_data, delimiter=',')
        np.savetxt(os.path.join(curr_dir, 'voltage_data.csv'), voltage_data, delimiter=',')
        for i in range(4):
            np.savetxt(os.path.join(curr_dir, 'esc_data{}.csv'.format(i)), esc_datas[i], delimiter=',')

        print("Saved data to [%s]" % curr_dir)


if __name__ == "__main__":
    # data_dir = rospy.get_param('/calculate_thrust_mapping/data_dir')
    # is_single_rotor_thrust = rospy.get_param('/calculate_thrust_mapping/single_rotor_thrust')
    # cycles = int(rospy.get_param('/calculate_thrust_mapping/cycles'))
    # min_cmd = float(rospy.get_param('/calculate_thrust_mapping/min_cmd'))
    # max_cmd = float(rospy.get_param('/calculate_thrust_mapping/max_cmd'))
    # zero_cmd = float(rospy.get_param('/calculate_thrust_mapping/zero_cmd'))
    # num_cmd_steps = int(rospy.get_param('/calculate_thrust_mapping/num_cmd_steps'))
    # step_duration = int(rospy.get_param('/calculate_thrust_mapping/step_duration'))

    data_dir = '/home/charmyoung/flygripper_ws/scripts/data'

    rospy.logout("")
    app = AcquireThrustMapping(data_dir)
    app.run()
