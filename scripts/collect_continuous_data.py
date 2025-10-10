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
    def __init__(self,
                 data_dir,
                 cycles,
                 min_cmd=0.3,
                 max_cmd=1.0,
                 zero_cmd=0.0,
                 half_cycle_duration=10.0,
                 ctrl_rate=100.0, record_rate=50.0):

        rospy.init_node('acquire_thrust_map')
        # command steps to perform between min_cmd and max_cmd

        # data written directory
        self.data_dir = data_dir
        # clycle number
        self.cycles = cycles
        # min and max command value
        self.min_cmd = min_cmd
        self.max_cmd = max_cmd
        # zero command value
        self.zero_command = zero_cmd
        # expected command values
        self.cmds = None
        # duration of each half cycle
        self.half_cycle_duration = half_cycle_duration

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

        # rate
        self.ctrl_rate = rospy.Rate(ctrl_rate)
        self.ctrl_rate_val = ctrl_rate
        self.record_div = int(ctrl_rate/record_rate)

        # Command publishers
        self._cmd_pub = rospy.Publisher('/mavros/actuator_control', ActuatorControl, queue_size=1)

        # Data acquisition
        self._rpm_sub = rospy.Subscriber("/mavros/esc_telemetry", ESCTelemetry, self.esc_callback)
        self._voltage_sub = rospy.Subscriber("/mavros/battery", BatteryState, self.voltage_callback)
        self._state_sub = rospy.Subscriber("/mavros/state", State, self.state_callback)

    def run(self):
        self.create_thrust_trajectory()
        # Wait some time to get things running; otherwise FMU is not armed
        time.sleep(1)

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

        rospy.logout("Start thrust mapping acquisition...")
        # Empty raw buffers before start
        self.reset_buffer()

        # Ramp up motor to first applied cmd
        last_applied_cmd = self.cmds[0]
        rospy.logout("Warm up motor from: cmd=%f to cmd=%f", self.zero_command, last_applied_cmd)
        warm_up_duration = 1.0
        start_warming_up_time = rospy.get_time()
        while not rospy.is_shutdown():
            cmd = (last_applied_cmd - self.zero_command) / warm_up_duration * (
                    rospy.get_time() - start_warming_up_time) + self.zero_command
            if cmd >= last_applied_cmd:
                # Stop motor completely by sending off command
                self.send_cmd(last_applied_cmd)
                break
            self.send_cmd(cmd)
            self.ctrl_rate.sleep()

        # Do steps
        iter_idx = 1
        for cmd in self.cmds:
            rospy.logout("Send command: %f", cmd)
            if rospy.is_shutdown():
                break
            self.send_cmd(cmd)
            if iter_idx % self.record_div == 0:
                iter_idx = 0
                # to avoid measuring at high rate
                if not self.log_data():
                    rospy.logerr('Messages are not well prepared, check if mavros node has been already started.')
                    return
            last_applied_cmd = cmd
            self.ctrl_rate.sleep()
            iter_idx += 1

        # Ramp down motor otherwise active breaking messes up PSU
        rospy.logout("Ramping down motor from: cmd=%f to cmd=%f", last_applied_cmd, self.zero_command)
        break_duration = 1.0
        start_breaking_time = rospy.get_time()
        while not rospy.is_shutdown():
            cmd = (self.zero_command - last_applied_cmd) / break_duration * (
                    rospy.get_time() - start_breaking_time) + last_applied_cmd
            if cmd <= self.zero_command:
                # Stop motor completely by sending off command
                self.send_cmd(self.zero_command)
                break
            self.send_cmd(cmd)
            self.ctrl_rate.sleep()

        self.send_zero_cmd(1.0)  # Stop motor

        if not len(self._raw_esc) > 0:
            rospy.logerr("No esc data captured; check connection!")
            sys.exit(1)

        # Finished run, save data
        self.write_data_to_disk()

        rospy.logout("Finished run")

    def create_thrust_trajectory(self):

        num_cmd_half_cycle = int(self.half_cycle_duration * self.ctrl_rate_val)

        # 生成锯齿状命令序列
        thrust_up = np.linspace(self.min_cmd, self.max_cmd, num_cmd_half_cycle)
        thrust_down = np.flip(thrust_up)[1:-1]

        # 插入2秒钟的不变值
        cmd_constant_duration = 2.0
        num_cmd_constant = int(cmd_constant_duration * self.ctrl_rate_val)
        thrust_up_hold = np.full(num_cmd_constant, self.max_cmd)
        thrust_down_hold = np.full(num_cmd_constant, self.min_cmd)

        # 合并命令序列并重复多次以获得所需长度的命令序列
        self.cmds = np.tile(np.concatenate([thrust_down_hold, thrust_up, thrust_up_hold, thrust_down]), cycles)

        total_duration = (self.half_cycle_duration + cmd_constant_duration) * 2 * cycles  # 考虑到插入的不变值
        print("Thrust mapping will take %.1f seconds" % total_duration)
        # import matplotlib.pyplot as plt
        # times = np.linspace(0.0, total_duration, len(self.cmds))
        # fig = plt.figure(figsize=(10, 8))
        # ax = fig.add_subplot(111)
        # ax.scatter(times, self.cmds)
        # ax.set_xlabel('t [s]')
        # ax.set_ylabel('cmd [0-1]')
        # plt.show()

    def esc_callback(self, msg):
        self.esc_msg = msg

    def voltage_callback(self, msg):
        self.voltage_msg = msg

    def state_callback(self, msg):
        self.state_msg = msg

    def log_data(self):
        if self.esc_msg is None:
            rospy.logerr("Not saving data since esc_msg has not been received")
            return False
        if self.cmd_msg is None:
            rospy.logerr("Not saving data since cmd_msg has not been published")
            return False
        if self.voltage_msg is None:
            rospy.logerr("Not saving data since voltage_msg has been received")
            return False
        # print(self.esc_msg)
        # print(self.cmd_msg)
        # print(self.voltage_msg)

        self._raw_esc.append(self.esc_msg)
        self._raw_cmds.append(self.cmd_msg)
        self._raw_voltage.append(self.voltage_msg)
        return True

    def reset_buffer(self):
        self._raw_esc = []
        self._raw_cmds = []
        self._raw_voltage = []

    def send_zero_cmd(self, duration):
        init_time = rospy.Time.now()
        while (rospy.Time.now() - init_time).to_sec() < duration and not rospy.is_shutdown():
            self.send_cmd(self.zero_command)
            self.ctrl_rate.sleep()

    def send_cmd(self, cmd):
        cmd_msg = ActuatorControl()
        cmd_msg.header.stamp = rospy.Time.now()
        cmd_msg.group_mix = cmd_msg.PX4_MIX_FLIGHT_CONTROL
        cmd_msg.controls = [cmd, cmd, cmd, cmd, 0.0, 0.0, 0.0, 0.0]

        self.cmd_msg = cmd_msg
        self._cmd_pub.publish(cmd_msg)

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

    data_dir = '/home/rock/experiment_ws/data'


    cycles = int(3)
    min_cmd = float(0.2)
    max_cmd = float(0.7)
    zero_cmd = float(0.0)
    half_cycle_duration = float(10.0)
    ctrl_rate = float(333)
    record_rate = float(50)

    rospy.logout("")
    app = AcquireThrustMapping(data_dir, cycles, min_cmd, max_cmd, zero_cmd, half_cycle_duration, ctrl_rate, record_rate)
    app.run()
