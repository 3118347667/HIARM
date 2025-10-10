import struct
import rospy
import numpy as np
from mavros_msgs.msg import Tunnel
from std_msgs.msg import Float64MultiArray
# Assuming `encoded_data` is the byte array containing the encoded data
class Test:
    def __init__(self):
        rospy.init_node('test_map')
        self.encoded_data = None

        self.rpm = []
        self.virtual_torque_x = []
        self.virtual_torque_y = []
        self.virtual_torque_z = []

        self.thrust_coeff_rpm = 4.76e-9
        self.torque_coeff_rpm = 7.536e-11

        x_pos = 0.145/2
        y_pos = 0.175/2
        torque_thrust_ratio = self.torque_coeff_rpm/self.thrust_coeff_rpm
        # self.G = np.array([1.0, 1.0, 1.0, 1.0],
        # [-y_pos, y_pos, y_pos, -y_pos],
        # [-x_pos, x_pos, -x_pos, x_pos],
        # [-torque_thrust_ratio,-torque_thrust_ratio,torque_thrust_ratio,torque_thrust_ratio])


        self.G = np.array([[1.0, 1.0, 1.0, 1.0],
                           [-y_pos, y_pos, y_pos, -y_pos],
                           [x_pos, -x_pos, x_pos, -x_pos],
                           [torque_thrust_ratio,torque_thrust_ratio,-torque_thrust_ratio,-torque_thrust_ratio]])


        # Data acquisition
        self._data_sub = rospy.Subscriber("/mavros/tunnel/out", Tunnel, self.data_callback)

        self._torque_pub = rospy.Publisher("/torque", Float64MultiArray, 10)

    def decode(self, encoded_data):
        idx = 0
        decoded_data = {}

        # Decode moment
        moment_x, moment_y, moment_z = struct.unpack_from('fff', encoded_data, idx)
        idx += struct.calcsize('fff')
        decoded_data['moment'] = {'x': moment_x, 'y': moment_y, 'z': moment_z}

        # Decode angular acceleration
        ang_acc_x, ang_acc_y, ang_acc_z = struct.unpack_from('fff', encoded_data, idx)
        idx += struct.calcsize('fff')
        decoded_data['angular_acceleration'] = {'x': ang_acc_x, 'y': ang_acc_y, 'z': ang_acc_z}

        # Decode rpm
        rpm = [struct.unpack_from('f', encoded_data, idx + i * struct.calcsize('f'))[0] for i in range(4)]
        idx += struct.calcsize('f') * 4
        decoded_data['rpm'] = np.array(rpm)

        # Decode delta rpm
        delta_rpm = [struct.unpack_from('f', encoded_data, idx + i * struct.calcsize('f'))[0] for i in range(4)]
        idx += struct.calcsize('f') * 4
        decoded_data['delta_rpm'] = delta_rpm

        # Decode motor factors
        motor_factors = []
        for _ in range(4):
            roll_factor, pitch_factor, yaw_factor = struct.unpack_from('fff', encoded_data, idx)
            idx += struct.calcsize('fff')
            motor_factors.append({'roll': roll_factor, 'pitch': pitch_factor, 'yaw': yaw_factor})
        decoded_data['motor_factors'] = motor_factors

        # Decode set point
        set_point_roll, set_point_pitch, set_point_yaw = struct.unpack_from('fff', encoded_data, idx)
        # decoded_data['set_point'] = {'roll': set_point_roll, 'pitch': set_point_pitch, 'yaw': set_point_yaw}
        decoded_data['set_point'] = [set_point_roll, set_point_pitch, set_point_yaw]
        #offline
        self.rpm.append(np.array(rpm))
        self.virtual_torque_x.append(set_point_roll)
        self.virtual_torque_y.append(set_point_pitch)
        self.virtual_torque_z.append(set_point_yaw)

        # online
        real_torque = np.dot(self.G, (self.thrust_coeff_rpm * np.array(rpm) * np.array(rpm)))[1:]
        virtual_torque = np.array([set_point_roll, set_point_pitch, set_point_yaw])

        torque_msg = Float64MultiArray()

        torque_msg.data = [real_torque[0], real_torque[1], real_torque[2],
                         virtual_torque[0], virtual_torque[1], virtual_torque[2]]  # Example torque values

        self._torque_pub.publish(torque_msg)
        # print(decoded_data)

    def data_callback(self, msg):
        self.encoded_data = msg.payload
        self.decode(self.encoded_data)

    def analyze(self):
        assert len(self.rpm) == len(self.virtual_torque_x)
        self.torque_x = []
        self.torque_y = []
        self.torque_z = []

        for rpms in self.rpm:
            torque = np.dot(self.G, (self.thrust_coeff_rpm * rpms * rpms))[1:]
            self.torque_x.append(torque[0])
            self.torque_y.append(torque[1])
            self.torque_z.append(torque[2])

        import matplotlib.pyplot as plt
        fig, (ax1, ax2) = plt.subplots(2)
        # 绘制图形
        ax1.plot(self.torque_x, label='Torque x')  # 绘制第一个向量
        ax1.plot(self.virtual_torque_x, label='Virtual torque x')  # 绘制第二个向量
        ax1.set_title('Two Torque Plot')
        ax1.set_xlabel('Index')
        ax1.set_ylabel('Torque')
        ax1.legend()


        ratio =np.array(self.virtual_torque_x)/np.array(self.torque_x)
        # 计算均值和标准差
        mean = np.mean(ratio)
        std_dev = np.std(ratio)
        print(mean)
        # 定义异常值的阈值，例如设为均值加上三倍标准差
        high_threshold = mean + 3 * std_dev
        low_threshold = mean - 3 * std_dev
        # 将超出阈值的值设为均值

        ratio_filtered = np.where(np.logical_or(ratio > high_threshold,ratio < low_threshold), mean, ratio)
        print(np.mean(ratio_filtered))
        ax2.plot(ratio, label='Torque Ratio x')  # 绘制第一个向量
        ax2.plot(ratio_filtered, label='Filtered Torque Ratio x')  # 绘制第二个向量
        ax2.set_title('Two Torque Ratio Plot')
        ax2.set_xlabel('Index')
        ax2.set_ylabel('Torque Ratio')
        ax2.legend()
        # 调整子图之间的间距
        plt.tight_layout()
        # 显示图形
        plt.show()

if __name__ == "__main__":
    rospy.logout("")
    app = Test()
    rate = rospy.Rate(100)
    while not rospy.is_shutdown():
        # if app.encoded_data:
        #     app.decode(app.encoded_data)
        rate.sleep()

    app.analyze()




