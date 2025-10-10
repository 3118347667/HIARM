#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Joy
from geometry_msgs.msg import WrenchStamped
import numpy as np
cnt = 0
def joy_to_wrench(joy_msg):
    global cnt
    # 创建WrenchStamped消息
    wrench_stamped = WrenchStamped()

    # 设置header信息
    wrench_stamped.header.stamp = rospy.Time.now()
    wrench_stamped.header.frame_id = "base_link"
    SCALE = 1.0

    # 根据Joy的四个通道设置力和扭矩
    force_x = SCALE*joy_msg.axes[3]
    force_y = SCALE*joy_msg.axes[2]
    force_z = SCALE*joy_msg.axes[1]
    torque_z = 0.0*SCALE*joy_msg.axes[0]
    NOISE_MEAN = 0.0
    NOISE_STDDEV = 0.01
    noise_x = np.random.normal(NOISE_MEAN, NOISE_STDDEV)
    noise_y = np.random.normal(NOISE_MEAN, NOISE_STDDEV)
    noise_z = np.random.normal(NOISE_MEAN, NOISE_STDDEV)
    # 设置力向量
    wrench_stamped.wrench.force.x = force_x + noise_x
    wrench_stamped.wrench.force.y = force_y + noise_y
    wrench_stamped.wrench.force.z = force_z + noise_z

    # 设置扭矩向量
    wrench_stamped.wrench.torque.x = 0.0
    wrench_stamped.wrench.torque.y = 0.0
    wrench_stamped.wrench.torque.z = torque_z

    if cnt == 0:
        print("External force: {} {} {}".format(wrench_stamped.wrench.force.x, wrench_stamped.wrench.force.y, wrench_stamped.wrench.force.z))
        print("External torque: ", wrench_stamped.wrench.torque.z)
    if cnt < 20:
        cnt += 1
    else:
        cnt = 0
    # 发布WrenchStamped消息
    wrench_pub.publish(wrench_stamped)

def joy_listener():
    # 初始化ROS节点
    rospy.init_node('joy_to_wrench', anonymous=True)

    # 创建WrenchStamped话题的发布者
    global wrench_pub
    # prefix = "/hummingbird"#"/flygripper"
    prefix = "/flygripper"#"/flygripper"
    wrench_pub = rospy.Publisher(prefix+'/applied_wrench', WrenchStamped, queue_size=10)

    # 订阅Joy话题
    rospy.Subscriber(prefix+'/joy', Joy, joy_to_wrench)

    # 循环以保持节点运行
    rospy.spin()

if __name__ == '__main__':
    try:
        joy_listener()
    except rospy.ROSInterruptException:
        pass
