#include "ros/ros.h"
#include "sensor_msgs/Joy.h"
#include "mavros_msgs/RCIn.h"

ros::Publisher rc_in_pub;
enum RcChannel{
  AXIS_ROLL=0,
  AXIS_PITCH=1,
  AXIS_THROTTLE=2,
  AXIS_YAW=3,
  AXIS_MORPH=4,
  AXIS_SIZE=5,
};

void joyCallback(const sensor_msgs::Joy::ConstPtr& joy_msg)
{
  mavros_msgs::RCIn rc_in;
  rc_in.rssi = 255;
  // Optional: Set a value for RSSI

  //  -------------- 8Bitdo Axes ------------
//             1                      3
//             Λ                      Λ
//             |                      |
//             |                      |
//    0  <—— ——             2  <—— ——

  rc_in.channels.resize(8);
  // Convert joy_msg to rc_in.channels
  rc_in.channels[AXIS_ROLL]     = 1500 + 500 * joy_msg->axes[2];
  rc_in.channels[AXIS_PITCH]    = 1500 + 500 * joy_msg->axes[3];
  rc_in.channels[AXIS_THROTTLE] = 1500 + 500 * joy_msg->axes[1];
  rc_in.channels[AXIS_YAW]      = 1500 + 500 * joy_msg->axes[0];
  rc_in.channels[AXIS_MORPH]    = 1500 + 500 * joy_msg->axes[5];
  // Publish the converted RCIn message
  rc_in_pub.publish(rc_in);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "teleop_joy_node");
  ros::NodeHandle nh;

  ros::Subscriber joy_sub = nh.subscribe<sensor_msgs::Joy>("joy_topic", 10, joyCallback);
  rc_in_pub = nh.advertise<mavros_msgs::RCIn>("rc_in_topic", 10);

  ros::spin();

  return 0;
}