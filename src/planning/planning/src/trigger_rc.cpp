#include <cmath>
#include <ros/ros.h>
#include <flygripper_common/filter/butterworth.hpp>
#include <flygripper_common/ros/conversions.hpp>
#include <flygripper_common/type/rc.hpp>
#include <flygripper_common/utils/realtime.hpp>
#include <geometry_msgs/PoseStamped.h>
#include <realtime_tools/realtime_buffer.h>
#include <realtime_tools/realtime_publisher.h>

using namespace flygripper_common;

#define TRIG_CHANNEL 5

ros::Subscriber rc_in_sub_;
ros::Publisher traj_trigger;
realtime_tools::RealtimeBuffer<Rc<10>> rc_buffer_;

void RcInCallback(const mavros_msgs::RCInConstPtr &msg) {
    Rc<10> rc;
    fromRosMsg(msg, rc, false);
    rc_buffer_.writeFromNonRT(rc);
}

void trigger_rc(const ros::TimerEvent &event) {

    ros::Time now = event.current_real;
    static double last_channel;
    auto rc = rc_buffer_.readFromRT();
    if (!rc->isReceived(now.toSec(), 1.0) || !rc->valid()) {
        return;
    }
    bool use_traj = rc->channels[TRIG_CHANNEL] > 0 && last_channel < 0;
    if(use_traj)
    {
        geometry_msgs::PoseStamped triger_msg;
        traj_trigger.publish(triger_msg);
    }
    last_channel = rc->channels[TRIG_CHANNEL];
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "trigger_rc");
    ros::NodeHandle nh;
    
    double control_freq_hz = 333;
    rc_in_sub_ = nh.subscribe("/mavros/rc/in", 1, RcInCallback);
    traj_trigger = nh.advertise<geometry_msgs::PoseStamped>("/triger", 10, true);
    ros::Timer timer = nh.createTimer(ros::Duration(1 / control_freq_hz), trigger_rc);
    // ros::Duration(1.0).sleep();
    ROS_WARN("[Trigger RC server]: ready.");

    ros::spin();
    return 0;
}