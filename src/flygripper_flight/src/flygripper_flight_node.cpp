#include <ros/ros.h>

#include "flygripper_flight/fsm.hpp"
using namespace flygripper_flight;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "flygripper_flight_node");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  // Run the hardware interface node
  // -------------------------------

  // We run the ROS loop in a separate thread as external calls, such
  // as service callbacks updating parameters or service callback,
  // can block the (main) control loop

  ros::AsyncSpinner spinner(2);
  spinner.start();

  // TODO: Set the scheduler to SCHED_FIFO with priority num
//  if (!setSchedulePriority(95)){
//    return 1;
//  }

  try
  {
    Fsm fsm(nh, pnh);
    // Wait until shutdown signal received
    ros::waitForShutdown();
  }
  catch (const ros::Exception& e)
  {
    ROS_FATAL_STREAM("Error in the transformer node:\n"
                     << "\t" << e.what());
    return 1;
  }

  return 0;
}