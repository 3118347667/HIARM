#include "flygripper_actuator/bldc/bldc_control.hpp"
#include "flygripper_common/utils/realtime.hpp"

using namespace flygripper_actuator::bldc;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "actuator_control_node");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  // Run the hardware interface node
  // -------------------------------

  // We run the ROS loop in a separate thread as external calls, such
  // as service callbacks loading controllers, can block the (main) control loop

  ros::AsyncSpinner spinner(2);
  spinner.start();

  // Set the scheduler to SCHED_FIFO with priority num
  if (!setSchedulePriority(95)){
    return 1;
  }

  try
  {
    ActuatorControl actuator_control(nh, pnh);
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