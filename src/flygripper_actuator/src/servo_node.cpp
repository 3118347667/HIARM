#include "flygripper_actuator/servo/servo_control.hpp"
#include "flygripper_common/utils/realtime.hpp"
#include <csignal>

using namespace flygripper_actuator::servo;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "actuator_control_node");
  std::signal(SIGTERM, [](int) { ros::requestShutdown(); });
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
    // Finish callbacks before closing the serial port or destroying their data.
    spinner.stop();
    // Disable torque output for the servo
    actuator_control.shutdown();
  }
  catch (const ros::Exception& e)
  {
    ROS_FATAL_STREAM("Error in the transformer node:\n"
                         << "\t" << e.what());
    return 1;
  }


  return 0;
}
