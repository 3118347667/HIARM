#include <ros/ros.h>
#include "flygripper_estimator/flygripper_estimator.hpp"


int main(int argc, char** argv)
{
  ros::init(argc, argv, "flygripper_estimator_node");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  // Create a Estimator object
  flygripper_estimator::Estimator est(nh, pnh);

  // Spin
  ros::spin();


  return 0;
}