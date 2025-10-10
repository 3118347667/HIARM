#include "ros/ros.h"
#include "gcopter/gcopter.hpp"

#include "flygripper_common/trajectory/polynomial_trajectory.hpp"
#include "flygripper_msgs/PolynomialTrajectory.h"

using namespace std;
using namespace ros;
using namespace Eigen;//
using namespace flygripper_common;

int main(int argc, char **argv){
  ros::init(argc, argv, "fsto_node");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  GCOPTER nonlinOpt;
  PolynomialTrajectory<3, 7> traj;
  ros::spin();

  return 0;
}