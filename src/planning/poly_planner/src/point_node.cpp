#include "fsto/point.h"

int main(int argc, char **argv) {
  ros::init(argc, argv, "point_node");
  ros::NodeHandle nh_;

  Config config;
  ros::NodeHandle nh_priv("~");

  Config::loadParameters(config, nh_priv);
  MavGlobalPlanner glbPlanner(config, nh_);

  ros::spin();

  return 0;
}
