#include <ros/ros.h>
#include <cmath>
#include "flygripper_flight/admittance_control.hpp"

using namespace flygripper_flight;
using namespace flygripper_common;
void test(){
  static AdmittanceControlSystem sys;
  static AdmittanceControlSystem::State init_state;
  init_state = AdmittanceControlSystem::State(Vector<3>(0,1,2),
                                              0.1,
                                              Vector<3>::Zero(),
                                              0.0);
  AdmittanceControlSystem::State ref_state = init_state;
  AdmittanceControlSystem::Input admittance_force;
//  admittance_force.data.setZero();
  admittance_force.data << 1.0,1.0,0.0,0.0;
  ROS_ERROR_STREAM("\n  first reference acc: " << sys.getAcc(ref_state, admittance_force).transpose());

}

void test2(){
  Vector<4> tests=Vector<4>::Constant(NAN);
  if (tests.allFinite()){
    ROS_INFO("NAN all finite");
  } else{
    ROS_INFO("NAN all finite not ");
  }

  Vector<4> tests2=Vector<4>::Constant(INF);
  if (tests2.allFinite()){
    ROS_INFO("INF all finite");
  } else{
    ROS_INFO("INF all finite not ");
  }

  Vector<4> tests3=Vector<4>::Constant(0.0);
  if (tests3.allFinite()){
    ROS_INFO("0 all finite");
  } else{
    ROS_INFO("0 all finite not ");
  }


}
int main(int argc, char** argv)
{
  ros::init(argc, argv, "flygripper_flight_calibr");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  ros::NodeHandle robot_param_nh(nh, "robot_parameter");
  test();
  test2();
  return 0;
}


