#include "ros/ros.h"
#include <memory>
#include <Eigen/Dense>
#include "flygripper_common/type/base.hpp"

#include "flygripper_common/ros/dynamic_parameter_config.hpp"
#include "flygripper_common/ros/parameters.hpp"
#include "flygripper_common/RosTestParameterConfig.h"

namespace flygripper_common{

class TestParameter{
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  using Ptr = std::shared_ptr<TestParameter>;
  /** Dynamic parameters **/
  Scalar dynamic_scalar{INF};
  Matrix<3, 3> dynamic_matrix{Vector<3>::Constant(INF).asDiagonal()};
  bool dynamic_bool{false};

  /** Static parameters **/
  std::string static_str;
  int static_int;

  TestParameter() = default;
  ~TestParameter() = default;

  // Print
  friend std::ostream& operator<<(std::ostream& os, const TestParameter& param){
    os.precision(3);
    os << std::scientific;
    os << "dynamic_scalar:         [ " << param.dynamic_scalar << " ]" <<std::endl;
    os << "dynamic_matrix:         [ " << param.dynamic_matrix.row(0) << " ]" << std::endl;
    os << "                        [ " << param.dynamic_matrix.row(1) << " ]" << std::endl;
    os << "                        [ " << param.dynamic_matrix.row(2) << " ]" << std::endl;
    os << "dynamic_bool: [ " << (param.dynamic_bool ? "true" : "false")<< " ]" << std::endl;

    os << "static_str: [ " << param.static_str << " ]" << std::endl;
    os << "static_int: [ " << param.static_int << " ]" << std::endl;

    os.precision();
    os.unsetf(std::ios::scientific);
    return os;
  }
};

class TestParameterConfig:
    public DynamicParameterConfig<TestParameter,
                                  RosTestParameterConfig> {
 public:
  TestParameterConfig() = default;
  virtual ~TestParameterConfig() = default;

  virtual bool initParams(ros::NodeHandle& pnh,
                          TestParameter& params) override{
    // config from ROS static parameters
    getParamDefault("static_int", params.static_int, 10, pnh );
    getParamDefault("static_str", params.static_str, std::string("hello"), pnh );

    // config from ROS dynamic parameters
    getParamDefault("dynamic_scalar", params.dynamic_scalar, 1.0, pnh );
    Vector<3> dynamic_matrix_diag = Vector<3>(0.0,1.0,2.0);
    if (!getParam("dynamic_matrix", dynamic_matrix_diag, pnh)){
      ROS_WARN("Failed to get dynamic matrix, use default");
    };
    params.dynamic_matrix = dynamic_matrix_diag.asDiagonal();
    getParamDefault("dynamic_bool", params.dynamic_bool, false, pnh);

    return true;
  }

  void dynamicParams2Config(const TestParameter& params,
                            RosTestParameterConfig& config) override{
    // convert params to dynamic reconfig
    config.dynamic_scalar = params.dynamic_scalar;
    config.dynamic_matrix_x = params.dynamic_matrix(0,0);
    config.dynamic_matrix_y = params.dynamic_matrix(1,1);
    config.dynamic_matrix_z = params.dynamic_matrix(2,2);
    config.dynamic_bool = params.dynamic_bool;
  }

  void config2DynamicParams(const RosTestParameterConfig& config,
                            TestParameter& params) override{
    // convert dynamic reconfig to params
    params.dynamic_scalar = config.dynamic_scalar;
    params.dynamic_matrix(0,0) = config.dynamic_matrix_x;
    params.dynamic_matrix(1,1) = config.dynamic_matrix_y;
    params.dynamic_matrix(2,2) = config.dynamic_matrix_z;
    params.dynamic_bool = config.dynamic_bool;
  }
};
}

using namespace flygripper_common;
int main(int argc, char** argv){
  ros::init(argc, argv, "test_node");
  ros::NodeHandle cnh("~");
  TestParameterConfig parameter_cfg;
  if(!parameter_cfg.init(cnh)){
    throw ros::Exception("Failed to initialize geometric controller parameter");
  }
  std::shared_ptr<TestParameter> parameter = std::make_shared<TestParameter>();

  ros::Rate rate(1);
  while (ros::ok()){
    ros::spinOnce();
    parameter_cfg.getParams(*parameter);
    std::cout<<*parameter
              <<"\n******************************************************************"<<std::endl;
    rate.sleep();
  }
  return 0;
}