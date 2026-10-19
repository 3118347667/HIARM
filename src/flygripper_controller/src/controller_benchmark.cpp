#include <algorithm>
#include <cstdlib>
#include <iostream>
#include <stdexcept>
#include <string>

#include <ros/ros.h>

#include "flygripper_common/RosRobotParameterConfig.h"
#include "flygripper_common/type/reference.hpp"
#include "flygripper_common/type/robot_parameter.hpp"
#include "flygripper_common/type/robot_state.hpp"
#include "flygripper_common/utils/runtime_profiler.hpp"
#include "flygripper_controller/geometric_controller.hpp"

namespace {

struct Options {
  std::size_t iterations = 10000;
  std::size_t warmup = 1000;
  double frequency = 333.0;
};

std::size_t parseSize(const char *value, const char *option) {
  try {
    const auto parsed = std::stoull(value);
    if (parsed == 0) {
      throw std::invalid_argument("zero");
    }
    return static_cast<std::size_t>(parsed);
  } catch (const std::exception &) {
    throw std::invalid_argument(std::string("invalid value for ") + option);
  }
}

double parseFrequency(const char *value) {
  try {
    const double parsed = std::stod(value);
    if (!(parsed > 0.0)) {
      throw std::invalid_argument("non-positive");
    }
    return parsed;
  } catch (const std::exception &) {
    throw std::invalid_argument("invalid value for --frequency");
  }
}

Options parseOptions(int argc, char **argv) {
  Options options;
  for (int index = 1; index < argc; ++index) {
    const std::string argument(argv[index]);
    if (argument == "--iterations" && index + 1 < argc) {
      options.iterations = parseSize(argv[++index], "--iterations");
    } else if (argument == "--warmup" && index + 1 < argc) {
      options.warmup = parseSize(argv[++index], "--warmup");
    } else if (argument == "--frequency" && index + 1 < argc) {
      options.frequency = parseFrequency(argv[++index]);
    } else if (argument == "--help") {
      std::cout << "Usage: controller_benchmark [options]\n"
                << "  --iterations N  measured controller calls (default: 10000)\n"
                << "  --warmup N      unmeasured calls (default: 1000)\n"
                << "  --frequency Hz  controller frequency (default: 333)\n";
      std::exit(EXIT_SUCCESS);
    } else {
      throw std::invalid_argument("unknown or incomplete option: " + argument);
    }
  }
  return options;
}

flygripper_common::RobotState makeState(
    const flygripper_common::RobotParameter &robot_parameter) {
  using namespace flygripper_common;
  RobotState state(0.0);
  state.pos << 0.0, 0.0, 0.5;
  state.setQ(0.0);
  state.vel << 0.05, -0.03, 0.0;
  state.omega << 0.01, -0.02, 0.015;
  state.acc.setZero();
  state.alpha.setZero();
  state.jerk.setZero();
  state.snap.setZero();
  state.rot_pos = robot_parameter.rotor_position;
  const double hover_rotor_speed = std::sqrt(
      robot_parameter.mass * 9.794 / (4.0 * robot_parameter.thrust_coeff));
  state.rot_speed.setConstant(hover_rotor_speed);
  return state;
}

flygripper_common::ReferenceVector makeReferences(
    const flygripper_common::RobotState &state) {
  using namespace flygripper_common;
  RobotState reference_state = state;
  reference_state.stamp = 0.0;
  reference_state.pos += Vector<3>(0.1, -0.05, 0.03);
  reference_state.vel.setZero();
  reference_state.acc.setZero();
  reference_state.alpha.setZero();
  reference_state.jerk.setZero();
  reference_state.snap.setZero();
  RobotCommand reference_command(0.0, RobotCommandMode::MODE_SINGLE_ROTOR);
  return ReferenceVector{Reference(reference_state, reference_command)};
}

} // namespace

int main(int argc, char **argv) {
  ros::init(argc, argv, "controller_benchmark", ros::init_options::AnonymousName);
  ros::NodeHandle node_handle;

  Options options;
  try {
    options = parseOptions(argc, argv);
  } catch (const std::exception &error) {
    std::cerr << error.what() << "\nUse --help for usage.\n";
    return EXIT_FAILURE;
  }

  if (!flygripper_common::RuntimeProfiler("controller_total", 1).enabled()) {
    std::cerr << "Set DYNAMIC_GRASP_PROFILE=1 before running the benchmark.\n";
    return EXIT_FAILURE;
  }

  ros::NodeHandle robot_parameter_node(node_handle, "robot_parameter");
  flygripper_common::RobotParameterConfig robot_parameter_config;
  if (!robot_parameter_config.init(robot_parameter_node)) {
    ROS_ERROR("Failed to initialize robot parameters");
    return EXIT_FAILURE;
  }
  auto robot_parameter = std::make_shared<flygripper_common::RobotParameter>();
  robot_parameter_config.getParams(*robot_parameter);
  if (!robot_parameter->valid()) {
    ROS_ERROR_STREAM("Robot parameters are invalid:\n" << *robot_parameter);
    return EXIT_FAILURE;
  }

  flygripper_controller::GeometricController controller(
      robot_parameter, options.frequency);
  flygripper_common::RobotState state = makeState(*robot_parameter);
  const flygripper_common::ReferenceVector references = makeReferences(state);
  flygripper_common::RobotCommand command(
      0.0, flygripper_common::RobotCommandMode::MODE_SINGLE_ROTOR);

  for (std::size_t index = 0; index < options.warmup; ++index) {
    controller.prepare(state);
    controller.getCommand(state, references, command,
                          static_cast<double>(index) / options.frequency);
  }

  flygripper_common::RuntimeProfiler total_profiler(
      "controller_total", options.iterations);
  flygripper_common::RuntimeProfiler control_profiler(
      "trajectory_tracking_control", options.iterations);
  flygripper_common::RuntimeProfiler prepare_profiler(
      "controller_prepare", options.iterations);
  volatile double checksum = 0.0;
  const double start_time =
      static_cast<double>(options.warmup) / options.frequency;
  for (std::size_t index = 0; index < options.iterations; ++index) {
    const double now = start_time + static_cast<double>(index) / options.frequency;
    flygripper_common::ScopedRuntimeSample total_sample(total_profiler);
    {
      flygripper_common::ScopedRuntimeSample prepare_sample(prepare_profiler);
      controller.prepare(state);
    }
    {
      flygripper_common::ScopedRuntimeSample control_sample(control_profiler);
      controller.getCommand(state, references, command, now);
    }
    checksum += command.rotor_velocities.sum() + command.collective_thrust;
  }

  std::cout << "[BENCHMARK] disarmed=true publishes_commands=false iterations="
            << options.iterations << " checksum=" << checksum << std::endl;
  return EXIT_SUCCESS;
}
