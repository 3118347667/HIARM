# Additional clean files
cmake_minimum_required(VERSION 3.16)

if("${CONFIG}" STREQUAL "" OR "${CONFIG}" STREQUAL "Debug")
  file(REMOVE_RECURSE
  "flygripper_planner/trajectory_generator/decomp_ros/decomp_ros_utils/CMakeFiles/decomp_rviz_plugins_autogen.dir/AutogenUsed.txt"
  "flygripper_planner/trajectory_generator/decomp_ros/decomp_ros_utils/CMakeFiles/decomp_rviz_plugins_autogen.dir/ParseCache.txt"
  "flygripper_planner/trajectory_generator/decomp_ros/decomp_ros_utils/decomp_rviz_plugins_autogen"
  )
endif()
