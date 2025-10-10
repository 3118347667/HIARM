# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "flygripper_msgs: 9 messages, 0 services")

set(MSG_I_FLAGS "-Iflygripper_msgs:/home/nv/flygripper_ws/src/flygripper_msgs/msg;-Istd_msgs:/opt/ros/noetic/share/std_msgs/cmake/../msg;-Igeometry_msgs:/opt/ros/noetic/share/geometry_msgs/cmake/../msg;-Inav_msgs:/opt/ros/noetic/share/nav_msgs/cmake/../msg;-Iactionlib_msgs:/opt/ros/noetic/share/actionlib_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(flygripper_msgs_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/nv/flygripper_ws/src/flygripper_msgs/msg/ActuatorState.msg" NAME_WE)
add_custom_target(_flygripper_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "flygripper_msgs" "/home/nv/flygripper_ws/src/flygripper_msgs/msg/ActuatorState.msg" "std_msgs/Header"
)

get_filename_component(_filename "/home/nv/flygripper_ws/src/flygripper_msgs/msg/ActuatorCommand.msg" NAME_WE)
add_custom_target(_flygripper_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "flygripper_msgs" "/home/nv/flygripper_ws/src/flygripper_msgs/msg/ActuatorCommand.msg" "std_msgs/Header"
)

get_filename_component(_filename "/home/nv/flygripper_ws/src/flygripper_msgs/msg/PidControllerState.msg" NAME_WE)
add_custom_target(_flygripper_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "flygripper_msgs" "/home/nv/flygripper_ws/src/flygripper_msgs/msg/PidControllerState.msg" "std_msgs/Header"
)

get_filename_component(_filename "/home/nv/flygripper_ws/src/flygripper_msgs/msg/RotorCommand.msg" NAME_WE)
add_custom_target(_flygripper_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "flygripper_msgs" "/home/nv/flygripper_ws/src/flygripper_msgs/msg/RotorCommand.msg" "geometry_msgs/Quaternion:std_msgs/Header:geometry_msgs/Vector3"
)

get_filename_component(_filename "/home/nv/flygripper_ws/src/flygripper_msgs/msg/RotorVelocityFeedback.msg" NAME_WE)
add_custom_target(_flygripper_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "flygripper_msgs" "/home/nv/flygripper_ws/src/flygripper_msgs/msg/RotorVelocityFeedback.msg" "std_msgs/Header"
)

get_filename_component(_filename "/home/nv/flygripper_ws/src/flygripper_msgs/msg/RobotState.msg" NAME_WE)
add_custom_target(_flygripper_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "flygripper_msgs" "/home/nv/flygripper_ws/src/flygripper_msgs/msg/RobotState.msg" "geometry_msgs/Vector3:std_msgs/Header:geometry_msgs/Quaternion"
)

get_filename_component(_filename "/home/nv/flygripper_ws/src/flygripper_msgs/msg/PolynomialTrajectory.msg" NAME_WE)
add_custom_target(_flygripper_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "flygripper_msgs" "/home/nv/flygripper_ws/src/flygripper_msgs/msg/PolynomialTrajectory.msg" "std_msgs/Header:flygripper_msgs/Polynomial"
)

get_filename_component(_filename "/home/nv/flygripper_ws/src/flygripper_msgs/msg/Polynomial.msg" NAME_WE)
add_custom_target(_flygripper_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "flygripper_msgs" "/home/nv/flygripper_ws/src/flygripper_msgs/msg/Polynomial.msg" ""
)

get_filename_component(_filename "/home/nv/flygripper_ws/src/flygripper_msgs/msg/GeometricControllerData.msg" NAME_WE)
add_custom_target(_flygripper_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "flygripper_msgs" "/home/nv/flygripper_ws/src/flygripper_msgs/msg/GeometricControllerData.msg" "geometry_msgs/Vector3:geometry_msgs/Quaternion"
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(flygripper_msgs
  "/home/nv/flygripper_ws/src/flygripper_msgs/msg/ActuatorState.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/flygripper_msgs
)
_generate_msg_cpp(flygripper_msgs
  "/home/nv/flygripper_ws/src/flygripper_msgs/msg/ActuatorCommand.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/flygripper_msgs
)
_generate_msg_cpp(flygripper_msgs
  "/home/nv/flygripper_ws/src/flygripper_msgs/msg/PidControllerState.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/flygripper_msgs
)
_generate_msg_cpp(flygripper_msgs
  "/home/nv/flygripper_ws/src/flygripper_msgs/msg/RotorCommand.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Vector3.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/flygripper_msgs
)
_generate_msg_cpp(flygripper_msgs
  "/home/nv/flygripper_ws/src/flygripper_msgs/msg/RotorVelocityFeedback.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/flygripper_msgs
)
_generate_msg_cpp(flygripper_msgs
  "/home/nv/flygripper_ws/src/flygripper_msgs/msg/RobotState.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/flygripper_msgs
)
_generate_msg_cpp(flygripper_msgs
  "/home/nv/flygripper_ws/src/flygripper_msgs/msg/PolynomialTrajectory.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/home/nv/flygripper_ws/src/flygripper_msgs/msg/Polynomial.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/flygripper_msgs
)
_generate_msg_cpp(flygripper_msgs
  "/home/nv/flygripper_ws/src/flygripper_msgs/msg/Polynomial.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/flygripper_msgs
)
_generate_msg_cpp(flygripper_msgs
  "/home/nv/flygripper_ws/src/flygripper_msgs/msg/GeometricControllerData.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/flygripper_msgs
)

### Generating Services

### Generating Module File
_generate_module_cpp(flygripper_msgs
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/flygripper_msgs
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(flygripper_msgs_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(flygripper_msgs_generate_messages flygripper_msgs_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/nv/flygripper_ws/src/flygripper_msgs/msg/ActuatorState.msg" NAME_WE)
add_dependencies(flygripper_msgs_generate_messages_cpp _flygripper_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/nv/flygripper_ws/src/flygripper_msgs/msg/ActuatorCommand.msg" NAME_WE)
add_dependencies(flygripper_msgs_generate_messages_cpp _flygripper_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/nv/flygripper_ws/src/flygripper_msgs/msg/PidControllerState.msg" NAME_WE)
add_dependencies(flygripper_msgs_generate_messages_cpp _flygripper_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/nv/flygripper_ws/src/flygripper_msgs/msg/RotorCommand.msg" NAME_WE)
add_dependencies(flygripper_msgs_generate_messages_cpp _flygripper_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/nv/flygripper_ws/src/flygripper_msgs/msg/RotorVelocityFeedback.msg" NAME_WE)
add_dependencies(flygripper_msgs_generate_messages_cpp _flygripper_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/nv/flygripper_ws/src/flygripper_msgs/msg/RobotState.msg" NAME_WE)
add_dependencies(flygripper_msgs_generate_messages_cpp _flygripper_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/nv/flygripper_ws/src/flygripper_msgs/msg/PolynomialTrajectory.msg" NAME_WE)
add_dependencies(flygripper_msgs_generate_messages_cpp _flygripper_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/nv/flygripper_ws/src/flygripper_msgs/msg/Polynomial.msg" NAME_WE)
add_dependencies(flygripper_msgs_generate_messages_cpp _flygripper_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/nv/flygripper_ws/src/flygripper_msgs/msg/GeometricControllerData.msg" NAME_WE)
add_dependencies(flygripper_msgs_generate_messages_cpp _flygripper_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(flygripper_msgs_gencpp)
add_dependencies(flygripper_msgs_gencpp flygripper_msgs_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS flygripper_msgs_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(flygripper_msgs
  "/home/nv/flygripper_ws/src/flygripper_msgs/msg/ActuatorState.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/flygripper_msgs
)
_generate_msg_eus(flygripper_msgs
  "/home/nv/flygripper_ws/src/flygripper_msgs/msg/ActuatorCommand.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/flygripper_msgs
)
_generate_msg_eus(flygripper_msgs
  "/home/nv/flygripper_ws/src/flygripper_msgs/msg/PidControllerState.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/flygripper_msgs
)
_generate_msg_eus(flygripper_msgs
  "/home/nv/flygripper_ws/src/flygripper_msgs/msg/RotorCommand.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Vector3.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/flygripper_msgs
)
_generate_msg_eus(flygripper_msgs
  "/home/nv/flygripper_ws/src/flygripper_msgs/msg/RotorVelocityFeedback.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/flygripper_msgs
)
_generate_msg_eus(flygripper_msgs
  "/home/nv/flygripper_ws/src/flygripper_msgs/msg/RobotState.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/flygripper_msgs
)
_generate_msg_eus(flygripper_msgs
  "/home/nv/flygripper_ws/src/flygripper_msgs/msg/PolynomialTrajectory.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/home/nv/flygripper_ws/src/flygripper_msgs/msg/Polynomial.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/flygripper_msgs
)
_generate_msg_eus(flygripper_msgs
  "/home/nv/flygripper_ws/src/flygripper_msgs/msg/Polynomial.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/flygripper_msgs
)
_generate_msg_eus(flygripper_msgs
  "/home/nv/flygripper_ws/src/flygripper_msgs/msg/GeometricControllerData.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/flygripper_msgs
)

### Generating Services

### Generating Module File
_generate_module_eus(flygripper_msgs
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/flygripper_msgs
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(flygripper_msgs_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(flygripper_msgs_generate_messages flygripper_msgs_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/nv/flygripper_ws/src/flygripper_msgs/msg/ActuatorState.msg" NAME_WE)
add_dependencies(flygripper_msgs_generate_messages_eus _flygripper_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/nv/flygripper_ws/src/flygripper_msgs/msg/ActuatorCommand.msg" NAME_WE)
add_dependencies(flygripper_msgs_generate_messages_eus _flygripper_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/nv/flygripper_ws/src/flygripper_msgs/msg/PidControllerState.msg" NAME_WE)
add_dependencies(flygripper_msgs_generate_messages_eus _flygripper_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/nv/flygripper_ws/src/flygripper_msgs/msg/RotorCommand.msg" NAME_WE)
add_dependencies(flygripper_msgs_generate_messages_eus _flygripper_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/nv/flygripper_ws/src/flygripper_msgs/msg/RotorVelocityFeedback.msg" NAME_WE)
add_dependencies(flygripper_msgs_generate_messages_eus _flygripper_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/nv/flygripper_ws/src/flygripper_msgs/msg/RobotState.msg" NAME_WE)
add_dependencies(flygripper_msgs_generate_messages_eus _flygripper_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/nv/flygripper_ws/src/flygripper_msgs/msg/PolynomialTrajectory.msg" NAME_WE)
add_dependencies(flygripper_msgs_generate_messages_eus _flygripper_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/nv/flygripper_ws/src/flygripper_msgs/msg/Polynomial.msg" NAME_WE)
add_dependencies(flygripper_msgs_generate_messages_eus _flygripper_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/nv/flygripper_ws/src/flygripper_msgs/msg/GeometricControllerData.msg" NAME_WE)
add_dependencies(flygripper_msgs_generate_messages_eus _flygripper_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(flygripper_msgs_geneus)
add_dependencies(flygripper_msgs_geneus flygripper_msgs_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS flygripper_msgs_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(flygripper_msgs
  "/home/nv/flygripper_ws/src/flygripper_msgs/msg/ActuatorState.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/flygripper_msgs
)
_generate_msg_lisp(flygripper_msgs
  "/home/nv/flygripper_ws/src/flygripper_msgs/msg/ActuatorCommand.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/flygripper_msgs
)
_generate_msg_lisp(flygripper_msgs
  "/home/nv/flygripper_ws/src/flygripper_msgs/msg/PidControllerState.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/flygripper_msgs
)
_generate_msg_lisp(flygripper_msgs
  "/home/nv/flygripper_ws/src/flygripper_msgs/msg/RotorCommand.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Vector3.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/flygripper_msgs
)
_generate_msg_lisp(flygripper_msgs
  "/home/nv/flygripper_ws/src/flygripper_msgs/msg/RotorVelocityFeedback.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/flygripper_msgs
)
_generate_msg_lisp(flygripper_msgs
  "/home/nv/flygripper_ws/src/flygripper_msgs/msg/RobotState.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/flygripper_msgs
)
_generate_msg_lisp(flygripper_msgs
  "/home/nv/flygripper_ws/src/flygripper_msgs/msg/PolynomialTrajectory.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/home/nv/flygripper_ws/src/flygripper_msgs/msg/Polynomial.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/flygripper_msgs
)
_generate_msg_lisp(flygripper_msgs
  "/home/nv/flygripper_ws/src/flygripper_msgs/msg/Polynomial.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/flygripper_msgs
)
_generate_msg_lisp(flygripper_msgs
  "/home/nv/flygripper_ws/src/flygripper_msgs/msg/GeometricControllerData.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/flygripper_msgs
)

### Generating Services

### Generating Module File
_generate_module_lisp(flygripper_msgs
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/flygripper_msgs
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(flygripper_msgs_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(flygripper_msgs_generate_messages flygripper_msgs_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/nv/flygripper_ws/src/flygripper_msgs/msg/ActuatorState.msg" NAME_WE)
add_dependencies(flygripper_msgs_generate_messages_lisp _flygripper_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/nv/flygripper_ws/src/flygripper_msgs/msg/ActuatorCommand.msg" NAME_WE)
add_dependencies(flygripper_msgs_generate_messages_lisp _flygripper_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/nv/flygripper_ws/src/flygripper_msgs/msg/PidControllerState.msg" NAME_WE)
add_dependencies(flygripper_msgs_generate_messages_lisp _flygripper_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/nv/flygripper_ws/src/flygripper_msgs/msg/RotorCommand.msg" NAME_WE)
add_dependencies(flygripper_msgs_generate_messages_lisp _flygripper_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/nv/flygripper_ws/src/flygripper_msgs/msg/RotorVelocityFeedback.msg" NAME_WE)
add_dependencies(flygripper_msgs_generate_messages_lisp _flygripper_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/nv/flygripper_ws/src/flygripper_msgs/msg/RobotState.msg" NAME_WE)
add_dependencies(flygripper_msgs_generate_messages_lisp _flygripper_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/nv/flygripper_ws/src/flygripper_msgs/msg/PolynomialTrajectory.msg" NAME_WE)
add_dependencies(flygripper_msgs_generate_messages_lisp _flygripper_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/nv/flygripper_ws/src/flygripper_msgs/msg/Polynomial.msg" NAME_WE)
add_dependencies(flygripper_msgs_generate_messages_lisp _flygripper_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/nv/flygripper_ws/src/flygripper_msgs/msg/GeometricControllerData.msg" NAME_WE)
add_dependencies(flygripper_msgs_generate_messages_lisp _flygripper_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(flygripper_msgs_genlisp)
add_dependencies(flygripper_msgs_genlisp flygripper_msgs_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS flygripper_msgs_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(flygripper_msgs
  "/home/nv/flygripper_ws/src/flygripper_msgs/msg/ActuatorState.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/flygripper_msgs
)
_generate_msg_nodejs(flygripper_msgs
  "/home/nv/flygripper_ws/src/flygripper_msgs/msg/ActuatorCommand.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/flygripper_msgs
)
_generate_msg_nodejs(flygripper_msgs
  "/home/nv/flygripper_ws/src/flygripper_msgs/msg/PidControllerState.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/flygripper_msgs
)
_generate_msg_nodejs(flygripper_msgs
  "/home/nv/flygripper_ws/src/flygripper_msgs/msg/RotorCommand.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Vector3.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/flygripper_msgs
)
_generate_msg_nodejs(flygripper_msgs
  "/home/nv/flygripper_ws/src/flygripper_msgs/msg/RotorVelocityFeedback.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/flygripper_msgs
)
_generate_msg_nodejs(flygripper_msgs
  "/home/nv/flygripper_ws/src/flygripper_msgs/msg/RobotState.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/flygripper_msgs
)
_generate_msg_nodejs(flygripper_msgs
  "/home/nv/flygripper_ws/src/flygripper_msgs/msg/PolynomialTrajectory.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/home/nv/flygripper_ws/src/flygripper_msgs/msg/Polynomial.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/flygripper_msgs
)
_generate_msg_nodejs(flygripper_msgs
  "/home/nv/flygripper_ws/src/flygripper_msgs/msg/Polynomial.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/flygripper_msgs
)
_generate_msg_nodejs(flygripper_msgs
  "/home/nv/flygripper_ws/src/flygripper_msgs/msg/GeometricControllerData.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/flygripper_msgs
)

### Generating Services

### Generating Module File
_generate_module_nodejs(flygripper_msgs
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/flygripper_msgs
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(flygripper_msgs_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(flygripper_msgs_generate_messages flygripper_msgs_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/nv/flygripper_ws/src/flygripper_msgs/msg/ActuatorState.msg" NAME_WE)
add_dependencies(flygripper_msgs_generate_messages_nodejs _flygripper_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/nv/flygripper_ws/src/flygripper_msgs/msg/ActuatorCommand.msg" NAME_WE)
add_dependencies(flygripper_msgs_generate_messages_nodejs _flygripper_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/nv/flygripper_ws/src/flygripper_msgs/msg/PidControllerState.msg" NAME_WE)
add_dependencies(flygripper_msgs_generate_messages_nodejs _flygripper_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/nv/flygripper_ws/src/flygripper_msgs/msg/RotorCommand.msg" NAME_WE)
add_dependencies(flygripper_msgs_generate_messages_nodejs _flygripper_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/nv/flygripper_ws/src/flygripper_msgs/msg/RotorVelocityFeedback.msg" NAME_WE)
add_dependencies(flygripper_msgs_generate_messages_nodejs _flygripper_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/nv/flygripper_ws/src/flygripper_msgs/msg/RobotState.msg" NAME_WE)
add_dependencies(flygripper_msgs_generate_messages_nodejs _flygripper_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/nv/flygripper_ws/src/flygripper_msgs/msg/PolynomialTrajectory.msg" NAME_WE)
add_dependencies(flygripper_msgs_generate_messages_nodejs _flygripper_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/nv/flygripper_ws/src/flygripper_msgs/msg/Polynomial.msg" NAME_WE)
add_dependencies(flygripper_msgs_generate_messages_nodejs _flygripper_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/nv/flygripper_ws/src/flygripper_msgs/msg/GeometricControllerData.msg" NAME_WE)
add_dependencies(flygripper_msgs_generate_messages_nodejs _flygripper_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(flygripper_msgs_gennodejs)
add_dependencies(flygripper_msgs_gennodejs flygripper_msgs_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS flygripper_msgs_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(flygripper_msgs
  "/home/nv/flygripper_ws/src/flygripper_msgs/msg/ActuatorState.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/flygripper_msgs
)
_generate_msg_py(flygripper_msgs
  "/home/nv/flygripper_ws/src/flygripper_msgs/msg/ActuatorCommand.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/flygripper_msgs
)
_generate_msg_py(flygripper_msgs
  "/home/nv/flygripper_ws/src/flygripper_msgs/msg/PidControllerState.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/flygripper_msgs
)
_generate_msg_py(flygripper_msgs
  "/home/nv/flygripper_ws/src/flygripper_msgs/msg/RotorCommand.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Vector3.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/flygripper_msgs
)
_generate_msg_py(flygripper_msgs
  "/home/nv/flygripper_ws/src/flygripper_msgs/msg/RotorVelocityFeedback.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/flygripper_msgs
)
_generate_msg_py(flygripper_msgs
  "/home/nv/flygripper_ws/src/flygripper_msgs/msg/RobotState.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/flygripper_msgs
)
_generate_msg_py(flygripper_msgs
  "/home/nv/flygripper_ws/src/flygripper_msgs/msg/PolynomialTrajectory.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/home/nv/flygripper_ws/src/flygripper_msgs/msg/Polynomial.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/flygripper_msgs
)
_generate_msg_py(flygripper_msgs
  "/home/nv/flygripper_ws/src/flygripper_msgs/msg/Polynomial.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/flygripper_msgs
)
_generate_msg_py(flygripper_msgs
  "/home/nv/flygripper_ws/src/flygripper_msgs/msg/GeometricControllerData.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/flygripper_msgs
)

### Generating Services

### Generating Module File
_generate_module_py(flygripper_msgs
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/flygripper_msgs
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(flygripper_msgs_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(flygripper_msgs_generate_messages flygripper_msgs_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/nv/flygripper_ws/src/flygripper_msgs/msg/ActuatorState.msg" NAME_WE)
add_dependencies(flygripper_msgs_generate_messages_py _flygripper_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/nv/flygripper_ws/src/flygripper_msgs/msg/ActuatorCommand.msg" NAME_WE)
add_dependencies(flygripper_msgs_generate_messages_py _flygripper_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/nv/flygripper_ws/src/flygripper_msgs/msg/PidControllerState.msg" NAME_WE)
add_dependencies(flygripper_msgs_generate_messages_py _flygripper_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/nv/flygripper_ws/src/flygripper_msgs/msg/RotorCommand.msg" NAME_WE)
add_dependencies(flygripper_msgs_generate_messages_py _flygripper_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/nv/flygripper_ws/src/flygripper_msgs/msg/RotorVelocityFeedback.msg" NAME_WE)
add_dependencies(flygripper_msgs_generate_messages_py _flygripper_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/nv/flygripper_ws/src/flygripper_msgs/msg/RobotState.msg" NAME_WE)
add_dependencies(flygripper_msgs_generate_messages_py _flygripper_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/nv/flygripper_ws/src/flygripper_msgs/msg/PolynomialTrajectory.msg" NAME_WE)
add_dependencies(flygripper_msgs_generate_messages_py _flygripper_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/nv/flygripper_ws/src/flygripper_msgs/msg/Polynomial.msg" NAME_WE)
add_dependencies(flygripper_msgs_generate_messages_py _flygripper_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/nv/flygripper_ws/src/flygripper_msgs/msg/GeometricControllerData.msg" NAME_WE)
add_dependencies(flygripper_msgs_generate_messages_py _flygripper_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(flygripper_msgs_genpy)
add_dependencies(flygripper_msgs_genpy flygripper_msgs_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS flygripper_msgs_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/flygripper_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/flygripper_msgs
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(flygripper_msgs_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()
if(TARGET geometry_msgs_generate_messages_cpp)
  add_dependencies(flygripper_msgs_generate_messages_cpp geometry_msgs_generate_messages_cpp)
endif()
if(TARGET nav_msgs_generate_messages_cpp)
  add_dependencies(flygripper_msgs_generate_messages_cpp nav_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/flygripper_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/flygripper_msgs
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(flygripper_msgs_generate_messages_eus std_msgs_generate_messages_eus)
endif()
if(TARGET geometry_msgs_generate_messages_eus)
  add_dependencies(flygripper_msgs_generate_messages_eus geometry_msgs_generate_messages_eus)
endif()
if(TARGET nav_msgs_generate_messages_eus)
  add_dependencies(flygripper_msgs_generate_messages_eus nav_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/flygripper_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/flygripper_msgs
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(flygripper_msgs_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()
if(TARGET geometry_msgs_generate_messages_lisp)
  add_dependencies(flygripper_msgs_generate_messages_lisp geometry_msgs_generate_messages_lisp)
endif()
if(TARGET nav_msgs_generate_messages_lisp)
  add_dependencies(flygripper_msgs_generate_messages_lisp nav_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/flygripper_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/flygripper_msgs
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(flygripper_msgs_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()
if(TARGET geometry_msgs_generate_messages_nodejs)
  add_dependencies(flygripper_msgs_generate_messages_nodejs geometry_msgs_generate_messages_nodejs)
endif()
if(TARGET nav_msgs_generate_messages_nodejs)
  add_dependencies(flygripper_msgs_generate_messages_nodejs nav_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/flygripper_msgs)
  install(CODE "execute_process(COMMAND \"/usr/bin/python3\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/flygripper_msgs\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/flygripper_msgs
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(flygripper_msgs_generate_messages_py std_msgs_generate_messages_py)
endif()
if(TARGET geometry_msgs_generate_messages_py)
  add_dependencies(flygripper_msgs_generate_messages_py geometry_msgs_generate_messages_py)
endif()
if(TARGET nav_msgs_generate_messages_py)
  add_dependencies(flygripper_msgs_generate_messages_py nav_msgs_generate_messages_py)
endif()
