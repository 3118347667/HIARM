cmake_minimum_required(VERSION 3.0.2)
project(flygripper_gazebo_plugins)

## Add support for C++11, supported in ROS Kinetic and newer
add_definitions(-std=c++11)

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS
        roscpp
        tf
        gazebo_ros
        nav_msgs
        sensor_msgs
        flygripper_msgs
        mavros_msgs
        flygripper_common
        )

# Depend on system install of Gazebo
find_package(gazebo REQUIRED)

catkin_package(
        CATKIN_DEPENDS
        roscpp
        tf
        gazebo_ros
        nav_msgs
        sensor_msgs
        flygripper_msgs
        mavros_msgs
        flygripper_common
        INCLUDE_DIRS
        include
        LIBRARIES
        gazebo_rotor_plugin
        gazebo_template_plugin
        gazebo_odometry_plugin
        gazebo_imu_plugin
        gazebo_reconfigurable_plugin
        gazebo_grasp_plugin
        gazebo_interactive_plugin
)

###########
## Build ##
###########

link_directories(${GAZEBO_LIBRARY_DIRS})
include_directories(include ${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS})

# For gazebo_template_plugin plugin
add_library(gazebo_template_plugin src/gazebo_template_plugin.cpp)
target_link_libraries(gazebo_template_plugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

# For gazebo_rotor_plugin plugin
add_library(gazebo_rotor_plugin src/gazebo_rotor_plugin.cpp)
target_link_libraries(gazebo_rotor_plugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

# For gazebo_odometry_plugin plugin
add_library(gazebo_odometry_plugin src/gazebo_odometry_plugin.cpp)
target_link_libraries(gazebo_odometry_plugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

# For gazebo_imu_plugin plugin
add_library(gazebo_imu_plugin src/gazebo_imu_plugin.cpp)
target_link_libraries(gazebo_imu_plugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

# For gazebo_reconfigurable_plugin plugin
add_library(gazebo_reconfigurable_plugin src/gazebo_reconfigurable_plugin.cpp)
target_link_libraries(gazebo_reconfigurable_plugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

# For gazebo_grasp_plugin plugin
add_library(gazebo_grasp_plugin src/gazebo_grasp_plugin.cpp)
target_link_libraries(gazebo_grasp_plugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

# For gazebo_interactive_plugin plugin
add_library(gazebo_interactive_plugin src/gazebo_interactive_plugin.cpp)
target_link_libraries(gazebo_interactive_plugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

install(DIRECTORY include/${PROJECT_NAME}/
DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(TARGETS
        gazebo_template_plugin
        gazebo_rotor_plugin
        gazebo_odometry_plugin
        gazebo_imu_plugin
        gazebo_reconfigurable_plugin
        gazebo_grasp_plugin
        gazebo_interactive_plugin
ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

