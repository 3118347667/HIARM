cmake_minimum_required(VERSION 3.1)
project(fake_mavros)

set(CMAKE_CXX_STANDARD 17)

find_package(catkin REQUIRED
        COMPONENTS
        roscpp
        std_msgs
        geometry_msgs
        mavros_msgs
        flygripper_msgs
        tf)

catkin_package()
include_directories(${catkin_INCLUDE_DIRS})

add_executable(mavros_node src/mavros_node.cpp)
add_dependencies(mavros_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(mavros_node ${catkin_LIBRARIES})

