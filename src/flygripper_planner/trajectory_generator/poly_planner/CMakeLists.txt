cmake_minimum_required(VERSION 3.0.2)

project(poly_planner)

set(CMAKE_CXX_FLAGS "-std=c++11")
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -fPIC")

find_package(Eigen3 REQUIRED)

find_package(catkin REQUIRED COMPONENTS
        roscpp
        std_msgs
        geometry_msgs
        nav_msgs
        nav_msgs
        flygripper_msgs
        flygripper_common
        visualization_msgs
        decomp_ros_utils
        )

include_directories(
        ${catkin_INCLUDE_DIRS}
        ${EIGEN3_INCLUDE_DIRS}
        include
)

catkin_package()

add_executable(${PROJECT_NAME}_node
        src/fsto_node.cpp
        src/fsto.cpp)

target_link_libraries(${PROJECT_NAME}_node
        ${catkin_LIBRARIES}
        )

add_executable(${PROJECT_NAME}_test
        src/test.cpp)

target_link_libraries(${PROJECT_NAME}_test
        ${catkin_LIBRARIES}
        )
add_dependencies(${PROJECT_NAME}_node ${catkin_EXPORTED_TARGETS})
add_dependencies(${PROJECT_NAME}_test ${catkin_EXPORTED_TARGETS})
