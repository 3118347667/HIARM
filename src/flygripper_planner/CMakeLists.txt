cmake_minimum_required(VERSION 3.0.2)

project(flygripper_planner)

set(CMAKE_CXX_FLAGS "-std=c++17")
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -fPIC")

find_package(Eigen3 REQUIRED)

find_package(catkin REQUIRED COMPONENTS
        roscpp
        geometry_msgs
        visualization_msgs
        flygripper_common
        dynamic_reconfigure
        )

generate_dynamic_reconfigure_options(
        cfg/RosTrajOptParameter.cfg
)

include_directories(
        ${catkin_INCLUDE_DIRS}
        ${EIGEN3_INCLUDE_DIRS}
        include
)

catkin_package()

#add_executable(online_test src/online_test.cpp)
#target_link_libraries(online_test
#        ${catkin_LIBRARIES})
#
#add_executable(offline_test src/offline_test.cpp)
#target_link_libraries(offline_test
#        ${catkin_LIBRARIES})
#
#add_executable(joy_test src/joy_test.cpp)
#target_link_libraries(joy_test
#        ${catkin_LIBRARIES})
#
#add_executable(grasp_test src/grasp_test.cpp)
#target_link_libraries(grasp_test
#        ${catkin_LIBRARIES})

add_executable(real src/real.cpp)
target_link_libraries(real
        ${catkin_LIBRARIES})
add_dependencies(real
        ${${PROJECT_NAME}_EXPORTED_TARGETS}
        ${catkin_EXPORTED_TARGETS}
        )

add_executable(grasp src/grasp.cpp)
target_link_libraries(grasp
        ${catkin_LIBRARIES})
add_dependencies(grasp
        ${${PROJECT_NAME}_EXPORTED_TARGETS}
        ${catkin_EXPORTED_TARGETS}
        )

add_executable(door src/door.cpp)
target_link_libraries(door
        ${catkin_LIBRARIES})
add_dependencies(door
        ${${PROJECT_NAME}_EXPORTED_TARGETS}
        ${catkin_EXPORTED_TARGETS}
        )

add_executable(goal src/goal.cpp)
target_link_libraries(goal
        ${catkin_LIBRARIES})
add_dependencies(goal
        ${${PROJECT_NAME}_EXPORTED_TARGETS}
        ${catkin_EXPORTED_TARGETS}
        )