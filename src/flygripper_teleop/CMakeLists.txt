cmake_minimum_required(VERSION 3.0.2)
project(flygripper_teleop)

add_compile_options(-std=c++17)

find_package(catkin REQUIRED COMPONENTS
        roscpp
        dynamic_reconfigure
        flygripper_common
        sensor_msgs
        mavros_msgs
        flygripper_msgs
        )
find_package(Eigen3 REQUIRED)

generate_dynamic_reconfigure_options(
        cfg/Parameters.cfg
)

catkin_package()
include_directories(
        include
        ${catkin_INCLUDE_DIRS}
        ${EIGEN3_INCLUDE_DIRS}
)


add_executable(teleop_joy_test src/teleop_joy_test.cpp)
target_link_libraries(teleop_joy_test
        ${CMAKE_THREAD_LIBS_INIT}
        ${catkin_LIBRARIES})
add_dependencies(teleop_joy_test
        ${${PROJECT_NAME}_EXPORTED_TARGETS}
        ${catkin_EXPORTED_TARGETS}
        )

add_executable(teleop_joy_node src/teleop_joy_node.cpp)
target_link_libraries(teleop_joy_node
        ${CMAKE_THREAD_LIBS_INIT}
        ${catkin_LIBRARIES})
add_dependencies(teleop_joy_node
        ${${PROJECT_NAME}_EXPORTED_TARGETS}
        ${catkin_EXPORTED_TARGETS}
        )

#############
## Install ##
#############

# Install targets
install(TARGETS teleop_joy_node
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
        )

# Install headers
install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        )

# Install launch and scripts files
install(DIRECTORY scripts launch cfg
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
        )




