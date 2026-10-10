cmake_minimum_required(VERSION 3.0.2)
project(flygripper_actuator)

add_compile_options(-std=c++17)

find_package(catkin REQUIRED COMPONENTS
        roscpp
        sensor_msgs
        dynamic_reconfigure
        urdf
        flygripper_msgs
        flygripper_common
        )
find_package(Threads REQUIRED)
find_package(Eigen3 REQUIRED)

generate_dynamic_reconfigure_options(
        cfg/BLDC.cfg
        cfg/Servo.cfg
)

catkin_package()
include_directories(
        include
        external/feetech_servo
        ${catkin_INCLUDE_DIRS}
        ${EIGEN3_INCLUDE_DIRS}
)

add_executable(bldc_node
        src/bldc_node.cpp)
target_link_libraries(bldc_node
        ${CMAKE_THREAD_LIBS_INIT}
        ${catkin_LIBRARIES})
add_dependencies(bldc_node
        ${${PROJECT_NAME}_EXPORTED_TARGETS}
        ${catkin_EXPORTED_TARGETS}
        )

# Feetech servo sdk
file(GLOB FEETECH_SRC ${CMAKE_CURRENT_SOURCE_DIR}/external/feetech_servo/[A-Za-z]*.cpp)
add_library(feetech_servo SHARED ${FEETECH_SRC})

add_executable(servo_node
        src/servo_node.cpp)
target_link_libraries(servo_node
        ${CMAKE_THREAD_LIBS_INIT}
        ${catkin_LIBRARIES}
        feetech_servo)
add_dependencies(servo_node
        ${${PROJECT_NAME}_EXPORTED_TARGETS}
        ${catkin_EXPORTED_TARGETS}
        )


#############
## Install ##
#############

# Install targets
install(TARGETS bldc_node servo_node
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



