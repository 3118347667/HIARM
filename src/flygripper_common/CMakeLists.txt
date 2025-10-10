cmake_minimum_required(VERSION 3.0.2)
project(flygripper_common)

add_compile_options(-std=c++17)
set(BUILD_TEST 0)

find_package(catkin REQUIRED COMPONENTS
        roscpp
        realtime_tools
        geometry_msgs
        nav_msgs
        sensor_msgs
        mavros_msgs
        flygripper_msgs
        dynamic_reconfigure)

find_package(Eigen3 REQUIRED)

generate_dynamic_reconfigure_options(
        cfg/RosRobotParameter.cfg
        cfg/RosTestParameter.cfg
)

# Header Only Libs
catkin_package(
        CATKIN_DEPENDS roscpp realtime_tools geometry_msgs nav_msgs sensor_msgs mavros_msgs flygripper_msgs
        INCLUDE_DIRS include
)

add_library(${PROJECT_NAME} INTERFACE)
target_include_directories(${PROJECT_NAME} INTERFACE include ${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME} INTERFACE ${catkin_LIBRARIES})


# Install headers
install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        )

# Shared Libs

#catkin_package(
#        CATKIN_DEPENDS roscpp realtime_tools geometry_msgs nav_msgs sensor_msgs mavros_msgs flygripper_msgs
#        LIBRARIES ${PROJECT_NAME}
#        INCLUDE_DIRS include
#)
#
#add_library(${PROJECT_NAME} SHARED src/flygripper_common.cpp)
#target_include_directories(${PROJECT_NAME} PUBLIC include ${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS})
#target_link_libraries(${PROJECT_NAME} PUBLIC ${catkin_LIBRARIES})
#
## Install targets
#install(TARGETS ${PROJECT_NAME}
#        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
#        )
#
## Install headers
#install(DIRECTORY include/${PROJECT_NAME}/
#        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#        )

if(BUILD_TEST)
    message(STATUS "Build test")
    add_executable(parameter_test test/parameter_test.cpp)
    target_include_directories(parameter_test PUBLIC include ${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS})
    target_link_libraries(parameter_test PUBLIC ${catkin_LIBRARIES})
else ()
    message(STATUS "Ignore building test")
endif ()

