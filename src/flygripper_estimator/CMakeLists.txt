cmake_minimum_required(VERSION 3.0.2)
project(flygripper_estimator)

add_compile_options(-std=c++17)

find_package(catkin REQUIRED COMPONENTS
        roscpp
        tf
        flygripper_common
        nav_msgs
        sensor_msgs
        mavros_msgs
        flygripper_msgs
        )
find_package(Threads REQUIRED)
find_package(Eigen3 REQUIRED)


catkin_package()
include_directories(
        include
        ${catkin_INCLUDE_DIRS}
        ${EIGEN3_INCLUDE_DIRS}
)

#add_executable(${PROJECT_NAME}_node src/${PROJECT_NAME}_node.cpp)
#target_link_libraries(${PROJECT_NAME}_node
#        ${CMAKE_THREAD_LIBS_INIT}
#        ${catkin_LIBRARIES})
#add_dependencies(${PROJECT_NAME}_node
#        ${${PROJECT_NAME}_EXPORTED_TARGETS}
#        ${catkin_EXPORTED_TARGETS}
#        )

add_executable(${PROJECT_NAME}_fake_node src/${PROJECT_NAME}_fake_node.cpp)
target_link_libraries(${PROJECT_NAME}_fake_node
        ${CMAKE_THREAD_LIBS_INIT}
        ${catkin_LIBRARIES})
add_dependencies(${PROJECT_NAME}_fake_node
        ${${PROJECT_NAME}_EXPORTED_TARGETS}
        ${catkin_EXPORTED_TARGETS}
        )

#############
## Install ##
#############

# Install targets
install(TARGETS ${PROJECT_NAME}_node
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




