cmake_minimum_required(VERSION 3.0.2)
project(flygripper_flight)

add_compile_options(-std=c++17)

find_package(catkin REQUIRED COMPONENTS
        roscpp
        diagnostic_msgs
        flygripper_common
        flygripper_msgs
        flygripper_controller
        mavros_msgs
        dynamic_reconfigure
        )
find_package(Threads REQUIRED)
find_package(Eigen3 REQUIRED)

generate_dynamic_reconfigure_options(
        cfg/RosFsmParameter.cfg
)

catkin_package()
include_directories(
        include
        ${catkin_INCLUDE_DIRS}
        ${EIGEN3_INCLUDE_DIRS}
)

if(CATKIN_ENABLE_TESTING)
  add_executable(test_runtime_io test/test_runtime_io.cpp)
  target_include_directories(test_runtime_io PRIVATE include)
  target_link_libraries(test_runtime_io ${CMAKE_THREAD_LIBS_INIT})
  add_test(NAME runtime_io_contract COMMAND test_runtime_io)
  add_executable(test_takeoff test/test_takeoff.cpp)
  target_include_directories(test_takeoff PRIVATE include)
  add_test(NAME takeoff_profile COMMAND test_takeoff)
  find_package(rostest REQUIRED)
  add_rostest_gtest(test_input_buffer test/input_buffer.test test/test_input_buffer.cpp)
  target_include_directories(test_input_buffer PRIVATE include ${catkin_INCLUDE_DIRS})
  target_link_libraries(test_input_buffer ${CMAKE_THREAD_LIBS_INIT} ${catkin_LIBRARIES})
  add_dependencies(test_input_buffer ${catkin_EXPORTED_TARGETS})
  add_rostest(test/auto_takeoff.test)
  add_rostest(test/perching.test)
  add_rostest(test/perching_ch6_exit.test)
endif()

add_executable(${PROJECT_NAME}_node
        src/flygripper_flight_node.cpp
        src/perching.cpp
        src/fsm.cpp)
target_link_libraries(${PROJECT_NAME}_node
        ${CMAKE_THREAD_LIBS_INIT}
        ${catkin_LIBRARIES})
add_dependencies(${PROJECT_NAME}_node
        ${${PROJECT_NAME}_EXPORTED_TARGETS}
        ${catkin_EXPORTED_TARGETS}
        )

#add_executable(${PROJECT_NAME}_realtime_node
#        src/flygripper_flight_realtime_node.cpp)
#target_link_libraries(${PROJECT_NAME}_realtime_node
#        ${CMAKE_THREAD_LIBS_INIT}
#        ${catkin_LIBRARIES})
#add_dependencies(${PROJECT_NAME}_realtime_node
#        ${${PROJECT_NAME}_EXPORTED_TARGETS}
#        ${catkin_EXPORTED_TARGETS}
#        )

add_executable(${PROJECT_NAME}_calibr
        src/calibr.cpp)
target_link_libraries(${PROJECT_NAME}_calibr
        ${CMAKE_THREAD_LIBS_INIT}
        ${catkin_LIBRARIES})
add_dependencies(${PROJECT_NAME}_calibr
        ${${PROJECT_NAME}_EXPORTED_TARGETS}
        ${catkin_EXPORTED_TARGETS}
        )

add_executable(${PROJECT_NAME}_calibr2
        src/calibr2.cpp)
target_link_libraries(${PROJECT_NAME}_calibr2
        ${CMAKE_THREAD_LIBS_INIT}
        ${catkin_LIBRARIES})
add_dependencies(${PROJECT_NAME}_calibr2
        ${${PROJECT_NAME}_EXPORTED_TARGETS}
        ${catkin_EXPORTED_TARGETS}
        )

add_executable(${PROJECT_NAME}_test
        src/test.cpp)
target_link_libraries(${PROJECT_NAME}_test
        ${CMAKE_THREAD_LIBS_INIT}
        ${catkin_LIBRARIES})
add_dependencies(${PROJECT_NAME}_test
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




