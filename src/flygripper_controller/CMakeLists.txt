cmake_minimum_required(VERSION 3.0.2)
project(flygripper_controller)

add_compile_options(-std=c++17)

find_package(catkin REQUIRED COMPONENTS
        roscpp
        flygripper_common
        flygripper_msgs
        dynamic_reconfigure
        )
find_package(Threads REQUIRED)
find_package(Eigen3 REQUIRED)

generate_dynamic_reconfigure_options(
        cfg/RosGeometricControllerParameter.cfg
)

catkin_package(
        CATKIN_DEPENDS roscpp flygripper_common flygripper_msgs
        INCLUDE_DIRS include
        LIBRARIES ${PROJECT_NAME}
)

add_library(${PROJECT_NAME} src/geometric_controller.cpp)
target_include_directories(${PROJECT_NAME} PUBLIC include ${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME} PUBLIC
        ${CMAKE_THREAD_LIBS_INIT}
        ${catkin_LIBRARIES})
add_dependencies(${PROJECT_NAME}
        ${${PROJECT_NAME}_EXPORTED_TARGETS}
        ${catkin_EXPORTED_TARGETS}
        )

add_executable(${PROJECT_NAME}_benchmark src/controller_benchmark.cpp)
target_include_directories(${PROJECT_NAME}_benchmark PUBLIC
        include ${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME}_benchmark
        ${PROJECT_NAME}
        ${CMAKE_THREAD_LIBS_INIT}
        ${catkin_LIBRARIES})
add_dependencies(${PROJECT_NAME}_benchmark
        ${${PROJECT_NAME}_EXPORTED_TARGETS}
        ${catkin_EXPORTED_TARGETS})

#add_executable(${PROJECT_NAME}_test src/test.cpp)
#target_include_directories(${PROJECT_NAME}_test PUBLIC
#        include ${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS})
#target_link_libraries(${PROJECT_NAME}_test
#        ${CMAKE_THREAD_LIBS_INIT}
#        ${catkin_LIBRARIES})
#add_dependencies(${PROJECT_NAME}_test
#        ${${PROJECT_NAME}_EXPORTED_TARGETS}
#        ${catkin_EXPORTED_TARGETS}
#        )


#############
## Install ##
#############

# Install targets
install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_benchmark
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
        )

# Install headers
install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        )

# Install launch and scripts files
install(DIRECTORY scripts params cfg
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
        )



