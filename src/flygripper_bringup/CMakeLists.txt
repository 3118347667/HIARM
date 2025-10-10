cmake_minimum_required(VERSION 3.0.2)
project(flygripper_bringup)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
        roscpp
        std_msgs
        )

catkin_package()

