cmake_minimum_required(VERSION 3.0.2)
project(flygripper_description)

add_compile_options(-std=c++17)

find_package(catkin REQUIRED COMPONENTS
        roscpp
        std_msgs
        )

catkin_package()

