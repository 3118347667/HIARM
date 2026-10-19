cmake_minimum_required(VERSION 3.1)
project(flygripper_msgs)

set(CMAKE_CXX_STANDARD 17)

find_package(catkin REQUIRED COMPONENTS
        std_msgs
        nav_msgs
        geometry_msgs
        message_generation
        )

# message
add_message_files(
        DIRECTORY msg
        FILES
        ActuatorState.msg
        ActuatorCommand.msg
        PidControllerState.msg
        RotorCommand.msg
        RotorVelocityFeedback.msg
        RobotState.msg
        PolynomialTrajectory.msg
        Polynomial.msg
        GeometricControllerData.msg
        TactSensor.msg
        TactState.msg
        GraspCommand.msg
        GraspState.msg
)

# dependencies
generate_messages(
        DEPENDENCIES
        std_msgs
        geometry_msgs
        nav_msgs
)

catkin_package(CATKIN_DEPENDS std_msgs geometry_msgs nav_msgs message_runtime)
