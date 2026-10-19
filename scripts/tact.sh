#!/bin/bash

source devel/setup.bash

roslaunch flygripper_tact tact_sensor.launch  &
sleep 1
roslaunch flygripper_tact grasp.launch
