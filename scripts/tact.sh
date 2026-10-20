#!/bin/bash

WORKSPACE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source "${WORKSPACE_DIR}/devel/setup.bash" || exit 1

roslaunch flygripper_tact tact_sensor.launch  &
sleep 1
roslaunch flygripper_tact grasp.launch
