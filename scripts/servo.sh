#!/bin/bash
set -e
CURRENT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" >/dev/null 2>&1 && pwd)"
MAV_NAME="flygripper" #"robot"
USE_SIM="false" #"true"
GUI="true"

if [ -z "$1" ]
then
  echo "No argument supplied, defaulting to ${MAV_NAME}"
else
  MAV_NAME=$1
fi

source "${CURRENT_DIR}/../devel/setup.bash"

echo "Start Running URDF and SERVO�~A"
roslaunch flygripper_bringup run.launch mav_name:=${MAV_NAME} use_sim:=${USE_SIM} gui:=${GUI} & sleep 1;

wait;
