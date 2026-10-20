#!/bin/bash
set -e
CURRENT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" >/dev/null 2>&1 && pwd)"
MAV_NAME="flygripper" #"robot"
USE_SIM="false"
GUI="false"

MOCAP_IP="10.1.1.198"

if [ -z "$1" ]
then
  echo "No argument supplied, defaulting to ${MAV_NAME}"
else
  MAV_NAME=$1
fi

source "${CURRENT_DIR}/../devel/setup.bash"

echo "Start Running Program！"
roslaunch poly_planner eight.launch mav_name:=${MAV_NAME} use_sim:=${USE_SIM} & sleep 1;
# roslaunch flygripper_planner grasp.launch mav_name:=${MAV_NAME} use_sim:=${USE_SIM} & sleep 1;
#roslaunch flygripper_planner real.launch mav_name:=${MAV_NAME} use_sim:=${USE_SIM} & sleep 1;
wait;
