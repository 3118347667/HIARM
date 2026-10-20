#!/bin/bash
set -e
CURRENT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" >/dev/null 2>&1 && pwd)"
MAV_NAME="flygripper" #"robot"
USE_SIM="false"       #"false"
GUI="false"

if [ -z "$1" ]; then
  echo "No argument supplied, defaulting to ${MAV_NAME}"
else
  MAV_NAME=$1
fi

source "${CURRENT_DIR}/../devel/setup.bash"

echo "Start Running URDF and SERVO！"
roslaunch flygripper_bringup run.launch mav_name:=${MAV_NAME} use_sim:=${USE_SIM} gui:=${GUI} &
sleep 4

if [ "$USE_SIM" == "false" ]; then
  echo "Start Running MAVROS！"
  sudo chmod +777 /dev/ttyS7
  #roslaunch mavros apm.launch & sleep 5;
  roslaunch mavros apm.launch fcu_url:=/dev/ttyS7:921600 &
  sleep 5
  #roslaunch mavros apm.launch gcs_url:=udp://@10.1.1.108:14550 & sleep 5;

  rosrun mavros mavsys message_interval --id=250 --rate=100   #DEBUG_VECTOR
  rosrun mavros mavsys message_interval --id=385 --rate=100   #TUNNEL
  rosrun mavros mavsys message_interval --id=245 --rate=5     # common::EXTENDED_SYS_STATE
  rosrun mavros mavsys message_interval --id=147 --rate=100   # common::BATTERY_STATUS
  rosrun mavros mavsys message_interval --id=30 --rate=333    # common::ATTITUDE
  rosrun mavros mavsys message_interval --id=27 --rate=333    # common::RAW_IMU
  rosrun mavros mavsys message_interval --id=65 --rate=100    # common::RC_CHANNELS
  rosrun mavros mavsys message_interval --id=11030 --rate=333 # ardupilotmega::ESC_TELEMETRY_1_TO_4
else
  echo "Start Running fake MAVROS！"
  roslaunch fake_mavros apm.launch mav_name:=${MAV_NAME} &
  sleep 1
fi
wait
