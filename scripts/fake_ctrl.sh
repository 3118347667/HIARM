#!/bin/bash
set -e
CURRENT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" >/dev/null 2>&1 && pwd)"
MAV_NAME="flygripper" #"robot"
USE_SIM="true"
GUI="true"

if [ -z "$1" ]
then
  echo "No argument supplied, defaulting to ${MAV_NAME}"
else
  MAV_NAME=$1
fi

source "${CURRENT_DIR}/../devel/setup.bash"

echo "Start Running Program！"
if [ "$USE_SIM" == "true" ]; then
  roslaunch fake_mavros apm.launch mav_name:=${MAV_NAME} & sleep 1;
else
  echo "Start Running MAVROS！"
  "${CURRENT_DIR}/require_serial_access.sh" /dev/ttyACM0
  roslaunch mavros apm.launch & sleep 5;

  rosrun mavros mavsys message_interval --id=245  --rate=5; # common::EXTENDED_SYS_STATE
  rosrun mavros mavsys message_interval --id=147  --rate=5; # common::BATTERY_STATUS
  rosrun mavros mavsys message_interval --id=30   --rate=333; # common::ATTITUDE
  rosrun mavros mavsys message_interval --id=27   --rate=333; # common::RAW_IMU
  rosrun mavros mavsys message_interval --id=65   --rate=100; # common::RC_CHANNELS
  rosrun mavros mavsys message_interval --id=11030 --rate=333; # ardupilotmega::ESC_TELEMETRY_1_TO_4
fi
roslaunch flygripper_estimator fake_estimator.launch mav_name:=${MAV_NAME} use_sim:=${USE_SIM} & sleep 1;
roslaunch flygripper_flight flight_root.launch mav_name:=${MAV_NAME} use_sim:=${USE_SIM} & sleep 1;
wait;
