#!/bin/bash
set -e
# SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# cd ${SCRIPT_PATH}/../bags && ./bag.sh

source "devel/setup.bash"
MAV_NAME="flygripper" #"robot"
USE_SIM="false"       #"false"
GUI="false"

echo "Start Running MAVROS！"
sudo chmod +777 /dev/ttyACM0
roslaunch mavros apm.launch fcu_url:=/dev/ttyACM0:57600 &
sleep 15 # wait roscore

rosrun mavros mavsys message_interval --id=27 --rate=333    # common::RAW_IMU
#rosrun mavros mavsys message_interval --id=27 --rate=100    # common::RAW_IMU
rosrun mavros mavsys message_interval --id=11030 --rate=333 # ardupilotmega::ESC_TELEMETRY_1_TO_4
rosrun mavros mavsys message_interval --id=65 --rate=100    # common::RC_CHANNELS
rosrun mavros mavsys message_interval --id=147 --rate=100   # common::BATTERY_STATUS
rosrun mavros mavsys message_interval --id=250 --rate=100   #DEBUG_VECTOR
rosrun mavros mavsys message_interval --id=385 --rate=100   #TUNNEL
rosrun mavros mavsys message_interval --id=245 --rate=5     # common::EXTENDED_SYS_STATE
#rosrun mavros mavsys message_interval --id=100 --rate=50    # optical_flow
rosrun mavros mavsys message_interval --id=100 --rate=100    # optical_flow
rosrun mavros mavsys message_interval --id=30 --rate=333    # common::ATTITUDE
#rosrun mavros mavsys message_interval --id=30 --rate=100    # common::ATTITUDE

#roslaunch vrpn_client_ros sample.launch server:=10.1.1.198 &
sleep 16 # wait imu rate to 333
#roslaunch eskf_fusion ekf_fusion_node.launch &
