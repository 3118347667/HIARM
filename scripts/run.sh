#!/bin/bash
set -x
WORKSPACE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source "${WORKSPACE_DIR}/devel/setup.bash"
MAV_NAME="flygripper" #"robot"
USE_SIM="false"       #"false"
GUI="false"
MOCAP_IP="10.1.1.198"
RUN_MODE="${1:-}"

echo "Start Running MAVROS！"
"${WORKSPACE_DIR}/scripts/require_serial_access.sh" /dev/ttyACM0 || exit 1
roslaunch mavros apm.launch &
sleep 10 # wait roscore
echo "Start Running Ctrl！"
roslaunch flygripper_flight flight.launch mav_name:=${MAV_NAME} use_sim:=${USE_SIM} &
sleep 4
echo "Start Running URDF and SERVO！"
roslaunch flygripper_bringup run.launch mav_name:=${MAV_NAME} use_sim:=${USE_SIM} gui:=${GUI} &

rosrun mavros mavsys message_interval --id=27 --rate=333    # common::RAW_IMU
#rosrun mavros mavsys message_interval --id=27 --rate=100    # common::RAW_IMU
#rosrun mavros mavsys message_interval --id=11030 --rate=333 # ardupilotmega::ESC_TELEMETRY_1_TO_4
rosrun mavros mavsys message_interval --id=65 --rate=100    # common::RC_CHANNELS
#rosrun mavros mavsys message_interval --id=147 --rate=100   # common::BATTERY_STATUS
#rosrun mavros mavsys message_interval --id=250 --rate=100   #DEBUG_VECTOR
#rosrun mavros mavsys message_interval --id=385 --rate=100   #TUNNEL
#rosrun mavros mavsys message_interval --id=245 --rate=5     # common::EXTENDED_SYS_STATE
#rosrun mavros mavsys message_interval --id=100 --rate=50    # optical_flow
#rosrun mavros mavsys message_interval --id=100 --rate=100    # optical_flow
rosrun mavros mavsys message_interval --id=30 --rate=333    # common::ATTITUDE
#rosrun mavros mavsys message_interval --id=30 --rate=100    # common::ATTITUDE

sleep 4 # wait imu rate to 333
echo "Start Running Mocap Localization！"
roslaunch vrpn_client_ros sample.launch server:=${MOCAP_IP} &
sleep 3
roslaunch ekf nokov.launch &
sleep 1
wait;

#rostopic pub /flygripper/actuator_command flygripper_msgs/ActuatorCommand "header:
#    seq: 0
#    stamp: {secs: 0, nsecs: 0}
#    frame_id: ''
#    mode: 0
#    reference_position: 0.0
#   reference_velocity: 0.0
#    reference_effort: 0.0" -1 &

#sleep 1
#echo "Start Recording ROSbag..."
#(cd "${WORKSPACE_DIR}/bags" && ./bag.sh)
