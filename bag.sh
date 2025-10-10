#!/bin/bash
set -e
CURRENT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" >/dev/null 2>&1 && pwd)"
MAV_NAME="flygripper" #"robot"

if [ -z "$1" ]
then
  echo "No argument supplied, defaulting to ${MAV_NAME}"
else
  MAV_NAME=$1
fi

source "${CURRENT_DIR}/devel/setup.bash"
CMD="rosbag record --tcpnodelay /mavros/esc_telemetry /mavros/battery /mavros/imu/data /mavros/state /mavros/rc/in /mavros/actuator_control /mavros/setpoint_raw/attitude /${MAV_NAME}/geometric_controller/data /${MAV_NAME}/actuator_command /${MAV_NAME}/actuator_state /mavros/tunnel/out /mavros/debug_value/debug_vector"
echo "Start Record Bag with Command: ${CMD}"
$CMD

wait;
