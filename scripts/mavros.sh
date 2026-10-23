#!/bin/bash
set -e
WORKSPACE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source "${WORKSPACE_DIR}/devel/setup.bash"
MAV_NAME="flygripper" #"robot"
USE_SIM="false"       #"false"
GUI="false"
# QGC_IP="${QGC_IP:-10.1.1.108}"
# QGC_IP="${QGC_IP:-192.168.71.213}"
QGC_IP="${QGC_IP:-10.1.1.91}"
QGC_PORT="${QGC_PORT:-14550}"

echo "Start Running MAVROS！"
echo "Forward MAVLink to QGC at ${QGC_IP}:${QGC_PORT}/UDP"
"${WORKSPACE_DIR}/scripts/require_serial_access.sh" /dev/ttyACM0
roslaunch mavros apm.launch \
  fcu_url:=/dev/ttyACM0:57600 \
  gcs_url:=udp://@${QGC_IP}:${QGC_PORT} &
sleep 15 # wait roscore

python3 "${WORKSPACE_DIR}/scripts/configure_mavlink_streams.py"

#roslaunch vrpn_client_ros sample.launch server:=10.1.1.198 &
sleep 16 # allow the requested telemetry rates to settle
#roslaunch eskf_fusion ekf_fusion_node.launch &
