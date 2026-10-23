#!/bin/bash
set -ex
WORKSPACE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source "${WORKSPACE_DIR}/devel/setup.bash"
MAV_NAME="flygripper" #"robot"
USE_SIM="false"       #"false"
GUI="false"
MOCAP_IP="10.1.1.198"
QGC_IP="${QGC_IP:-10.1.1.91}"
QGC_PORT="${QGC_PORT:-14550}"
RUN_MODE="${1:-}"

echo "Start Running MAVROS！"
echo "Forward MAVLink to QGC at ${QGC_IP}:${QGC_PORT}/UDP"
"${WORKSPACE_DIR}/scripts/require_serial_access.sh" /dev/ttyACM0 || exit 1
roslaunch mavros apm.launch "gcs_url:=udp://@${QGC_IP}:${QGC_PORT}" &
sleep 10 # wait roscore
# Configure telemetry before the controller; stop startup on a failed core ACK.
python3 "${WORKSPACE_DIR}/scripts/configure_mavlink_streams.py"

sleep 4 # allow the requested telemetry rates to settle
echo "Start Running Mocap Localization！"
roslaunch vrpn_client_ros sample.launch server:=${MOCAP_IP} &
sleep 3
roslaunch ekf nokov.launch &
sleep 1
echo "Start Running Ctrl！"
roslaunch flygripper_flight flight.launch mav_name:=${MAV_NAME} use_sim:=${USE_SIM} &
sleep 4
echo "Start Running URDF and SERVO！"
roslaunch flygripper_bringup run.launch mav_name:=${MAV_NAME} use_sim:=${USE_SIM} gui:=${GUI} &
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
