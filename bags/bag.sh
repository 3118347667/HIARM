#!/bin/bash
set -e
CURRENT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" >/dev/null 2>&1 && pwd)"
MAV_NAME="flygripper" #"robot"
OPTION=""

if [ -z "$1" ]; then
  echo "No argument supplied, defaulting to timestamp"
else
  OPTION="-o $1"
fi

source "${CURRENT_DIR}/../devel/setup.bash"
#CMD="rosbag record --tcpnodelay /vrpn_client_node/HIARM3/pose /ekf/ekf_odom /mavros/esc_telemetry /mavros/battery /mavros/imu/data /mavros/state /mavros/rc/in /mavros/actuator_control /mavros/setpoint_raw/attitude /${MAV_NAME}/geometric_controller/data /${MAV_NAME}/actuator_command /${MAV_NAME}/actuator_state /mavros/tunnel/out /mavros/debug_value/debug_vector /vins_estimator/imu_propagate /tact_sensor/tact_state /tact_sensor/reference_mag /grasp_state /${MAV_NAME}/trajectory ${OPTION}" # control
CMD="rosbag record --tcpnodelay /vrpn_client_node/HIARM3/pose /vrpn_client_node/HIARM3/twist /ekf/ekf_odom /mavros/esc_telemetry /mavros/battery /mavros/imu/data /mavros/state /mavros/rc/in /mavros/actuator_control /mavros/setpoint_raw/attitude /${MAV_NAME}/geometric_controller/data /${MAV_NAME}/actuator_command /${MAV_NAME}/actuator_state /mavros/tunnel/out /mavros/debug_value/debug_vector /vins_estimator/imu_propagate /tact_sensor/tact_state /tact_sensor/reference_mag /grasp_state /drone0/gridmap /drone0/gridmap_inflate /drone0/mapping_vis/vs_gridmap /drone0/mapping_vis/vs_gridmap_inflate /yolo/bboxes /yolo/visualization /target_ekf_node/target_odom /target_ekf_node/yolo_odom /drone0/position_cmd /${MAV_NAME}/trajectory /optical_flow/optical_flow_rad /mavros/px4flow/raw/optical_flow_rad /ekf_interface_fusion_node/fusion_odom /policy/policy_action /vrpn_client_node/bottle2/pose /policy/state_input /ekf_interface_fusion_node/filter_optical_flow_speed /policy/goal /move_base_simple/goal /camera/mid/image_raw /camera/front/image_raw ${OPTION}" # with image
echo "Start Record Bag with Command: ${CMD}"
$CMD

wait
if [ -n "$2" ]; then
  sshpass -p "$3" scp -o StrictHostKeyChecking=no $1 $2
fi
