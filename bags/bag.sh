#!/bin/bash
set -e
CURRENT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" >/dev/null 2>&1 && pwd)"
MAV_NAME="flygripper" #"robot"
if [ -z "$1" ]; then
  echo "No argument supplied, defaulting to timestamp"
fi

source "${CURRENT_DIR}/../devel/setup.bash"
TOPICS=(
  # Motion capture and state estimation
  /vrpn_client_node/HIARM3/pose
  /vrpn_client_node/HIARM3/twist
  /ekf/ekf_odom
  /vins_estimator/imu_propagate
  /target_ekf_node/target_odom
  /target_ekf_node/yolo_odom
  /ekf_interface_fusion_node/fusion_odom
  /vrpn_client_node/bottle2/pose

  # MAVROS and flight control
  /mavros/esc_telemetry
  /mavros/battery
  /mavros/imu/data
  /mavros/state
  /mavros/extended_state
  /mavros/rc/in
  /mavros/actuator_control
  /mavros/setpoint_raw/attitude
  "/${MAV_NAME}/geometric_controller/data"
  # Initial ROS parameters and runtime changes to flight control settings
  "/${MAV_NAME}/geometric_controller/parameter_updates"
  "/${MAV_NAME}/robot_parameter/parameter_updates"
  "/${MAV_NAME}/flygripper_flight_node/parameter_updates"
  "/${MAV_NAME}/actuator_command"
  "/${MAV_NAME}/actuator_state"
  /mavros/tunnel/out
  /mavros/debug_value/debug_vector
  "/${MAV_NAME}/trajectory"
  "/${MAV_NAME}/reference"

  # Tactile and grasp state
  /tact_sensor/tact_state
  /tact_sensor/reference_mag
  /grasp_state

  # Mapping, perception, and navigation
  /drone0/gridmap
  /drone0/gridmap_inflate
  /drone0/mapping_vis/vs_gridmap
  /drone0/mapping_vis/vs_gridmap_inflate
  /yolo/bboxes
  /yolo/visualization
  /drone0/position_cmd
  /move_base_simple/goal

  # Optical flow, policy, and cameras
  /optical_flow/optical_flow_rad
  /mavros/px4flow/raw/optical_flow_rad
  /ekf_interface_fusion_node/filter_optical_flow_speed
  /policy/policy_action
  /policy/state_input
  /policy/goal
  /camera/mid/image_raw
  /camera/front/image_raw
)

SNAPSHOT_DIR="$(mktemp -d)"
SNAPSHOT_PID=""
cleanup_snapshot() {
  if [ -n "$SNAPSHOT_PID" ]; then
    kill "$SNAPSHOT_PID" 2>/dev/null || true
    wait "$SNAPSHOT_PID" 2>/dev/null || true
  fi
  rm -f "$SNAPSHOT_DIR/ready" "$SNAPSHOT_DIR/skipped"
  rmdir "$SNAPSHOT_DIR"
}
trap cleanup_snapshot EXIT

python3 "${CURRENT_DIR}/publish_param_snapshot.py" "$MAV_NAME" "$SNAPSHOT_DIR/ready" "$SNAPSHOT_DIR/skipped" &
SNAPSHOT_PID=$!
for ((i = 0; i < 100; i++)); do
  if [ -f "$SNAPSHOT_DIR/ready" ] || [ -f "$SNAPSHOT_DIR/skipped" ]; then
    break
  fi
  if ! kill -0 "$SNAPSHOT_PID" 2>/dev/null; then
    break
  fi
  sleep 0.1
done
if [ -f "$SNAPSHOT_DIR/ready" ]; then
  TOPICS+=("/${MAV_NAME}/rosparam_snapshot")
else
  if [ ! -f "$SNAPSHOT_DIR/skipped" ]; then
    echo "ROS parameter snapshot unavailable; recording other topics without it." >&2
  fi
  kill "$SNAPSHOT_PID" 2>/dev/null || true
  wait "$SNAPSHOT_PID" 2>/dev/null || true
  SNAPSHOT_PID=""
fi

CMD=(rosbag record --tcpnodelay "${TOPICS[@]}")
BAG_FILE=""
if [ -n "$2" ]; then
  BAG_FILE="${1:-rosbag}_$(date +%Y-%m-%d-%H-%M-%S)_$$.bag"
  CMD+=(-O "$BAG_FILE")
elif [ -n "$1" ]; then
  CMD+=(-o "$1")
fi

printf 'Start Record Bag with Command:'
printf ' %q' "${CMD[@]}"
printf '\n'
"${CMD[@]}"

if [ -n "$2" ]; then
  if [ ! -f "$BAG_FILE" ]; then
    echo "Recorded bag not found: $BAG_FILE" >&2
    exit 1
  fi
  sshpass -p "$3" scp -o StrictHostKeyChecking=no -- "$BAG_FILE" "$2"
fi
