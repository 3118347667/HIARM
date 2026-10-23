#!/bin/bash
set -e
CURRENT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" >/dev/null 2>&1 && pwd)"
MAV_NAME="${MAV_NAME:-flygripper}"
SNAPSHOT_TIMEOUT_SECONDS="${SNAPSHOT_TIMEOUT_SECONDS:-30}"
[[ "$SNAPSHOT_TIMEOUT_SECONDS" =~ ^[1-9][0-9]*$ ]] || {
  echo "SNAPSHOT_TIMEOUT_SECONDS must be a positive integer." >&2
  exit 2
}
if [ -z "$1" ]; then
  echo "No argument supplied, defaulting to timestamp"
fi

source "${CURRENT_DIR}/../devel/setup.bash"
TOPICS=(
  "/${MAV_NAME}/rosparam_snapshot"
  /rosout
  /rosout_agg
  /diagnostics
  "/${MAV_NAME}/runtime_diagnostics"
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
  /mavros/imu/data_raw
  /mavros/timesync_status
  /mavros/time_reference
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
  rm -f "$SNAPSHOT_DIR/ready" "$SNAPSHOT_DIR/failed"
  rmdir "$SNAPSHOT_DIR"
}
trap cleanup_snapshot EXIT

python3 "${CURRENT_DIR}/publish_param_snapshot.py" "$MAV_NAME" "$SNAPSHOT_DIR/ready" "$SNAPSHOT_DIR/failed" "$SNAPSHOT_TIMEOUT_SECONDS" &
SNAPSHOT_PID=$!
for ((i = 0; i < (SNAPSHOT_TIMEOUT_SECONDS + 5) * 10; i++)); do
  if [ -f "$SNAPSHOT_DIR/ready" ] || [ -f "$SNAPSHOT_DIR/failed" ]; then
    break
  fi
  if ! kill -0 "$SNAPSHOT_PID" 2>/dev/null; then
    break
  fi
  sleep 0.1
done
if [ ! -f "$SNAPSHOT_DIR/ready" ] || ! kill -0 "$SNAPSHOT_PID" 2>/dev/null; then
  printf '\033[1;31mParameter snapshot failed. Recording was NOT started.\033[0m\n' >&2
  if [ -f "$SNAPSHOT_DIR/failed" ]; then cat "$SNAPSHOT_DIR/failed" >&2; fi
  exit 1
fi
echo "Parameter snapshot ready; starting recording with parameters and ROS logs."

BAG_FILE="${1:+${1}_}$(date +%Y-%m-%d-%H-%M-%S).bag"
if [ -e "$BAG_FILE" ] || [ -e "$BAG_FILE.active" ]; then
  BAG_FILE="${BAG_FILE%.bag}_$$.bag"
fi
CMD=(rosbag record --tcpnodelay -O "$BAG_FILE" "${TOPICS[@]}")

printf 'Start Record Bag with Command:'
printf ' %q' "${CMD[@]}"
printf '\n'
# Allow Ctrl-C to close rosbag cleanly, then inspect the actual recorded file.
trap ':' INT
if "${CMD[@]}"; then
  RECORD_STATUS=0
else
  RECORD_STATUS=$?
fi
if [ "$RECORD_STATUS" -ne 0 ] && [ "$RECORD_STATUS" -ne 130 ]; then
  echo "rosbag failed with status $RECORD_STATUS" >&2
  exit "$RECORD_STATUS"
fi
python3 "${CURRENT_DIR}/verify_param_recording.py" "$BAG_FILE" "$MAV_NAME"

if [ -n "$2" ]; then
  if [ ! -f "$BAG_FILE" ]; then
    echo "Recorded bag not found: $BAG_FILE" >&2
    exit 1
  fi
  sshpass -p "$3" scp -o StrictHostKeyChecking=no -- "$BAG_FILE" "$2"
fi
