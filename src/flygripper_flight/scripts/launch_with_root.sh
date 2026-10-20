#!/bin/bash
CURRENT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" >/dev/null 2>&1 && pwd)"
PACKAGE=$1
LAUNCH_FILE=$2
# arguments of given roslaunch file
MAV_NAME=$3
USE_SIM=$4

ROSLAUNCH_CMD="roslaunch ${PACKAGE} ${LAUNCH_FILE} mav_name:=${MAV_NAME} use_sim:=${USE_SIM}"
# This script lives at <workspace>/src/flygripper_flight/scripts.
WORKSPACE_DIR="$(cd "${CURRENT_DIR}/../../.." && pwd)"
if [[ ! -f "${WORKSPACE_DIR}/devel/setup.bash" ]]; then
    echo "Missing catkin setup: ${WORKSPACE_DIR}/devel/setup.bash" >&2
    exit 1
fi

echo "home directory: ${HOME}"
echo "workspace directory: ${WORKSPACE_DIR}"
echo "mav_name: ${MAV_NAME}"
echo "use_sim: ${USE_SIM}"
echo "roslaunch command: ${ROSLAUNCH_CMD}"

# fetch NODE_NAME_LIST
source "${WORKSPACE_DIR}/devel/setup.bash"
NODE_NAME_LIST=$(${ROSLAUNCH_CMD} --nodes)
if [ $? -eq 0 ]; then
  # Extract individual node names and build the rosnode kill command
  ROS_NODES=""
  while IFS= read -r node_name; do
      ROS_NODES+="$node_name "
  done <<< "$NODE_NAME_LIST"
  echo "ros nodes: ${ROS_NODES}"
else
  echo "Error executing roslaunch command. Exiting."
  exit 1
fi

function signal_handle() {
    # Kill all nodes using rosnode kill
    echo "------------------------ rosnode kill $ROS_NODES --------------------------"
    rosnode kill $ROS_NODES
}
trap signal_handle SIGINT SIGTERM

echo "Starting flight nodes as $(id -un)."
bash -ic "source '${WORKSPACE_DIR}/devel/setup.bash' && exec ${ROSLAUNCH_CMD}" &
launcher_pid=$!
wait "$launcher_pid"
