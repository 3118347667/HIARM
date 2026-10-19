#!/bin/bash
CURRENT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" >/dev/null 2>&1 && pwd)"
PACKAGE=$1
LAUNCH_FILE=$2
MAV_NAME=$3
ACTUATOR_BUS=$4
ACTUATOR_INTERFACE=$5
ROSLAUNCH_CMD="roslaunch ${PACKAGE} ${LAUNCH_FILE} mav_name:=${MAV_NAME} actuator_bus:=${ACTUATOR_BUS} actuator_interface:=${ACTUATOR_INTERFACE}"
# Find catkin workspace root directory
cd ${CURRENT_DIR}
# Function to find catkin workspace root directory
find_catkin_workspace() {
  local current_dir="${CURRENT_DIR}"
  local ws_dir="$current_dir"

  while [ ! -d "$ws_dir/devel" ] && [ "$ws_dir" != "/" ]; do
    ws_dir="$(dirname "$ws_dir")"
  done

  if [ ! -d "$ws_dir/devel" ]; then
    echo "Error: Could not find ROS workspace devel directory" >&2
    return 1
  fi

  echo "$ws_dir"
}
#WORKSPACE_DIR="$(find_catkin_workspace)"
WORKSPACE_DIR="$HOME/dynamic_grasp"
#WORKSPACE_DIR="$(catkin locate)"

echo "home directory: ${HOME}"
echo "workspace directory: ${WORKSPACE_DIR}"
echo "actuator bus: ${ACTUATOR_BUS}"
echo "actuator interface: ${ACTUATOR_INTERFACE}"
echo "roslaunch command: ${ROSLAUNCH_CMD}"

# fetch NODE_NAME_LIST
source ${WORKSPACE_DIR}/devel/setup.bash
NODE_NAME_LIST=$(${ROSLAUNCH_CMD} --nodes)
if [ $? -eq 0 ]; then
  # Extract individual node names and build the rosnode kill command
  ROS_NODES=""
  while IFS= read -r node_name; do
    ROS_NODES+="$node_name "
  done <<<"$NODE_NAME_LIST"
  echo "ros nodes: ${ROS_NODES}"
else
  echo "Error executing roslaunch command. Exiting."
  exit 1
fi

# Setup the actuator interface
if [ "$ACTUATOR_BUS" == "socket_can" ]; then
  # Check if can0 interface is already up
  if ip link show ${ACTUATOR_INTERFACE} | grep -q "state UP"; then
    echo "${ACTUATOR_INTERFACE} interface is already up"
  else
    # Set up the can0 interface
    sudo -S ip link set ${ACTUATOR_INTERFACE} type can bitrate 1000000
    sudo -S ifconfig ${ACTUATOR_INTERFACE} up
    echo "can0 interface is set up and brought up"
  fi
elif [ "$ACTUATOR_BUS" == "socket_can" ]; then
  sudo chmod +777 ${ACTUATOR_INTERFACE}
fi

function signal_handle() {
  # Kill all nodes using rosnode kill
  echo "------------------------ rosnode kill $ROS_NODES --------------------------"
  rosnode kill $ROS_NODES
}
trap signal_handle SIGINT SIGTERM

sudo -S bash -ic "source ${HOME}/.bashrc && source ${WORKSPACE_DIR}/devel/setup.bash && ${ROSLAUNCH_CMD}" &
echo "--------------------- Launch with root privilege successfully -------------------"
wait
echo "--------------------- Launch with root privilege terminate ----------------------"
