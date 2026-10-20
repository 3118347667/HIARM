#!/usr/bin/env bash
set -euo pipefail

workspace="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
image="dynamic-grasp:ros-one-jammy-arm64"
board_workspace="/home/rock/dynamic_grasp"
container_args=(-e "REFRESH_DEPS=${REFRESH_DEPS:-0}")

[[ "${REFRESH_DEPS:-0}" == 0 || "${REFRESH_DEPS:-0}" == 1 ]] || {
  echo "REFRESH_DEPS must be 0 or 1." >&2
  exit 2
}
[[ "${CATKIN_JOBS:-2}" =~ ^[1-9][0-9]*$ ]] || {
  echo "CATKIN_JOBS must be a positive integer." >&2
  exit 2
}
command -v docker >/dev/null || { echo "Docker is required." >&2; exit 1; }

if [[ -n "${CONTAINER_PROXY:-}" ]]; then
  container_args+=(
    -e "http_proxy=$CONTAINER_PROXY"
    -e "https_proxy=$CONTAINER_PROXY"
    -e "HTTP_PROXY=$CONTAINER_PROXY"
    -e "HTTPS_PROXY=$CONTAINER_PROXY"
  )
  echo "Using configured container proxy."
fi

echo "[1/2] Building ARM64 Docker image..."
docker build --platform linux/arm64 -t "$image" \
  -f "$workspace/docker/ros-one-jammy/Dockerfile" "$workspace/docker/ros-one-jammy"

if [[ -f "$workspace/devel/.built_by" && "$(cat "$workspace/devel/.built_by")" == catkin_make ]]; then
  backup_devel="$workspace/devel_catkin_make_backup"
  [[ ! -e "$backup_devel" ]] || {
    echo "Cannot preserve catkin_make output: $backup_devel already exists." >&2
    exit 1
  }
  echo "Preserving catkin_make devel space at $backup_devel"
  mv "$workspace/devel" "$backup_devel"
fi

echo "[2/2] Building catkin workspace in ARM64 container..."
docker run --rm --platform linux/arm64 \
  -e CATKIN_JOBS="${CATKIN_JOBS:-2}" \
  "${container_args[@]}" \
  -v "$workspace:$board_workspace" \
  -w "$board_workspace" "$image" bash -lc '
    set -eo pipefail
    source /opt/ros/one/setup.bash
    set -u
    [[ "$(uname -m)" == aarch64 && "$ROS_DISTRO" == one ]]
    packages=(
      src/planning/DecompROS/catkin_simple
      src/planning/DecompROS/decomp_ros_msgs
      src/planning/DecompROS/decomp_ros_utils
      src/ekf_pose
      src/flygripper_actuator
      src/flygripper_bringup
      src/flygripper_common
      src/flygripper_controller
      src/flygripper_description
      src/flygripper_flight
      src/flygripper_msgs
      src/flygripper_planner
      src/planning/planning
      src/planning/poly_planner
      src/utils/quadrotor_msgs
      src/planning/traj_opt
      src/vrpn_client_ros
    )
    if [[ "$REFRESH_DEPS" == 1 ]]; then
      echo "[container] Refreshing apt and rosdep dependencies..."
      apt-get update -o APT::Update::Error-Mode=any
      rosdep update
      rosdep install --from-paths "${packages[@]}" --ignore-src --rosdistro one -y \
        -t build -t buildtool -t build_export -t buildtool_export -t exec \
        --skip-keys vrpn
    fi
    mkdir -p .catkin_tools/profiles
    catkin config --workspace /home/rock/dynamic_grasp --init \
      --extend /opt/ros/one --merge-devel \
      --build-space build_catkin --devel-space devel \
      --log-space logs_catkin --install-space install_catkin \
      --cmake-args -DCMAKE_BUILD_TYPE=Release -DCATKIN_ENABLE_TESTING=OFF
    catkin build --workspace /home/rock/dynamic_grasp --no-status --summarize \
      -j"$CATKIN_JOBS" -p1 \
      catkin_simple decomp_ros_msgs decomp_ros_utils ekf \
      flygripper_actuator flygripper_bringup flygripper_common \
      flygripper_controller flygripper_description flygripper_flight \
      flygripper_msgs flygripper_planner planning poly_planner \
      quadrotor_msgs traj_opt vrpn_client_ros
    file devel/lib/ekf/ekf
  '

echo "Build complete. Upload explicitly with push_ros_one_arm64_adb.sh or push_ros_one_arm64.sh."
