#!/usr/bin/env bash
set -euo pipefail
workspace="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
image="hiarm:arducopter-sitl"
output=".runtime_captures/apm-sitl/$(date +%Y%m%d-%H%M%S)"
docker build --platform linux/arm64 -t "$image" \
  --build-arg "COPTER_REF=${COPTER_REF:-Copter-4.7.2}" \
  -f "$workspace/docker/arducopter-sitl/Dockerfile" "$workspace/docker/arducopter-sitl"
echo "Test artifacts: $workspace/$output"
docker run --rm --network none --platform linux/arm64 \
  -e ROS_IP=127.0.0.1 -e ROS_MASTER_URI=http://127.0.0.1:11311 \
  -v "$workspace:/home/rock/dynamic_grasp" -w /home/rock/dynamic_grasp \
  "$image" bash -lc '
    set -eo pipefail
    source /opt/ros/one/setup.bash
    catkin build --workspace /home/rock/dynamic_grasp --no-status --summarize -j4 -p1 flygripper_flight
    source devel/setup.bash
    python3 scripts/sitl/test_auto_takeoff.py --output "$1"
  ' bash "$output"
