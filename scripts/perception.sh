#!/bin/bash
set -e

source devel/setup.bash
TARGET_CAMERA="${1:-mid}"


if [[ "${TARGET_CAMERA}" != "mid" && "${TARGET_CAMERA}" != "front" ]]; then
  echo "Unsupported target camera: ${TARGET_CAMERA}. Use mid or front." >&2
  exit 2
fi

roslaunch perception_bringup usb_camera.launch camera:=both &
sleep 1
roslaunch perception_bringup run.launch camera:=${TARGET_CAMERA}
