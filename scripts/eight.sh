#!/bin/bash

WORKSPACE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source "${WORKSPACE_DIR}/devel/setup.bash" || exit 1
roslaunch poly_planner eight.launch mav_name:=flygripper use_sim:=false
