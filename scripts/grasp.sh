#!/bin/bash

WORKSPACE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source "${WORKSPACE_DIR}/devel/setup.bash" || exit 1

roslaunch flygripper_planner real.launch use_sim:=false
