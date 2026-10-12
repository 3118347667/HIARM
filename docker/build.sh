#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/noetic/setup.bash
cd /ws
python3 -c "from pathlib import Path; Path('devel/.hiarm-build-ok').unlink(missing_ok=True)"
# 配置目录是独立卷，首次挂载为空；先创建 catkin_tools 的 profiles 目录。
mkdir -p .catkin_tools/profiles
catkin config --init --extend /opt/ros/noetic \
  --source-space /ws/src --build-space /ws/build --devel-space /ws/devel \
  --log-space /ws/build/logs --merge-devel --no-install \
  -j2 -l2 --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
catkin build -j2 -p1 -l2 --no-status --summarize
python3 docker/merge-compile-commands.py
touch devel/.hiarm-build-ok
