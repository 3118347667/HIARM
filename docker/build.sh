#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/noetic/setup.bash
cd /ws
python3 docker/sync-source.py
python3 -c "from pathlib import Path; Path('devel/.hiarm-build-ok').unlink(missing_ok=True)"
catkin_make -j2 -l2 -DCMAKE_BUILD_TYPE=Release
touch devel/.hiarm-build-ok
