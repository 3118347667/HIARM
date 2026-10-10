#!/usr/bin/env bash
set -eo pipefail
source /ws/devel/setup.bash
export DISABLE_ROS1_EOL_WARNINGS=1
case "${1:-rviz}" in
  rviz) wmctrl -a RViz || exec rviz -d /ws/docker/hiarm.rviz ;;
  gazebo) wmctrl -a Gazebo || exec gzclient ;;
  terminal) exec xterm -bg '#202833' -fg white -e bash --rcfile /ws/devel/setup.bash ;;
esac
