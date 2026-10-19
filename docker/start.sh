#!/usr/bin/env bash
set -eo pipefail
cd /ws
# 防止手动启动与自动启动同时占用 ROS/Gazebo 端口。
exec 9>/tmp/hiarm-simulation.lock
flock -n 9 || { echo 'Simulation is already running.'; exit 1; }
# 容器日志集中放在 docker/logs；首次启动或构建标记缺失时先编译工作区。
mkdir -p docker/logs
source /opt/ros/noetic/setup.bash
if [[ ! -f devel/.hiarm-build-ok ]]; then
  bash docker/build.sh 2>&1 | tee docker/logs/catkin-build.log
fi
source devel/setup.bash
# 后台启动 ROS/Gazebo 主 launch，并把它的 PID 留给清理函数。
roslaunch docker/sim.launch gui:="${HIARM_GUI:-false}" mission:="${HIARM_MISSION:-original}" >docker/logs/simulation.log 2>&1 &
sim_pid=$!
cleanup() {
  # 收到退出信号时先中断 roslaunch，再清理脚本启动的辅助后台进程。
  kill -INT "$sim_pid" 2>/dev/null || true
  wait "$sim_pid" 2>/dev/null || true
  local children
  children=$(jobs -pr)
  if [[ -n "$children" ]]; then kill $children 2>/dev/null || true; fi
  wait || true
}
trap 'exit 0' TERM INT
trap cleanup EXIT
# 等待 ROS 仿真时钟和机器人描述参数；主 launch 提前退出时立即失败。
for i in {1..120}; do
  rosparam get /use_sim_time >/dev/null 2>&1 && break
  kill -0 "$sim_pid" || exit 1
  sleep 1
done
rosparam get /use_sim_time >/dev/null 2>&1 || { echo 'ROS startup timed out'; exit 1; }
for i in {1..120}; do
  rosparam get /flygripper/robot_description >/dev/null 2>&1 && break
  kill -0 "$sim_pid" || exit 1
  sleep 1
done
# 启动仅用于 RViz 的真值 TF、虚拟遥控输入和浏览器 WebGL 服务。
python3 docker/rviz_bridge.py >docker/logs/rviz-bridge.log 2>&1 &
if [[ "${HIARM_DESKTOP:-false}" == true ]]; then
  bash docker/open-window.sh rviz >docker/logs/rviz.log 2>&1 &
fi
python3 docker/virtual_rc.py >docker/logs/virtual-rc.log 2>&1 &
python3 -u docker/web_server.py >docker/logs/web.log 2>&1 &
web_pid=$!
# 仿真脚本等待 ROS 或 Web 服务退出；只清理本次仿真的子进程。
echo 'HIARM WebGL: http://localhost:8080'
echo 'HIARM noVNC: http://localhost:6080/vnc.html?autoconnect=true&resize=scale'
if [[ "${HIARM_DESKTOP:-false}" == true ]]; then
  echo 'noVNC desktop: enabled.'
else
  echo 'noVNC desktop: disabled (set HIARM_DESKTOP=true to enable).'
fi
wait -n "$sim_pid" "$web_pid"
