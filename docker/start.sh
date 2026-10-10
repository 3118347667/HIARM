#!/usr/bin/env bash
set -eo pipefail
cd /ws
# 容器日志集中放在 docker/logs；首次启动或构建标记缺失时先编译工作区。
mkdir -p docker/logs
source /opt/ros/noetic/setup.bash
if [[ ! -f devel/.hiarm-build-ok ]]; then
  bash docker/build.sh 2>&1 | tee docker/logs/catkin-build.log
fi
source devel/setup.bash
# 创建用于 Gazebo/OpenGL 的虚拟 X 显示；异常重启后先清理可能残留的锁文件。
# A container restart can leave X11 lock/socket files after forced termination.
if ! glxinfo -B >/dev/null 2>&1; then
  python3 -c "from pathlib import Path; [Path(p).unlink(missing_ok=True) for p in ('/tmp/.X1-lock', '/tmp/.X11-unix/X1')]"
fi
Xvfb :1 -screen 0 1920x1080x24 +extension GLX +render -noreset >docker/logs/xvfb.log 2>&1 &
# 短暂轮询虚拟显示是否可用，确认后保存图形栈信息供诊断。
for i in {1..50}; do glxinfo -B >/dev/null 2>&1 && break; sleep .1; done
glxinfo -B >/dev/null 2>&1 || { echo 'Virtual display failed; see docker/logs/xvfb.log'; exit 1; }
glxinfo -B >docker/logs/graphics.log 2>&1
# 只有显式启用桌面时才启动窗口管理器、VNC 和 noVNC；headless 仿真不需要它们。
if [[ "${HIARM_DESKTOP:-false}" == true ]]; then
  mkdir -p /root/.config/openbox
  cp /ws/docker/desktop/menu.xml /root/.config/openbox/menu.xml
  openbox >docker/logs/openbox.log 2>&1 &
  xsetroot -solid '#283442'
  xset s off -dpms || true
  tint2 -c /ws/docker/desktop/tint2rc >docker/logs/panel.log 2>&1 &
  x11vnc -display :1 -forever -shared -nopw -rfbport 5900 -localhost >docker/logs/vnc.log 2>&1 &
  websockify --web=/usr/share/novnc 6080 localhost:5900 >docker/logs/novnc.log 2>&1 &
fi
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
trap cleanup TERM INT EXIT
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
# 容器主进程持续等待仿真或 Web 服务任一退出；trap 会负责回收其余子进程。
echo 'HIARM WebGL: http://localhost:8080'
echo 'HIARM noVNC: http://localhost:6080/vnc.html?autoconnect=true&resize=scale'
if [[ "${HIARM_DESKTOP:-false}" == true ]]; then
  echo 'noVNC desktop: enabled.'
else
  echo 'noVNC desktop: disabled (set HIARM_DESKTOP=true to enable).'
fi
wait -n "$sim_pid" "$web_pid"
