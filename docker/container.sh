#!/usr/bin/env bash
set -eo pipefail
cd /ws
mkdir -p docker/logs
# 桌面由容器入口管理，不随 ROS/Gazebo 退出。
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

cleanup() {
  trap - TERM INT EXIT
  if [[ -n "${simulation_pid:-}" ]]; then
    kill -TERM "$simulation_pid" 2>/dev/null || true
    wait "$simulation_pid" 2>/dev/null || true
  fi
  local children
  children=$(jobs -pr)
  if [[ -n "$children" ]]; then kill $children 2>/dev/null || true; fi
  wait || true
}
trap 'exit 0' TERM INT
trap cleanup EXIT
rm -f /tmp/hiarm-start.exit
bash docker/start.sh &
simulation_pid=$!
if wait "$simulation_pid"; then status=0; else status=$?; fi
simulation_pid=
printf '%s\n' "$status" > /tmp/hiarm-start.exit
echo "Simulation stopped (exit $status). Container and desktop remain available."
echo 'Start simulation again inside the container: bash /ws/docker/start.sh'
# 仿真结束后保留开发环境，直到 Docker 停止容器。
sleep infinity &
wait "$!"
