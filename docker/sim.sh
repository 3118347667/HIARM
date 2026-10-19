#!/usr/bin/env bash
set -euo pipefail
# 从脚本位置回到仓库根目录，使 compose 文件路径与调用时的当前目录无关。
cd "$(dirname "$0")/.."
# 把公共的 compose 前缀存成数组；后续展开时各参数仍保持独立。
compose=(docker compose -f docker/compose.yaml)
# demo/mission 使用后台容器；前台任务中断或失败时停止仿真。
stop_failed_task() {
  local status="$1"
  trap - INT TERM
  echo "Task stopped (exit $status); stopping HIARM simulation..." >&2
  if ! "${compose[@]}" down; then
    echo 'Failed to stop HIARM simulation; run ./docker/sim.sh stop.' >&2
  fi
  exit "$status"
}
wait_for_workspace() {
  local running
  echo 'Waiting for workspace build. Progress: docker compose -f docker/compose.yaml logs -f'
  # 成功标记由 build.sh 在编译完成后生成；此时才能加载环境和导入项目消息。
  until "${compose[@]}" exec -T sim test -f /ws/devel/.hiarm-build-ok 2>/dev/null; do
    if "${compose[@]}" exec -T sim test -f /tmp/hiarm-start.exit 2>/dev/null; then
      echo 'Simulation startup ended before workspace build completed; check container logs.' >&2
      return 1
    fi
    if ! running=$("${compose[@]}" ps --status running -q sim) || [[ -z "$running" ]]; then
      echo 'Simulation container stopped before workspace build completed; check container logs.' >&2
      return 1
    fi
    sleep 2
  done
}
run_task() {
  local profile="$1" script="$2" status
  trap 'stop_failed_task 130' INT
  trap 'stop_failed_task 143' TERM
  if HIARM_MISSION="$profile" "${compose[@]}" up -d --force-recreate; then
    if wait_for_workspace && "${compose[@]}" exec -T sim bash -c "source devel/setup.bash && python3 -u docker/$script"; then
      trap - INT TERM
      return 0
    else
      status=$?
    fi
  else
    status=$?
  fi
  stop_failed_task "$status"
}
# 第一个参数选择操作；未提供时默认 start。
case "${1:-start}" in
  build)
    # 先构建镜像，再启动一次性 sim 容器编译 catkin 工作区。
    "${compose[@]}" build
    "${compose[@]}" run --rm --no-deps sim bash docker/build.sh
    ;;
  start)
    # 后台启动并构建服务，然后显示浏览器入口和桌面启用状态。
    "${compose[@]}" up -d --build
    echo 'WebGL: http://localhost:8080'
    echo 'noVNC: http://localhost:6080/vnc.html?autoconnect=true&resize=scale'
    desktop_enabled=$("${compose[@]}" exec -T sim printenv HIARM_DESKTOP 2>/dev/null || true)
    case "$desktop_enabled" in
      true) echo 'noVNC desktop: enabled (allow a few seconds for startup).' ;;
      false) echo 'noVNC desktop: disabled. Enable with: HIARM_DESKTOP=true HIARM_GUI=true ./docker/sim.sh start' ;;
      *) echo 'noVNC desktop: status unavailable; check container logs.' ;;
    esac
    echo 'First start compiles the workspace. Progress: docker compose -f docker/compose.yaml logs -f'
    ;;
  demo)
    # 以 demo 配置重建仿真，并在容器内运行短轨迹验收脚本。
    run_task demo verify.py
    ;;
  mission)
    # 以 original 配置重建仿真，再运行完整十阶段任务协调脚本。
    run_task original mission.py
    ;;
  # 以下是直接转发给 Docker Compose 的生命周期/调试命令。
  restart) "${compose[@]}" restart sim ;;
  stop) "${compose[@]}" down ;;
  logs) "${compose[@]}" logs -f ;;
  shell) "${compose[@]}" exec sim bash --rcfile /ws/devel/setup.bash ;;
  # 未知参数时打印用法到标准错误，并返回非零状态码。
  *) echo "Usage: $0 {build|start|demo|mission|restart|stop|logs|shell}" >&2; exit 2 ;;
esac
