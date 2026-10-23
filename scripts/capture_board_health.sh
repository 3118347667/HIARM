#!/bin/bash
# Run alongside a ground bag to correlate input delay with CPU/USB/network logs.
set -eu
[[ "$(uname -s)" == Linux ]] || { echo "Run this script on the Linux flight board." >&2; exit 1; }
HEALTH_DIR="${1:-board-health-$(date +%Y-%m-%d-%H-%M-%S)}"
mkdir -p "$HEALTH_DIR"
CHILD_PIDS=()
cleanup() {
  for task_pid in "${CHILD_PIDS[@]}"; do kill "$task_pid" 2>/dev/null || true; done
  for task_pid in "${CHILD_PIDS[@]}"; do wait "$task_pid" 2>/dev/null || true; done
}
trap cleanup EXIT
trap 'exit 0' INT TERM
if command -v journalctl >/dev/null; then
  journalctl -k -f --since now -o short-iso > "$HEALTH_DIR/kernel.log" 2>&1 &
  CHILD_PIDS+=("$!")
  journalctl -u NetworkManager -f --since now -o short-iso > "$HEALTH_DIR/network.log" 2>&1 &
  CHILD_PIDS+=("$!")
fi
if command -v vmstat >/dev/null; then
  vmstat -t 1 > "$HEALTH_DIR/vmstat.log" 2>&1 &
  CHILD_PIDS+=("$!")
fi
echo "Capturing board health in $HEALTH_DIR; Ctrl-C to stop."
while true; do
  {
    date -Ins
    cat /proc/loadavg
    for task_pressure in /proc/pressure/cpu /proc/pressure/io /proc/pressure/memory; do
      if [[ -r "$task_pressure" ]]; then echo "$task_pressure"; cat "$task_pressure"; fi
    done
    # Process names only: avoid capturing sensitive command arguments.
    ps -eo pid,comm,pcpu,pmem,nlwp,stat --sort=-pcpu | head -16
  } >> "$HEALTH_DIR/processes.log"
  sleep 1
done
