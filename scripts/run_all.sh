#!/usr/bin/env bash
set -euo pipefail

WORKSPACE_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
SESSION_NAME="hiarm-run-all"

if ! command -v tmux >/dev/null 2>&1; then
  echo "tmux is required but was not found in PATH." >&2
  exit 1
fi

# Redraw the shell prompt after prefill to clear any early terminal echo.
prefill_pane() {
  tmux send-keys -t "$1" -l "$2"
  tmux send-keys -t "$1" C-l
}

if ! tmux has-session -t "=${SESSION_NAME}" 2>/dev/null; then
  tmux new-session -d -s "$SESSION_NAME" -n run-all -c "$WORKSPACE_DIR"
  run_pane="$(tmux display-message -p -t "$SESSION_NAME:run-all" '#{pane_id}')"
  bag_pane="$(tmux split-window -v -P -F '#{pane_id}' -t "$run_pane" -c "$WORKSPACE_DIR")"
  ekf_pane="$(tmux split-window -h -P -F '#{pane_id}' -t "$bag_pane" -c "$WORKSPACE_DIR")"
  spare_pane="$(tmux split-window -h -P -F '#{pane_id}' -t "$run_pane" -c "$WORKSPACE_DIR")"
  grasp_pane="$(tmux split-window -v -P -F '#{pane_id}' -t "$spare_pane" -c "$WORKSPACE_DIR")"

  tmux select-layout -t "$SESSION_NAME:run-all" tiled
  tmux set-option -w -t "$SESSION_NAME:run-all" pane-border-status top
  tmux select-pane -t "$run_pane" -T run
  tmux select-pane -t "$bag_pane" -T bag
  tmux select-pane -t "$ekf_pane" -T htop
  tmux select-pane -t "$spare_pane" -T ekf
  tmux select-pane -t "$grasp_pane" -T grasp_car

  prefill_pane "$run_pane" './scripts/run.sh'
  prefill_pane "$bag_pane" './bags/bag.sh'
  prefill_pane "$ekf_pane" 'htop'
  prefill_pane "$spare_pane" 'rostopic echo /ekf/ekf_odom'
  prefill_pane "$grasp_pane" './scripts/grasp_car.sh'
  tmux select-pane -t "$run_pane"
fi

if ! tmux list-windows -t "=$SESSION_NAME" -F '#{window_name}' | grep -Fxq rates; then
  raw_hz_pane="$(tmux new-window -d -P -F '#{pane_id}' -t "$SESSION_NAME" -n rates -c "$WORKSPACE_DIR")"
  imu_hz_pane="$(tmux split-window -h -P -F '#{pane_id}' -t "$raw_hz_pane" -c "$WORKSPACE_DIR")"
  ekf_hz_pane="$(tmux split-window -v -P -F '#{pane_id}' -t "$imu_hz_pane" -c "$WORKSPACE_DIR")"

  tmux select-layout -t "$SESSION_NAME:rates" tiled
  tmux set-option -w -t "$SESSION_NAME:rates" pane-border-status top
  tmux select-pane -t "$raw_hz_pane" -T imu_raw_hz
  tmux select-pane -t "$imu_hz_pane" -T imu_hz
  tmux select-pane -t "$ekf_hz_pane" -T ekf_hz

  # tmux panes may use zsh; load catkin's setup.bash in Bash explicitly.
  # rostopic hz reports once per second while sampling all incoming messages.
  prefill_pane "$raw_hz_pane" "bash -c 'source ./devel/setup.bash && exec rostopic hz /mavros/imu/data_raw'"
  prefill_pane "$imu_hz_pane" "bash -c 'source ./devel/setup.bash && exec rostopic hz /mavros/imu/data'"
  prefill_pane "$ekf_hz_pane" "bash -c 'source ./devel/setup.bash && exec rostopic hz /ekf/ekf_odom'"
fi

echo "tmux windows: run-all (run, bag, htop, ekf, grasp_car) and rates (raw IMU, IMU, EKF frequencies)."
echo "Press Ctrl-b then w to choose a window. Press Enter in a pane to execute its prefilled command."

if [[ -n "${TMUX:-}" ]]; then
  tmux switch-client -t "$SESSION_NAME"
else
  exec tmux attach-session -t "$SESSION_NAME"
fi
