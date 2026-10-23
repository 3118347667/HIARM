#!/usr/bin/env bash
set -euo pipefail

if [[ $# -gt 1 ]]; then
  echo "Usage: $0 [USER@BOARD_IP]" >&2
  exit 2
fi
if [[ "${PUSH_TRANSPORT:-ssh}" != ssh ]]; then
  echo "This script uploads over SSH. Use push_ros_one_arm64_adb.sh for ADB." >&2
  exit 2
fi

target="${1:-${BOARD_SSH_TARGET:-rock@192.168.69.117}}"
if [[ "$target" == -* || "$target" != *@* ]]; then
  echo "Invalid SSH target: $target" >&2
  echo "Usage: $0 [USER@BOARD_IP]" >&2
  exit 2
fi
workspace="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
remote_workspace="/home/rock/dynamic_grasp"
binary="$workspace/devel/lib/ekf/ekf"
ssh_options=(
  -o BatchMode=yes
  -o ConnectTimeout=15
  -o ServerAliveInterval=15
  -o ServerAliveCountMax=4
)
rsync_ssh="ssh -o BatchMode=yes -o ConnectTimeout=15 -o ServerAliveInterval=15 -o ServerAliveCountMax=4"

retry() {
  local attempt
  for attempt in 1 2 3; do
    if "$@"; then return 0; fi
    if [[ "$attempt" == 3 ]]; then return 1; fi
    echo "Connection failed (attempt $attempt/3); retrying in 3 seconds..." >&2
    sleep 3
  done
}

command -v file >/dev/null || { echo "Missing command: file" >&2; exit 1; }
[[ -f "$binary" && -f "$workspace/devel/setup.bash" ]] || {
  echo "Missing build output. Run ./scripts/build_ros_one_arm64.sh first." >&2
  exit 1
}
file "$binary" | grep -q 'ARM aarch64' || {
  echo "EKF binary is not an AArch64 ELF file." >&2
  exit 1
}

for cmd in ssh rsync; do
  command -v "$cmd" >/dev/null || { echo "Missing command: $cmd" >&2; exit 1; }
done

retry ssh "${ssh_options[@]}" "$target" "test \"\$(uname -m)\" = aarch64 && test -f /opt/ros/one/setup.bash && test -d '$remote_workspace' && mkdir -p '$remote_workspace/src' '$remote_workspace/scripts' '$remote_workspace/devel' '$remote_workspace/params' '$remote_workspace/bags'"
for directory in src scripts devel params bags; do
  echo "Uploading $directory to $target..."
  retry rsync -az --timeout=90 --partial-dir=.rsync-partial --delay-updates \
    --exclude='/.wifi_credentials' --exclude='*.bag' --exclude='*.bag.active' --exclude='__pycache__/' \
    -e "$rsync_ssh" "$workspace/$directory/" "$target:$remote_workspace/$directory/"
done

board_check() {
  ssh "${ssh_options[@]}" "$target" bash -se <<'BOARD_CHECK'
set -eo pipefail
source /opt/ros/one/setup.bash
export LD_LIBRARY_PATH="/home/rock/dynamic_grasp/devel/lib:${LD_LIBRARY_PATH:-}"
for binary in \
  /home/rock/dynamic_grasp/devel/lib/ekf/ekf \
  /home/rock/dynamic_grasp/devel/lib/flygripper_flight/flygripper_flight_node \
  /home/rock/dynamic_grasp/devel/lib/libplanning_nodelet.so \
  /home/rock/dynamic_grasp/devel/lib/vrpn_client_ros/vrpn_client_node; do
  test -f "$binary"
  file "$binary" | grep -q 'ARM aarch64'
  if ldd "$binary" | grep 'not found'; then
    echo "Missing runtime library for $binary" >&2
    exit 1
  fi
done
for script in bags/bag.sh bags/publish_param_snapshot.py bags/verify_param_recording.py; do
  test -f "/home/rock/dynamic_grasp/$script"
done
bash -n /home/rock/dynamic_grasp/bags/bag.sh
BOARD_CHECK
}
retry board_check
echo "Uploaded to $target:$remote_workspace. The active workspace files were updated."
