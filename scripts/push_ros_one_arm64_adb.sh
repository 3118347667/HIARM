#!/usr/bin/env bash
set -euo pipefail

[[ $# -le 1 ]] || { echo "Usage: $0 [ADB_SERIAL]" >&2; exit 2; }
workspace="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
remote_workspace=/home/rock/dynamic_grasp

for cmd in adb tar shasum base64 file; do
  command -v "$cmd" >/dev/null || { echo "Missing command: $cmd" >&2; exit 1; }
done
serial="${1:-${BOARD_ADB_SERIAL:-}}"
if [[ -z "$serial" ]]; then
  devices=$(adb devices)
  connected_count=0
  while IFS= read -r connected_serial; do
    [[ -n "$connected_serial" ]] || continue
    serial=$connected_serial
    connected_count=$((connected_count + 1))
  done < <(printf '%s\n' "$devices" | awk '$2 == "device" {print $1}')
  if (( connected_count != 1 )); then
    echo "Expected one authorized ADB device; found $connected_count. Set BOARD_ADB_SERIAL or pass ADB_SERIAL." >&2
    exit 1
  fi
fi
adb -s "$serial" get-state | grep -qx device || {
  echo "ADB device $serial is not connected and authorized." >&2
  exit 1
}
[[ -f "$workspace/devel/setup.bash" && -f "$workspace/devel/lib/ekf/ekf" ]] || {
  echo "Missing ARM64 build output. Run ./scripts/build_ros_one_arm64.sh first." >&2
  exit 1
}
file "$workspace/devel/lib/ekf/ekf" | grep -q 'ARM aarch64' || {
  echo "EKF binary is not an AArch64 ELF file." >&2
  exit 1
}

# This board's adbd may return zero even when its remote shell command fails.
adb_shell_checked() {
  local output_file last_line
  output_file=$(mktemp "${TMPDIR:-/tmp}/hiarm-adb-output.XXXXXX") || return 1
  if ! adb -s "$serial" shell "$1; status=\$?; printf '\n__HIARM_ADB_STATUS__:%s\n' \"\$status\"" | tr -d '\r' > "$output_file"; then
    cat "$output_file" >&2
    unlink "$output_file"
    return 1
  fi
  last_line=$(tail -n 1 "$output_file")
  if [[ "$last_line" != '__HIARM_ADB_STATUS__:0' ]]; then
    printf 'ADB board command failed (%s).\n' "$last_line" >&2
    sed '$d' "$output_file" >&2
    unlink "$output_file"
    return 1
  fi
  sed '$d' "$output_file"
  unlink "$output_file"
}

echo "Using ADB device $serial."
adb_shell_checked "test \"\$(uname -m)\" = aarch64 && test -f /opt/ros/one/setup.bash && test -d '$remote_workspace' && test \"\$(id -u)\" = 0 && test \"\$(stat -c %U '$remote_workspace')\" = rock"
board_epoch=$(adb_shell_checked "date +%s")
host_epoch=$(date +%s)
if [[ "$board_epoch" =~ ^[0-9]+$ ]]; then
  clock_skew=$((host_epoch - board_epoch))
  if (( clock_skew > 300 )); then
    printf 'Board clock is about %d hours behind Mac; suppressing repeated tar timestamp warnings.\n' "$(((clock_skew + 1800) / 3600))"
  elif (( clock_skew < -300 )); then
    printf 'Board clock is about %d hours ahead of Mac; suppressing repeated tar timestamp warnings.\n' "$(((-clock_skew + 1800) / 3600))"
  fi
fi

local_archive=$(mktemp "${TMPDIR:-/tmp}/hiarm-upload.XXXXXX")
remote_archive="/tmp/${local_archive##*/}"
cleanup_archive() {
  unlink "$local_archive" 2>/dev/null || true
  adb_shell_checked "test ! -f '$remote_archive' || unlink '$remote_archive'" >/dev/null 2>&1 || true
}
trap cleanup_archive EXIT

echo "Packing src, scripts, devel, and params..."
COPYFILE_DISABLE=1 tar --no-xattrs --exclude='scripts/.wifi_credentials' \
  -C "$workspace" -czf "$local_archive" src scripts devel params
local_hash=$(shasum -a 256 "$local_archive" | awk '{print $1}')
adb -s "$serial" push "$local_archive" "$remote_archive"
remote_hash=$(adb_shell_checked "sha256sum '$remote_archive'")
[[ "${remote_hash%% *}" == "$local_hash" ]] || {
  echo "ADB archive checksum mismatch; board files were not changed." >&2
  exit 1
}

echo "Archive checksum matches. Extracting as rock..."
adb_shell_checked "su -s /bin/sh rock -c 'tar --warning=no-timestamp -C $remote_workspace -xzf $remote_archive'"
echo "Archive extracted. Checking board binaries..."

board_check_script=$(cat <<'BOARD_CHECK'
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
BOARD_CHECK
)
encoded_check=$(printf '%s\n' "$board_check_script" | base64 | tr -d '\n')
adb_shell_checked "printf %s '$encoded_check' | base64 -d | bash -se"
echo "Uploaded via ADB to $remote_workspace. The active workspace files were updated."
