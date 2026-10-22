#!/usr/bin/env bash
set -euo pipefail

if [[ $# -gt 1 ]]; then
  echo "Usage: $0 [ADB_SERIAL]" >&2
  exit 2
fi

command -v adb >/dev/null || { echo "adb is required on the Mac." >&2; exit 1; }

serial="${1:-${BOARD_ADB_SERIAL:-}}"
if [[ -z "$serial" ]]; then
  devices=$(adb devices)
  count=0
  while IFS= read -r connected_serial; do
    [[ -n "$connected_serial" ]] || continue
    serial=$connected_serial
    count=$((count + 1))
  done < <(printf '%s\n' "$devices" | awk '$2 == "device" {print $1}')
  if (( count != 1 )); then
    echo "Expected one authorized ADB device; found $count. Pass ADB_SERIAL or set BOARD_ADB_SERIAL." >&2
    exit 1
  fi
fi

state=$(adb -s "$serial" get-state | tr -d '\r\n')
if [[ "$state" != device ]]; then
  echo "ADB device $serial is not connected and authorized." >&2
  exit 1
fi

uid=$(adb -s "$serial" shell 'id -u' | tr -d '\r\n')
if [[ "$uid" != 0 ]]; then
  echo "ADB shell on $serial must run as root to set the board clock (current uid: $uid)." >&2
  exit 1
fi

board_before=$(adb -s "$serial" shell 'date -u "+%Y-%m-%d %H:%M:%S UTC"' | tr -d '\r')
echo "ADB device: $serial"
echo "Board before: $board_before"

# Sample the Mac clock immediately before sending the date command.
mac_epoch=$(date -u +%s)
board_after_epoch=$(adb -s "$serial" shell "date -u -s '@$mac_epoch' >/dev/null && date -u +%s" | tr -d '\r\n')
if [[ ! "$board_after_epoch" =~ ^[0-9]+$ ]]; then
  echo "Could not set or read the board system clock." >&2
  exit 1
fi

board_after=$(adb -s "$serial" shell 'date -u "+%Y-%m-%d %H:%M:%S UTC"' | tr -d '\r')
echo "Mac now:      $(date -u '+%Y-%m-%d %H:%M:%S UTC')"
echo "Board after:  $board_after"
echo "Board system clock updated. Network NTP may adjust it again when available."
