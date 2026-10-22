#!/usr/bin/env bash
set -euo pipefail

if [[ $# -gt 1 ]]; then
  echo "Usage: $0 [USER@BOARD_IP]" >&2
  exit 2
fi

target="${1:-${BOARD_SSH_TARGET:-}}"
if [[ -z "$target" || "$target" == -* || "$target" != *@* ]]; then
  echo "Invalid SSH target: $target" >&2
  echo "Usage: $0 USER@BOARD_IP (or set BOARD_SSH_TARGET)" >&2
  exit 2
fi

command -v ssh >/dev/null || { echo "ssh is required on the Mac." >&2; exit 1; }

# Reuse one authenticated connection so the clock sample is taken after login.
socket_dir=$(mktemp -d /tmp/hiarm-time-ssh.XXXXXX)
socket="$socket_dir/control"
ssh_options=(-o ConnectTimeout=15 -o ServerAliveInterval=15 -o ServerAliveCountMax=4)
cleanup() {
  ssh -S "$socket" -O exit -o BatchMode=yes "$target" >/dev/null 2>&1 || true
  rm -rf -- "$socket_dir"
}
trap cleanup EXIT

ssh -M -S "$socket" -fN "${ssh_options[@]}" "$target"
remote() {
  ssh -S "$socket" -o BatchMode=yes "${ssh_options[@]}" "$target" "$1"
}

board_before=$(remote 'date -u "+%Y-%m-%d %H:%M:%S UTC"')
echo "SSH target:   $target"
echo "Board before: $board_before"

sudo_mode=$(remote 'if [ "$(id -u)" -eq 0 ]; then echo root; elif sudo -n /usr/bin/date -u +%s >/dev/null 2>&1; then echo nopass; else echo password; fi')
if [[ "$sudo_mode" == password ]]; then
  if [[ ! -r /dev/tty ]]; then
    echo "Board sudo requires a password; run this script from an interactive terminal." >&2
    exit 1
  fi
  read -r -s -p "Sudo password for $target: " sudo_password < /dev/tty
  printf '\n' > /dev/tty
elif [[ "$sudo_mode" != root && "$sudo_mode" != nopass ]]; then
  echo "Could not determine board sudo access." >&2
  exit 1
fi

# GNU date on the Ubuntu board accepts the Mac's UTC Unix timestamp.
mac_epoch=$(date -u +%s)
case "$sudo_mode" in
  root)
    board_after_epoch=$(remote "date -u -s '@$mac_epoch' >/dev/null && date -u +%s")
    ;;
  nopass)
    board_after_epoch=$(remote "sudo -n /usr/bin/date -u -s '@$mac_epoch' >/dev/null && date -u +%s")
    ;;
  password)
    board_after_epoch=$(printf '%s\n' "$sudo_password" | remote "sudo -S -p '' /usr/bin/date -u -s '@$mac_epoch' >/dev/null && date -u +%s")
    unset sudo_password
    ;;
esac

if [[ ! "$board_after_epoch" =~ ^[0-9]+$ ]]; then
  echo "Could not set or read the board system clock." >&2
  exit 1
fi

board_after=$(remote 'date -u "+%Y-%m-%d %H:%M:%S UTC"')
echo "Mac now:      $(date -u '+%Y-%m-%d %H:%M:%S UTC')"
echo "Board after:  $board_after"
echo "Board system clock updated. Network NTP may adjust it again when available."
