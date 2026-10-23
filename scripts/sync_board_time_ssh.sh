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

# Board SSH and sudo password, provided by the user for unattended use.
board_password='rock'

# Reuse one authenticated connection so the clock sample is taken after login.
socket_dir=$(mktemp -d /tmp/hiarm-time-ssh.XXXXXX)
socket="$socket_dir/control"
ssh_options=(-o ConnectTimeout=15 -o ServerAliveInterval=15 -o ServerAliveCountMax=4 -o StrictHostKeyChecking=accept-new)
cleanup() {
  ssh -S "$socket" -O exit -o BatchMode=yes "$target" >/dev/null 2>&1 || true
  rm -rf -- "$socket_dir"
}
trap cleanup EXIT

if ! ssh -M -S "$socket" -fN -o BatchMode=yes "${ssh_options[@]}" "$target" </dev/null 2>/dev/null; then
  command -v expect >/dev/null || { echo "SSH password login needs expect on the Mac." >&2; exit 1; }
  rm -f -- "$socket"
  BOARD_SSH_TARGET="$target" BOARD_SSH_SOCKET="$socket" BOARD_SSH_PASSWORD="$board_password" expect <<'EXPECT'
set timeout 25
log_user 1
spawn ssh -M -S $env(BOARD_SSH_SOCKET) -fN \
  -o ConnectTimeout=15 \
  -o ServerAliveInterval=15 \
  -o ServerAliveCountMax=4 \
  -o StrictHostKeyChecking=accept-new \
  -o PubkeyAuthentication=no \
  -o PreferredAuthentications=password,keyboard-interactive \
  -o NumberOfPasswordPrompts=1 \
  $env(BOARD_SSH_TARGET)
expect {
  -re {(?i)(password|passphrase):[ \t]*$} {
    send -- "$env(BOARD_SSH_PASSWORD)\r"
    exp_continue
  }
  eof {}
  timeout { puts stderr "SSH password authentication timed out."; exit 1 }
}
set result [wait]
exit [lindex $result 3]
EXPECT
fi

remote() {
  ssh -S "$socket" -o BatchMode=yes "${ssh_options[@]}" "$target" "$1"
}

board_before=$(remote 'date -u "+%Y-%m-%d %H:%M:%S UTC"')
echo "SSH target:   $target"
echo "Board before: $board_before"

sudo_mode=$(remote 'if [ "$(id -u)" -eq 0 ]; then echo root; elif sudo -n /usr/bin/date -u +%s >/dev/null 2>&1; then echo nopass; else echo password; fi')
if [[ "$sudo_mode" != root && "$sudo_mode" != nopass && "$sudo_mode" != password ]]; then
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
    board_after_epoch=$(printf '%s\n' "$board_password" | remote "sudo -S -p '' /usr/bin/date -u -s '@$mac_epoch' >/dev/null && date -u +%s")
    ;;
esac
unset board_password

if [[ ! "$board_after_epoch" =~ ^[0-9]+$ ]]; then
  echo "Could not set or read the board system clock." >&2
  exit 1
fi

board_after=$(remote 'date -u "+%Y-%m-%d %H:%M:%S UTC"')
echo "Mac now:      $(date -u '+%Y-%m-%d %H:%M:%S UTC')"
echo "Board after:  $board_after"
echo "Board system clock updated. Network NTP may adjust it again when available."
