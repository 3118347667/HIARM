#!/usr/bin/env bash
set -euo pipefail

account="${1:-rock}"
if [[ "$EUID" -ne 0 ]]; then
  echo "Run this one-time board setup as root (for example through a root ADB shell)." >&2
  exit 1
fi
if [[ ! "$account" =~ ^[a-z_][a-z0-9_-]*$ ]]; then
  echo "Invalid account name: $account" >&2
  exit 2
fi
id "$account" >/dev/null
getent group dialout >/dev/null || {
  echo "Missing dialout group on this board." >&2
  exit 1
}

usermod -aG dialout "$account"
cat > /etc/udev/rules.d/70-hiarm-serial.rules <<'RULES'
SUBSYSTEM=="tty", KERNEL=="ttyACM[0-9]*", GROUP:="dialout", MODE:="0660"
SUBSYSTEM=="tty", KERNEL=="ttyUSB[0-9]*", GROUP:="dialout", MODE:="0660"
SUBSYSTEM=="tty", KERNEL=="ttyS4", GROUP:="dialout", MODE:="0660"
SUBSYSTEM=="tty", KERNEL=="ttyS7", GROUP:="dialout", MODE:="0660"
RULES
chmod 0644 /etc/udev/rules.d/70-hiarm-serial.rules

cat > /etc/security/limits.d/90-hiarm-rtprio.conf <<LIMITS
$account soft rtprio 95
$account hard rtprio 95
LIMITS
chmod 0644 /etc/security/limits.d/90-hiarm-rtprio.conf

udevadm control --reload-rules
for device in /dev/ttyACM[0-9]* /dev/ttyUSB[0-9]* /dev/ttyS4 /dev/ttyS7; do
  [[ -e "$device" ]] || continue
  chgrp dialout "$device"
  chmod 0660 "$device"
done

echo "Configured serial access and realtime priority for $account."
echo "Disconnect and reconnect SSH so the new group and rtprio limits take effect."
