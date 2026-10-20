#!/usr/bin/env bash
set -euo pipefail

interface="${1:-can0}"
if ip link show "$interface" | grep -q "state UP"; then
  echo "$interface is already up"
  exit 0
fi

if [[ "$EUID" -eq 0 ]]; then
  ip link set "$interface" type can bitrate 1000000
  ip link set "$interface" up
elif sudo -n true 2>/dev/null; then
  sudo -n ip link set "$interface" type can bitrate 1000000
  sudo -n ip link set "$interface" up
else
  echo "Configuring $interface needs root. Run this setup as root once." >&2
  exit 1
fi
echo "$interface is set up and brought up"
