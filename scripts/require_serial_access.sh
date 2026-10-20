#!/usr/bin/env bash
set -euo pipefail

device="${1:-}"
if [[ -z "$device" ]]; then
  echo "Usage: $0 DEVICE" >&2
  exit 2
fi
if [[ ! -e "$device" ]]; then
  echo "Serial device does not exist: $device" >&2
  exit 1
fi
if [[ ! -r "$device" || ! -w "$device" ]]; then
  echo "Current user $(id -un) cannot read and write $device." >&2
  ls -l "$device" >&2
  echo "Configure board serial permissions once as root, then reconnect this SSH session:" >&2
  echo "  ./scripts/setup_serial_permissions.sh (run as root)" >&2
  exit 1
fi
