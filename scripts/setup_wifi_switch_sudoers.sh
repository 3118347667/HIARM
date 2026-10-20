#!/usr/bin/env bash
set -euo pipefail

if [[ $EUID -ne 0 ]]; then
  echo "Run this one-time setup as root on the board (for example through a root ADB shell)." >&2
  exit 1
fi

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
rule_file="$script_dir/wifi_switch.sudoers"
target=/etc/sudoers.d/hiarm-wifi-switch

command -v visudo >/dev/null || { echo "visudo is required to validate the sudoers rule." >&2; exit 1; }
id rock >/dev/null || { echo "The rock account is missing." >&2; exit 1; }
[[ -x /usr/bin/nmcli ]] || { echo "/usr/bin/nmcli is missing." >&2; exit 1; }

# Validate the source before replacing the active rule.
visudo -cf "$rule_file"
install -o root -g root -m 0440 "$rule_file" "$target"
visudo -cf "$target"
echo "Installed $target for rock. The two saved Wi-Fi profiles can now be activated without a sudo password."
