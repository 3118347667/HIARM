#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
interface=wlan0

usage() {
  echo "Usage: $0" >&2
  exit 2
}

if [[ $# -eq 0 ]]; then
  printf 'Choose Wi-Fi for %s:\n  1) DIiffRobot_2_5G\n  2) dongbudemo\n' "$interface"
  read -r -p 'Enter 1 or 2: ' choice || exit 1
  interactive=true
elif [[ $# -eq 2 && $1 == --apply ]]; then
  choice=$2
  interactive=false
else
  usage
fi

case "$choice" in
  1) ssid=DIiffRobot_2_5G ;;
  2) ssid=dongbudemo ;;
  *) echo 'Invalid selection; enter 1 or 2.' >&2; exit 2 ;;
esac

command -v nmcli >/dev/null || { echo 'nmcli is required on the board.' >&2; exit 1; }
nmcli -g connection.id connection show "$ssid" >/dev/null 2>&1 || {
  echo "No saved NetworkManager profile named $ssid. Configure it on the board first." >&2
  exit 1
}

active_profile=$(nmcli -g GENERAL.CONNECTION device show "$interface" 2>/dev/null || true)
if [[ -n "$active_profile" && "$active_profile" != -- ]]; then
  active_ssid=$(nmcli -g 802-11-wireless.ssid connection show "$active_profile" 2>/dev/null || true)
  if [[ "$active_ssid" == "$ssid" ]]; then
    echo "Already connected to $ssid on $interface."
    exit 0
  fi
fi

if [[ "$interactive" == true && -n ${SSH_CONNECTION:-} ]]; then
  log_file="$HOME/.cache/hiarm-wifi-switch.log"
  mkdir -p "$(dirname "$log_file")"
  umask 077
  nohup "$script_dir/switch_wifi.sh" --apply "$choice" > "$log_file" 2>&1 < /dev/null &
  echo "Switching to $ssid in the background. SSH may disconnect."
  echo "After reconnecting, see $log_file for the result."
  exit 0
fi

echo "Switching $interface to $ssid..."
if [[ $EUID -eq 0 ]]; then
  /usr/bin/nmcli --wait 90 connection up id "$ssid" ifname "$interface"
else
  sudo -n /usr/bin/nmcli --wait 90 connection up id "$ssid" ifname "$interface" || {
    echo "Could not activate $ssid. Check hotspot availability and run scripts/setup_wifi_switch_sudoers.sh as root on the board." >&2
    exit 1
  }
fi
echo "Connected to $ssid."
NMCLI_PAGER=cat nmcli -g IP4.ADDRESS device show "$interface" || true
