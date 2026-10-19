#!/bin/bash

# RK3588 Temperature Monitor Script

# Define thermal zones and their corresponding components
declare -A thermal_zones=(
  ["SOC_Center"]="thermal_zone0"
  ["CPU_A76_0_1"]="thermal_zone1"
  ["CPU_A76_2_3"]="thermal_zone2"
  ["CPU_A55_0_1_2_3"]="thermal_zone3"
  ["PD_Center"]="thermal_zone4"
  ["GPU"]="thermal_zone5"
  ["NPU"]="thermal_zone6"
)

# Function to read temperature and convert to Celsius
read_temp() {
  local temp_raw=$(cat /sys/class/thermal/$1/temp 2>/dev/null)
  if [ -n "$temp_raw" ]; then
    echo "scale=2; $temp_raw/1000" | bc
  else
    echo "N/A"
  fi
}

# Main function
main() {
  echo "====================================="
  echo "    RK3588 Temperature Monitoring    "
  echo "====================================="
  echo "$(date '+%Y-%m-%d %H:%M:%S')"
  echo "-------------------------------------"

  for component in "${!thermal_zones[@]}"; do
    zone=${thermal_zones[$component]}
    temp_c=$(read_temp $zone)
    printf "%-15s: %5s °C\n" "$component" "$temp_c"
  done

  echo "====================================="
}

# Execute main function
main
