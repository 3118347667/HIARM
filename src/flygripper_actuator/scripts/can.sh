#!/bin/bash
# Check if can0 interface is already up
if ip link show can0 | grep -q "state UP"; then
  echo "can0 interface is already up"
else
  # Set up the can0 interface
  echo "${PASSWORD}" | sudo -S ip link set can0 type can bitrate 1000000
  echo "${PASSWORD}" | sudo -S ifconfig can0 up
  echo "can0 interface is set up and brought up"
fi