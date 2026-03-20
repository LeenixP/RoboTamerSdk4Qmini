#!/bin/bash

# Wait until PS4 controller appears
while ! grep -ql "Wireless Controller" /sys/class/input/event*/device/name 2>/dev/null; do
  echo "Waiting for PS4 controller..."
  sleep 5
done

# Set the relevant variables
LOG_TIME=$(date +%Y_%m_%d-%H)
LOG_FILE_NAGE=run_interface_$(date +%Y%m%d_%H%M).log

# Create log directory
mkdir -p /home/lckfb/robot-logs/$LOG_TIME

cd /home/lckfb/RoboTamerSdk4Qmini/bin/

# Run Qmini binary
exec /home/lckfb/RoboTamerSdk4Qmini/bin/run_interface 2>&1 | tee /home/lckfb/robot-logs/$LOG_TIME/$LOG_FILE_NAGE
