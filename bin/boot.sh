#!/bin/bash

# Wait until PS4 controller appears (via evdev)
while ! grep -ql "Wireless Controller" /sys/class/input/event*/device/name 2>/dev/null; do
  echo "Waiting for PS4 controller..."
  sleep 5
done

# Run Qmini binary
exec /home/lckfb/RoboTamerSdk4Qmini/bin/run_interface
