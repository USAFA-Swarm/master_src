#!/usr/bin/env bash
# view_camera.sh — Run on the GROUND LAPTOP to see live camera feed from Pi.
#
# Strategy: runs the ROS2 image viewer ON THE PI via SSH with X11 forwarding
# so the window appears on your ground laptop screen. This avoids the
# Humble (ground) / Jazzy (Pi) DDS incompatibility that crashes camera_ros
# when the ground laptop subscribes directly.
#
# Prerequisites (one-time, on Pi):
#   sudo apt install -y ros-jazzy-image-tools
#
# Prerequisites (one-time, on ground laptop):
#   sudo apt install -y openssh-client   # already installed on Ubuntu
#   # X server must be running — it is by default on Ubuntu desktop
#
# Usage:
#   1. Start camera on Pi first (separate SSH terminal):
#        export ROS_DOMAIN_ID=13 && export ROS_LOCALHOST_ONLY=1
#        source /opt/ros/jazzy/setup.bash && source ~/master_src/install/setup.bash
#        ros2 launch onboard camera_only.launch.py
#
#   2. Then on ground laptop run this script:
#        chmod +x scripts/view_camera.sh
#        ./scripts/view_camera.sh
#
# The showimage window will appear on your ground laptop screen.
# Press Ctrl-C to close.

PI_HOST="hare@192.168.168.102"
TOPIC="/camera/image_raw"

echo "[view_camera] Connecting to Pi ($PI_HOST) with X11 forwarding..."
echo "[view_camera] A window should appear within a few seconds."
echo "[view_camera] Press Ctrl-C to exit."
echo ""

ssh -X "$PI_HOST" bash -s << 'REMOTE'
set -e
export ROS_DOMAIN_ID=13
export ROS_LOCALHOST_ONLY=1
source /opt/ros/jazzy/setup.bash
source ~/master_src/install/setup.bash

# Wait briefly for DDS discovery if camera was just started
sleep 1

# Confirm topic is publishing before launching viewer
if ! ros2 topic list 2>/dev/null | grep -q "/camera/image_raw"; then
    echo "[view_camera] ERROR: /camera/image_raw not found."
    echo "  Start camera first with: ros2 launch onboard camera_only.launch.py"
    exit 1
fi

echo "[view_camera] Topic found. Opening image viewer..."
exec ros2 run image_tools showimage \
    --ros-args \
    -r image:=/camera/image_raw \
    -p reliability:=best_effort
REMOTE
