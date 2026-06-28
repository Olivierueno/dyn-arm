#!/usr/bin/env bash
# One launcher to replace the old pile of *.sh scripts.
# Sources ROS + the workspace, finds the VcXsrv DISPLAY (for RViz), then ros2 launch.
#
#   bash tools/ros_launch.sh arm.launch.py          # sliders + twin + real motors
#   bash tools/ros_launch.sh display.launch.py      # RViz twin + sliders (no motors)
#   bash tools/ros_launch.sh floor_test.launch.py   # back-drive + validate the virtual floor
set -e
# Locate the repo from this script's own path, so it works wherever it is cloned.
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source /opt/ros/humble/setup.bash
source "$REPO/ros2_ws/install/setup.bash"

# Find an X server on port 6000 (VcXsrv) so RViz can render (WSLg is broken on this machine).
for ip in $(ip route list default 2>/dev/null | awk '{print $3}') \
          $(awk '/nameserver/{print $2}' /etc/resolv.conf 2>/dev/null) 127.0.0.1; do
  if timeout 2 bash -c "echo > /dev/tcp/$ip/6000" 2>/dev/null; then export DISPLAY="$ip:0.0"; break; fi
done
export LIBGL_ALWAYS_SOFTWARE=1
export QT_QPA_PLATFORM=xcb

ros2 launch dyn_arm "$1"
