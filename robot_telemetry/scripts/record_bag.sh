#!/usr/bin/env bash
# ----------------------------------------------------------------------
# Trigger standard ROS 2 bag recording of the robot telemetry topics.
#
# Usage:
#   record_bag.sh [output_dir] [extra ros2 bag record args]
#
# Examples:
#   record_bag.sh                          # -> ~/ros2_ws/logs/bags/bag_<ts>
#   timeout 10 record_bag.sh /tmp/mybags   # record for 10 seconds
# ----------------------------------------------------------------------
set -eo pipefail

if [ -z "${ROS_DISTRO:-}" ]; then
  # shellcheck disable=SC1091
  source /opt/ros/jazzy/setup.bash
fi
if [ -f "$HOME/ros2_ws/install/setup.bash" ]; then
  # shellcheck disable=SC1091
  source "$HOME/ros2_ws/install/setup.bash"
fi

set -u

BAG_DIR="${1:-$HOME/ros2_ws/logs/bags}"
if [ $# -gt 0 ]; then shift; fi
mkdir -p "$BAG_DIR"

TOPICS="/cmd_vel /odom /scan /joint_states /arm_pose /pick_place_status"
BAG_NAME="bag_$(date +%Y%m%d_%H%M%S)"

echo "[record_bag] recording:$TOPICS"
echo "[record_bag] output:  $BAG_DIR/$BAG_NAME"
ros2 bag record -o "$BAG_DIR/$BAG_NAME" $TOPICS "$@"
echo "[record_bag] done:    $BAG_DIR/$BAG_NAME"
