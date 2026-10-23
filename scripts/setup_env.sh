#!/usr/bin/env bash
# Source this at the top of every terminal:  source scripts/setup_env.sh
#
# Every terminal in a session must end up with the same ROS_DOMAIN_ID and the
# same workspaces. Mismatches produce topics that exist but never deliver, which
# is slow to debug. Adjust the paths below to your machine.

FRANKA_WS="${FRANKA_WS:-$HOME/franka_ros2_ws}"
LAMBDA_WS="${LAMBDA_WS:-$HOME/lambda-07_ws}"
FR3_WS="${FR3_WS:-$HOME/fr3_ws}"          # RealSense + AprilTag, optional

# A conda python on the PATH shadows the system rclpy.
conda deactivate 2>/dev/null || true

source /opt/ros/jazzy/setup.bash

[ -f "$FRANKA_WS/install/setup.bash" ] && source "$FRANKA_WS/install/setup.bash"
[ -f "$LAMBDA_WS/install/setup.bash" ] && source "$LAMBDA_WS/install/setup.bash"
[ -f "$FR3_WS/install/setup.bash" ]    && source "$FR3_WS/install/setup.bash"

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"

echo "ROS_DISTRO=$ROS_DISTRO  DOMAIN=$ROS_DOMAIN_ID  RMW=${RMW_IMPLEMENTATION:-default}"
echo "python3=$(command -v python3)"
