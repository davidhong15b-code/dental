
#!/usr/bin/env bash

# MTRN4231 Dental Hygiene Robot Environment
# Usage: source scripts/source_ros.sh

if [[ "${BASH_SOURCE[0]}" == "$0" ]]; then
    echo "ERROR: Source this script instead of executing it."
    exit 1
fi

DENTAL_WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

# Base ROS 2 installation
source /opt/ros/humble/setup.bash || return 1

# MoveIt 2 underlay
source "$HOME/ws_moveit2/install/local_setup.bash" || return 1

# Universal Robots driver underlay
source "$HOME/ws_ur_driver/install/local_setup.bash" || return 1

# ROS 2 DDS Domain
# Default project domain; allow user override.
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-50}"

# Dental project overlay (if built)
# Skip while building the workspace itself.
if [[ "${1:-}" != "--underlays-only" && \
      -f "$DENTAL_WS/install/local_setup.bash" ]]; then
    source "$DENTAL_WS/install/local_setup.bash" || return 1
fi

echo "ROS 2 Humble environment ready."
echo "Project workspace: $DENTAL_WS"
echo "ROS_DOMAIN_ID: $ROS_DOMAIN_ID"

unset DENTAL_WS
