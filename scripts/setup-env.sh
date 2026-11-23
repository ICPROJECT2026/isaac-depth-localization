#!/usr/bin/env bash
# Source this file in each ROS terminal. Isaac itself uses an isolated environment.
project_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
if [[ ! -f /opt/ros/humble/setup.bash ]]; then
  echo 'ROS 2 Humble is required at /opt/ros/humble.' >&2
  return 1
fi
source /opt/ros/humble/setup.bash
if [[ -f "$project_root/.env.local" ]]; then
  source "$project_root/.env.local"
fi
if [[ -f "${ROS_OVERLAY_SETUP:-$project_root/ros_ws/install/setup.bash}" ]]; then
  source "${ROS_OVERLAY_SETUP:-$project_root/ros_ws/install/setup.bash}"
fi
# The supplied scene ROS2Context is authored with domain 26.
export ROS_DOMAIN_ID=26 ROS_LOCALHOST_ONLY=0 RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export OMP_NUM_THREADS=4 OPENBLAS_NUM_THREADS=2
