#!/usr/bin/env bash
set -e
project_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
mode="${1:-help}"
if [[ "$mode" == help || "$mode" == --help || "$mode" == -h ]]; then
  echo 'Usage: bash scripts/run.sh {simulation|wheel|localization|rviz|teleop|record} [extra arguments]'
  exit 0
fi
case "$mode" in
  simulation|wheel|localization|rviz|teleop|record) ;;
  *) echo "Unknown mode: $mode" >&2; exit 2 ;;
esac
shift
if [[ ! -f "$project_root/.env.local" ]]; then
  echo 'Run python3 scripts/configure.py first; see README.' >&2
  exit 1
fi
source "$project_root/.env.local"
mkdir -p "$project_root/logs"
if [[ "$mode" == simulation ]]; then
  # The simulator embeds a different Python version from system ROS Humble.
  (
    source "$project_root/scripts/setup-env.sh"
    exec /usr/bin/python3 "$project_root/runtime/astra-depth-range.py"
  ) > "$project_root/logs/depth-range.log" 2>&1 &
  range_pid=$!
  trap 'kill "$range_pid" 2>/dev/null || true' EXIT
  unset PYTHONPATH PYTHONHOME AMENT_PREFIX_PATH
  export ROS_DISTRO=humble ROS_DOMAIN_ID=26 ROS_LOCALHOST_ONLY=0 RMW_IMPLEMENTATION=rmw_fastrtps_cpp
  export LD_LIBRARY_PATH="$ISAAC_SIM_ROOT/exts/isaacsim.ros2.core/humble/lib"
  cd "$ISAAC_SIM_ROOT"
  ./isaac-sim.sh --/isaac/startup/ros_bridge_extension=isaacsim.ros2.bridge \
    --exec "$project_root/runtime/open-localization-scene.py" "$@"
  exit $?
fi
source "$project_root/scripts/setup-env.sh"
case "$mode" in
  wheel) exec ros2 launch "$project_root/runtime/wheel-control.launch.py" "$@" ;;
  localization) exec ros2 launch "$project_root/runtime/guarded-localization.launch.py" "$@" ;;
  rviz) exec ros2 run rviz2 rviz2 -d "$project_root/runtime/localization.rviz" --ros-args -p use_sim_time:=true "$@" ;;
  teleop) exec ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p speed:=0.1 -p turn:=0.3 "$@" ;;
  record)
    cd "$project_root"
    exec ros2 bag record -o "data/manual-$(date +%Y%m%d-%H%M%S)" \
      /clock /tf /tf_static /depth/points /wheel/odom /wheel/joint_states \
      /pcl_pose /alignment_status /ground_truth/pose /ground_truth/rear_axle_pose \
      /wheel_reference/pose /cmd_vel "$@"
    ;;
esac
