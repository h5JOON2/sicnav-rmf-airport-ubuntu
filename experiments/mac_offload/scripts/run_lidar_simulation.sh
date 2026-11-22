#!/bin/bash
set -euo pipefail
SCRIPT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
TASK_DIR=$(dirname "$SCRIPT_DIR")
# ROS underlays sometimes read unset variables while sourcing.
set +u
source /opt/ros/jazzy/setup.bash
if [[ -f "$HOME/sicnav_airport_ws/install/setup.bash" ]]; then source "$HOME/sicnav_airport_ws/install/setup.bash"; fi
set -u
SIM_ASSETS="${SICNAV_SIM_ASSETS:-$HOME/social_nav_ws/src/sicnav-rmf-airport-main}"
[[ -d "$SIM_ASSETS/models" && -d "$SIM_ASSETS/maps" ]] || { echo 'Set SICNAV_SIM_ASSETS to the airport package with models/ and maps/.' >&2; exit 1; }
export GZ_SIM_RESOURCE_PATH="$SIM_ASSETS/models:/opt/ros/jazzy/share:${GZ_SIM_RESOURCE_PATH:-}"
# Menge expects one directory, not a colon-separated search path.
export MENGE_RESOURCE_PATH="$SIM_ASSETS/maps"
PLUGIN_PREFIX=$(ros2 pkg prefix rmf_building_sim_gz_plugins)
export GZ_SIM_SYSTEM_PLUGIN_PATH="$PLUGIN_PREFIX/lib/rmf_building_sim_gz_plugins:$PLUGIN_PREFIX/lib:/opt/ros/jazzy/lib:${GZ_SIM_SYSTEM_PLUGIN_PATH:-}"
PIDS=()
cleanup() {
  trap - EXIT INT TERM
  # Background jobs can inherit SIGINT ignored. TERM also works after startup failure.
  for p in "${PIDS[@]}"; do kill -TERM "$p" 2>/dev/null || true; done
  for p in "${PIDS[@]}"; do wait "$p" 2>/dev/null || true; done
}
trap cleanup EXIT INT TERM
# EGL offscreen rendering; camera topics and camera inference are not started.
gz sim -s -r --headless-rendering "$TASK_DIR/worlds/airport_lidar_world.sdf" & PIDS+=("$!")
BRIDGE="$(ros2 pkg prefix ros_gz_bridge)/lib/ros_gz_bridge/parameter_bridge"
"$BRIDGE" '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock' '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry' '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist' --ros-args -r __node:=lidar_sim_state_bridge & PIDS+=("$!")
"$BRIDGE" '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan' --ros-args -r __node:=lidar_scan_bridge -p override_frame_id:=tb4_lidar & PIDS+=("$!")
wait "${PIDS[0]}"
