#!/usr/bin/env bash
set -eo pipefail
ROOT=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
source /opt/ros/jazzy/setup.bash
if [[ -f "$HOME/sicnav_airport_ws/install/setup.bash" ]]; then
  source "$HOME/sicnav_airport_ws/install/setup.bash"
fi
exec /usr/bin/python3 "$ROOT/experiments/mac_offload/scripts/robot_offload_gateway.py" "$@"
