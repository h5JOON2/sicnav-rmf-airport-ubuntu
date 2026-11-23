#!/usr/bin/env bash
set -euo pipefail
ROOT=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
export SICNAV_SIM_ASSETS="${SICNAV_SIM_ASSETS:-$ROOT}"
export SICNAV_HEADLESS="${SICNAV_HEADLESS:-0}"
exec bash "$ROOT/experiments/mac_offload/scripts/run_lidar_simulation.sh" "$@"
