#!/usr/bin/env bash
set -e
SICNAV_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
SICNAV_WORKSPACE="$(cd "$SICNAV_ROOT/../.." && pwd)"
SICNAV_PYTHON="${SICNAV_PYTHON:-$SICNAV_WORKSPACE/.venv/bin/python}"
source /opt/ros/jazzy/setup.bash
if [ -f "$HOME/sicnav_airport_ws/install/setup.bash" ]; then
    source "$HOME/sicnav_airport_ws/install/setup.bash"
fi
source "$SICNAV_WORKSPACE/install/setup.bash"
export ACADOS_SOURCE_DIR="$SICNAV_ROOT/acados"
export LD_LIBRARY_PATH="$ACADOS_SOURCE_DIR/lib:${LD_LIBRARY_PATH:-}"
export OMP_NUM_THREADS="${OMP_NUM_THREADS:-1}"
export OPENBLAS_NUM_THREADS="${OPENBLAS_NUM_THREADS:-1}"
export MKL_NUM_THREADS="${MKL_NUM_THREADS:-1}"
# CPU VM profile: retain 15 MPC samples, generate 25 rather than the model's 100.
export SICNAV_JMID_SAMPLES="${SICNAV_JMID_SAMPLES:-25}"
export MPLBACKEND=Agg
export PYTHONUNBUFFERED=1
cd "$SICNAV_WORKSPACE"
exec ros2 launch sicnav_airport_bringup airport_cropped_tb4.launch.py \
    transport:=local sicnav_python:="$SICNAV_PYTHON" sicnav_root:="$SICNAV_ROOT/sicnav" "$@"
