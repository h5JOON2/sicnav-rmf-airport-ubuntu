#!/bin/zsh
set -eu
TASK_DIR="${0:A:h}"
export SICNAV_MP_ROOT="${SICNAV_MP_ROOT:?Set SICNAV_MP_ROOT to your existing controller environment root}"
CONTROLLER_DIR="${SICNAV_CONTROLLER_DIR:-$TASK_DIR/controller}"
if [[ ! -f "$CONTROLLER_DIR/run_mp_tcp.command" ]]; then CONTROLLER_DIR="$TASK_DIR/../mp_tcp"; fi
exec zsh "$CONTROLLER_DIR/run_mp_tcp.command" --host 127.0.0.1 --port 9150 --device mps --origin-x 136.5 --origin-y -41.7 --mpc-horizon 8 --work-dir "$TASK_DIR/runtime-controller" "$@"
