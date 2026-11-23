#!/bin/zsh
set -eu
TASK_DIR="${0:A:h}"
export SICNAV_MP_ROOT="${SICNAV_MP_ROOT:?Set SICNAV_MP_ROOT to your existing controller environment root}"
CONTROLLER_DIR="${SICNAV_CONTROLLER_DIR:-$TASK_DIR/controller}"
[[ -f "$CONTROLLER_DIR/run_mp_tcp.command" ]] || { print -u2 'Controller entry point is missing.'; exit 1; }
exec zsh "$CONTROLLER_DIR/run_mp_tcp.command" --host 127.0.0.1 --port 9150 --device mps --origin-x 136.5 --origin-y -41.7 --mpc-horizon 4 --variable-humans --max-speed .31 --max-omega 1.9 --work-dir "$TASK_DIR/runtime-controller" "$@"
