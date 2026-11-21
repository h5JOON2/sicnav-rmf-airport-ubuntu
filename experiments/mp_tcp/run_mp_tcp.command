#!/bin/zsh
set -eu
TASK_DIR="${0:A:h}"
MP_ROOT="${SICNAV_MP_ROOT:?Set SICNAV_MP_ROOT to the environment containing work/repo, work/venv and work/acados}"
unset SWEEP_RANK_LIB DIRECT_BACKEND ORCA_LM ORCA_RELAX_MODE ORCA_RELAX_MU SWEEP_SQP SWEEP_QP
export SICNAV_MP_ROOT="$MP_ROOT"
export ACADOS_SOURCE_DIR="$MP_ROOT/work/acados"
export DYLD_LIBRARY_PATH="$MP_ROOT/work/acados/lib${DYLD_LIBRARY_PATH:+:$DYLD_LIBRARY_PATH}"
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 VECLIB_MAXIMUM_THREADS=1 NUMEXPR_NUM_THREADS=1
export MPLBACKEND=Agg
export MPLCONFIGDIR="$TASK_DIR/runtime/mpl" PYTHONDONTWRITEBYTECODE=1
mkdir -p "$MPLCONFIGDIR"
exec "$MP_ROOT/work/venv/bin/python" -u "$TASK_DIR/run_mp_tcp.py" "$@"
